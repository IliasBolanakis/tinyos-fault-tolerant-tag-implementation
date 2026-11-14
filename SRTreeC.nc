/**
 * @file SRTreeC.nc
 * @brief Core TAG data-plane aggregation module (`AM_MYMSG = 23`).
 *
 * Manages bottom-up level-slotted epoch timers (`MyTimer`), bounded `+/-25%`
 * random-walk sensor sampling, query-specific minimal partial aggregates
 * (`SUM`, `AVG`, `VARIANCE`, `MAX`), duplicate-filtered child folding, and
 * synchronous MAC-layer ACK failure recovery coordinated with `TreeTopology`.
 */
#include "SimpleRoutingTree.h"
#ifdef PRINTFDBG_MODE
    #include "printf.h"
#endif

module SRTreeC {
    uses {
        interface Boot;
        interface Leds;
        interface SplitControl as RadioControl;
        interface Random;
        interface ParameterInit<uint16_t> as SeedInit;

        /* Data aggregation Active Message layer (AM 23, unicast partial aggregates) */
        interface Packet as MyPacket;
        interface AMSend as MyAMSend;
        interface AMPacket as MyAMPacket;
        interface Receive as MyReceive;
        interface PacketAcknowledgements as MyPacketAcknowledgements;

        /* TAG bottom-up epoch slotting timer and data queues */
        interface Timer<TMilli> as MyTimer;
        interface PacketQueue as MySendQueue;
        interface PacketQueue as MyReceiveQueue;

        /* Extracted control-plane and helper modules */
        interface TreeTopology;
        interface BackupTable;
        interface DupFilter;
    }
}
implementation {
    /* Current epoch sequence index */
    uint16_t epochCounter = 0;

    /* Static transmission buffer and radio lock for AM_MYMSG */
    message_t radioMySendPkt;
    bool MySendBusy = FALSE;

    /* Failover retry and epoch-slotting state */
    uint8_t txRetryCount = 0;
    uint8_t parentFailCount = 0;
    uint32_t lastMyFireTime = 0;             // Anchor timestamp for TAG slot shifting & deadline checks
    uint8_t lastFireDepth = DEPTH_UNATTACHED;

    /* Saved outbound epoch snapshot for zero-loss rerouting upon parent/root failure */
    uint32_t savedSumSq = 0;
    uint16_t savedSum = 0;
    uint8_t savedCount = 0;
    uint8_t savedMax = 0;
    uint32_t savedEpochTime = 0;             // Timestamp & depth when snapshot was captured
    uint8_t savedEpochDepth = DEPTH_UNATTACHED;
    bool hasUnsentEpochData = FALSE;
    bool hasPendingChildAgg = FALSE;         // TRUE when a post-fire child aggregate awaits forwarding

    /* Full cumulative epoch snapshot (local reading + all children received in current epoch) */
    uint32_t epochSumSq = 0;
    uint16_t epochSum = 0;
    uint8_t epochCount = 0;
    uint8_t epochMax = 0;
    uint16_t lastAckedDest = INVALID_NODE_ID; // Parent ID that last ACKed our primary epoch packet
    uint8_t txPktToggle = 0;                  // 1-bit MSB toggle distinguishing post-fire deltas from MAC retries

    /* Cached query configuration and local sensor measurement */
    uint8_t queryType = 0;
    uint8_t epochDuration = 0;
    uint8_t myVal = 0;

    /* Running subtree accumulators for the current epoch */
    uint32_t aggSumSq = 0;
    uint16_t aggSum = 0;
    uint8_t aggCount = 0;
    uint8_t aggMax = 0;

    task void sendMyTask();
    task void receiveMyTask();

    void resetEpochAccumulators() {
        aggSumSq = 0;
        aggSum = 0;
        aggCount = 0;
        aggMax = 0;
        hasPendingChildAgg = FALSE;
    }

    /**
     * @brief Flushes stale or superseded outbound data packets from `MySendQueue`.
     */
    void clearMySendQueue() {
        while (!(call MySendQueue.empty())) {
            call MySendQueue.dequeue();
        }
    }

    /**
     * @brief Folds any pending post-fire child deltas (`agg*`) into both `epoch*`
     *        and `saved*` accumulators and clears `agg*`.
     */
    void foldPendingChildAggIntoSnapshots() {
        epochSum += aggSum;
        epochSumSq += aggSumSq;
        epochCount += aggCount;
        if (aggMax > epochMax) {
            epochMax = aggMax;
        }
        savedSum += aggSum;
        savedSumSq += aggSumSq;
        savedCount += aggCount;
        if (aggMax > savedMax) {
            savedMax = aggMax;
        }
        resetEpochAccumulators();
    }

    /**
     * @brief Shifts `MyTimer`'s periodic phase relative to the node's last firing slot
     *        while keeping `t0` within `(now - epochMillis, now]` to prevent unsigned
     *        underflow in TinyOS `Timer.startPeriodicAt(t0, dt)`.
     */
    void rescheduleMyTimerForDepthChange(uint8_t newDepth) {
        uint32_t now = call MyTimer.getNow();
        uint32_t epochMillis = SEC_TO_BMILLI(epochDuration);
        int32_t shift = ((int32_t)lastFireDepth - (int32_t)newDepth) * (int32_t)TAG_SLOT_MILLIS;
        uint32_t t0 = (uint32_t)((int32_t)lastMyFireTime + shift);
        bool futureSlot = ((int32_t)(now - t0) < 0);

        // TinyOS VirtualizeTimerC computes elapsed = now - t0 unsigned; keep t0 in (now - epochMillis, now]
        while ((int32_t)(now - t0) < 0) {
            t0 -= epochMillis;
        }
        while ((now - t0) >= epochMillis) {
            t0 += epochMillis;
        }

        // Only advance the fire anchor if the shifted slot has already passed in this epoch
        if (!futureSlot || epochCounter == 0) {
            lastMyFireTime = t0;
            lastFireDepth = newDepth;
        }
        call MyTimer.startPeriodicAt(t0, epochMillis);
    }

    /**
     * @brief Generates the node's sensor reading for the current epoch (+/-25% random walk).
     */
    uint8_t sampleSensorReading() {
        if (epochCounter <= 1 || myVal == 0) {
            myVal = (uint8_t)((call Random.rand16() % 100) + 1);
        } else {
            uint16_t delta = ((uint16_t)myVal * 25) / 100;
            uint16_t minVal = (myVal > delta) ? (myVal - delta) : 1;
            uint16_t maxVal = myVal + delta;
            uint16_t span;

            if (minVal < 1)   minVal = 1;
            if (maxVal > 100) maxVal = 100;

            span = maxVal - minVal + 1;
            myVal = (uint8_t)(minVal + (call Random.rand16() % span));
        }
        return myVal;
    }

    /**
     * @brief Constructs and enqueues a query-specific minimal data packet to `parentID`.
     *        Packs the 1-bit `txPktToggle` into the unused MSB of `sum` (bit 15) or
     *        `maxVal` (bit 7) so the parent's duplicate filter distinguishes legitimate
     *        post-fire deltas from MAC retries with 0 extra packet bytes.
     */
    void enqueueDataPacketToParent(uint16_t tSum, uint32_t tSumSq, uint8_t tCount, uint8_t tMax) {
        message_t tmp;
        uint16_t curParent = call TreeTopology.getParent();

        if ((call TreeTopology.isRoot()) || curParent >= INVALID_NODE_ID || call MySendQueue.full()) {
            return;
        }
        memset(&tmp, 0, sizeof(message_t));

        if (queryType == QUERY_SUM) {
            SumMsg* pkt = (SumMsg*)(call MyPacket.getPayload(&tmp, sizeof(SumMsg)));
            if (pkt == NULL) return;
            pkt->sum = PACK_SUM_TOGGLE(tSum, txPktToggle);
            call MyAMPacket.setDestination(&tmp, curParent);
            call MyPacket.setPayloadLength(&tmp, sizeof(SumMsg));
        } else if (queryType == QUERY_AVG) {
            AvgMsg* pkt = (AvgMsg*)(call MyPacket.getPayload(&tmp, sizeof(AvgMsg)));
            if (pkt == NULL) return;
            pkt->sum = PACK_SUM_TOGGLE(tSum, txPktToggle);
            pkt->count = tCount;
            call MyAMPacket.setDestination(&tmp, curParent);
            call MyPacket.setPayloadLength(&tmp, sizeof(AvgMsg));
        } else if (queryType == QUERY_VARIANCE) {
            VarMsg* pkt = (VarMsg*)(call MyPacket.getPayload(&tmp, sizeof(VarMsg)));
            if (pkt == NULL) return;
            pkt->sumSq = tSumSq;
            pkt->sum = PACK_SUM_TOGGLE(tSum, txPktToggle);
            pkt->count = tCount;
            call MyAMPacket.setDestination(&tmp, curParent);
            call MyPacket.setPayloadLength(&tmp, sizeof(VarMsg));
        } else if (queryType == QUERY_MAX) {
            MaxMsg* pkt = (MaxMsg*)(call MyPacket.getPayload(&tmp, sizeof(MaxMsg)));
            if (pkt == NULL) return;
            pkt->maxVal = PACK_MAX_TOGGLE(tMax, txPktToggle);
            call MyAMPacket.setDestination(&tmp, curParent);
            call MyPacket.setPayloadLength(&tmp, sizeof(MaxMsg));
        } else {
            return;
        }

        // Request synchronous hardware/MAC-layer ACK from parentID (0 extra bytes)
        call MyPacketAcknowledgements.requestAck(&tmp);

        if (call MySendQueue.enqueue(tmp) == SUCCESS) {
            if (!MySendBusy && !(call TreeTopology.isRoutingSendBusy())) {
                post sendMyTask();
            }
        }
    }

    /**
     * @brief Forwards saved epoch snapshot to the active parent if the epoch window
     *        at the Root is still open (prevents ghost children).
     */
    void forwardSavedEpochIfOpen(bool isNewTreeSeq, uint32_t now) {
        bool epochStillOpen;
        uint32_t myJitter = SLOT_JITTER_MILLIS(TOS_NODE_ID);
        if (!hasUnsentEpochData) {
            return;
        }

        epochStillOpen = isNewTreeSeq
            ? ((now - savedEpochTime) < HALF_SEC_TO_BMILLI(epochDuration))
            : (savedEpochDepth < DEPTH_UNATTACHED &&
               ((now - savedEpochTime) + HOP_DELAY_MILLIS) < (((uint32_t)savedEpochDepth * TAG_SLOT_MILLIS) + myJitter));

        clearMySendQueue();
        if (epochStillOpen) {
            if (hasPendingChildAgg) {
                foldPendingChildAggIntoSnapshots();
            }
            if (!MySendBusy) {
                txRetryCount = 0;
                enqueueDataPacketToParent(savedSum, savedSumSq, savedCount, savedMax);
            }
        } else {
            hasUnsentEpochData = FALSE;
        }
    }

    /**
     * @brief Outputs the final network-wide aggregate at the active Root node.
     */
    void logRootEpochResult(uint16_t tSum, uint32_t tSumSq, uint8_t tCount, uint8_t tMax) {
        if (queryType == QUERY_SUM) {
            dbg("SRTreeC", ">>> [EPOCH %u RESULT (Root=%u) | Query=SUM | EpochDur=%us] Final SUM = %u (from %u nodes) <<<\n",
                epochCounter, TOS_NODE_ID, epochDuration, tSum, tCount);
        } else if (queryType == QUERY_AVG) {
            double avg = (tCount > 0) ? ((double)tSum / (double)tCount) : 0.0;
            dbg("SRTreeC", ">>> [EPOCH %u RESULT (Root=%u) | Query=AVG | EpochDur=%us] Final AVG = %.2f (Sum=%u, Count=%u) <<<\n",
                epochCounter, TOS_NODE_ID, epochDuration, avg, tSum, tCount);
        } else if (queryType == QUERY_VARIANCE) {
            double mean = (tCount > 0) ? ((double)tSum / (double)tCount) : 0.0;
            double meanSq = (tCount > 0) ? ((double)tSumSq / (double)tCount) : 0.0;
            double variance = meanSq - (mean * mean);
            if (variance < 0.0) variance = 0.0;
            dbg("SRTreeC", ">>> [EPOCH %u RESULT (Root=%u) | Query=VARIANCE | EpochDur=%us] Final VARIANCE = %.2f (Sum=%u, SumSq=%lu, Count=%u) <<<\n",
                epochCounter, TOS_NODE_ID, epochDuration, variance, tSum, (unsigned long)tSumSq, tCount);
        } else if (queryType == QUERY_MAX) {
            dbg("SRTreeC", ">>> [EPOCH %u RESULT (Root=%u) | Query=MAX | EpochDur=%us] Final MAX = %u <<<\n",
                epochCounter, TOS_NODE_ID, epochDuration, tMax);
        }
    }

    event void Boot.booted() {
        uint16_t seed = (uint16_t)((sim_time() % 1024) + (TOS_NODE_ID * 137) + 1);

        call RadioControl.start();
        MySendBusy = FALSE;
        resetEpochAccumulators();
        call DupFilter.reset();
        epochCounter = 0;
        lastMyFireTime = 0;
        lastFireDepth = DEPTH_UNATTACHED;
        savedEpochTime = 0;
        savedEpochDepth = DEPTH_UNATTACHED;
        hasUnsentEpochData = FALSE;
        lastAckedDest = INVALID_NODE_ID;
        txPktToggle = 0;
        txRetryCount = 0;
        parentFailCount = 0;

        call SeedInit.init(seed);
        call Random.rand16();
        call Random.rand16();

        if (TOS_NODE_ID == 0) {
            queryType = (uint8_t)((call Random.rand16() % 4) + 1);
            epochDuration = (uint8_t)(((call Random.rand16() % 8) + 3) * 10);
            call TreeTopology.initBootState(TRUE, queryType, epochDuration);

            dbg("Boot", "ROOT (Node 0) booted: selected Query=%u (%s), EpochDuration=%u sec\n",
                queryType, call TreeTopology.getQueryName(queryType), epochDuration);
        } else {
            queryType = 0;
            epochDuration = 0;
            call TreeTopology.initBootState(FALSE, 0, 0);

            dbg("Boot", "Sensor Node %u booted: curdepth=%u, parentID=%u\n",
                TOS_NODE_ID, call TreeTopology.getDepth(), call TreeTopology.getParent());
        }
    }

    event void RadioControl.startDone(error_t err) {
        if (err == SUCCESS) {
            dbg("Radio", "Radio initialized successfully!\n");
            call TreeTopology.onRadioStarted(call MyTimer.getNow());
        } else {
            call RadioControl.start();
        }
    }

    event void RadioControl.stopDone(error_t err) {
        dbg("Radio", "Radio stopped!\n");
    }

    /**
     * @brief Epoch timer handler: fires in bottom-up TAG level order.
     */
    event void MyTimer.fired() {
        uint8_t sample;
        uint32_t totalSumSq;
        uint16_t totalSum;
        uint8_t totalCount;
        uint8_t totalMax;
        uint32_t now = call MyTimer.getNow();
        uint8_t curDepth = call TreeTopology.getDepth();
        uint16_t curParent = call TreeTopology.getParent();
        bool rootFlag = call TreeTopology.isRoot();

        // If MyTimer was phase-shifted to a shallower slot within the same epoch,
        // absorb the alignment tick without double-sampling or advancing epochCounter.
        if (epochCounter > 0 && lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED &&
            (now - lastMyFireTime) < HALF_SEC_TO_BMILLI(epochDuration)) {
            lastMyFireTime = now;
            if (curDepth < DEPTH_UNATTACHED) {
                lastFireDepth = curDepth;
            }
            return;
        }

        lastMyFireTime = now;
        if (curDepth < DEPTH_UNATTACHED) {
            lastFireDepth = curDepth;
        }
        epochCounter++;
        sample = sampleSensorReading();

        clearMySendQueue();

        totalSum = aggSum + sample;
        totalSumSq = aggSumSq + ((uint32_t)sample * (uint32_t)sample);
        totalCount = aggCount + 1;
        totalMax = (sample > aggMax) ? sample : aggMax;

        // Save both full cumulative epoch totals and current outbound packet snapshot
        epochSum = totalSum;
        epochSumSq = totalSumSq;
        epochCount = totalCount;
        epochMax = totalMax;
        savedSum = totalSum;
        savedSumSq = totalSumSq;
        savedCount = totalCount;
        savedMax = totalMax;
        savedEpochTime = now;
        savedEpochDepth = lastFireDepth;
        lastAckedDest = INVALID_NODE_ID;
        txPktToggle = 0;

        resetEpochAccumulators();
        call DupFilter.reset();

        dbg("Lab1", "Epoch %u | Node %u (depth %u, parent %u) sampled local reading = %u\n",
            epochCounter, TOS_NODE_ID, curDepth, curParent, sample);

        if (rootFlag) {
            hasUnsentEpochData = FALSE;
            logRootEpochResult(totalSum, totalSumSq, totalCount, totalMax);
            // Broadcast an authoritative depth-0 beacon at epoch end so any Depth-1 child
            // with a pending false election timer immediately aborts before promoting.
            call TreeTopology.sendRoutingBroadcast(0);
            return;
        }

        hasUnsentEpochData = TRUE;
        txRetryCount = 0;
        if (curParent >= INVALID_NODE_ID || curDepth >= DEPTH_ROOT_PROBE) {
            if (!(call TreeTopology.isElectionPending()) &&
                (call TreeTopology.getOrphanPhase()) == ORPHAN_NONE) {
                call TreeTopology.enterOrphanState();
            }
            return;
        }
        enqueueDataPacketToParent(totalSum, totalSumSq, totalCount, totalMax);
    }

    /**
     * @brief Data transmission callback with synchronous MAC-layer ACK inspection,
     *        fast retry, intermediate backup parent failover, and Root Election trigger.
     */
    event void MyAMSend.sendDone(message_t* msg, error_t err) {
        bool acked = FALSE;
        uint16_t txDest = call MyAMPacket.destination(msg);
        uint16_t curParent = call TreeTopology.getParent();
        uint8_t curDepth = call TreeTopology.getDepth();
        bool rootFlag = call TreeTopology.isRoot();

        MySendBusy = FALSE;

        if (err == SUCCESS) {
            acked = call MyPacketAcknowledgements.wasAcked(msg);
        } else {
            dbg("SRTreeC", "WARN: Node %u MyAMSend.sendDone err=%u (dest=%u, retry=%u)\n",
                TOS_NODE_ID, (unsigned)err, txDest, txRetryCount);
        }

        if (acked) {
            uint32_t now = call MyTimer.getNow();
            bool inRecoveryWindow = call TreeTopology.isInRecoveryWindow(now);
            uint32_t myJitter = SLOT_JITTER_MILLIS(TOS_NODE_ID);

            // Parent is alive and received the aggregate
            txRetryCount = 0;
            parentFailCount = 0;
            hasUnsentEpochData = FALSE;
            lastAckedDest = txDest;

            // If a failover child's packet arrived while our packet was on the air,
            // forward that pending delta now if still before the Root's epoch deadline
            if (!rootFlag && hasPendingChildAgg &&
                (inRecoveryWindow || (lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED &&
                 (now - lastMyFireTime) < (((uint32_t)lastFireDepth * TAG_SLOT_MILLIS) + myJitter)))) {
                uint16_t fSum = aggSum;
                uint32_t fSumSq = aggSumSq;
                uint8_t fCount = aggCount;
                uint8_t fMax = aggMax;

                epochSum += fSum;
                epochSumSq += fSumSq;
                epochCount += fCount;
                if (fMax > epochMax) {
                    epochMax = fMax;
                }
                savedSum = fSum;
                savedSumSq = fSumSq;
                savedCount = fCount;
                savedMax = fMax;

                txPktToggle ^= 1;
                resetEpochAccumulators();
                hasUnsentEpochData = TRUE;
                enqueueDataPacketToParent(fSum, fSumSq, fCount, fMax);
            }
        } else if (!rootFlag && hasUnsentEpochData) {
            // No MAC ACK received from parentID
            if (curParent >= INVALID_NODE_ID || curDepth >= DEPTH_UNATTACHED) {
                // Already detached while packet was in flight
                txRetryCount = 0;
                clearMySendQueue();
            } else if (txDest != curParent) {
                // Switched parent while old packet was in flight; forward to new parent
                txRetryCount = 0;
                clearMySendQueue();
                enqueueDataPacketToParent(savedSum, savedSumSq, savedCount, savedMax);
            } else if (txRetryCount < MAX_MAC_RETRIES) {
                txRetryCount++;
                dbg("SRTreeC", "Node %u missed ACK from parent %u (retry %u/%u)\n",
                    TOS_NODE_ID, curParent, txRetryCount, MAX_MAC_RETRIES);
                clearMySendQueue();
                enqueueDataPacketToParent(savedSum, savedSumSq, savedCount, savedMax);
            } else {
                // Retries exhausted: parentID is unreachable/dead.
                // Fold any pending post-fire child aggregate only now (before failover/orphan)
                // so active MAC retries to the same parentID always transmit an immutable payload.
                uint32_t now = call MyTimer.getNow();
                bool inRecoveryWindow = call TreeTopology.isInRecoveryWindow(now);
                uint32_t myJitter = SLOT_JITTER_MILLIS(TOS_NODE_ID);

                if (hasPendingChildAgg &&
                    (inRecoveryWindow || (lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED &&
                     (now - lastMyFireTime) < (((uint32_t)lastFireDepth * TAG_SLOT_MILLIS) + myJitter)))) {
                    foldPendingChildAggIntoSnapshots();
                }

                txRetryCount = 0;
                parentFailCount++;
                call BackupTable.remove(curParent);
                clearMySendQueue();
                if (lastAckedDest == curParent) {
                    // Only if parentID previously ACKed our primary aggregate: restore full
                    // cumulative epoch totals (epoch*) into saved* and flip txPktToggle so the
                    // outgoing payload is distinguished from the failed delta.
                    savedSum = epochSum;
                    savedSumSq = epochSumSq;
                    savedCount = epochCount;
                    savedMax = epochMax;
                    txPktToggle ^= 1;
                }

                if (parentFailCount >= MAX_PARENT_FAILS) {
                    parentFailCount = 0;

                    if (curDepth == 1) {
                        // Any node at depth 1 is a direct child of the active Root:
                        // probe Root liveness and start slotted election timer.
                        call TreeTopology.startRootElection();
                    } else if (call TreeTopology.tryFailoverToBackup()) {
                        // Immediately reroute current epoch's saved aggregate to new parent
                        enqueueDataPacketToParent(savedSum, savedSumSq, savedCount, savedMax);
                    } else {
                        // Backup table exhausted: issue fast 1-hop local orphan pull request
                        dbg("SRTreeC", "ORPHAN: Node %u lost parent %u with no backups; sending local pull\n",
                            TOS_NODE_ID, curParent);
                        call TreeTopology.enterOrphanState();
                    }
                }
            }
        } else if (!rootFlag) {
            dbg("SRTreeC", "WARN: Node %u unacked sendDone without hasUnsentEpochData (dest=%u, err=%u)\n",
                TOS_NODE_ID, txDest, (unsigned)err);
        }

        if (!(call MySendQueue.empty())) {
            post sendMyTask();
        }
        call TreeTopology.kickRoutingSend();
    }

    event message_t* MyReceive.receive(message_t* msg, void* payload, uint8_t len) {
        message_t tmp;
        memcpy(&tmp, msg, sizeof(message_t));
        if (call MyReceiveQueue.enqueue(tmp) == SUCCESS) {
            post receiveMyTask();
        }
        return msg;
    }

    task void sendMyTask() {
        uint8_t mlen;
        uint16_t mdest;
        error_t sendDone;

        if (call MySendQueue.empty()) {
            return;
        }
        if (MySendBusy || (call TreeTopology.isRoutingSendBusy())) {
            return;
        }

        // Peek via head() first; only dequeue() after AMSend.send() accepts the packet
        // so a transient radio busy/fail return never drops the packet from the queue.
        radioMySendPkt = call MySendQueue.head();
        mlen = call MyPacket.payloadLength(&radioMySendPkt);
        mdest = call MyAMPacket.destination(&radioMySendPkt);

        // Ensure ACK request flag is set on the outgoing buffer
        call MyPacketAcknowledgements.requestAck(&radioMySendPkt);

        sendDone = call MyAMSend.send(mdest, &radioMySendPkt, mlen);
        if (sendDone == SUCCESS) {
            call MySendQueue.dequeue();
            MySendBusy = TRUE;
        } else {
            post sendMyTask();
        }
    }

    /**
     * @brief Dequeues incoming child data packets and folds their partial state
     *        into the current epoch's running accumulators.
     */
    task void receiveMyTask() {
        message_t recPkt;
        uint8_t len;
        uint16_t childID;
        bool validPacket = FALSE;
        uint32_t now;
        uint32_t sinceLastFire;
        uint32_t halfEpoch;
        uint32_t rootDeadline;
        bool isPostFireWindow;
        bool inRecoveryWindow;
        bool rootFlag = call TreeTopology.isRoot();
        bool rootPromotedFlag = call TreeTopology.isRootJustPromoted();
        bool electPendingFlag = call TreeTopology.isElectionPending();
        uint16_t curParent = call TreeTopology.getParent();

        if (call MyReceiveQueue.empty()) {
            return;
        }

        recPkt = call MyReceiveQueue.dequeue();
        len = call MyPacket.payloadLength(&recPkt);
        childID = call MyAMPacket.source(&recPkt);

        now = call MyTimer.getNow();
        sinceLastFire = now - lastMyFireTime;
        halfEpoch = HALF_SEC_TO_BMILLI(epochDuration);
        inRecoveryWindow = call TreeTopology.isInRecoveryWindow(now);
        // A node at depth d fires (d * TAG_SLOT_MILLIS + myJitter) before the Root;
        // packets arriving before that offset belong to the current epoch's failover window.
        rootDeadline = rootFlag ? 0 : (((uint32_t)lastFireDepth * TAG_SLOT_MILLIS) + SLOT_JITTER_MILLIS(TOS_NODE_ID));
        isPostFireWindow = (!rootPromotedFlag && epochCounter > 0 &&
                            lastMyFireTime > 0 && sinceLastFire < halfEpoch);

        // Drop late straggler packets that arrive after the Root has already fired for the epoch
        // (unless the node just joined a new Root failover wave within ROOT_RECOVERY_DELAY)
        if (isPostFireWindow && !inRecoveryWindow &&
            (rootFlag || lastFireDepth >= DEPTH_UNATTACHED || sinceLastFire >= rootDeadline)) {
            if (!(call MyReceiveQueue.empty())) post receiveMyTask();
            return;
        }

        if (len == sizeof(MaxMsg) && queryType == QUERY_MAX) {
            MaxMsg* mr = (MaxMsg*)(call MyPacket.getPayload(&recPkt, len));
            uint8_t cleanMax = UNPACK_MAX(mr->maxVal);
            if (call DupFilter.isDuplicate(childID, mr->maxVal, now)) {
                if (!(call MyReceiveQueue.empty())) post receiveMyTask();
                return;
            }
            if (cleanMax > aggMax) {
                aggMax = cleanMax;
            }
            hasPendingChildAgg = TRUE;
            validPacket = TRUE;
            dbg("Lab1", "Node %u rx MaxMsg (1B) from child %u: childMax=%u -> runningMax=%u\n",
                TOS_NODE_ID, childID, cleanMax, aggMax);
        } else if (len == sizeof(SumMsg) && queryType == QUERY_SUM) {
            SumMsg* mr = (SumMsg*)(call MyPacket.getPayload(&recPkt, len));
            uint16_t cleanSum = UNPACK_SUM(mr->sum);
            if (call DupFilter.isDuplicate(childID, mr->sum, now)) {
                if (!(call MyReceiveQueue.empty())) post receiveMyTask();
                return;
            }
            aggSum += cleanSum;
            hasPendingChildAgg = TRUE;
            validPacket = TRUE;
            dbg("Lab1", "Node %u rx SumMsg (2B) from child %u: childSum=%u -> runningSum=%u\n",
                TOS_NODE_ID, childID, cleanSum, aggSum);
        } else if (len == sizeof(AvgMsg) && queryType == QUERY_AVG) {
            AvgMsg* mr = (AvgMsg*)(call MyPacket.getPayload(&recPkt, len));
            uint16_t cleanSum = UNPACK_SUM(mr->sum);
            uint32_t sig = ((uint32_t)mr->sum << 8) | mr->count;
            if (call DupFilter.isDuplicate(childID, sig, now)) {
                if (!(call MyReceiveQueue.empty())) post receiveMyTask();
                return;
            }
            aggSum += cleanSum;
            aggCount += mr->count;
            hasPendingChildAgg = TRUE;
            validPacket = TRUE;
            dbg("Lab1", "Node %u rx AvgMsg (3B) from child %u: (sum=%u, cnt=%u) -> running=(%u, %u)\n",
                TOS_NODE_ID, childID, cleanSum, mr->count, aggSum, aggCount);
        } else if (len == sizeof(VarMsg) && queryType == QUERY_VARIANCE) {
            VarMsg* mr = (VarMsg*)(call MyPacket.getPayload(&recPkt, len));
            uint16_t cleanSum = UNPACK_SUM(mr->sum);
            uint32_t sig = mr->sumSq ^ ((uint32_t)mr->sum << 8) ^ mr->count;
            if (call DupFilter.isDuplicate(childID, sig, now)) {
                if (!(call MyReceiveQueue.empty())) post receiveMyTask();
                return;
            }
            aggSumSq += mr->sumSq;
            aggSum += cleanSum;
            aggCount += mr->count;
            hasPendingChildAgg = TRUE;
            validPacket = TRUE;
            dbg("Lab1", "Node %u rx VarMsg (7B) from child %u: (sum=%u, sumSq=%lu, cnt=%u)\n",
                TOS_NODE_ID, childID, cleanSum, (unsigned long)mr->sumSq, mr->count);
        }

        // If a sibling/orphan failover packet arrived after this node fired its MyTimer
        // (or during a new Root recovery window), fold into epoch* and forward upstream
        // only when no un-ACKed primary/retry packet is currently in flight (keeps saved* immutable).
        if (validPacket && (isPostFireWindow || inRecoveryWindow)) {
            if (!hasUnsentEpochData && !MySendBusy && !electPendingFlag) {
                uint16_t fSum = aggSum;
                uint32_t fSumSq = aggSumSq;
                uint8_t fCount = aggCount;
                uint8_t fMax = aggMax;

                epochSum += fSum;
                epochSumSq += fSumSq;
                epochCount += fCount;
                if (fMax > epochMax) {
                    epochMax = fMax;
                }
                if (curParent < INVALID_NODE_ID) {
                    savedSum = fSum;
                    savedSumSq = fSumSq;
                    savedCount = fCount;
                    savedMax = fMax;
                } else {
                    savedSum = epochSum;
                    savedSumSq = epochSumSq;
                    savedCount = epochCount;
                    savedMax = epochMax;
                }
                txPktToggle ^= 1;
                resetEpochAccumulators();
                clearMySendQueue();
                hasUnsentEpochData = TRUE;
                txRetryCount = 0;
                if (curParent < INVALID_NODE_ID) {
                    enqueueDataPacketToParent(fSum, fSumSq, fCount, fMax);
                }
            }
        }

        if (!(call MyReceiveQueue.empty())) {
            post receiveMyTask();
        }
    }

    /* ===================================================================== */
    /* TreeTopology Event Handlers (Control-Plane -> Data-Plane Coordination) */
    /* ===================================================================== */

    event bool TreeTopology.isDataSendBusy() {
        return MySendBusy;
    }

    event uint16_t TreeTopology.getEpochCounter() {
        return epochCounter;
    }

    event void TreeTopology.routingSendCompleted() {
        if (!(call MySendQueue.empty()) && !MySendBusy) {
            post sendMyTask();
        }
    }

    event void TreeTopology.initialRootStart(uint32_t now) {
        if (epochCounter == 0 && lastFireDepth == DEPTH_UNATTACHED) {
            uint32_t epochMillis = SEC_TO_BMILLI(epochDuration);
            uint32_t t0 = now + FIRST_EPOCH_DELAY - epochMillis;

            lastMyFireTime = t0;
            lastFireDepth = 0;
            call MyTimer.startPeriodicAt(t0, epochMillis);
            dbg("SRTreeC", "####################################################\n");
            dbg("SRTreeC", "### TAG DISSEMINATION START: Query=%s, Epoch=%us ###\n",
                call TreeTopology.getQueryName(queryType), epochDuration);
            dbg("SRTreeC", "####################################################\n");
        }
    }

    event void TreeTopology.onEnteredOrphan() {
        clearMySendQueue();
        // Shed stale child aggregates while retaining local sensor reading (myVal)
        // since DEPTH_UNATTACHED instructs direct children to re-route their subtrees.
        if (epochCounter > 0 && myVal > 0) {
            epochSum = myVal;
            epochSumSq = (uint32_t)myVal * (uint32_t)myVal;
            epochCount = 1;
            epochMax = myVal;
            savedSum = myVal;
            savedSumSq = (uint32_t)myVal * (uint32_t)myVal;
            savedCount = 1;
            savedMax = myVal;
            resetEpochAccumulators();
            call DupFilter.reset();
        }
    }

    event void TreeTopology.depthChanged(uint8_t oldDepth, uint8_t newDepth) {
        if (newDepth != oldDepth && lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED) {
            rescheduleMyTimerForDepthChange(newDepth);
        }
    }

    event void TreeTopology.prepareParentInvalidated(uint16_t oldParent, uint32_t now) {
        if (hasPendingChildAgg) {
            foldPendingChildAggIntoSnapshots();
        }
        if (lastAckedDest == oldParent && savedEpochTime > 0 &&
            (now - savedEpochTime) < HALF_SEC_TO_BMILLI(epochDuration)) {
            savedSum = epochSum;
            savedSumSq = epochSumSq;
            savedCount = epochCount;
            savedMax = epochMax;
            txPktToggle ^= 1;
            hasUnsentEpochData = TRUE;
        }
    }

    event void TreeTopology.forwardSavedEpoch(bool isNewTreeSeq, uint32_t now) {
        forwardSavedEpochIfOpen(isNewTreeSeq, now);
    }

    event void TreeTopology.onRootPromoted(uint8_t newTreeSeq, uint32_t now) {
        uint32_t epochMillis = SEC_TO_BMILLI(epochDuration);

        // Merge this node's saved subtree snapshot with any child data received during election delay
        clearMySendQueue();
        aggSum += savedSum;
        aggSumSq += savedSumSq;
        aggCount += savedCount;
        if (savedMax > aggMax) {
            aggMax = savedMax;
        }
        hasUnsentEpochData = FALSE;

        // Schedule periodic epoch timer for subsequent epochs
        lastMyFireTime = now;
        lastFireDepth = 0;
        call MyTimer.startPeriodicAt(now, epochMillis);

        dbg("SRTreeC", "####################################################################\n");
        dbg("SRTreeC", "### ROOT FAILOVER: Node %u ELECTED NEW ROOT (treeSeq=%u, Epoch=%u) ###\n",
            TOS_NODE_ID, newTreeSeq, epochCounter);
        dbg("SRTreeC", "####################################################################\n");
    }

    event void TreeTopology.onRootSettleComplete() {
        uint16_t tSum = aggSum;
        uint32_t tSumSq = aggSumSq;
        uint8_t tCount = aggCount;
        uint8_t tMax = aggMax;

        resetEpochAccumulators();
        call DupFilter.reset();
        logRootEpochResult(tSum, tSumSq, tCount, tMax);
    }

    event void TreeTopology.onAdoptSuperiorTree(bool wasPromotedRoot, uint16_t prevRootID,
                                                uint16_t newRootID, uint32_t now) {
        // If this node had just promoted itself to Root and was waiting in its settle
        // window, move its accumulated epoch state back into saved* so it can forward
        // the subtree aggregate upstream to the winning lower-ID Root.
        if (wasPromotedRoot) {
            savedSum = aggSum;
            savedSumSq = aggSumSq;
            savedCount = aggCount;
            savedMax = aggMax;
            epochSum = aggSum;
            epochSumSq = aggSumSq;
            epochCount = aggCount;
            epochMax = aggMax;
            resetEpochAccumulators();
            hasUnsentEpochData = TRUE;
            txPktToggle ^= 1;
        } else if (savedEpochDepth == 1 && lastAckedDest == prevRootID &&
                   prevRootID != newRootID && savedEpochTime > 0 &&
                   (now - savedEpochTime) < HALF_SEC_TO_BMILLI(epochDuration)) {
            // Only restore saved* = epoch* if this Depth-1 node's primary packet was ACKed
            // by the previous Root that died (prevents double-counting across split-brain waves).
            savedSum = epochSum;
            savedSumSq = epochSumSq;
            savedCount = epochCount;
            savedMax = epochMax;
            hasUnsentEpochData = TRUE;
            txPktToggle ^= 1;
        }
        parentFailCount = 0;
        lastFireDepth = DEPTH_UNATTACHED;
    }

    event void TreeTopology.onParentDepthIncreased(uint8_t newDepth, uint32_t now) {
        if (lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED && newDepth != lastFireDepth) {
            rescheduleMyTimerForDepthChange(newDepth);
        }
    }

    event void TreeTopology.onParentSelected(bool isSuperiorTree, uint8_t newDepth,
                                             uint8_t qType, uint8_t epochDur,
                                             uint32_t radioStartTime, uint32_t now) {
        uint32_t epochMillis;
        uint32_t slotLead;
        uint32_t jitter;
        uint32_t t0;
        uint8_t curTreeSeq = call TreeTopology.getTreeSeq();

        queryType = qType;
        epochDuration = epochDur;
        epochMillis = SEC_TO_BMILLI(epochDuration);

        slotLead = (uint32_t)newDepth * TAG_SLOT_MILLIS;
        // Spread sibling transmissions across 16 buckets (0..120 ms within the 200 ms level slot)
        // to reduce CSMA/ACK collisions in high-fanout star topologies
        jitter = SLOT_JITTER_MILLIS(TOS_NODE_ID);

        if (curTreeSeq == 0 && epochCounter == 0 &&
            (lastFireDepth != DEPTH_UNATTACHED ||
             (int32_t)(now - (radioStartTime + TIMER_FAST_PERIOD + FIRST_EPOCH_DELAY - slotLead - jitter)) < 0)) {
            // Initial dissemination wave (including late boot-watchdog pulls): lock Epoch 1
            // directly onto Root's FIRST_EPOCH_DELAY baseline from radioStartTime.
            uint32_t firstEpochTarget = radioStartTime + TIMER_FAST_PERIOD + FIRST_EPOCH_DELAY - slotLead - jitter;
            if ((int32_t)(now - firstEpochTarget) < 0) {
                t0 = firstEpochTarget - epochMillis;
                lastMyFireTime = t0;
                lastFireDepth = newDepth;
                call MyTimer.startPeriodicAt(t0, epochMillis);
            } else if (newDepth != lastFireDepth) {
                rescheduleMyTimerForDepthChange(newDepth);
            }
        } else if (!isSuperiorTree && lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED) {
            // Mid-epoch orphan recovery or depth change under the existing Root schedule
            if (newDepth != lastFireDepth) {
                rescheduleMyTimerForDepthChange(newDepth);
            }
        } else {
            // New Root failover wave (or late first join): anchor phase to the new Root's broadcast
            uint32_t fastWaveStep = (isSuperiorTree && epochCounter > 0)
                ? (HOP_DELAY_MILLIS + 3)
                : TIMER_FAST_PERIOD;
            uint32_t fastRoutingElapsed = (uint32_t)(newDepth - 1) * fastWaveStep +
                                          (uint32_t)newDepth * HOP_DELAY_MILLIS;
            t0 = now - fastRoutingElapsed - slotLead - jitter;
            while ((int32_t)(now - t0) < 0) {
                t0 -= epochMillis;
            }
            while ((now - t0) >= epochMillis) {
                t0 += epochMillis;
            }
            lastMyFireTime = t0;
            lastFireDepth = newDepth;
            call MyTimer.startPeriodicAt(t0, epochMillis);
        }
    }
}
