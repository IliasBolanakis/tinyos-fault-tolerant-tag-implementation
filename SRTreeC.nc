/**
 * @file SRTreeC.nc
 * @brief Core TAG module with energy-efficient fail-safety:
 *        - Synchronous MAC-layer ACK failure detection (0 extra packet bytes on data)
 *        - Passive backup parent table (strictly shallower upstream candidates)
 *        - Two-phase orphan pull recovery with periodic retry & boot watchdog
 *        - Pre-election Root liveness probe (code 30) to prevent false elections
 *        - Slotted Root Election with rootID split-brain tie-breaking
 *        - Staggered post-epoch top-down routing refresh beacons
 */
#include "SimpleRoutingTree.h"
#ifdef PRINTFDBG_MODE
    #include "printf.h"
#endif

module SRTreeC {
    // Platform bootstrap, LEDs, radio power control, and PRNG services
    uses interface Boot;
    uses interface Leds;
    uses interface SplitControl as RadioControl;
    uses interface Random;
    uses interface ParameterInit<uint16_t> as SeedInit;

    // Routing Active Message layer (AM 22, broadcast tree dissemination & repair)
    uses interface Packet as RoutingPacket;
    uses interface AMSend as RoutingAMSend;
    uses interface AMPacket as RoutingAMPacket;
    uses interface Receive as RoutingReceive;

    // Data aggregation Active Message layer (AM 23, unicast partial aggregates)
    uses interface Packet as MyPacket;
    uses interface AMSend as MyAMSend;
    uses interface AMPacket as MyAMPacket;
    uses interface Receive as MyReceive;
    uses interface PacketAcknowledgements as MyPacketAcknowledgements;

    // Hardware millisecond timers: routing/election delay and TAG epoch slotting
    uses interface Timer<TMilli> as RoutingMsgTimer;
    uses interface Timer<TMilli> as MyTimer;

    // Inbound and outbound FIFO packet queues
    uses interface PacketQueue as RoutingSendQueue;
    uses interface PacketQueue as RoutingReceiveQueue;
    uses interface PacketQueue as MySendQueue;
    uses interface PacketQueue as MyReceiveQueue;
}
implementation {
    // Current epoch sequence index
    uint16_t epochCounter = 0;

    // Static transmission buffers for the CC2420 radio driver
    message_t radioRoutingSendPkt;
    message_t radioMySendPkt;

    // Mutual-exclusion radio lock flags
    bool RoutingSendBusy = FALSE;
    bool MySendBusy = FALSE;

    // Tree topology and dynamic root state
    uint8_t curdepth = DEPTH_UNATTACHED;
    uint16_t parentID = 65535;
    uint16_t currentRootID = 0;
    uint8_t treeSeq = 0;
    bool isRoot = FALSE;

    // Passive Backup Parent Table (populated with 0 extra packets during routing wave)
    uint16_t backupID[MAX_BACKUP_PARENTS];
    uint8_t backupDepth[MAX_BACKUP_PARENTS];

    // Fault-tolerance & failover tracking state
    uint8_t txRetryCount = 0;
    uint8_t parentFailCount = 0;
    bool electionPending = FALSE;
    bool rootJustPromoted = FALSE;
    bool orphanPullPending = FALSE;
    uint8_t orphanPhase = ORPHAN_NONE;
    uint8_t orphanMaxNeighborDepth = DEPTH_UNATTACHED;
    uint32_t lastMyFireTime = 0;             // Anchor timestamp for TAG slot shifting & deadline checks
    uint32_t radioStartTime = 0;             // Boot synchronization baseline from RadioControl.startDone
    uint8_t lastFireDepth = DEPTH_UNATTACHED;

    // Saved epoch snapshot for zero-loss rerouting upon parent/root failure
    uint32_t savedSumSq = 0;
    uint16_t savedSum = 0;
    uint8_t savedCount = 0;
    uint8_t savedMax = 0;
    uint32_t savedEpochTime = 0;             // Timestamp & depth when snapshot was captured
    uint8_t savedEpochDepth = DEPTH_UNATTACHED;
    bool hasUnsentEpochData = FALSE;
    bool hasPendingChildAgg = FALSE;         // TRUE when a post-fire child aggregate awaits forwarding

    // Full cumulative epoch snapshot (local reading + all children received in current epoch)
    uint32_t epochSumSq = 0;
    uint16_t epochSum = 0;
    uint8_t epochCount = 0;
    uint8_t epochMax = 0;
    uint16_t lastAckedDest = 65535;          // Parent ID that last ACKed our primary epoch packet
    uint32_t treeJoinTime = 0;               // Timestamp when node adopted a new treeSeq wave
    uint8_t txPktToggle = 0;                 // 1-bit MSB toggle distinguishing post-fire deltas from MAC retries

    // Active query configuration and local sensor measurement
    uint8_t queryType = 0;
    uint8_t epochDuration = 0;
    uint8_t myVal = 0;

    /* Running subtree accumulators for the current epoch */
    uint32_t aggSumSq = 0;
    uint16_t aggSum = 0;
    uint8_t aggCount = 0;
    uint8_t aggMax = 0;

    /* Per-epoch duplicate MAC retry filter (0 extra packet bytes) */
    #define MAX_TRACKED_CHILDREN 16
    #define DUP_RETRY_WINDOW_MILLIS 500
    uint16_t rxChildID[MAX_TRACKED_CHILDREN];
    uint32_t rxChildSig[MAX_TRACKED_CHILDREN];
    uint32_t rxChildTime[MAX_TRACKED_CHILDREN];
    uint8_t rxChildUsed = 0;

    // Forward task declarations
    task void sendRoutingTask();
    task void sendMyTask();
    task void receiveRoutingTask();
    task void receiveMyTask();

    void setRoutingSendBusy(bool state) {
        atomic { RoutingSendBusy = state; }
    }

    void setMySendBusy(bool state) {
        atomic { MySendBusy = state; }
    }

    void resetEpochAccumulators() {
        atomic {
            aggSumSq = 0;
            aggSum = 0;
            aggCount = 0;
            aggMax = 0;
            hasPendingChildAgg = FALSE;
        }
    }

    /**
     * @brief Clears the per-epoch child duplicate filter. Invoked strictly at epoch
     *        boundaries (never during mid-epoch delta forwarding or failover) so retransmitted
     *        child packets arriving after orphan recovery in the same epoch are still filtered.
     */
    void resetDuplicateFilter() {
        atomic {
            rxChildUsed = 0;
        }
    }

    /**
     * @brief Flushes any stale or superseded outbound data packets from MySendQueue.
     */
    void clearMySendQueue() {
        while (!(call MySendQueue.empty())) {
            call MySendQueue.dequeue();
        }
    }

    /**
     * @brief Flushes any stale outbound routing advertisements from RoutingSendQueue
     *        so a node never transmits an outdated depth after detaching or updating.
     */
    void clearRoutingSendQueue() {
        while (!(call RoutingSendQueue.empty())) {
            call RoutingSendQueue.dequeue();
        }
    }

    /**
     * @brief Initializes or clears the passive backup parent table.
     */
    void clearBackupTable() {
        uint8_t i;
        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            backupID[i] = 65535;
            backupDepth[i] = DEPTH_UNATTACHED;
        }
    }

    /**
     * @brief Records or updates a neighbor's depth in the backup parent table.
     *        Only strictly shallower upstream neighbors (nDepth < curdepth)
     *        are retained to prevent sibling/descendant routing loops and avoid
     *        wasting a backup slot on the active parentID.
     */
    void updateBackupCandidate(uint16_t nodeID, uint8_t nDepth) {
        uint8_t i;
        uint8_t worstIdx = 0;
        uint8_t worstDepth = 0;
        bool invalidCand = (nodeID == parentID) ||
                           (nDepth >= DEPTH_ROOT_PROBE) ||
                           (curdepth < DEPTH_ROOT_PROBE && nDepth >= curdepth);

        if (nodeID == TOS_NODE_ID) return;

        // Update or evict existing entry if neighbor is already in table
        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            if (backupID[i] == nodeID) {
                if (invalidCand) {
                    backupID[i] = 65535;
                    backupDepth[i] = DEPTH_UNATTACHED;
                } else {
                    backupDepth[i] = nDepth;
                }
                return;
            }
        }

        if (invalidCand) return;

        // Insert into first empty slot
        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            if (backupID[i] == 65535) {
                backupID[i] = nodeID;
                backupDepth[i] = nDepth;
                return;
            }
            if (backupDepth[i] > worstDepth) {
                worstDepth = backupDepth[i];
                worstIdx = i;
            }
        }

        // Replace worst entry if new candidate is closer to the root
        if (nDepth < worstDepth) {
            backupID[worstIdx] = nodeID;
            backupDepth[worstIdx] = nDepth;
        }
    }

    /**
     * @brief Evicts any backup candidates at or deeper than the node's current depth.
     */
    void pruneDeeperBackups() {
        uint8_t i;
        if (curdepth >= DEPTH_UNATTACHED) return;
        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            if (backupID[i] != 65535 && backupDepth[i] >= curdepth) {
                backupID[i] = 65535;
                backupDepth[i] = DEPTH_UNATTACHED;
            }
        }
    }

    /**
     * @brief Removes a dead neighbor from the backup parent table.
     */
    void removeBackupCandidate(uint16_t deadNodeID) {
        uint8_t i;
        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            if (backupID[i] == deadNodeID) {
                backupID[i] = 65535;
                backupDepth[i] = DEPTH_UNATTACHED;
            }
        }
    }

    /**
     * @brief Selects the best available backup parent (minimum depth < curdepth, lowest ID tie-break).
     */
    uint16_t selectBestBackupParent(uint8_t* outNeighborDepth) {
        uint8_t i;
        uint16_t bestID = 65535;
        uint8_t bestD = DEPTH_UNATTACHED;

        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            if (backupID[i] != 65535 &&
                backupDepth[i] < DEPTH_ROOT_PROBE &&
                (curdepth >= DEPTH_ROOT_PROBE || backupDepth[i] < curdepth)) {
                if (backupDepth[i] < bestD || (backupDepth[i] == bestD && backupID[i] < bestID)) {
                    bestID = backupID[i];
                    bestD = backupDepth[i];
                }
            }
        }
        *outNeighborDepth = bestD;
        return bestID;
    }

    /**
     * @brief Shifts MyTimer's periodic phase relative to the node's last firing slot
     *        while keeping t0 within [now - epochMillis + 1, now] to prevent unsigned
     *        underflow in TinyOS Timer.startPeriodicAt(t0, dt).
     */
    void rescheduleMyTimerForDepthChange() {
        uint32_t now = call MyTimer.getNow();
        uint32_t epochMillis = (uint32_t)epochDuration * 1024;
        int32_t shift = ((int32_t)lastFireDepth - (int32_t)curdepth) * (int32_t)TAG_SLOT_MILLIS;
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
            lastFireDepth = curdepth;
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

    const char* getQueryName(uint8_t qType) {
        switch (qType) {
            case QUERY_SUM:      return "SUM";
            case QUERY_AVG:      return "AVG";
            case QUERY_VARIANCE: return "VARIANCE";
            case QUERY_MAX:      return "MAX";
            default:             return "UNKNOWN";
        }
    }

    /**
     * @brief Constructs and enqueues a query-specific minimal data packet to parentID.
     *        Packs the 1-bit txPktToggle into the unused MSB of sum (bit 15) or maxVal (bit 7)
     *        so the parent's duplicate filter distinguishes legitimate post-fire deltas from
     *        MAC retries with 0 extra packet bytes.
     */
    void enqueueDataPacketToParent(uint16_t tSum, uint32_t tSumSq, uint8_t tCount, uint8_t tMax) {
        message_t tmp;

        if (isRoot || parentID >= 65535 || call MySendQueue.full()) return;
        memset(&tmp, 0, sizeof(message_t));

        if (queryType == QUERY_SUM) {
            SumMsg* pkt = (SumMsg*)(call MyPacket.getPayload(&tmp, sizeof(SumMsg)));
            if (pkt == NULL) return;
            atomic { pkt->sum = (tSum & 0x7FFF) | ((uint16_t)(txPktToggle & 0x01) << 15); }
            call MyAMPacket.setDestination(&tmp, parentID);
            call MyPacket.setPayloadLength(&tmp, sizeof(SumMsg));
        } else if (queryType == QUERY_AVG) {
            AvgMsg* pkt = (AvgMsg*)(call MyPacket.getPayload(&tmp, sizeof(AvgMsg)));
            if (pkt == NULL) return;
            atomic {
                pkt->sum = (tSum & 0x7FFF) | ((uint16_t)(txPktToggle & 0x01) << 15);
                pkt->count = tCount;
            }
            call MyAMPacket.setDestination(&tmp, parentID);
            call MyPacket.setPayloadLength(&tmp, sizeof(AvgMsg));
        } else if (queryType == QUERY_VARIANCE) {
            VarMsg* pkt = (VarMsg*)(call MyPacket.getPayload(&tmp, sizeof(VarMsg)));
            if (pkt == NULL) return;
            atomic {
                pkt->sumSq = tSumSq;
                pkt->sum = (tSum & 0x7FFF) | ((uint16_t)(txPktToggle & 0x01) << 15);
                pkt->count = tCount;
            }
            call MyAMPacket.setDestination(&tmp, parentID);
            call MyPacket.setPayloadLength(&tmp, sizeof(VarMsg));
        } else if (queryType == QUERY_MAX) {
            MaxMsg* pkt = (MaxMsg*)(call MyPacket.getPayload(&tmp, sizeof(MaxMsg)));
            if (pkt == NULL) return;
            atomic { pkt->maxVal = (tMax & 0x7F) | ((txPktToggle & 0x01) << 7); }
            call MyAMPacket.setDestination(&tmp, parentID);
            call MyPacket.setPayloadLength(&tmp, sizeof(MaxMsg));
        } else {
            return;
        }

        // Request synchronous hardware/MAC-layer ACK from parentID (0 extra bytes)
        call MyPacketAcknowledgements.requestAck(&tmp);

        if (call MySendQueue.enqueue(tmp) == SUCCESS) {
            if (!MySendBusy && !RoutingSendBusy) {
                post sendMyTask();
            }
        }
    }

    /**
     * @brief Constructs and enqueues a 3-byte broadcast RoutingMsg carrying the
     *        active treeSeq, advertisedDepth (or control code 30/31), query/epoch
     *        parameters, and currentRootID.
     */
    void enqueueRoutingBroadcast(uint8_t advertisedDepth) {
        message_t tmp;
        RoutingMsg* mrpkt;

        clearRoutingSendQueue();
        if (call RoutingSendQueue.full()) return;
        memset(&tmp, 0, sizeof(message_t));

        mrpkt = (RoutingMsg*)(call RoutingPacket.getPayload(&tmp, sizeof(RoutingMsg)));
        if (mrpkt == NULL) return;

        atomic {
            mrpkt->depth = PACK_SEQ_DEPTH(treeSeq, advertisedDepth);
            mrpkt->queryAndEpoch = (uint8_t)(((epochDuration / 10) << 4) | (queryType & 0x0F));
            mrpkt->rootID = (uint8_t)currentRootID;
        }

        call RoutingAMPacket.setDestination(&tmp, AM_BROADCAST_ADDR);
        call RoutingPacket.setPayloadLength(&tmp, sizeof(RoutingMsg));

        if (call RoutingSendQueue.enqueue(tmp) == SUCCESS) {
            if (!RoutingSendBusy && !MySendBusy) {
                post sendRoutingTask();
            }
        }
    }

    /**
     * @brief Detaches the node into Phase 1 orphan recovery while preserving its
     *        last attached depth bound so descendants cannot be selected as parents.
     *        Because broadcasting DEPTH_UNATTACHED instructs all direct children to
     *        detach and re-route their own subtrees (via handleParentInvalidated),
     *        this node sheds any previously accumulated child aggregates and retains
     *        only its own local sensor sample (myVal) to prevent double-counting.
     */
    void enterOrphanState() {
        if (curdepth < DEPTH_ROOT_PROBE) {
            orphanMaxNeighborDepth = curdepth;
        }
        orphanPhase = ORPHAN_PULL_P1;
        parentID = 65535;
        curdepth = DEPTH_UNATTACHED;
        clearBackupTable();
        clearMySendQueue();
        clearRoutingSendQueue();
        if (epochCounter > 0 && myVal > 0) {
            atomic {
                epochSum = myVal;
                epochSumSq = (uint32_t)myVal * (uint32_t)myVal;
                epochCount = 1;
                epochMax = myVal;
                savedSum = myVal;
                savedSumSq = (uint32_t)myVal * (uint32_t)myVal;
                savedCount = 1;
                savedMax = myVal;
            }
            resetEpochAccumulators();
            resetDuplicateFilter();
        }
        orphanPullPending = TRUE;
        if (!electionPending && !rootJustPromoted) {
            // Issue a single Phase-1 pull immediately and enter ORPHAN_WAIT_P1;
            // avoids a duplicate P1 broadcast that would reset neighbors' reply timers.
            orphanPhase = ORPHAN_WAIT_P1;
            enqueueRoutingBroadcast(DEPTH_UNATTACHED);
            call RoutingMsgTimer.startOneShot(ORPHAN_DEEPER_DELAY + (uint32_t)(TOS_NODE_ID % 8) * 3);
        }
    }

    /**
     * @brief Switches to the best strictly shallower backup parent if available.
     *        Always schedules a RoutingMsg broadcast (even when depth is unchanged)
     *        so neighbors refresh their backup tables and any stale-backup cycle is
     *        exposed on the next routing exchange.
     * @return TRUE if failover succeeded, FALSE if backup table is empty.
     */
    bool tryFailoverToBackup() {
        uint8_t candDepth = DEPTH_UNATTACHED;
        uint16_t newParent = selectBestBackupParent(&candDepth);

        if (newParent != 65535 && candDepth < DEPTH_ROOT_PROBE) {
            uint8_t oldDepth = curdepth;
            uint16_t deadParent = parentID;

            parentID = newParent;
            curdepth = candDepth + 1;
            if (candDepth == 0) {
                currentRootID = newParent;
            }
            orphanPullPending = FALSE;
            orphanPhase = ORPHAN_NONE;
            orphanMaxNeighborDepth = DEPTH_UNATTACHED;
            removeBackupCandidate(parentID);
            pruneDeeperBackups();

            dbg("SRTreeC", "FAILOVER: Node %u switched from parent %u to backup parent %u (depth %u -> %u)\n",
                TOS_NODE_ID, deadParent, parentID, oldDepth, curdepth);

            if (curdepth != oldDepth && lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED) {
                rescheduleMyTimerForDepthChange();
            }
            // Broadcast updated depth immediately so downstream children refresh quickly without colliding
            if (!electionPending && !rootJustPromoted) {
                call RoutingMsgTimer.startOneShot(HOP_DELAY_MILLIS + (uint32_t)(TOS_NODE_ID % 8) * 3);
            }
            return TRUE;
        }
        return FALSE;
    }

    /**
     * @brief Forwards saved epoch snapshot to the active parent if the epoch window
     *        at the Root is still open (prevents ghost children).
     */
    void forwardSavedEpochIfOpen(bool isNewTreeSeq, uint32_t now) {
        bool epochStillOpen;
        uint32_t myJitter = SLOT_JITTER_MILLIS(TOS_NODE_ID);
        if (!hasUnsentEpochData) return;

        epochStillOpen = isNewTreeSeq
            ? ((now - savedEpochTime) < ((uint32_t)epochDuration * 512))
            : (savedEpochDepth < DEPTH_UNATTACHED &&
               ((now - savedEpochTime) + HOP_DELAY_MILLIS) < (((uint32_t)savedEpochDepth * TAG_SLOT_MILLIS) + myJitter));

        clearMySendQueue();
        if (epochStillOpen) {
            if (hasPendingChildAgg) {
                atomic {
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
                }
                resetEpochAccumulators();
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
     * @brief Handles invalidation or depth worsening of the active parent by
     *        failing over to a strictly shallower backup or detaching into orphan recovery.
     *        If the invalidated parent had already ACKed our primary epoch packet, restores
     *        the full cumulative epoch snapshot so data trapped in the detached parent is re-sent.
     */
    void handleParentInvalidated(uint32_t now) {
        // If no ACKed parent data can be invalidated further, still fold any pending child delta
        if (hasPendingChildAgg) {
            atomic {
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
            }
            resetEpochAccumulators();
        }
        if (lastAckedDest == parentID && savedEpochTime > 0 &&
            (now - savedEpochTime) < ((uint32_t)epochDuration * 512)) {
            atomic {
                savedSum = epochSum;
                savedSumSq = epochSumSq;
                savedCount = epochCount;
                savedMax = epochMax;
            }
            txPktToggle ^= 1;
            hasUnsentEpochData = TRUE;
        }
        if (tryFailoverToBackup()) {
            forwardSavedEpochIfOpen(FALSE, now);
        } else {
            enterOrphanState();
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
        setRoutingSendBusy(FALSE);
        setMySendBusy(FALSE);
        resetEpochAccumulators();
        resetDuplicateFilter();
        clearBackupTable();
        epochCounter = 0;
        treeSeq = 0;
        lastMyFireTime = 0;
        radioStartTime = 0;
        lastFireDepth = DEPTH_UNATTACHED;
        savedEpochTime = 0;
        savedEpochDepth = DEPTH_UNATTACHED;
        hasUnsentEpochData = FALSE;
        lastAckedDest = 65535;
        treeJoinTime = 0;
        txPktToggle = 0;
        orphanPullPending = FALSE;
        orphanPhase = ORPHAN_NONE;
        orphanMaxNeighborDepth = DEPTH_UNATTACHED;

        call SeedInit.init(seed);
        call Random.rand16();
        call Random.rand16();

        if (TOS_NODE_ID == 0) {
            isRoot = TRUE;
            currentRootID = 0;
            curdepth = 0;
            parentID = 0;
            queryType = (uint8_t)((call Random.rand16() % 4) + 1);
            epochDuration = (uint8_t)(((call Random.rand16() % 8) + 3) * 10);

            dbg("Boot", "ROOT (Node 0) booted: selected Query=%u (%s), EpochDuration=%u sec\n",
                queryType, getQueryName(queryType), epochDuration);
        } else {
            isRoot = FALSE;
            currentRootID = 0;
            curdepth = DEPTH_UNATTACHED;
            parentID = 65535;
            dbg("Boot", "Sensor Node %u booted: curdepth=%u, parentID=%u\n",
                TOS_NODE_ID, curdepth, parentID);
        }
    }

    event void RadioControl.startDone(error_t err) {
        if (err == SUCCESS) {
            dbg("Radio", "Radio initialized successfully!\n");
            radioStartTime = call MyTimer.getNow();
            if (isRoot) {
                call RoutingMsgTimer.startOneShot(TIMER_FAST_PERIOD);
            } else {
                // Boot watchdog: if a sensor node misses every broadcast in the initial
                // dissemination wave, this fires well before Epoch 1 to pull routing state.
                call RoutingMsgTimer.startOneShot(2000 + (uint32_t)(TOS_NODE_ID % 16) * 25);
            }
        } else {
            call RadioControl.start();
        }
    }

    event void RadioControl.stopDone(error_t err) {
        dbg("Radio", "Radio stopped!\n");
    }

    /**
     * @brief Handles initial tree dissemination, local repair broadcasts,
     *        slotted Root Election promotion, and post-election epoch readout.
     */
    event void RoutingMsgTimer.fired() {
        uint8_t advertisedDepth;

        // Case 1: Newly promoted Root finishes its settle window and outputs recovered epoch
        if (isRoot && rootJustPromoted) {
            uint16_t tSum;
            uint32_t tSumSq;
            uint8_t tCount;
            uint8_t tMax;

            rootJustPromoted = FALSE;
            atomic {
                tSum = aggSum;
                tSumSq = aggSumSq;
                tCount = aggCount;
                tMax = aggMax;
            }
            resetEpochAccumulators();
            resetDuplicateFilter();
            logRootEpochResult(tSum, tSumSq, tCount, tMax);
            return;
        }

        // Case 2: Depth-1 node's election timer expires without hearing a lower-ID new Root
        if (electionPending) {
            uint32_t now = call MyTimer.getNow();
            uint32_t epochMillis = (uint32_t)epochDuration * 1024;

            electionPending = FALSE;
            isRoot = TRUE;
            currentRootID = TOS_NODE_ID;
            parentID = TOS_NODE_ID;
            curdepth = 0;
            orphanPullPending = FALSE;
            orphanPhase = ORPHAN_NONE;
            orphanMaxNeighborDepth = DEPTH_UNATTACHED;
            treeSeq = (treeSeq + 1) & 0x07;

            // Merge this node's saved subtree snapshot with any child data received during election delay
            clearMySendQueue();
            atomic {
                aggSum += savedSum;
                aggSumSq += savedSumSq;
                aggCount += savedCount;
                if (savedMax > aggMax) {
                    aggMax = savedMax;
                }
            }
            hasUnsentEpochData = FALSE;
            rootJustPromoted = TRUE;

            // Schedule periodic epoch timer for subsequent epochs
            lastMyFireTime = now;
            lastFireDepth = 0;
            call MyTimer.startPeriodicAt(now, epochMillis);

            dbg("SRTreeC", "####################################################################\n");
            dbg("SRTreeC", "### ROOT FAILOVER: Node %u ELECTED NEW ROOT (treeSeq=%u, Epoch=%u) ###\n",
                TOS_NODE_ID, treeSeq, epochCounter);
            dbg("SRTreeC", "####################################################################\n");

            // Schedule settle window so sibling Depth-1 nodes can forward their saved aggregates
            call RoutingMsgTimer.startOneShot(ROOT_RECOVERY_DELAY);
        }

        // Case 3: Initial Root startup synchronization (guarded so it only runs once)
        if (isRoot && treeSeq == 0 && epochCounter == 0 && lastFireDepth == DEPTH_UNATTACHED) {
            uint32_t now = call MyTimer.getNow();
            uint32_t epochMillis = (uint32_t)epochDuration * 1024;
            uint32_t t0 = now + FIRST_EPOCH_DELAY - epochMillis;

            lastMyFireTime = t0;
            lastFireDepth = 0;
            call MyTimer.startPeriodicAt(t0, epochMillis);
            dbg("SRTreeC", "####################################################\n");
            dbg("SRTreeC", "### TAG DISSEMINATION START: Query=%s, Epoch=%us ###\n",
                getQueryName(queryType), epochDuration);
            dbg("SRTreeC", "####################################################\n");
        }

        // If an unattached node's boot watchdog fired, transition into active orphan pull
        if (!isRoot && curdepth == DEPTH_UNATTACHED && orphanPhase == ORPHAN_NONE) {
            orphanPhase = ORPHAN_PULL_P2;
            orphanMaxNeighborDepth = DEPTH_UNATTACHED;
            orphanPullPending = TRUE;
        }

        advertisedDepth = orphanPullPending ? DEPTH_UNATTACHED : curdepth;
        orphanPullPending = FALSE;
        enqueueRoutingBroadcast(advertisedDepth);

        // Two-phase orphan recovery + fast Phase-2 retry + periodic fallback:
        // - Phase 1 (ORPHAN_PULL_P1 -> ORPHAN_WAIT_P1): settle window accepting <= former depth.
        // - Phase 2 entry (ORPHAN_WAIT_P1 -> ORPHAN_PULL_P2): relax depth constraint and schedule
        //   a fast retry (ORPHAN_P2_RETRY_MILLIS + jitter) before backing off to ORPHAN_RETRY_PERIOD.
        // - Phase 2 steady (ORPHAN_WAIT_P2): retry every ORPHAN_RETRY_PERIOD so partitioned nodes
        //   do not storm the channel while still recovering seamlessly if reconnected.
        if (!isRoot && advertisedDepth == DEPTH_UNATTACHED &&
            curdepth == DEPTH_UNATTACHED && !electionPending) {
            if (orphanPhase == ORPHAN_PULL_P1) {
                orphanPhase = ORPHAN_WAIT_P1;
                orphanPullPending = TRUE;
                call RoutingMsgTimer.startOneShot(ORPHAN_DEEPER_DELAY + (uint32_t)(TOS_NODE_ID % 8) * 3);
            } else if (orphanPhase == ORPHAN_WAIT_P1) {
                orphanPhase = ORPHAN_PULL_P2;
                orphanMaxNeighborDepth = DEPTH_UNATTACHED;
                orphanPullPending = TRUE;
                call RoutingMsgTimer.startOneShot(ORPHAN_P2_RETRY_MILLIS + (uint32_t)(TOS_NODE_ID % 8) * 5);
            } else {
                orphanPhase = ORPHAN_WAIT_P2;
                orphanMaxNeighborDepth = DEPTH_UNATTACHED;
                orphanPullPending = TRUE;
                call RoutingMsgTimer.startOneShot(ORPHAN_RETRY_PERIOD + (uint32_t)(TOS_NODE_ID % 8) * 25);
            }
        }
    }

    /**
     * @brief Epoch timer handler: fires in bottom-up level order.
     */
    event void MyTimer.fired() {
        uint8_t sample;
        uint32_t totalSumSq;
        uint16_t totalSum;
        uint8_t totalCount;
        uint8_t totalMax;
        uint32_t now = call MyTimer.getNow();

        // If MyTimer was phase-shifted to a shallower slot within the same epoch,
        // absorb the alignment tick without double-sampling or advancing epochCounter.
        if (epochCounter > 0 && lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED &&
            (now - lastMyFireTime) < ((uint32_t)epochDuration * 512)) {
            lastMyFireTime = now;
            if (curdepth < DEPTH_UNATTACHED) {
                lastFireDepth = curdepth;
            }
            return;
        }

        lastMyFireTime = now;
        if (curdepth < DEPTH_UNATTACHED) {
            lastFireDepth = curdepth;
        }
        epochCounter++;
        sample = sampleSensorReading();

        clearMySendQueue();

        atomic {
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
            lastAckedDest = 65535;
            txPktToggle = 0;
        }

        resetEpochAccumulators();
        resetDuplicateFilter();

        dbg("Lab1", "Epoch %u | Node %u (depth %u, parent %u) sampled local reading = %u\n",
            epochCounter, TOS_NODE_ID, curdepth, parentID, sample);

        if (isRoot) {
            hasUnsentEpochData = FALSE;
            logRootEpochResult(totalSum, totalSumSq, totalCount, totalMax);
            // Broadcast an authoritative depth-0 beacon at epoch end so any Depth-1 child
            // with a pending false election timer immediately aborts before promoting.
            enqueueRoutingBroadcast(0);
            return;
        }

        hasUnsentEpochData = TRUE;
        txRetryCount = 0;
        if (parentID >= 65535 || curdepth >= DEPTH_ROOT_PROBE) {
            if (!electionPending && orphanPhase == ORPHAN_NONE) {
                enterOrphanState();
            }
            return;
        }
        enqueueDataPacketToParent(totalSum, totalSumSq, totalCount, totalMax);
    }

    event void RoutingAMSend.sendDone(message_t* msg, error_t err) {
        setRoutingSendBusy(FALSE);
        if (!(call RoutingSendQueue.empty())) {
            post sendRoutingTask();
        }
        if (!(call MySendQueue.empty()) && !MySendBusy) {
            post sendMyTask();
        }
    }

    /**
     * @brief Data transmission callback with synchronous MAC-layer ACK inspection,
     *        fast retry, intermediate backup parent failover, and Root Election trigger.
     */
    event void MyAMSend.sendDone(message_t* msg, error_t err) {
        bool acked = FALSE;
        uint16_t txDest = call MyAMPacket.destination(msg);
        setMySendBusy(FALSE);

        if (err == SUCCESS) {
            acked = call MyPacketAcknowledgements.wasAcked(msg);
        } else {
            dbg("SRTreeC", "WARN: Node %u MyAMSend.sendDone err=%u (dest=%u, retry=%u)\n",
                TOS_NODE_ID, (unsigned)err, txDest, txRetryCount);
        }

        if (acked) {
            uint32_t now = call MyTimer.getNow();
            bool inRecoveryWindow = (treeJoinTime > 0 && (now - treeJoinTime) < ROOT_RECOVERY_DELAY);
            uint32_t myJitter = SLOT_JITTER_MILLIS(TOS_NODE_ID);
            // Parent is alive and received the aggregate
            txRetryCount = 0;
            parentFailCount = 0;
            hasUnsentEpochData = FALSE;
            lastAckedDest = txDest;

            // If a failover child's packet arrived while our packet was on the air,
            // forward that pending delta now if still before the Root's epoch deadline
            if (!isRoot && hasPendingChildAgg &&
                (inRecoveryWindow || (lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED &&
                 (now - lastMyFireTime) < (((uint32_t)lastFireDepth * TAG_SLOT_MILLIS) + myJitter)))) {
                uint16_t fSum;
                uint32_t fSumSq;
                uint8_t fCount;
                uint8_t fMax;
                atomic {
                    fSum = aggSum;
                    fSumSq = aggSumSq;
                    fCount = aggCount;
                    fMax = aggMax;
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
                }
                txPktToggle ^= 1;
                resetEpochAccumulators();
                hasUnsentEpochData = TRUE;
                enqueueDataPacketToParent(fSum, fSumSq, fCount, fMax);
            }
        } else if (!isRoot && hasUnsentEpochData) {
            // No MAC ACK received from parentID
            if (parentID >= 65535 || curdepth >= DEPTH_UNATTACHED) {
                // Already detached while packet was in flight
                txRetryCount = 0;
                clearMySendQueue();
            } else if (txDest != parentID) {
                // Switched parent while old packet was in flight; forward to new parent
                txRetryCount = 0;
                clearMySendQueue();
                enqueueDataPacketToParent(savedSum, savedSumSq, savedCount, savedMax);
            } else if (txRetryCount < MAX_MAC_RETRIES) {
                txRetryCount++;
                dbg("SRTreeC", "Node %u missed ACK from parent %u (retry %u/%u)\n",
                    TOS_NODE_ID, parentID, txRetryCount, MAX_MAC_RETRIES);
                clearMySendQueue();
                enqueueDataPacketToParent(savedSum, savedSumSq, savedCount, savedMax);
            } else {
                // Retries exhausted: parentID is unreachable/dead.
                // Fold any pending post-fire child aggregate only now (before failover/orphan)
                // so active MAC retries to the same parentID always transmit an immutable payload.
                uint32_t now = call MyTimer.getNow();
                bool inRecoveryWindow = (treeJoinTime > 0 && (now - treeJoinTime) < ROOT_RECOVERY_DELAY);
                uint32_t myJitter = SLOT_JITTER_MILLIS(TOS_NODE_ID);
                if (hasPendingChildAgg &&
                    (inRecoveryWindow || (lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED &&
                     (now - lastMyFireTime) < (((uint32_t)lastFireDepth * TAG_SLOT_MILLIS) + myJitter)))) {
                    atomic {
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
                    }
                    resetEpochAccumulators();
                }

                txRetryCount = 0;
                parentFailCount++;
                removeBackupCandidate(parentID);
                clearMySendQueue();
                if (lastAckedDest == parentID) {
                    // Only if parentID previously ACKed our primary aggregate: restore full
                    // cumulative epoch totals (epoch*) into saved* and flip txPktToggle so the
                    // outgoing payload is distinguished from the failed delta. If lastAckedDest != parentID,
                    // saved* already equals epoch*; keeping txPktToggle unchanged lets the same
                    // parent's duplicate filter drop the packet if orphan recovery reattaches to it.
                    atomic {
                        savedSum = epochSum;
                        savedSumSq = epochSumSq;
                        savedCount = epochCount;
                        savedMax = epochMax;
                    }
                    txPktToggle ^= 1;
                }

                if (parentFailCount >= MAX_PARENT_FAILS) {
                    parentFailCount = 0;

                    if (curdepth == 1) {
                        // Any node at depth 1 is a direct child of the active Root (even if it
                        // reached depth 1 via backup failover without hearing msgDepth == 0 directly).
                        // Broadcast a DEPTH_ROOT_PROBE (code 30) to verify whether the Root is truly
                        // dead before the slotted election timer expires (prevents false elections).
                        uint32_t electDelay = ELECTION_BASE_MILLIS + (uint32_t)TOS_NODE_ID * ELECTION_STEP_MILLIS;
                        electionPending = TRUE;
                        currentRootID = parentID;
                        enqueueRoutingBroadcast(DEPTH_ROOT_PROBE);
                        dbg("SRTreeC", "Node %u (depth 1) detected ROOT %u FAILURE! Probing & starting election timer (%lu ms)\n",
                            TOS_NODE_ID, currentRootID, (unsigned long)electDelay);
                        call RoutingMsgTimer.startOneShot(electDelay);
                    } else if (tryFailoverToBackup()) {
                        // Immediately reroute current epoch's saved aggregate to new parent
                        enqueueDataPacketToParent(savedSum, savedSumSq, savedCount, savedMax);
                    } else {
                        // Backup table exhausted: issue fast 1-hop local orphan pull request
                        dbg("SRTreeC", "ORPHAN: Node %u lost parent %u with no backups; sending local pull\n",
                            TOS_NODE_ID, parentID);
                        enterOrphanState();
                    }
                }
            }
        } else if (!isRoot) {
            dbg("SRTreeC", "WARN: Node %u unacked sendDone without hasUnsentEpochData (dest=%u, err=%u)\n",
                TOS_NODE_ID, txDest, (unsigned)err);
        }

        if (!(call MySendQueue.empty())) {
            post sendMyTask();
        }
        if (!(call RoutingSendQueue.empty()) && !RoutingSendBusy) {
            post sendRoutingTask();
        }
    }

    event message_t* RoutingReceive.receive(message_t* msg, void* payload, uint8_t len) {
        message_t tmp;
        if (len != sizeof(RoutingMsg)) {
            return msg;
        }
        atomic {
            memcpy(&tmp, msg, sizeof(message_t));
        }
        if (call RoutingReceiveQueue.enqueue(tmp) == SUCCESS) {
            post receiveRoutingTask();
        }
        return msg;
    }

    event message_t* MyReceive.receive(message_t* msg, void* payload, uint8_t len) {
        message_t tmp;
        atomic {
            memcpy(&tmp, msg, sizeof(message_t));
        }
        if (call MyReceiveQueue.enqueue(tmp) == SUCCESS) {
            post receiveMyTask();
        }
        return msg;
    }

    task void sendRoutingTask() {
        uint8_t mlen;
        uint16_t mdest;
        error_t sendDone;

        if (call RoutingSendQueue.empty()) return;
        if (RoutingSendBusy || MySendBusy) return;

        // Peek via head() first; only dequeue() after AMSend.send() accepts the packet
        // so a transient radio busy/fail return never drops the packet from the queue.
        radioRoutingSendPkt = call RoutingSendQueue.head();
        mlen = call RoutingPacket.payloadLength(&radioRoutingSendPkt);
        mdest = call RoutingAMPacket.destination(&radioRoutingSendPkt);
        if (mlen != sizeof(RoutingMsg)) {
            call RoutingSendQueue.dequeue();
            return;
        }

        sendDone = call RoutingAMSend.send(mdest, &radioRoutingSendPkt, mlen);
        if (sendDone == SUCCESS) {
            call RoutingSendQueue.dequeue();
            setRoutingSendBusy(TRUE);
        } else {
            post sendRoutingTask();
        }
    }

    task void sendMyTask() {
        uint8_t mlen;
        uint16_t mdest;
        error_t sendDone;

        if (call MySendQueue.empty()) return;
        if (MySendBusy || RoutingSendBusy) return;

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
            setMySendBusy(TRUE);
        } else {
            post sendMyTask();
        }
    }

    /**
     * @brief Processes RoutingMsg packets:
     *        - Passively populates the backup parent table
     *        - Responds to local orphan pull requests (msgDepth == DEPTH_UNATTACHED)
     *        - Adopts new treeSeq waves when a new Root is elected
     *        - Updates shortest-path parent and level-staggered timer slots
     */
    task void receiveRoutingTask() {
        message_t radioRoutingRecPkt;
        uint8_t len;

        if (call RoutingReceiveQueue.empty()) return;

        radioRoutingRecPkt = call RoutingReceiveQueue.dequeue();
        len = call RoutingPacket.payloadLength(&radioRoutingRecPkt);

        if (len == sizeof(RoutingMsg)) {
            RoutingMsg* mpkt = (RoutingMsg*)(call RoutingPacket.getPayload(&radioRoutingRecPkt, len));
            uint16_t senderID = call RoutingAMPacket.source(&radioRoutingRecPkt);
            uint8_t msgSeq = UNPACK_SEQ(mpkt->depth);
            uint8_t msgDepth = UNPACK_DEPTH(mpkt->depth);
            uint16_t msgRootID = mpkt->rootID;
            uint8_t offeredDepth = msgDepth + 1;
            // 3-bit modular wraparound comparison (half-space < 4 treats ahead-by-1..3 as newer)
            bool isNewTreeSeq = ((uint8_t)((msgSeq - treeSeq) & 0x07) > 0) &&
                                ((uint8_t)((msgSeq - treeSeq) & 0x07) < 4);
            bool isOlderTreeSeq = ((uint8_t)((treeSeq - msgSeq) & 0x07) > 0) &&
                                  ((uint8_t)((treeSeq - msgSeq) & 0x07) < 4);
            // Deterministic split-brain tie-breaker: if two non-adjacent depth-1 nodes both
            // promote to Root on the same treeSeq, the lower rootID wins across the network.
            bool isSameSeqBetterRoot = (msgSeq == treeSeq) &&
                                       (msgDepth < DEPTH_ROOT_PROBE) &&
                                       (msgRootID < currentRootID);
            bool isSameSeqWorseRoot = (msgSeq == treeSeq) &&
                                      (msgDepth < DEPTH_ROOT_PROBE) &&
                                      (msgRootID > currentRootID);
            bool isSuperiorTree = isNewTreeSeq || isSameSeqBetterRoot;

            // 0. Root liveness probe (code 30): only the active Root replies immediately
            //    (plus a 25 ms follow-up beacon in case of noise); all non-root nodes ignore it.
            if (msgDepth == DEPTH_ROOT_PROBE) {
                if (isRoot && !rootJustPromoted && msgSeq == treeSeq) {
                    enqueueRoutingBroadcast(0);
                    call RoutingMsgTimer.startOneShot(25);
                }
                if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
                return;
            }

            // 1. Handle local orphan pull / parent-detachment notifications
            if (msgDepth == DEPTH_UNATTACHED) {
                if (msgSeq == treeSeq && !isRoot) {
                    if (senderID == parentID) {
                        // Active parent detached: fail over to a shallower backup or detach too
                        dbg("SRTreeC", "Node %u parent %u became unattached; recovering\n",
                            TOS_NODE_ID, senderID);
                        handleParentInvalidated(call MyTimer.getNow());
                        if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
                        return;
                    }
                    updateBackupCandidate(senderID, DEPTH_UNATTACHED);
                }

                // Reply after 2*HOP_DELAY_MILLIS so direct children's poison reaches grandchildren first
                if (!electionPending && !rootJustPromoted &&
                    (isRoot || (parentID < 65535 && curdepth < DEPTH_ROOT_PROBE)) &&
                    (msgSeq == treeSeq || isOlderTreeSeq)) {
                    call RoutingMsgTimer.startOneShot((2 * HOP_DELAY_MILLIS) + (TOS_NODE_ID % 8) * 5);
                }
                if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
                return;
            }

            // If a depth-1 node has an election pending and hears its Root is still alive (msgDepth == 0),
            // abort the false election and re-forward the saved epoch aggregate.
            if (!isRoot && electionPending && msgSeq == treeSeq && msgDepth == 0 && senderID == parentID) {
                electionPending = FALSE;
                call RoutingMsgTimer.stop();
                dbg("SRTreeC", "Node %u aborted Root Election: Root %u is still alive!\n",
                    TOS_NODE_ID, senderID);
                forwardSavedEpochIfOpen(FALSE, call MyTimer.getNow());
                if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
                return;
            }

            // Split-brain defense: if a higher-ID node also promoted on the same treeSeq,
            // ignore its wave and immediately rebroadcast our lower-ID Root's state to heal it.
            if (isSameSeqWorseRoot) {
                if (!electionPending && !rootJustPromoted &&
                    (isRoot || (parentID < 65535 && curdepth < DEPTH_ROOT_PROBE))) {
                    call RoutingMsgTimer.startOneShot(HOP_DELAY_MILLIS + (TOS_NODE_ID % 8) * 5);
                }
                if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
                return;
            }

            // If an active Root hears another challenger advertising a different rootID on the
            // same treeSeq, immediately rebroadcast its authoritative depth-0 beacon to heal it.
            if (isRoot && !rootJustPromoted && msgSeq == treeSeq &&
                msgDepth < DEPTH_ROOT_PROBE && msgRootID != TOS_NODE_ID) {
                call RoutingMsgTimer.startOneShot(HOP_DELAY_MILLIS);
                if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
                return;
            }

            // 2. If a newer treeSeq arrives OR a lower-ID Root won a simultaneous election on
            //    the same treeSeq, reset topology state and adopt the winning Root's wave.
            if (isSuperiorTree) {
                uint32_t now = call MyTimer.getNow();
                uint16_t prevRootID = currentRootID;
                treeJoinTime = now;
                // If this node had just promoted itself to Root and was waiting in its settle
                // window, move its accumulated epoch state back into saved* so it can forward
                // the subtree aggregate upstream to the winning lower-ID Root.
                if (isRoot && rootJustPromoted) {
                    atomic {
                        savedSum = aggSum;
                        savedSumSq = aggSumSq;
                        savedCount = aggCount;
                        savedMax = aggMax;
                        epochSum = aggSum;
                        epochSumSq = aggSumSq;
                        epochCount = aggCount;
                        epochMax = aggMax;
                    }
                    resetEpochAccumulators();
                    hasUnsentEpochData = TRUE;
                    txPktToggle ^= 1;
                } else if (savedEpochDepth == 1 && lastAckedDest == prevRootID &&
                           prevRootID != msgRootID && savedEpochTime > 0 &&
                           (now - savedEpochTime) < ((uint32_t)epochDuration * 512)) {
                    // Only restore saved* = epoch* if this Depth-1 node's primary packet was ACKed
                    // by the previous Root that died (prevents double-counting across split-brain waves).
                    atomic {
                        savedSum = epochSum;
                        savedSumSq = epochSumSq;
                        savedCount = epochCount;
                        savedMax = epochMax;
                    }
                    hasUnsentEpochData = TRUE;
                    txPktToggle ^= 1;
                }
                treeSeq = msgSeq;
                currentRootID = msgRootID;
                if (electionPending || rootJustPromoted) {
                    call RoutingMsgTimer.stop();
                }
                isRoot = FALSE;
                electionPending = FALSE;
                rootJustPromoted = FALSE;
                parentFailCount = 0;
                orphanPullPending = FALSE;
                orphanPhase = ORPHAN_NONE;
                orphanMaxNeighborDepth = DEPTH_UNATTACHED;
                clearBackupTable();
                parentID = 65535;
                curdepth = DEPTH_UNATTACHED;
                lastFireDepth = DEPTH_UNATTACHED;
            }

            // During Phase 1 orphan recovery, reject candidates deeper than our former depth
            // so children and grandchildren cannot be selected before subtree poison settles
            if (!isRoot && msgSeq == treeSeq && curdepth == DEPTH_UNATTACHED &&
                (orphanPhase == ORPHAN_PULL_P1 || orphanPhase == ORPHAN_WAIT_P1) &&
                msgDepth > orphanMaxNeighborDepth) {
                if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
                return;
            }

            // 3. Passively record sender as a backup parent candidate (if same treeSeq and not active parent)
            if (msgSeq == treeSeq && !isRoot && senderID != parentID) {
                updateBackupCandidate(senderID, msgDepth);
            }

            // 4. Parent selection or current-parent depth update
            if (!isRoot && msgSeq == treeSeq &&
                (parentID >= 65535 || curdepth > offeredDepth ||
                 (senderID == parentID && curdepth != offeredDepth))) {

                uint32_t now = call MyTimer.getNow();
                uint32_t epochMillis;
                uint32_t slotLead;
                uint32_t jitter;
                uint32_t t0;
                bool wasUnattached = (curdepth >= DEPTH_ROOT_PROBE);
                bool wasOrphanPull = (orphanPhase != ORPHAN_NONE);

                // If current parent increased its depth, prefer a strictly shallower backup;
                // otherwise smoothly adopt a small (+1..+2) depth increase without detaching.
                if (senderID == parentID && offeredDepth > curdepth) {
                    if (tryFailoverToBackup()) {
                        forwardSavedEpochIfOpen(FALSE, now);
                        if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
                        return;
                    }
                    if (offeredDepth > curdepth + 2 || offeredDepth >= DEPTH_ROOT_PROBE) {
                        dbg("SRTreeC", "Node %u parent %u depth jumped (%u -> %u); detaching\n",
                            TOS_NODE_ID, senderID, curdepth, offeredDepth);
                        handleParentInvalidated(now);
                        if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
                        return;
                    }
                    curdepth = offeredDepth;
                    removeBackupCandidate(parentID);
                    pruneDeeperBackups();
                    dbg("SRTreeC", "Node %u selected parentID=%u, depth=%u (treeSeq=%u, root=%u, Query=%s, Epoch=%us)\n",
                        TOS_NODE_ID, parentID, curdepth, treeSeq, currentRootID, getQueryName(queryType), epochDuration);
                    if (lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED && curdepth != lastFireDepth) {
                        rescheduleMyTimerForDepthChange();
                    }
                    forwardSavedEpochIfOpen(FALSE, now);
                    if (!electionPending && !rootJustPromoted) {
                        call RoutingMsgTimer.startOneShot(HOP_DELAY_MILLIS + (uint32_t)(TOS_NODE_ID % 8) * 3);
                    }
                    if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
                    return;
                }

                if (offeredDepth >= DEPTH_ROOT_PROBE) {
                    if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
                    return;
                }

                parentID = senderID;
                curdepth = offeredDepth;
                currentRootID = msgRootID;
                orphanPullPending = FALSE;
                orphanPhase = ORPHAN_NONE;
                orphanMaxNeighborDepth = DEPTH_UNATTACHED;
                removeBackupCandidate(parentID);
                pruneDeeperBackups();

                queryType = mpkt->queryAndEpoch & 0x0F;
                epochDuration = (mpkt->queryAndEpoch >> 4) * 10;
                epochMillis = (uint32_t)epochDuration * 1024;

                slotLead = (uint32_t)curdepth * TAG_SLOT_MILLIS;
                // Spread sibling transmissions across 16 buckets (0..120 ms within the 200 ms level slot)
                // to reduce CSMA/ACK collisions in high-fanout star topologies
                jitter = SLOT_JITTER_MILLIS(TOS_NODE_ID);

                if (treeSeq == 0 && epochCounter == 0 &&
                    (lastFireDepth != DEPTH_UNATTACHED ||
                     (int32_t)(now - (radioStartTime + TIMER_FAST_PERIOD + FIRST_EPOCH_DELAY - slotLead - jitter)) < 0)) {
                    // Initial dissemination wave (including late boot-watchdog pulls): lock Epoch 1
                    // directly onto Root's FIRST_EPOCH_DELAY baseline from radioStartTime.
                    uint32_t firstEpochTarget = radioStartTime + TIMER_FAST_PERIOD + FIRST_EPOCH_DELAY - slotLead - jitter;
                    if ((int32_t)(now - firstEpochTarget) < 0) {
                        t0 = firstEpochTarget - epochMillis;
                        lastMyFireTime = t0;
                        lastFireDepth = curdepth;
                        call MyTimer.startPeriodicAt(t0, epochMillis);
                    } else if (curdepth != lastFireDepth) {
                        rescheduleMyTimerForDepthChange();
                    }
                } else if (!isSuperiorTree && lastMyFireTime > 0 && lastFireDepth < DEPTH_UNATTACHED) {
                    // Mid-epoch orphan recovery or depth change under the existing Root schedule
                    if (curdepth != lastFireDepth) {
                        rescheduleMyTimerForDepthChange();
                    }
                } else {
                    // New Root failover wave (or late first join): anchor phase to the new Root's broadcast
                    uint32_t fastWaveStep = (isSuperiorTree && epochCounter > 0)
                        ? (HOP_DELAY_MILLIS + 3)
                        : TIMER_FAST_PERIOD;
                    uint32_t fastRoutingElapsed = (uint32_t)(curdepth - 1) * fastWaveStep +
                                                  (uint32_t)curdepth * HOP_DELAY_MILLIS;
                    t0 = now - fastRoutingElapsed - slotLead - jitter;
                    while ((int32_t)(now - t0) < 0) {
                        t0 -= epochMillis;
                    }
                    while ((now - t0) >= epochMillis) {
                        t0 += epochMillis;
                    }
                    lastMyFireTime = t0;
                    lastFireDepth = curdepth;
                    call MyTimer.startPeriodicAt(t0, epochMillis);
                }

                dbg("SRTreeC", "Node %u selected parentID=%u, depth=%u (treeSeq=%u, root=%u, Query=%s, Epoch=%us)\n",
                    TOS_NODE_ID, parentID, curdepth, treeSeq, currentRootID, getQueryName(queryType), epochDuration);

                // If this node had unsent epoch data from a failed parent/root, forward it only
                // if the current epoch's window at the Root is still open (prevents ghost children)
                forwardSavedEpochIfOpen(isSuperiorTree, now);

                // Propagate routing advertisement downward:
                // - Tight fast-relay (8..14 ms) for new-Root failover waves so lower-ID winners suppress
                //   distant election timers across multi-hop paths before secondary promotions occur
                // - Standard fast propagation (8..29 ms) on post-boot orphan/reattach events
                // - Standard TIMER_FAST_PERIOD (+ jitter post-boot) otherwise
                if (!electionPending && !rootJustPromoted) {
                    uint32_t advDelay = (isSuperiorTree && epochCounter > 0)
                        ? (HOP_DELAY_MILLIS + (uint32_t)(TOS_NODE_ID % 4) * 2)
                        : ((wasOrphanPull || (wasUnattached && epochCounter > 0))
                            ? (HOP_DELAY_MILLIS + (uint32_t)(TOS_NODE_ID % 8) * 3)
                            : ((epochCounter > 0)
                                ? (TIMER_FAST_PERIOD + (uint32_t)(TOS_NODE_ID % 8) * 5)
                                : TIMER_FAST_PERIOD));
                    call RoutingMsgTimer.startOneShot(advDelay);
                }
            }
        }

        if (!(call RoutingReceiveQueue.empty())) post receiveRoutingTask();
    }

    /**
     * @brief Detects duplicate child packets caused by lost MAC ACK frames within the same epoch.
     */
    bool isDuplicateMacRetry(uint16_t childID, uint32_t sig, uint32_t now) {
        uint8_t i;
        for (i = 0; i < rxChildUsed; i++) {
            if (rxChildID[i] == childID) {
                if (rxChildSig[i] == sig && (now - rxChildTime[i]) < DUP_RETRY_WINDOW_MILLIS) {
                    rxChildTime[i] = now;
                    return TRUE;
                }
                rxChildSig[i] = sig;
                rxChildTime[i] = now;
                return FALSE;
            }
        }
        if (rxChildUsed < MAX_TRACKED_CHILDREN) {
            rxChildID[rxChildUsed] = childID;
            rxChildSig[rxChildUsed] = sig;
            rxChildTime[rxChildUsed] = now;
            rxChildUsed++;
        }
        return FALSE;
    }

    /**
     * @brief Dequeues incoming child data packets and folds their partial state
     *        into the current epoch's running accumulators. If a rerouted failover
     *        packet arrives shortly after this node already fired for the epoch
     *        (but before the Root's epoch deadline), it is folded and immediately
     *        forwarded upstream; late stragglers after the Root's deadline are
     *        dropped so they cannot create ghost children in the next epoch.
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

        if (call MyReceiveQueue.empty()) return;

        recPkt = call MyReceiveQueue.dequeue();
        len = call MyPacket.payloadLength(&recPkt);
        childID = call MyAMPacket.source(&recPkt);

        now = call MyTimer.getNow();
        sinceLastFire = now - lastMyFireTime;
        halfEpoch = (uint32_t)epochDuration * 512;
        inRecoveryWindow = (treeJoinTime > 0 && (now - treeJoinTime) < ROOT_RECOVERY_DELAY);
        // A node at depth d fires (d * TAG_SLOT_MILLIS + myJitter) before the Root;
        // packets arriving before that offset belong to the current epoch's failover window.
        rootDeadline = isRoot ? 0 : (((uint32_t)lastFireDepth * TAG_SLOT_MILLIS) + SLOT_JITTER_MILLIS(TOS_NODE_ID));
        isPostFireWindow = (!rootJustPromoted && epochCounter > 0 &&
                            lastMyFireTime > 0 && sinceLastFire < halfEpoch);

        // Drop late straggler packets that arrive after the Root has already fired for the epoch
        // (unless the node just joined a new Root failover wave within ROOT_RECOVERY_DELAY)
        if (isPostFireWindow && !inRecoveryWindow &&
            (isRoot || lastFireDepth >= DEPTH_UNATTACHED || sinceLastFire >= rootDeadline)) {
            if (!(call MyReceiveQueue.empty())) post receiveMyTask();
            return;
        }

        if (len == sizeof(MaxMsg) && queryType == QUERY_MAX) {
            MaxMsg* mr = (MaxMsg*)(call MyPacket.getPayload(&recPkt, len));
            uint8_t cleanMax = mr->maxVal & 0x7F;
            if (isDuplicateMacRetry(childID, mr->maxVal, now)) {
                if (!(call MyReceiveQueue.empty())) post receiveMyTask();
                return;
            }
            atomic {
                if (cleanMax > aggMax) {
                    aggMax = cleanMax;
                }
                hasPendingChildAgg = TRUE;
            }
            validPacket = TRUE;
            dbg("Lab1", "Node %u rx MaxMsg (1B) from child %u: childMax=%u -> runningMax=%u\n",
                TOS_NODE_ID, childID, cleanMax, aggMax);
        } else if (len == sizeof(SumMsg) && queryType == QUERY_SUM) {
            SumMsg* mr = (SumMsg*)(call MyPacket.getPayload(&recPkt, len));
            uint16_t cleanSum = mr->sum & 0x7FFF;
            if (isDuplicateMacRetry(childID, mr->sum, now)) {
                if (!(call MyReceiveQueue.empty())) post receiveMyTask();
                return;
            }
            atomic {
                aggSum += cleanSum;
                hasPendingChildAgg = TRUE;
            }
            validPacket = TRUE;
            dbg("Lab1", "Node %u rx SumMsg (2B) from child %u: childSum=%u -> runningSum=%u\n",
                TOS_NODE_ID, childID, cleanSum, aggSum);
        } else if (len == sizeof(AvgMsg) && queryType == QUERY_AVG) {
            AvgMsg* mr = (AvgMsg*)(call MyPacket.getPayload(&recPkt, len));
            uint16_t cleanSum = mr->sum & 0x7FFF;
            uint32_t sig = ((uint32_t)mr->sum << 8) | mr->count;
            if (isDuplicateMacRetry(childID, sig, now)) {
                if (!(call MyReceiveQueue.empty())) post receiveMyTask();
                return;
            }
            atomic {
                aggSum += cleanSum;
                aggCount += mr->count;
                hasPendingChildAgg = TRUE;
            }
            validPacket = TRUE;
            dbg("Lab1", "Node %u rx AvgMsg (3B) from child %u: (sum=%u, cnt=%u) -> running=(%u, %u)\n",
                TOS_NODE_ID, childID, cleanSum, mr->count, aggSum, aggCount);
        } else if (len == sizeof(VarMsg) && queryType == QUERY_VARIANCE) {
            VarMsg* mr = (VarMsg*)(call MyPacket.getPayload(&recPkt, len));
            uint16_t cleanSum = mr->sum & 0x7FFF;
            uint32_t sig = mr->sumSq ^ ((uint32_t)mr->sum << 8) ^ mr->count;
            if (isDuplicateMacRetry(childID, sig, now)) {
                if (!(call MyReceiveQueue.empty())) post receiveMyTask();
                return;
            }
            atomic {
                aggSumSq += mr->sumSq;
                aggSum += cleanSum;
                aggCount += mr->count;
                hasPendingChildAgg = TRUE;
            }
            validPacket = TRUE;
            dbg("Lab1", "Node %u rx VarMsg (7B) from child %u: (sum=%u, sumSq=%lu, cnt=%u)\n",
                TOS_NODE_ID, childID, cleanSum, (unsigned long)mr->sumSq, mr->count);
        }

        // If a sibling/orphan failover packet arrived after this node fired its MyTimer
        // (or during a new Root recovery window), fold into epoch* and forward upstream
        // only when no un-ACKed primary/retry packet is currently in flight (keeps saved* immutable).
        if (validPacket && (isPostFireWindow || inRecoveryWindow)) {
            if (!hasUnsentEpochData && !MySendBusy && !electionPending) {
                uint16_t fSum;
                uint32_t fSumSq;
                uint8_t fCount;
                uint8_t fMax;
                atomic {
                    fSum = aggSum;
                    fSumSq = aggSumSq;
                    fCount = aggCount;
                    fMax = aggMax;
                    epochSum += fSum;
                    epochSumSq += fSumSq;
                    epochCount += fCount;
                    if (fMax > epochMax) {
                        epochMax = fMax;
                    }
                    if (parentID < 65535) {
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
                }
                txPktToggle ^= 1;
                resetEpochAccumulators();
                clearMySendQueue();
                hasUnsentEpochData = TRUE;
                txRetryCount = 0;
                if (parentID < 65535) {
                    enqueueDataPacketToParent(fSum, fSumSq, fCount, fMax);
                }
            }
        }

        if (!(call MyReceiveQueue.empty())) {
            post receiveMyTask();
        }
    }
}