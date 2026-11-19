/**
 * @file TreeRoutingP.nc
 * @brief Private TEP-3 control-plane module implementing fault-tolerant TAG
 *        routing tree construction, two-phase orphan recovery, and slotted
 *        Root failover election (`AM_ROUTINGMSG = 22`).
 *
 * Uses three dedicated virtualized `Timer<TMilli>` interfaces (`DisseminationTimer`,
 * `OrphanTimer`, and `ElectionTimer`) so beacon advertisements, orphan pull
 * backoff, and Root election/settle deadlines never overwrite one another.
 */
#include "SimpleRoutingTree.h"

module TreeRoutingP {
    provides interface TreeTopology;

    uses {
        interface Packet as RoutingPacket;
        interface AMSend as RoutingAMSend;
        interface AMPacket as RoutingAMPacket;
        interface Receive as RoutingReceive;
        interface PacketQueue as RoutingSendQueue;
        interface PacketQueue as RoutingReceiveQueue;
        interface Timer<TMilli> as DisseminationTimer;
        interface Timer<TMilli> as OrphanTimer;
        interface Timer<TMilli> as ElectionTimer;
        interface BackupTable;
    }
}
implementation {
    /* Static transmission buffer and radio lock for AM_ROUTINGMSG */
    message_t radioRoutingSendPkt;
    bool RoutingSendBusy = FALSE;

    /* Core tree topology and active query metadata */
    uint8_t curdepth = DEPTH_UNATTACHED;
    uint16_t parentID = INVALID_NODE_ID;
    uint16_t currentRootID = 0;
    uint8_t treeSeq = 0;
    bool isRoot = FALSE;
    uint8_t queryType = 0;
    uint8_t epochDuration = 0;

    /* Failover, orphan recovery, and Root election FSM state */
    bool electionPending = FALSE;
    bool rootJustPromoted = FALSE;
    bool orphanPullPending = FALSE;
    uint8_t orphanPhase = ORPHAN_NONE;
    uint8_t orphanMaxNeighborDepth = DEPTH_UNATTACHED;
    uint32_t radioStartTime = 0;
    uint32_t treeJoinTime = 0;
    /** @brief TRUE once `treeJoinTime` records a valid superior-Root wave adoption timestamp. */
    bool hasTreeJoinTime = FALSE;

    /** @brief TRUE when an attached node enters orphan recovery and sheds child aggregates, cleared once its post-orphan attached routing beacon completes `RoutingAMSend.sendDone(SUCCESS)`. */
    bool myShedPending = FALSE;
    /** @brief Bounded retry counter for failed post-orphan shed beacon transmissions. */
    uint8_t shedBeaconRetryCount = 0;

    task void sendRoutingTask();
    task void receiveRoutingTask();

    /**
     * @brief Flushes stale outbound routing messages so outdated depths are
     *        never transmitted after detaching or updating depth.
     */
    void clearRoutingSendQueue() {
        while (!(call RoutingSendQueue.empty())) {
            call RoutingSendQueue.dequeue();
        }
    }

    /**
     * @brief Constructs and enqueues a 3-byte `RoutingMsg` broadcast carrying
     *        `treeSeq`, `advertisedDepth` (or control code 30/31), packed
     *        `queryAndEpoch`, and `currentRootID`.
     */
    void enqueueRoutingBroadcast(uint8_t advertisedDepth) {
        message_t tmp;
        RoutingMsg* mrpkt;

        clearRoutingSendQueue();
        if (call RoutingSendQueue.full()) {
            return;
        }
        memset(&tmp, 0, sizeof(message_t));

        mrpkt = (RoutingMsg*)(call RoutingPacket.getPayload(&tmp, sizeof(RoutingMsg)));
        if (mrpkt == NULL) {
            return;
        }

        mrpkt->depth = PACK_SEQ_DEPTH(treeSeq, advertisedDepth);
        mrpkt->queryAndEpoch = PACK_QUERY_EPOCH_SHED(epochDuration, queryType, myShedPending ? 1 : 0);
        mrpkt->rootID = (uint8_t)currentRootID;

        call RoutingAMPacket.setDestination(&tmp, AM_BROADCAST_ADDR);
        call RoutingPacket.setPayloadLength(&tmp, sizeof(RoutingMsg));

        if (call RoutingSendQueue.enqueue(tmp) == SUCCESS) {
            if (!RoutingSendBusy && !(signal TreeTopology.isDataSendBusy())) {
                post sendRoutingTask();
            }
        }
    }

    /**
     * @brief Invalidates the active parent, preserves any recoverable epoch
     *        aggregates via `prepareParentInvalidated`, and fails over to a
     *        strictly shallower backup or detaches into orphan recovery.
     */
    void handleParentInvalidated(uint32_t now) {
        signal TreeTopology.prepareParentInvalidated(parentID, now);
        if (call TreeTopology.tryFailoverToBackup()) {
            signal TreeTopology.forwardSavedEpoch(FALSE, now);
        } else {
            call TreeTopology.enterOrphanState();
        }
    }

    command void TreeTopology.initBootState(bool rootNode, uint8_t initQueryType, uint8_t initEpochDur) {
        RoutingSendBusy = FALSE;
        treeSeq = 0;
        radioStartTime = 0;
        treeJoinTime = 0;
        hasTreeJoinTime = FALSE;
        electionPending = FALSE;
        rootJustPromoted = FALSE;
        orphanPullPending = FALSE;
        orphanPhase = ORPHAN_NONE;
        orphanMaxNeighborDepth = DEPTH_UNATTACHED;
        myShedPending = FALSE;
        shedBeaconRetryCount = 0;
        call BackupTable.clear();

        if (rootNode) {
            isRoot = TRUE;
            currentRootID = 0;
            curdepth = 0;
            parentID = 0;
            queryType = initQueryType;
            epochDuration = initEpochDur;
        } else {
            isRoot = FALSE;
            currentRootID = 0;
            curdepth = DEPTH_UNATTACHED;
            parentID = INVALID_NODE_ID;
            queryType = 0;
            epochDuration = 0;
        }
    }

    command void TreeTopology.onRadioStarted(uint32_t startTime) {
        radioStartTime = startTime;
        if (isRoot) {
            call DisseminationTimer.startOneShot(TIMER_FAST_PERIOD);
        } else {
            // Boot watchdog: if a sensor node misses every broadcast in the initial
            // dissemination wave, this fires well before Epoch 1 to pull routing state.
            call OrphanTimer.startOneShot(2000 + (uint32_t)(TOS_NODE_ID % 16) * 25);
        }
    }

    command bool TreeTopology.isRoot() { return isRoot; }
    command uint8_t TreeTopology.getDepth() { return curdepth; }
    command uint16_t TreeTopology.getParent() { return parentID; }
    command uint16_t TreeTopology.getRootID() { return currentRootID; }
    command uint8_t TreeTopology.getTreeSeq() { return treeSeq; }
    command uint8_t TreeTopology.getQueryType() { return queryType; }
    command uint8_t TreeTopology.getEpochDuration() { return epochDuration; }
    command bool TreeTopology.isElectionPending() { return electionPending; }
    command bool TreeTopology.isRootJustPromoted() { return rootJustPromoted; }
    command uint8_t TreeTopology.getOrphanPhase() { return orphanPhase; }

    command bool TreeTopology.isOrphanShedPending() {
        return (!isRoot && (curdepth >= DEPTH_ROOT_PROBE || myShedPending));
    }

    command bool TreeTopology.isInRecoveryWindow(uint32_t now) {
        return (hasTreeJoinTime && (now - treeJoinTime) < ROOT_RECOVERY_DELAY);
    }

    command bool TreeTopology.isRoutingSendBusy() {
        return RoutingSendBusy;
    }

    command bool TreeTopology.hasPendingRoutingWork() {
        if (RoutingSendBusy ||
            !(call RoutingSendQueue.empty()) ||
            !(call RoutingReceiveQueue.empty())) {
            return TRUE;
        }
        if (electionPending || rootJustPromoted ||
            orphanPhase != ORPHAN_NONE || orphanPullPending) {
            return TRUE;
        }
        if (!isRoot && curdepth >= DEPTH_ROOT_PROBE) {
            return TRUE;
        }
        if ((call DisseminationTimer.isRunning()) ||
            (call OrphanTimer.isRunning()) ||
            (call ElectionTimer.isRunning())) {
            return TRUE;
        }
        return FALSE;
    }

    command void TreeTopology.kickRoutingSend() {
        if (!(call RoutingSendQueue.empty()) && !RoutingSendBusy) {
            post sendRoutingTask();
        }
    }

    command void TreeTopology.sendRoutingBroadcast(uint8_t advertisedDepth) {
        enqueueRoutingBroadcast(advertisedDepth);
    }

    command void TreeTopology.startRootElection() {
        // Compensate for intra-slot TAG jitter so Depth-1 candidates' promotion deadlines
        // remain strictly ordered by TOS_NODE_ID with a uniform ELECTION_STEP_MILLIS gap.
        uint32_t electDelay = ELECTION_BASE_MILLIS +
                              (uint32_t)TOS_NODE_ID * ELECTION_STEP_MILLIS +
                              SLOT_JITTER_MILLIS(TOS_NODE_ID);
        uint32_t probeRetryDelay = ROOT_PROBE_RETRY_MILLIS + (uint32_t)(TOS_NODE_ID % 8) * 5;
        electionPending = TRUE;
        currentRootID = parentID;
        call OrphanTimer.stop();
        if (!(signal TreeTopology.isRadioOn())) {
            signal TreeTopology.requestRadioWake();
        }
        enqueueRoutingBroadcast(DEPTH_ROOT_PROBE);
        dbg("SRTreeC", "Node %u (depth 1) detected ROOT %u FAILURE! Probing & starting election timer (%lu ms)\n",
            TOS_NODE_ID, currentRootID, (unsigned long)electDelay);
        // Schedule one staggered follow-up liveness probe after transient high-fanout
        // sibling data bursts subside, still well before the earliest election expiry.
        call DisseminationTimer.startOneShot(probeRetryDelay);
        call ElectionTimer.startOneShot(electDelay);
    }

    command bool TreeTopology.tryFailoverToBackup() {
        uint8_t candDepth = DEPTH_UNATTACHED;
        uint16_t newParent = call BackupTable.selectBest(curdepth, &candDepth);

        if (newParent != INVALID_NODE_ID && candDepth < DEPTH_ROOT_PROBE) {
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
            call OrphanTimer.stop();
            call BackupTable.remove(parentID);
            call BackupTable.pruneDeeper(curdepth);

            dbg("SRTreeC", "FAILOVER: Node %u switched from parent %u to backup parent %u (depth %u -> %u)\n",
                TOS_NODE_ID, deadParent, parentID, oldDepth, curdepth);

            if (curdepth != oldDepth) {
                signal TreeTopology.depthChanged(oldDepth, curdepth);
            }
            // Broadcast updated depth immediately so downstream children refresh quickly without colliding
            if (!electionPending && !rootJustPromoted) {
                call DisseminationTimer.startOneShot(HOP_DELAY_MILLIS + (uint32_t)(TOS_NODE_ID % 8) * 3);
            }
            return TRUE;
        }
        return FALSE;
    }

    command void TreeTopology.enterOrphanState() {
        if (curdepth < DEPTH_ROOT_PROBE) {
            orphanMaxNeighborDepth = curdepth;
            myShedPending = TRUE;
            shedBeaconRetryCount = 0;
        }
        orphanPhase = ORPHAN_PULL_P1;
        parentID = INVALID_NODE_ID;
        curdepth = DEPTH_UNATTACHED;
        call BackupTable.clear();
        clearRoutingSendQueue();
        signal TreeTopology.onEnteredOrphan();

        orphanPullPending = TRUE;
        call DisseminationTimer.stop();
        if (!electionPending && !rootJustPromoted) {
            if (!(signal TreeTopology.isRadioOn())) {
                signal TreeTopology.requestRadioWake();
            }
            // Issue a single Phase-1 pull immediately and enter ORPHAN_WAIT_P1;
            // avoids a duplicate P1 broadcast that would reset neighbors' reply timers.
            orphanPhase = ORPHAN_WAIT_P1;
            enqueueRoutingBroadcast(DEPTH_UNATTACHED);
            call OrphanTimer.startOneShot(ORPHAN_DEEPER_DELAY + (uint32_t)(TOS_NODE_ID % 8) * 3);
        }
    }

    command const char* TreeTopology.getQueryName(uint8_t qType) {
        switch (qType) {
            case QUERY_SUM:      return "SUM";
            case QUERY_AVG:      return "AVG";
            case QUERY_VARIANCE: return "VARIANCE";
            case QUERY_MAX:      return "MAX";
            default:             return "UNKNOWN";
        }
    }

    /**
     * @brief Handles slotted Root failover election promotion and the
     *        post-election `ROOT_RECOVERY_DELAY` settle readout.
     */
    event void ElectionTimer.fired() {
        // Case 1: Newly promoted Root finishes its settle window and outputs recovered epoch
        if (isRoot && rootJustPromoted) {
            rootJustPromoted = FALSE;
            signal TreeTopology.onRootSettleComplete();
            signal TreeTopology.checkRadioSleep();
            return;
        }

        // Case 2: Depth-1 node's election timer expires without hearing a lower-ID new Root
        if (electionPending) {
            uint32_t now = call ElectionTimer.getNow();

            electionPending = FALSE;
            isRoot = TRUE;
            currentRootID = TOS_NODE_ID;
            parentID = TOS_NODE_ID;
            myShedPending = FALSE;
            shedBeaconRetryCount = 0;
            curdepth = 0;
            orphanPullPending = FALSE;
            orphanPhase = ORPHAN_NONE;
            orphanMaxNeighborDepth = DEPTH_UNATTACHED;
            treeSeq = (treeSeq + 1) & 0x07;
            rootJustPromoted = TRUE;
            treeJoinTime = 0;
            hasTreeJoinTime = FALSE;

            call DisseminationTimer.stop();
            call OrphanTimer.stop();
            signal TreeTopology.onRootPromoted(treeSeq, now);

            // Schedule settle window so sibling Depth-1 nodes can forward their saved aggregates
            // and emit one follow-up depth-0 beacon after initial failover bursts subside
            call ElectionTimer.startOneShot(ROOT_RECOVERY_DELAY);
            call DisseminationTimer.startOneShot(ROOT_PROBE_REPLY_MILLIS);
            enqueueRoutingBroadcast(0);
            return;
        }

        signal TreeTopology.checkRadioSleep();
    }

    /**
     * @brief Handles initial Root dissemination, staggered depth-update /
     *        local-repair routing advertisements, and follow-up Root liveness
     *        probing when a Depth-1 election timer is pending.
     */
    event void DisseminationTimer.fired() {
        if (rootJustPromoted) {
            if (isRoot) {
                enqueueRoutingBroadcast(0);
                return;
            }
            signal TreeTopology.checkRadioSleep();
            return;
        }
        if (electionPending) {
            if (!isRoot) {
                enqueueRoutingBroadcast(DEPTH_ROOT_PROBE);
            } else {
                signal TreeTopology.checkRadioSleep();
            }
            return;
        }
        if (!isRoot && curdepth == DEPTH_UNATTACHED) {
            signal TreeTopology.checkRadioSleep();
            return;
        }

        // Initial Root startup synchronization (data plane guards single execution)
        if (isRoot && treeSeq == 0) {
            signal TreeTopology.initialRootStart(call DisseminationTimer.getNow());
        }

        enqueueRoutingBroadcast(curdepth);
    }

    /**
     * @brief Drives the two-phase cycle-free orphan pull state machine and
     *        boot watchdog fallback while the node remains unattached.
     */
    event void OrphanTimer.fired() {
        if (isRoot || curdepth != DEPTH_UNATTACHED || electionPending || rootJustPromoted) {
            signal TreeTopology.checkRadioSleep();
            return;
        }

        if (!(signal TreeTopology.isRadioOn())) {
            signal TreeTopology.requestRadioWake();
        }

        // If an unattached node's boot watchdog fired, transition into active Phase-2 pull
        if (orphanPhase == ORPHAN_NONE) {
            orphanPhase = ORPHAN_PULL_P2;
            orphanMaxNeighborDepth = DEPTH_UNATTACHED;
            orphanPullPending = TRUE;
        }

        orphanPullPending = FALSE;
        enqueueRoutingBroadcast(DEPTH_UNATTACHED);

        // Two-phase orphan recovery + fast Phase-2 retry + periodic fallback:
        // - Phase 1 (ORPHAN_PULL_P1 -> ORPHAN_WAIT_P1): settle window accepting <= former depth.
        // - Phase 2 entry (ORPHAN_WAIT_P1 -> ORPHAN_PULL_P2): relax depth constraint and schedule
        //   a fast retry (ORPHAN_P2_RETRY_MILLIS + jitter) before backing off to ORPHAN_RETRY_PERIOD.
        // - Phase 2 steady (ORPHAN_WAIT_P2): retry every ORPHAN_RETRY_PERIOD so partitioned nodes
        //   do not storm the channel while still recovering seamlessly if reconnected.
        if (orphanPhase == ORPHAN_PULL_P1) {
            orphanPhase = ORPHAN_WAIT_P1;
            orphanPullPending = TRUE;
            call OrphanTimer.startOneShot(ORPHAN_DEEPER_DELAY + (uint32_t)(TOS_NODE_ID % 8) * 3);
        } else if (orphanPhase == ORPHAN_WAIT_P1) {
            orphanPhase = ORPHAN_PULL_P2;
            orphanMaxNeighborDepth = DEPTH_UNATTACHED;
            orphanPullPending = TRUE;
            call OrphanTimer.startOneShot(ORPHAN_P2_RETRY_MILLIS + (uint32_t)(TOS_NODE_ID % 8) * 5);
        } else {
            orphanPhase = ORPHAN_WAIT_P2;
            orphanMaxNeighborDepth = DEPTH_UNATTACHED;
            orphanPullPending = TRUE;
            call OrphanTimer.startOneShot(ORPHAN_RETRY_PERIOD + (uint32_t)(TOS_NODE_ID % 8) * 25);
        }
    }

    event void RoutingAMSend.sendDone(message_t* msg, error_t err) {
        RoutingMsg* spkt;

        RoutingSendBusy = FALSE;
        if (!isRoot && curdepth < DEPTH_ROOT_PROBE && myShedPending &&
            (call RoutingSendQueue.empty())) {
            spkt = (RoutingMsg*)(call RoutingPacket.getPayload(msg, sizeof(RoutingMsg)));
            if (spkt != NULL && UNPACK_DEPTH(spkt->depth) < DEPTH_ROOT_PROBE) {
                if (err == SUCCESS) {
                    myShedPending = FALSE;
                    shedBeaconRetryCount = 0;
                } else if (!electionPending && !rootJustPromoted &&
                           shedBeaconRetryCount < MAX_SHED_BEACON_RETRIES) {
                    shedBeaconRetryCount++;
                    call DisseminationTimer.startOneShot(HOP_DELAY_MILLIS + (uint32_t)(((TOS_NODE_ID * 7U) & 0x0FU) * 3U));
                } else {
                    myShedPending = FALSE;
                    shedBeaconRetryCount = 0;
                }
            }
        }
        if (!(call RoutingSendQueue.empty())) {
            post sendRoutingTask();
        }
        signal TreeTopology.routingSendCompleted();
        if (call RoutingSendQueue.empty() && !RoutingSendBusy) {
            signal TreeTopology.checkRadioSleep();
        }
    }

    event message_t* RoutingReceive.receive(message_t* msg, void* payload, uint8_t len) {
        message_t tmp;
        if (len != sizeof(RoutingMsg)) {
            return msg;
        }
        memcpy(&tmp, msg, sizeof(message_t));
        if (call RoutingReceiveQueue.enqueue(tmp) == SUCCESS) {
            post receiveRoutingTask();
        }
        return msg;
    }

    task void sendRoutingTask() {
        uint8_t mlen;
        uint16_t mdest;
        error_t sendDone;

        if (call RoutingSendQueue.empty()) {
            signal TreeTopology.checkRadioSleep();
            return;
        }
        if (RoutingSendBusy || (signal TreeTopology.isDataSendBusy())) {
            return;
        }
        if (!(signal TreeTopology.isRadioOn())) {
            signal TreeTopology.requestRadioWake();
            return;
        }

        // Peek via head() first; only dequeue() after AMSend.send() accepts the packet
        // so a transient radio busy/fail return never drops the packet from the queue.
        radioRoutingSendPkt = call RoutingSendQueue.head();
        mlen = call RoutingPacket.payloadLength(&radioRoutingSendPkt);
        mdest = call RoutingAMPacket.destination(&radioRoutingSendPkt);
        if (mlen != sizeof(RoutingMsg)) {
            call RoutingSendQueue.dequeue();
            if (!(call RoutingSendQueue.empty())) {
                post sendRoutingTask();
            } else {
                signal TreeTopology.routingSendCompleted();
                signal TreeTopology.checkRadioSleep();
            }
            return;
        }

        sendDone = call RoutingAMSend.send(mdest, &radioRoutingSendPkt, mlen);
        if (sendDone == SUCCESS) {
            call RoutingSendQueue.dequeue();
            RoutingSendBusy = TRUE;
        } else if (sendDone == EOFF) {
            dbg("Radio", "WARN: Node %u RoutingAMSend.send returned EOFF\n", TOS_NODE_ID);
            signal TreeTopology.handleRadioEoff();
            return;
        } else if (sendDone == ESIZE) {
            dbg("SRTreeC", "WARN: Node %u RoutingAMSend.send returned ESIZE (len=%u); dropping\n",
                TOS_NODE_ID, mlen);
            call RoutingSendQueue.dequeue();
            if (!(call RoutingSendQueue.empty())) {
                post sendRoutingTask();
            } else {
                signal TreeTopology.routingSendCompleted();
                signal TreeTopology.checkRadioSleep();
            }
        } else {
            post sendRoutingTask();
        }
    }

    /**
     * @brief Continues draining `RoutingReceiveQueue` or signals `SRTreeC` to
     *        re-evaluate radio sleep once all queued control frames are handled.
     */
    void scheduleNextOrCheckSleepRoutingRx() {
        if (!(call RoutingReceiveQueue.empty())) {
            post receiveRoutingTask();
        } else {
            signal TreeTopology.checkRadioSleep();
        }
    }

    /**
     * @brief Processes inbound `RoutingMsg` frames:
     *        - Root liveness probes (`DEPTH_ROOT_PROBE = 30`)
     *        - Orphan pulls / parent detachment poison (`DEPTH_UNATTACHED = 31`)
     *        - False-election abortion and split-brain tie-breaking
     *        - Backup table maintenance and shortest-path parent selection
     */
    task void receiveRoutingTask() {
        message_t radioRoutingRecPkt;
        uint8_t len;

        if (call RoutingReceiveQueue.empty()) {
            signal TreeTopology.checkRadioSleep();
            return;
        }

        radioRoutingRecPkt = call RoutingReceiveQueue.dequeue();
        len = call RoutingPacket.payloadLength(&radioRoutingRecPkt);

        if (len == sizeof(RoutingMsg)) {
            RoutingMsg* mpkt = (RoutingMsg*)(call RoutingPacket.getPayload(&radioRoutingRecPkt, len));
            uint16_t senderID = call RoutingAMPacket.source(&radioRoutingRecPkt);
            uint8_t msgSeq = UNPACK_SEQ(mpkt->depth);
            uint8_t msgDepth = UNPACK_DEPTH(mpkt->depth);
            uint8_t msgShed = UNPACK_SHED_TOGGLE(mpkt->queryAndEpoch);
            uint16_t msgRootID = mpkt->rootID;
            uint8_t offeredDepth = msgDepth + 1;

            // 3-bit modular wraparound comparison (half-space < 4 treats ahead-by-1..3 as newer)
            bool isNewTreeSeq = ((uint8_t)((msgSeq - treeSeq) & 0x07) > 0) &&
                                ((uint8_t)((msgSeq - treeSeq) & 0x07) < 4);
            bool isOlderTreeSeq = ((uint8_t)((treeSeq - msgSeq) & 0x07) > 0) &&
                                  ((uint8_t)((treeSeq - msgSeq) & 0x07) < 4);

            // Deterministic split-brain tie-breaker: if two non-adjacent Depth-1 nodes both
            // promote to Root on the same treeSeq, the lower rootID wins across the network.
            bool isSameSeqBetterRoot = (msgSeq == treeSeq) &&
                                       (msgDepth < DEPTH_ROOT_PROBE) &&
                                       (msgRootID < currentRootID);
            bool isSameSeqWorseRoot = (msgSeq == treeSeq) &&
                                      (msgDepth < DEPTH_ROOT_PROBE) &&
                                      (msgRootID > currentRootID);
            bool isSuperiorTree = isNewTreeSeq || isSameSeqBetterRoot;
            bool parentShed = FALSE;

            // 0. Root liveness probe (code 30): only the active Root replies immediately
            //    (plus a follow-up beacon after ROOT_PROBE_REPLY_MILLIS once busy sibling bursts
            //    clear); all non-root nodes ignore it.
            if (msgDepth == DEPTH_ROOT_PROBE) {
                if (isRoot && !rootJustPromoted && msgSeq == treeSeq) {
                    enqueueRoutingBroadcast(0);
                    call DisseminationTimer.startOneShot(ROOT_PROBE_REPLY_MILLIS);
                }
                scheduleNextOrCheckSleepRoutingRx();
                return;
            }

            // 1. Handle local orphan pull / parent-detachment notifications
            if (msgDepth == DEPTH_UNATTACHED) {
                if (msgSeq == treeSeq && !isRoot) {
                    if (senderID == parentID) {
                        // Active parent detached: fail over to a shallower backup or detach too
                        dbg("SRTreeC", "Node %u parent %u became unattached; recovering\n",
                            TOS_NODE_ID, senderID);
                        handleParentInvalidated(call DisseminationTimer.getNow());
                        scheduleNextOrCheckSleepRoutingRx();
                        return;
                    }
                    call BackupTable.updateCandidate(senderID, DEPTH_UNATTACHED, parentID, curdepth);
                }

                // Reply after 2*HOP_DELAY_MILLIS so direct children's poison reaches grandchildren first
                if (!electionPending && !rootJustPromoted &&
                    (isRoot || (parentID < INVALID_NODE_ID && curdepth < DEPTH_ROOT_PROBE)) &&
                    (msgSeq == treeSeq || isOlderTreeSeq)) {
                    call DisseminationTimer.startOneShot((2 * HOP_DELAY_MILLIS) + (TOS_NODE_ID % 8) * 5);
                }
                scheduleNextOrCheckSleepRoutingRx();
                return;
            }

            // If a Depth-1 node has an election pending and hears its Root is still alive (msgDepth == 0),
            // abort the false election, cancel any follow-up probe, and re-forward the saved epoch aggregate.
            if (!isRoot && electionPending && msgSeq == treeSeq && msgDepth == 0 && senderID == parentID) {
                electionPending = FALSE;
                call ElectionTimer.stop();
                call DisseminationTimer.stop();
                dbg("SRTreeC", "Node %u aborted Root Election: Root %u is still alive!\n",
                    TOS_NODE_ID, senderID);
                signal TreeTopology.forwardSavedEpoch(FALSE, call DisseminationTimer.getNow());
                scheduleNextOrCheckSleepRoutingRx();
                return;
            }

            // Split-brain defense: if a higher-ID node also promoted on the same treeSeq,
            // ignore its wave and immediately rebroadcast our lower-ID Root's state to heal it.
            if (isSameSeqWorseRoot) {
                if (!electionPending && !rootJustPromoted &&
                    (isRoot || (parentID < INVALID_NODE_ID && curdepth < DEPTH_ROOT_PROBE))) {
                    call DisseminationTimer.startOneShot(HOP_DELAY_MILLIS + (TOS_NODE_ID % 8) * 5);
                }
                scheduleNextOrCheckSleepRoutingRx();
                return;
            }

            // If an active Root hears another challenger advertising a different rootID on the
            // same treeSeq, immediately rebroadcast its authoritative depth-0 beacon to heal it.
            if (isRoot && !rootJustPromoted && msgSeq == treeSeq &&
                msgDepth < DEPTH_ROOT_PROBE && msgRootID != TOS_NODE_ID) {
                call DisseminationTimer.startOneShot(HOP_DELAY_MILLIS);
                scheduleNextOrCheckSleepRoutingRx();
                return;
            }

            // 2. If a newer treeSeq arrives OR a lower-ID Root won a simultaneous election on
            //    the same treeSeq, reset topology state and adopt the winning Root's wave.
            if (isSuperiorTree) {
                uint32_t now = call DisseminationTimer.getNow();
                uint16_t prevRootID = currentRootID;
                bool wasPromotedRoot = (isRoot && rootJustPromoted);
                treeJoinTime = now;
                hasTreeJoinTime = TRUE;

                signal TreeTopology.onAdoptSuperiorTree(wasPromotedRoot, prevRootID, msgRootID, now);

                treeSeq = msgSeq;
                currentRootID = msgRootID;
                if (electionPending || rootJustPromoted) {
                    call ElectionTimer.stop();
                }
                call OrphanTimer.stop();
                call DisseminationTimer.stop();
                isRoot = FALSE;
                electionPending = FALSE;
                rootJustPromoted = FALSE;
                orphanPullPending = FALSE;
                orphanPhase = ORPHAN_NONE;
                orphanMaxNeighborDepth = DEPTH_UNATTACHED;
                myShedPending = FALSE;
                shedBeaconRetryCount = 0;
                call BackupTable.clear();
                parentID = INVALID_NODE_ID;
                curdepth = DEPTH_UNATTACHED;
            }

            // During Phase-1 orphan recovery, reject candidates deeper than our former depth
            // so children and grandchildren cannot be selected before subtree poison settles.
            if (!isRoot && msgSeq == treeSeq && curdepth == DEPTH_UNATTACHED &&
                (orphanPhase == ORPHAN_PULL_P1 || orphanPhase == ORPHAN_WAIT_P1) &&
                msgDepth > orphanMaxNeighborDepth) {
                scheduleNextOrCheckSleepRoutingRx();
                return;
            }

            // 3. Passively record sender as a backup parent candidate (if same treeSeq and not active parent)
            if (msgSeq == treeSeq && !isRoot && senderID != parentID) {
                call BackupTable.updateCandidate(senderID, msgDepth, parentID, curdepth);
            }

            parentShed = (parentID < INVALID_NODE_ID && senderID == parentID && msgShed != 0);

            // 4. Parent selection or current-parent depth update
            if (!isRoot && msgSeq == treeSeq &&
                (parentID >= INVALID_NODE_ID || curdepth > offeredDepth ||
                 (senderID == parentID && (curdepth != offeredDepth || parentShed)))) {

                uint32_t now = call DisseminationTimer.getNow();
                uint16_t epCounter;
                bool wasUnattached = (curdepth >= DEPTH_ROOT_PROBE);
                bool wasOrphanPull = (orphanPhase != ORPHAN_NONE);

                // If current parent increased its depth, prefer a strictly shallower backup;
                // otherwise smoothly adopt a small (+1..+2) depth increase without detaching.
                if (senderID == parentID && offeredDepth > curdepth) {
                    uint16_t oldParent = parentID;

                    if (call TreeTopology.tryFailoverToBackup()) {
                        if (parentShed) {
                            signal TreeTopology.prepareParentInvalidated(oldParent, now);
                        }
                        signal TreeTopology.forwardSavedEpoch(FALSE, now);
                        scheduleNextOrCheckSleepRoutingRx();
                        return;
                    }
                    if (offeredDepth > curdepth + 2 || offeredDepth >= DEPTH_ROOT_PROBE) {
                        dbg("SRTreeC", "Node %u parent %u depth jumped (%u -> %u); detaching\n",
                            TOS_NODE_ID, senderID, curdepth, offeredDepth);
                        handleParentInvalidated(now);
                        scheduleNextOrCheckSleepRoutingRx();
                        return;
                    }
                    if (parentShed) {
                        signal TreeTopology.prepareParentInvalidated(parentID, now);
                    }
                    curdepth = offeredDepth;
                    call BackupTable.remove(parentID);
                    call BackupTable.pruneDeeper(curdepth);
                    dbg("SRTreeC", "Node %u selected parentID=%u, depth=%u (treeSeq=%u, root=%u, Query=%s, Epoch=%us)\n",
                        TOS_NODE_ID, parentID, curdepth, treeSeq, currentRootID,
                        call TreeTopology.getQueryName(queryType), epochDuration);
                    signal TreeTopology.onParentDepthIncreased(curdepth, now);
                    signal TreeTopology.forwardSavedEpoch(FALSE, now);
                    if (!electionPending && !rootJustPromoted) {
                        call DisseminationTimer.startOneShot(HOP_DELAY_MILLIS + (uint32_t)(TOS_NODE_ID % 8) * 3);
                    }
                    scheduleNextOrCheckSleepRoutingRx();
                    return;
                }

                if (offeredDepth >= DEPTH_ROOT_PROBE) {
                    scheduleNextOrCheckSleepRoutingRx();
                    return;
                }

                if (senderID == parentID && parentShed) {
                    signal TreeTopology.prepareParentInvalidated(parentID, now);
                }

                parentID = senderID;
                curdepth = offeredDepth;
                currentRootID = msgRootID;
                orphanPullPending = FALSE;
                orphanPhase = ORPHAN_NONE;
                orphanMaxNeighborDepth = DEPTH_UNATTACHED;
                call OrphanTimer.stop();
                call BackupTable.remove(parentID);
                call BackupTable.pruneDeeper(curdepth);

                queryType = UNPACK_QUERY(mpkt->queryAndEpoch);
                epochDuration = UNPACK_EPOCH_DUR(mpkt->queryAndEpoch);

                signal TreeTopology.onParentSelected(isSuperiorTree, curdepth, queryType,
                                                     epochDuration, radioStartTime, now);

                dbg("SRTreeC", "Node %u selected parentID=%u, depth=%u (treeSeq=%u, root=%u, Query=%s, Epoch=%us)\n",
                    TOS_NODE_ID, parentID, curdepth, treeSeq, currentRootID,
                    call TreeTopology.getQueryName(queryType), epochDuration);

                // If this node had unsent epoch data from a failed parent/root, forward it only
                // if the current epoch's window at the Root is still open (prevents ghost children).
                signal TreeTopology.forwardSavedEpoch(isSuperiorTree, now);

                // Propagate routing advertisement downward:
                // - Collision-staggered fast relay (16..61 ms, coprime mod-16 stride so
                //   grid neighbors never collide mod 4) when clearing post-orphan shed beacons
                // - Staggered post-burst relay (16..51 ms) for new-Root failover waves so
                //   relays avoid the 0..15 ms forwardSavedEpoch burst and colliding siblings
                // - Standard fast propagation (8..29 ms) on post-boot orphan/reattach events
                // - Standard TIMER_FAST_PERIOD (+ jitter post-boot) otherwise
                epCounter = signal TreeTopology.getEpochCounter();
                if (!electionPending && !rootJustPromoted) {
                    uint32_t advDelay = myShedPending
                        ? ((2 * HOP_DELAY_MILLIS) + (uint32_t)(((TOS_NODE_ID * 7U) & 0x0FU) * 3U))
                        : ((isSuperiorTree && epCounter > 0)
                            ? ((2 * HOP_DELAY_MILLIS) + (uint32_t)(TOS_NODE_ID % 8) * 5)
                            : ((wasOrphanPull || (wasUnattached && epCounter > 0))
                                ? (HOP_DELAY_MILLIS + (uint32_t)(TOS_NODE_ID % 8) * 3)
                                : ((epCounter > 0)
                                    ? (TIMER_FAST_PERIOD + (uint32_t)(TOS_NODE_ID % 8) * 5)
                                    : TIMER_FAST_PERIOD)));
                    call DisseminationTimer.startOneShot(advDelay);
                }
            }
        }

        scheduleNextOrCheckSleepRoutingRx();
    }
}

