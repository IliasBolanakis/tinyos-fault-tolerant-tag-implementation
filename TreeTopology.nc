/**
 * @file TreeTopology.nc
 * @brief Control-plane / data-plane boundary interface between `TreeRoutingP`
 *        (AM_ROUTINGMSG = 22) and the TAG aggregation engine (`SRTreeC`,
 *        AM_MYMSG = 23).
 *
 * Encapsulates routing tree state (parent, depth, Root ID, `treeSeq`, query
 * metadata, orphan state machine, and Root election state) while delivering
 * lifecycle events to the TAG data plane for epoch timer alignment, aggregate
 * preservation, and deadline-guarded forwarding.
 */
#include "SimpleRoutingTree.h"

interface TreeTopology {
    /**
     * @brief Initializes control-plane topology state at node boot.
     *
     * @param rootNode      TRUE if this node boots as the initial Root (Node 0).
     * @param initQueryType Selected TAG query ID (`QUERY_SUM..QUERY_MAX`) if Root.
     * @param initEpochDur  Selected epoch duration in seconds (`30..100`) if Root.
     */
    command void initBootState(bool rootNode, uint8_t initQueryType, uint8_t initEpochDur);

    /**
     * @brief Starts initial Root dissemination or non-root boot watchdog once
     *        the radio stack reports `RadioControl.startDone(SUCCESS)`.
     *
     * @param startTime Local millisecond timestamp when the radio started.
     */
    command void onRadioStarted(uint32_t startTime);

    /** @return TRUE if this node is currently acting as the active tree Root. */
    command bool isRoot();

    /** @return Current hop depth (`0..29`, or `DEPTH_UNATTACHED`). */
    command uint8_t getDepth();

    /** @return Active parent node ID (`TOS_NODE_ID` if Root, `INVALID_NODE_ID` if detached). */
    command uint16_t getParent();

    /** @return Node ID of the currently tracked active Root. */
    command uint16_t getRootID();

    /** @return Current 3-bit modular tree epoch sequence `[0..7]`. */
    command uint8_t getTreeSeq();

    /** @return Active TAG query type (`QUERY_SUM..QUERY_MAX`). */
    command uint8_t getQueryType();

    /** @return Active epoch duration in seconds (`30..100`). */
    command uint8_t getEpochDuration();

    /** @return TRUE while a Depth-1 Root failover election timer is armed. */
    command bool isElectionPending();

    /** @return TRUE while a newly elected Root is in its `ROOT_RECOVERY_DELAY` settle window. */
    command bool isRootJustPromoted();

    /** @return Current orphan state machine phase (`ORPHAN_NONE..ORPHAN_WAIT_P2`). */
    command uint8_t getOrphanPhase();

    /**
     * @brief Checks whether the node is within `ROOT_RECOVERY_DELAY` ms of
     *        joining a newly elected Root's tree wave.
     *
     * @param now Current local millisecond timestamp.
     * @return TRUE if within the Root failover recovery window.
     */
    command bool isInRecoveryWindow(uint32_t now);

    /** @return TRUE while `RoutingAMSend` holds the shared radio channel. */
    command bool isRoutingSendBusy();

    /**
     * @brief Attempts to schedule a pending routing transmission once data-plane
     *        radio traffic completes.
     */
    command void kickRoutingSend();

    /**
     * @brief Enqueues and transmits a 3-byte `RoutingMsg` broadcast carrying
     *        the specified `advertisedDepth` (or control code `30`/`31`).
     *
     * @param advertisedDepth Depth or control code (`DEPTH_ROOT_PROBE` / `DEPTH_UNATTACHED`).
     */
    command void sendRoutingBroadcast(uint8_t advertisedDepth);

    /**
     * @brief Initiates Depth-1 Root liveness probing (`DEPTH_ROOT_PROBE`) and
     *        arms the ID-slotted Root election timer.
     */
    command void startRootElection();

    /**
     * @brief Attempts immediate local failover to the best strictly shallower
     *        backup parent in `BackupTable`.
     *
     * @return TRUE if a valid backup parent was adopted; FALSE if backup table is empty.
     */
    command bool tryFailoverToBackup();

    /**
     * @brief Detaches the node into Phase-1 orphan recovery (`DEPTH_UNATTACHED`)
     *        and notifies the data plane to shed stale child aggregates.
     */
    command void enterOrphanState();

    /**
     * @brief Maps a query ID (`1..4`) to its human-readable name string.
     *
     * @param qType Query type constant.
     * @return Constant string `"SUM"`, `"AVG"`, `"VARIANCE"`, `"MAX"`, or `"UNKNOWN"`.
     */
    command const char* getQueryName(uint8_t qType);

    /** @return TRUE while `MyAMSend` holds the shared radio channel. */
    event bool isDataSendBusy();

    /** @return Current `epochCounter` from the data plane. */
    event uint16_t getEpochCounter();

    /**
     * @brief Signaled when a routing transmission completes so queued data-plane
     *        packets can be dispatched.
     */
    event void routingSendCompleted();

    /**
     * @brief Signaled when the initial Root's dissemination timer fires so the
     *        data plane can schedule Epoch 1 (`FIRST_EPOCH_DELAY`).
     *
     * @param now Current local millisecond timestamp.
     */
    event void initialRootStart(uint32_t now);

    /**
     * @brief Signaled when entering orphan state so the data plane flushes
     *        outbound queues and sheds child aggregates while preserving `myVal`.
     */
    event void onEnteredOrphan();

    /**
     * @brief Signaled when failover to a backup parent changes tree depth.
     *
     * @param oldDepth Previous tree depth.
     * @param newDepth Newly adopted tree depth.
     */
    event void depthChanged(uint8_t oldDepth, uint8_t newDepth);

    /**
     * @brief Signaled immediately before invalidating an unreachable or detached
     *        active parent so the data plane can fold pending child deltas and
     *        restore `epoch*` snapshots if `oldParent` had already ACKed primary data.
     *
     * @param oldParent Parent node ID being invalidated.
     * @param now       Current local millisecond timestamp.
     */
    event void prepareParentInvalidated(uint16_t oldParent, uint32_t now);

    /**
     * @brief Requests the data plane to forward any unsent epoch snapshot if
     *        the Root's epoch deadline is still open.
     *
     * @param isNewTreeSeq TRUE if triggered by a `treeSeq` / lower-Root adoption.
     * @param now          Current local millisecond timestamp.
     */
    event void forwardSavedEpoch(bool isNewTreeSeq, uint32_t now);

    /**
     * @brief Signaled when the node wins a slotted Root election and promotes
     *        to Root for `newTreeSeq`.
     *
     * @param newTreeSeq Newly incremented 3-bit `treeSeq`.
     * @param now        Current local millisecond timestamp.
     */
    event void onRootPromoted(uint8_t newTreeSeq, uint32_t now);

    /**
     * @brief Signaled when a newly promoted Root finishes its `ROOT_RECOVERY_DELAY`
     *        settle window and should emit the recovered epoch aggregate.
     */
    event void onRootSettleComplete();

    /**
     * @brief Signaled when a newer `treeSeq` or lower-ID split-brain Root wave
     *        supersedes the current tree state.
     *
     * @param wasPromotedRoot TRUE if this node was in its `rootJustPromoted` settle window.
     * @param prevRootID      Previously tracked Root ID.
     * @param newRootID       Winning Root ID from the superior wave.
     * @param now             Current local millisecond timestamp.
     */
    event void onAdoptSuperiorTree(bool wasPromotedRoot, uint16_t prevRootID,
                                     uint16_t newRootID, uint32_t now);

    /**
     * @brief Signaled when the active parent increases its depth by +1 or +2 hops.
     *
     * @param newDepth Newly updated local depth.
     * @param now      Current local millisecond timestamp.
     */
    event void onParentDepthIncreased(uint8_t newDepth, uint32_t now);

    /**
     * @brief Signaled when a node selects a parent (initial join, better depth,
     *        orphan recovery, or superior Root wave) so the data plane updates
     *        query parameters and reschedules its bottom-up `MyTimer` slot.
     *
     * @param isSuperiorTree TRUE if triggered by adopting a superior tree wave.
     * @param newDepth       Newly selected tree depth.
     * @param qType          Active query type from `RoutingMsg`.
     * @param epochDur       Active epoch duration (seconds) from `RoutingMsg`.
     * @param radioStartTime Radio boot baseline timestamp.
     * @param now            Current local millisecond timestamp.
     */
    event void onParentSelected(bool isSuperiorTree, uint8_t newDepth,
                                  uint8_t qType, uint8_t epochDur,
                                  uint32_t radioStartTime, uint32_t now);
}

