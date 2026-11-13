/**
 * @file SimpleRoutingTree.h
 * @brief Header defining AM types, timing constants, fault-tolerance parameters,
 *        and minimal TAG nx_structs.
 */
#ifndef SIMPLEROUTINGTREE_H
#define SIMPLEROUTINGTREE_H

enum {
    SENDER_QUEUE_SIZE = 20,
    RECEIVER_QUEUE_SIZE = 20,
    AM_SIMPLEROUTINGTREEMSG = 22,
    AM_ROUTINGMSG = 22,
    AM_NOTIFYPARENTMSG = 12,
    AM_MYMSG = 23,
    TIMER_PERIOD_MILLI = 150000,
    TIMER_FAST_PERIOD = 200,
    TAG_SLOT_MILLIS = 200,
    SLOT_JITTER_STEP = 8,
    FIRST_EPOCH_DELAY = 5120,
    HOP_DELAY_MILLIS = 8,
    QUERY_SUM = 1,
    QUERY_AVG = 2,
    QUERY_VARIANCE = 3,
    QUERY_MAX = 4,

    /* Fault-Tolerance & Fail-Safety Constants */
    MAX_BACKUP_PARENTS = 4,
    MAX_MAC_RETRIES = 5,
    MAX_PARENT_FAILS = 1,
    ELECTION_BASE_MILLIS = 75,
    ELECTION_STEP_MILLIS = 85,
    ROOT_RECOVERY_DELAY = 350,
    ORPHAN_DEEPER_DELAY = 60,
    ORPHAN_P2_RETRY_MILLIS = 100,
    ORPHAN_RETRY_PERIOD = 1200,
    ORPHAN_NONE = 0,
    ORPHAN_PULL_P1 = 1,
    ORPHAN_WAIT_P1 = 2,
    ORPHAN_PULL_P2 = 3,
    ORPHAN_WAIT_P2 = 4,
    DEPTH_MASK = 0x1F,
    DEPTH_ROOT_PROBE = 30,
    DEPTH_UNATTACHED = 31
};

/* Intra-slot deterministic transmission stagger across 16 ID buckets */
#define SLOT_JITTER_MILLIS(id) ((uint32_t)((id) % 16) * SLOT_JITTER_STEP)

/* Bit-packing macros for RoutingMsg.depth: [7:5] = treeSeq (0..7), [4:0] = depth (0..31) */
#define PACK_SEQ_DEPTH(seq, d)   ((uint8_t)((((seq) & 0x07) << 5) | ((d) & DEPTH_MASK)))
#define UNPACK_SEQ(seqDepth)     ((uint8_t)(((seqDepth) >> 5) & 0x07))
#define UNPACK_DEPTH(seqDepth)   ((uint8_t)((seqDepth) & DEPTH_MASK))

/**
 * @brief Top-down TAG routing & query dissemination message (3 bytes total).
 * - depth: Upper 3 bits = treeSeq [0..7],
 *          Lower 5 bits = tree depth [0..29] (30 = root liveness probe, 31 = orphan pull).
 * - queryAndEpoch: Upper 4 bits = (epochDuration / 10) in [3..10],
 *                  Lower 4 bits = queryType in [1..4].
 * - rootID: Node ID of the active Root for deterministic split-brain tie-breaking
 *           and accurate Root tracking across all tree depths.
 */
typedef nx_struct RoutingMsg {
    nx_uint8_t depth;
    nx_uint8_t queryAndEpoch;
    nx_uint8_t rootID;
} RoutingMsg;

/**
 * @brief Query 1 (SUM) partial aggregate message (2 bytes).
 */
typedef nx_struct SumMsg {
    nx_uint16_t sum;
} SumMsg;

/**
 * @brief Query 2 (AVG) partial aggregate message (3 bytes).
 */
typedef nx_struct AvgMsg {
    nx_uint16_t sum;
    nx_uint8_t count;
} AvgMsg;

/**
 * @brief Query 3 (VARIANCE) partial aggregate message (7 bytes).
 */
typedef nx_struct VarMsg {
    nx_uint32_t sumSq;
    nx_uint16_t sum;
    nx_uint8_t count;
} VarMsg;

/**
 * @brief Query 4 (MAX) partial aggregate message (1 byte).
 */
typedef nx_struct MaxMsg {
    nx_uint8_t maxVal;
} MaxMsg;

/**
 * @brief Kept solely for backward compatibility with the mig rule in Makefile.
 */
typedef nx_struct NotifyParentMsg {
    nx_uint16_t senderID;
    nx_uint16_t parentID;
    nx_uint8_t depth;
} NotifyParentMsg;

#endif