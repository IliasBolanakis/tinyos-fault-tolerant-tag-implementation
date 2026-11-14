/**
 * @file SimpleRoutingTree.h
 * @brief Protocol constants, AM channel IDs, bit-packing macros, and minimal
 *        TAG wire-format structures for the fault-tolerant Tiny AGgregation
 *        (TAG) sensor network application.
 *
 * Wire-format design summary:
 * - Control plane (`AM_ROUTINGMSG = 22`): 3-byte `RoutingMsg` packing a 3-bit
 *   `treeSeq`, 5-bit `depth`/control code, 4-bit epoch duration, 4-bit query
 *   type, and 8-bit `rootID`.
 * - Data plane (`AM_MYMSG = 23`): Query-specific minimal payloads (`1–7 B`)
 *   stealing the unused MSB of `sum` (bit 15) or `maxVal` (bit 7) to carry
 *   `txPktToggle`, enabling parent duplicate filters to separate MAC retries
 *   from post-fire child deltas with zero extra packet bytes.
 */
#ifndef SIMPLEROUTINGTREE_H
#define SIMPLEROUTINGTREE_H

/** Sentinel 16-bit node ID indicating an unattached or invalid parent/target. */
#define INVALID_NODE_ID 65535U

enum {
    /* Packet queue capacities (used by PacketQueueC) */
    SENDER_QUEUE_SIZE = 20,
    RECEIVER_QUEUE_SIZE = 20,

    /* Active Message type identifiers */
    AM_SIMPLEROUTINGTREEMSG = 22,
    AM_ROUTINGMSG = 22,
    AM_NOTIFYPARENTMSG = 12,
    AM_MYMSG = 23,

    /* TAG slotted epoch & propagation timing constants (TinyOS binary ms) */
    TIMER_PERIOD_MILLI = 150000,
    TIMER_FAST_PERIOD = 200,
    TAG_SLOT_MILLIS = 200,
    SLOT_JITTER_STEP = 8,
    FIRST_EPOCH_DELAY = 5120,
    HOP_DELAY_MILLIS = 8,

    /* Supported in-network TAG aggregation queries */
    QUERY_SUM = 1,
    QUERY_AVG = 2,
    QUERY_VARIANCE = 3,
    QUERY_MAX = 4,

    /* Fault-Tolerance, Backup Table, Duplicate Filter & Recovery Constants */
    MAX_BACKUP_PARENTS = 4,
    MAX_TRACKED_CHILDREN = 16,
    DUP_RETRY_WINDOW_MILLIS = 500,
    MAX_MAC_RETRIES = 5,
    MAX_PARENT_FAILS = 1,
    ELECTION_BASE_MILLIS = 75,
    ELECTION_STEP_MILLIS = 85,
    ROOT_PROBE_RETRY_MILLIS = 55,
    ROOT_PROBE_REPLY_MILLIS = 50,
    ROOT_RECOVERY_DELAY = 350,
    ORPHAN_DEEPER_DELAY = 60,
    ORPHAN_P2_RETRY_MILLIS = 100,
    ORPHAN_RETRY_PERIOD = 1200,

    /* Two-phase orphan recovery FSM states */
    ORPHAN_NONE = 0,
    ORPHAN_PULL_P1 = 1,
    ORPHAN_WAIT_P1 = 2,
    ORPHAN_PULL_P2 = 3,
    ORPHAN_WAIT_P2 = 4,

    /* 5-bit depth field mask and reserved control codes */
    DEPTH_MASK = 0x1F,
    DEPTH_ROOT_PROBE = 30,
    DEPTH_UNATTACHED = 31
};

/**
 * @brief Intra-slot deterministic transmission stagger across 16 ID buckets
 *        (0..120 ms within a 200 ms TAG level slot) to reduce CSMA collisions.
 */
#define SLOT_JITTER_MILLIS(id) ((uint32_t)((id) % 16) * SLOT_JITTER_STEP)

/**
 * @brief Bit-packing macros for `RoutingMsg.depth`:
 *        bits [7:5] = `treeSeq` (0..7), bits [4:0] = `depth` / control code (0..31).
 */
#define PACK_SEQ_DEPTH(seq, d)   ((uint8_t)((((seq) & 0x07) << 5) | ((d) & DEPTH_MASK)))
#define UNPACK_SEQ(seqDepth)     ((uint8_t)(((seqDepth) >> 5) & 0x07))
#define UNPACK_DEPTH(seqDepth)   ((uint8_t)((seqDepth) & DEPTH_MASK))

/**
 * @brief Bit-packing macros for `RoutingMsg.queryAndEpoch`:
 *        bits [7:4] = (`epochDuration` / 10) in [3..10],
 *        bits [3:0] = `queryType` in [1..4].
 */
#define PACK_QUERY_EPOCH(epochDur, qType) ((uint8_t)((((epochDur) / 10) << 4) | ((qType) & 0x0F)))
#define UNPACK_QUERY(qe)                  ((uint8_t)((qe) & 0x0F))
#define UNPACK_EPOCH_DUR(qe)              ((uint8_t)(((qe) >> 4) * 10))

/**
 * @brief Bit-stealing MSB toggle macros for data-plane duplicate vs. delta
 *        discrimination without increasing payload sizes:
 *        - `SumMsg`, `AvgMsg`, `VarMsg`: bit 15 of `sum` = `txPktToggle`,
 *          bits [14:0] = partial sum (0..32767).
 *        - `MaxMsg`: bit 7 of `maxVal` = `txPktToggle`,
 *          bits [6:0] = partial maximum (0..127).
 */
#define PACK_SUM_TOGGLE(s, toggle)  ((uint16_t)(((uint16_t)(s) & 0x7FFFU) | (((uint16_t)((toggle) & 0x01U)) << 15)))
#define UNPACK_SUM(rawSum)          ((uint16_t)((rawSum) & 0x7FFFU))
#define PACK_MAX_TOGGLE(m, toggle)  ((uint8_t)(((uint8_t)(m) & 0x7FU) | (((uint8_t)((toggle) & 0x01U)) << 7)))
#define UNPACK_MAX(rawMax)          ((uint8_t)((rawMax) & 0x7FU))

/**
 * @brief Seconds to TinyOS binary milliseconds (1024 binary ticks/second).
 */
#define SEC_TO_BMILLI(sec)          ((uint32_t)(sec) * 1024UL)
#define HALF_SEC_TO_BMILLI(sec)     ((uint32_t)(sec) * 512UL)

/**
 * @brief Top-down TAG routing & query dissemination message (3 bytes total).
 * - `depth`: Upper 3 bits = `treeSeq` [0..7],
 *            Lower 5 bits = tree depth [0..29] (`30` = root liveness probe,
 *            `31` = orphan pull / parent detachment).
 * - `queryAndEpoch`: Upper 4 bits = (`epochDuration` / 10) in [3..10],
 *                    Lower 4 bits = `queryType` in [1..4].
 * - `rootID`: Node ID of the active Root for deterministic split-brain
 *             tie-breaking and Root tracking across all tree depths.
 */
typedef nx_struct RoutingMsg {
    nx_uint8_t depth;
    nx_uint8_t queryAndEpoch;
    nx_uint8_t rootID;
} RoutingMsg;

/**
 * @brief Query 1 (SUM) partial aggregate message (2 bytes).
 * - `sum`: Bit 15 carries `txPktToggle` (1-bit retry/delta toggle);
 *          bits [14:0] carry the 15-bit unsigned subtree partial sum (`<= 32767`).
 */
typedef nx_struct SumMsg {
    nx_uint16_t sum;
} SumMsg;

/**
 * @brief Query 2 (AVG) partial aggregate message (3 bytes).
 * - `sum`: Bit 15 carries `txPktToggle` (1-bit retry/delta toggle);
 *          bits [14:0] carry the 15-bit unsigned subtree partial sum (`<= 32767`).
 * - `count`: Number of contributing nodes in the subtree aggregate (`1..255`).
 */
typedef nx_struct AvgMsg {
    nx_uint16_t sum;
    nx_uint8_t count;
} AvgMsg;

/**
 * @brief Query 3 (VARIANCE) partial aggregate message (7 bytes).
 * - `sumSq`: 32-bit sum of squared readings for single-pass variance.
 * - `sum`: Bit 15 carries `txPktToggle` (1-bit retry/delta toggle);
 *          bits [14:0] carry the 15-bit unsigned subtree partial sum (`<= 32767`).
 * - `count`: Number of contributing nodes in the subtree aggregate (`1..255`).
 */
typedef nx_struct VarMsg {
    nx_uint32_t sumSq;
    nx_uint16_t sum;
    nx_uint8_t count;
} VarMsg;

/**
 * @brief Query 4 (MAX) partial aggregate message (1 byte).
 * - `maxVal`: Bit 7 carries `txPktToggle` (1-bit retry/delta toggle);
 *             bits [6:0] carry the 7-bit maximum sensor reading (`1..100`).
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
