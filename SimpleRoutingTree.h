/**
 * @file SimpleRoutingTree.h
 * @brief Protocol constants, AM channel IDs, bit-packing macros, and minimal
 *        TAG wire-format structures for the fault-tolerant Tiny AGgregation
 *        (TAG) sensor network application.
 *
 * Wire-format design summary:
 * - Control plane (`AM_ROUTINGMSG = 22`): 3-byte `RoutingMsg` packing a 3-bit
 *   `treeSeq`, 5-bit `depth`/control code, 4-bit epoch duration, 1-bit
 *   `shedFlag`, 3-bit query type, and 8-bit `rootID`.
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

    /* TAG-slot-aligned radio duty-cycling timing constants (TinyOS binary ms) */
    RADIO_WAKE_LEAD_MILLIS = 550,        /* Wakes > 320ms child slot lead + 2-hop backup margin + radio startup latency before MyTimer.fired() */
    RADIO_POST_EPOCH_MILLIS = 1000,      /* Keeps nodes awake after t_root for end-of-epoch beacon, backup/orphan replies, and Root election waves */
    RADIO_ROOT_POST_BEACON_MILLIS = 1100,/* Keeps Root awake across sendRoutingBroadcast(0) + late probes + high-ID election delays */
    RADIO_MIN_SLEEP_MILLIS = 150,        /* Skip stopping the radio if next wake deadline is within 150ms */
    RADIO_RETRY_DELAY_MILLIS = 10,       /* Short timer retry if SplitControl.start()/stop() returns transient EBUSY */
    MAX_RADIO_START_RETRIES = 5,         /* Bounded retry cap on start()/startDone() failure so hardware failures never spin */
    MAX_RADIO_STOP_RETRIES = 3,          /* Bounded retry cap on stop()/stopDone() failure so hardware failures never deadlock */

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
    ELECTION_BASE_MILLIS = 140,
    ELECTION_STEP_MILLIS = 75,
    ROOT_PROBE_RETRY_MILLIS = 55,
    ROOT_PROBE_REPLY_MILLIS = 50,
    ROOT_RECOVERY_DELAY = 350,
    ORPHAN_DEEPER_DELAY = 60,
    ORPHAN_P2_RETRY_MILLIS = 100,
    ORPHAN_RETRY_PERIOD = 1200,
    MAX_SHED_BEACON_RETRIES = 3,

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
 * @brief Explicit `SplitControl` radio power FSM states owned exclusively by `SRTreeC`.
 */
typedef enum {
    RADIO_OFF = 0,
    RADIO_STARTING = 1,
    RADIO_ON = 2,
    RADIO_STOPPING = 3
} radio_state_t;

/**
 * @brief Operational phases multiplexed onto `SRTreeC`'s `RadioWakeTimer`.
 */
typedef enum {
    DUTY_TIMER_IDLE = 0,
    DUTY_TIMER_LISTEN_WINDOW = 1,
    DUTY_TIMER_WAKE = 2,
    DUTY_TIMER_START_RETRY = 3,
    DUTY_TIMER_STOP_RETRY = 4
} duty_timer_phase_t;

/**
 * @brief Intra-slot deterministic transmission stagger across 16 ID buckets
 *        (0..120 ms within a 200 ms TAG level slot) using a 4-bit bit-reversed
 *        coprime permutation so adjacent/hidden-terminal siblings at the same
 *        tree depth are separated by 16..48 ms rather than colliding 8 ms apart.
 */
#define SLOT_JITTER_BUCKET(id) (((uint32_t)(id) * 3UL) & 0x0FUL)
#define SLOT_JITTER_BITREV(b)  ((((b) & 0x01UL) << 3) | (((b) & 0x02UL) << 1) | (((b) & 0x04UL) >> 1) | (((b) & 0x08UL) >> 3))
#define SLOT_JITTER_MILLIS(id) (SLOT_JITTER_BITREV(SLOT_JITTER_BUCKET(id)) * (uint32_t)SLOT_JITTER_STEP)

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
 *        bit  [3]   = `shedFlag` (`1` during orphan-shed recovery & post-orphan beacon, `0` otherwise),
 *        bits [2:0] = `queryType` in [1..4].
 */
#define PACK_QUERY_EPOCH(epochDur, qType)            ((uint8_t)((((epochDur) / 10) << 4) | ((qType) & 0x07)))
#define PACK_QUERY_EPOCH_SHED(epochDur, qType, shed) ((uint8_t)((((epochDur) / 10) << 4) | (((uint8_t)(shed) & 0x01) << 3) | ((qType) & 0x07)))
#define UNPACK_QUERY(qe)                             ((uint8_t)((qe) & 0x07))
#define UNPACK_SHED_TOGGLE(qe)                       ((uint8_t)(((qe) >> 3) & 0x01))
#define UNPACK_EPOCH_DUR(qe)                         ((uint8_t)(((qe) >> 4) * 10))

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
 * - `queryAndEpoch`: Bits [7:4] = (`epochDuration` / 10) in [3..10],
 *                    Bit  [3]   = `shedFlag` (`1` during orphan-shed recovery
 *                                 and post-orphan attached beacon, `0` otherwise),
 *                    Bits [2:0] = `queryType` in [1..4].
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
