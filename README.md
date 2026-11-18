# Fault-Tolerant Tiny Aggregation (TAG) for TinyOS / TOSSIM

A fault-tolerant **Tiny Aggregation (TAG)** implementation in nesC for **TinyOS 2.x / TOSSIM** (`micaz sim`), engineered with ultra-compact packet formats (`1–7 bytes`), bottom-up slotted epoch aggregation, TAG-slot-aligned radio duty-cycling, MAC-layer ACK monitoring, loop-free local repair, and slotted Root failover election.

Verified across **20 automated TOSSIM test scenarios** and **8 formal invariants** under realistic empirical RF noise (`meyer-heavy.txt`), covering all query types, diverse topologies, cascading intermediate hub deaths, consecutive Root failures, network partitions, and multi-epoch radio duty-cycling lifecycles.

---

## Features

- **Four In-Network TAG Queries** (selected randomly at Root boot from `{SUM, AVG, VARIANCE, MAX}` with random epoch period `30–100s` in `10s` steps):
  - `QUERY_SUM` (`2 B`), `QUERY_AVG` (`3 B`), `QUERY_VARIANCE` (`7 B` single-pass $\sum x^2, \sum x, n$), `QUERY_MAX` (`1 B`)
- **Zero-Overhead Packet Optimization:**
  - `RoutingMsg` packs a 3-bit `treeSeq` `[0..7]` and 5-bit `depth` (`0..29`, `30 = DEPTH_ROOT_PROBE`, `31 = DEPTH_UNATTACHED`) into a single byte, plus `queryAndEpoch` (`1 B`) and `rootID` (`1 B`).
  - `txPktToggle` (1-bit toggle) is packed into the unused MSB of `sum` (bit 15) or `maxVal` (bit 7) so parents distinguish MAC retries from post-fire child deltas without inflating packet sizes.
- **Bounded Random-Walk Sensor Model:** Each node generates readings in `[1..100]` bounded within `±25%` of its previous epoch value.
- **Slotted Bottom-Up Scheduling:** Nodes fire `depth * TAG_SLOT_MILLIS + SLOT_JITTER_MILLIS(TOS_NODE_ID)` ahead of the Root (`200 ms` per tree level, `16` deterministic jitter buckets spaced by `8 ms`) to serialize children ahead of parents and minimize CSMA collisions.
- **Multi-Tier Self-Healing:**
  1. **Synchronous MAC ACK Retries:** Up to `MAX_MAC_RETRIES = 5` fast retries per hop on missed link-layer ACKs.
  2. **Backup Parent Failover:** Maintains up to `MAX_BACKUP_PARENTS = 4` passively discovered shallower/sibling candidates for immediate rerouting without dropping the active epoch's aggregate.
  3. **Two-Phase Cycle-Free Orphan Recovery:** Broadcasts `DEPTH_UNATTACHED` (`31`) and enforces a depth-bounded settle window (`ORPHAN_WAIT_P1`) before accepting deeper neighbors (`ORPHAN_PULL_P2` → `ORPHAN_WAIT_P2`), preventing routing loops when subtrees detach simultaneously.
  4. **Slotted Root Failover Election & Liveness Probe:** Depth-1 nodes probe suspected Root failure with `DEPTH_ROOT_PROBE` (`30`) before promoting on an ID-slotted timer (`ELECTION_BASE_MILLIS + ID * ELECTION_STEP_MILLIS`), breaking any multi-hop split-brain deterministically via modular `treeSeq` and lowest `rootID`.

---

## Minimal Packet Formats

Defined in [SimpleRoutingTree.h](SimpleRoutingTree.h):

| Struct | AM Type | Size | Fields |
| :--- | :---: | :---: | :--- |
| `RoutingMsg` | `22` | **3 B** | `depth` (`[7:5]` `treeSeq`, `[4:0]` `depth/code`), `queryAndEpoch` (`[7:4]` `epoch/10`, `[3]` `shedFlag`, `[2:0]` `queryType`), `rootID` |
| `MaxMsg` | `23` | **1 B** | `maxVal` (`[7]` `txPktToggle`, `[6:0]` value) |
| `SumMsg` | `23` | **2 B** | `sum` (`[15]` `txPktToggle`, `[14:0]` partial sum) |
| `AvgMsg` | `23` | **3 B** | `sum` (`[15]` `txPktToggle`, `[14:0]` partial sum), `count` (`1 B`) |
| `VarMsg` | `23` | **7 B** | `sumSq` (`4 B`), `sum` (`2 B`, bit 15 = `txPktToggle`), `count` (`1 B`) |

*(Note: `NotifyParentMsg` remains in [SimpleRoutingTree.h](SimpleRoutingTree.h) solely for `mig` compatibility in [Makefile](Makefile).)*

---

## Modular TEP-3 Architecture

The nesC application separates control-plane tree management, data-plane TAG aggregation, backup candidate tracking, and duplicate filtering into cohesive TinyOS TEP-3 components:

- **Control Plane ([TreeTopology.nc](TreeTopology.nc) / [TreeRoutingP.nc](TreeRoutingP.nc)):** Owns `AM_ROUTINGMSG` (`22`) tree dissemination, `DEPTH_ROOT_PROBE` (`30`) liveness probing, slotted Root election with split-brain tie-breaking, and two-phase cycle-free orphan recovery (`DEPTH_UNATTACHED = 31`) across three dedicated `TimerMilliC` instances (`DisseminationTimerC`, `OrphanTimerC`, `ElectionTimerC`). Coordinates radio availability with `SRTreeC` via `TreeTopology` hooks (`hasPendingRoutingWork()`, `isRadioOn()`, `requestRadioWake()`, `checkRadioSleep()`, `handleRadioEoff()`) without wiring `SplitControl` into the routing module.
- **Data Plane & Radio Lifecycle ([SRTreeC.nc](SRTreeC.nc)):** Exclusively owns `SplitControl` (`RadioControl`) and `RadioWakeTimerC`, driving the 4-state `radio_state_t` FSM and TAG-slot-aligned duty cycling alongside `AM_MYMSG` (`23`) epoch execution (`MyTimerC`), `±25%` random-walk sensor sampling, duplicate-filtered subtree folding, and synchronous MAC ACK retries/failover snapshots.
- **Supporting Modules:** [BackupTableP.nc](BackupTableP.nc) (`@safe()`) maintains strictly shallower backup parents (`nDepth < curDepth`) with lowest-ID tie-breaking; [DupFilterP.nc](DupFilterP.nc) (`@safe()`) suppresses link-layer MAC retries within `DUP_RETRY_WINDOW_MILLIS` while allowing post-fire delta updates via the MSB `txPktToggle`.

---

## TAG-Slot-Aligned Radio Duty-Cycling

`SRTreeC` powers the radio down between TAG epochs while guaranteeing all nodes stay awake across initial tree discovery (`epochCounter == 0`), parent/backup child transmission slots, Root beacons, orphan recovery, and Root failover elections.

### 4-State `radio_state_t` FSM (`SRTreeC`)

- **`RADIO_OFF` (`0`):** Radio powered down; `RadioWakeTimer` is armed in `DUTY_TIMER_WAKE` to fire `RADIO_WAKE_LEAD_MILLIS = 550` ms before the node's next scheduled `MyTimer.fired()`.
- **`RADIO_STARTING` (`1`):** `RadioControl.start()` accepted (`SUCCESS`); awaiting `startDone(SUCCESS)`.
- **`RADIO_ON` (`2`):** Radio active; transmissions allowed only in this state (`sendMyTask` / `sendRoutingTask` gate on `RADIO_ON` to avoid `EOFF`).
- **`RADIO_STOPPING` (`3`):** `RadioControl.stop()` accepted (`SUCCESS`); awaiting `stopDone()`. Any wake request (`requestRadioWake()`, `openEpochListenWindow()`) sets `pendingRadioStart = TRUE` so `stopDone()` immediately re-starts the radio instead of sleeping.

### Multiplexed `RadioWakeTimerC` (`duty_timer_phase_t`)

- `DUTY_TIMER_IDLE = 0` — No duty timer armed.
- `DUTY_TIMER_LISTEN_WINDOW = 1` — Post-epoch / recovery listen window open (`listenWindowClosed = FALSE`; closes at `listenWindowEnd` after `RADIO_POST_EPOCH_MILLIS = 1000` ms or `RADIO_ROOT_POST_BEACON_MILLIS = 1100` ms at the Root).
- `DUTY_TIMER_WAKE = 2` — Sleeping until `nextFire - RADIO_WAKE_LEAD_MILLIS` (skipped via `RADIO_MIN_SLEEP_MILLIS = 150` if next wake is imminent).
- `DUTY_TIMER_START_RETRY = 3` — Bounded backoff (`RADIO_RETRY_DELAY_MILLIS = 10` ms, up to `MAX_RADIO_START_RETRIES = 5`) on transient `RadioControl.start()` / `startDone()` failures.
- `DUTY_TIMER_STOP_RETRY = 4` — Bounded backoff (`RADIO_RETRY_DELAY_MILLIS = 10` ms, up to `MAX_RADIO_STOP_RETRIES = 3`) on transient `RadioControl.stop()` / `stopDone()` failures.

### TAG-Slot-Aligned Duty-Cycle Timeline (Per Epoch $E \ge 1$)

```text
  t_d - 550ms         t_d = t_root - d*200 - jitter              t_root            t_root + 1000ms       t_root + 1100ms
  |                   |                                          |                 |                     |
  v                   v                                          v                 v                     v
  [RadioWakeTimer]    [MyTimer.fired()]                          [Root MyTimer]    [Child Stop]          [Root Stop]
  DUTY_TIMER_WAKE     - Sample & send AM_MYMSG to parent         - Log Root result - Window closes       - Window closes
  -> Radio ON         - Open listen window:                      - Broadcast       - evaluateRadioSleep  - evaluateRadioSleep
  (receives children    d*200 + jitter + RADIO_POST_EPOCH(1000)    depth=0 beacon  -> SplitControl.stop   -> SplitControl.stop
   & 2-hop backups)   - Window closes at t_root + 1000ms         - Window = 1100ms -> scheduleNextWake   -> scheduleNextWake
```

Sleep (`evaluateRadioSleep()`) executes **only** when `radioState == RADIO_ON`, `!pendingRadioStart`, `epochCounter >= 1`, `listenWindowClosed == TRUE`, `dutyTimerPhase == DUTY_TIMER_IDLE`, all data queues/flags (`MySendBusy`, `MySendQueue`, `MyReceiveQueue`, `hasUnsentEpochData`, `hasPendingChildAgg`) are clear, and `TreeTopology.hasPendingRoutingWork()` returns `FALSE`.

---

## Repository Structure

| File | Description |
| :--- | :--- |
| [SRTreeAppC.nc](SRTreeAppC.nc) | Top-level nesC configuration wiring `SRTreeC`, `TreeRoutingP`, `BackupTableP`, `DupFilterP`, `ActiveMessageC`, dual `AMSenderC`/`AMReceiverC` stacks (`AM 22` / `AM 23`), `PacketAcknowledgements`, `RandomC`, 4 `PacketQueueC(20)` queues, and 5 demultiplexed `TimerMilliC` instances (`DisseminationTimerC`, `OrphanTimerC`, `ElectionTimerC`, `MyTimerC`, `RadioWakeTimerC`). |
| [SRTreeC.nc](SRTreeC.nc) | TAG data-plane aggregation & radio power manager (`AM_MYMSG = 23`), handling bottom-up `MyTimer` epoch slotting, bounded `±25%` random-walk sensor sampling, duplicate-filtered child folding, MAC ACK retry/failover snapshots, and the `SplitControl` 4-state `radio_state_t` duty-cycling FSM (`RadioWakeTimer`). |
| [TreeTopology.nc](TreeTopology.nc) / [TreeRoutingP.nc](TreeRoutingP.nc) | Control-plane routing interface and private module (`AM_ROUTINGMSG = 22`), handling tree dissemination, two-phase cycle-free orphan recovery (`DEPTH_UNATTACHED = 31`), `DEPTH_ROOT_PROBE` (`30`) liveness probing, slotted Root election, split-brain tie-breaking, and radio coordination hooks (`hasPendingRoutingWork`, `isRadioOn`, `requestRadioWake`, `checkRadioSleep`, `handleRadioEoff`). |
| [BackupTable.nc](BackupTable.nc) / [BackupTableP.nc](BackupTableP.nc) | Passive backup parent table interface and `@safe()` module (`MAX_BACKUP_PARENTS = 4`), enforcing strictly shallower failover candidate selection (`nDepth < curDepth`) with lowest-ID tie-breaking. |
| [DupFilter.nc](DupFilter.nc) / [DupFilterP.nc](DupFilterP.nc) | Per-epoch duplicate MAC retry filter interface and `@safe()` module (`MAX_TRACKED_CHILDREN = 16`, `DUP_RETRY_WINDOW_MILLIS = 500`), distinguishing link-layer retransmissions from post-fire child deltas via the MSB `txPktToggle`. |
| [SimpleRoutingTree.h](SimpleRoutingTree.h) | Protocol constants, duty-cycling constants & enums (`radio_state_t`, `duty_timer_phase_t`), standardized bit-packing/unpacking and time-conversion macros (`INVALID_NODE_ID`, `PACK_SEQ_DEPTH` / `UNPACK_SEQ` / `UNPACK_DEPTH`, `PACK_QUERY_EPOCH` / `PACK_QUERY_EPOCH_SHED` / `UNPACK_QUERY` / `UNPACK_SHED_TOGGLE` (`shedFlag`) / `UNPACK_EPOCH_DUR`, `PACK_SUM_TOGGLE` / `UNPACK_SUM`, `PACK_MAX_TOGGLE` / `UNPACK_MAX`, `SEC_TO_BMILLI`, `HALF_SEC_TO_BMILLI`), and minimal `nx_struct` packet definitions. |
| [PacketQueue.nc](PacketQueue.nc) / [PacketQueueC.nc](PacketQueueC.nc) | Generic FIFO `message_t` queue interface and ring-buffer component (`SENDER_QUEUE_SIZE = 20`, `RECEIVER_QUEUE_SIZE = 20`). |
| [Makefile](Makefile) | TinyOS build configuration targeting `SRTreeAppC` (`-DSERIAL_EN`). |
| [create_topology.py](create_topology.py) | Generates a `D x D` grid `topology.txt` for TOSSIM given diameter `D` and radio range `R` (`-50.0 dBm` symmetric links). |
| [mySimulation.py](mySimulation.py) | Standalone 800-second TOSSIM runner with `meyer-heavy.txt` noise and two fault injections (intermediate Node `4` at `t=250s`, Root `0` at `t=450s`). |
| [test_suite.py](test_suite.py) | Automated 20-scenario Python 2.7 verification suite running isolated TOSSIM subprocesses and asserting 8 routing, radio-lifecycle, and mathematical invariants. |
| [run_sim.sh](run_sim.sh) / [run_tests.sh](run_tests.sh) | Helper shell scripts for copying all `.nc`, `.h`, `.py`, and test files into `/home/tinyos/local/src/tinyos-2x/apps/tinyOS`, compiling `micaz sim`, and running simulation/tests inside the Ubuntu TinyOS VM. |
| [test_results.txt](test_results.txt) | Recorded pre-fix 20-scenario TOSSIM baseline output (`19/20 PASSED`, capturing `T15` Epoch 4 prior to the `shedFlag` and `txInFlightInvalidated` fixes). |

---

## Getting Started (TinyOS 2.x / TOSSIM)

### Prerequisites

- TinyOS 2.x toolchain (`nescc`, `mig`, `make micaz sim`) with `TINYOS_ROOT_DIR` configured
- Python 2.7 with TOSSIM Python bindings (`_TOSSIMmodule.so`)
- Empirical noise trace at `$TINYOS_ROOT_DIR/tos/lib/tossim/noise/meyer-heavy.txt`

### Build for TOSSIM

```bash
make micaz sim
```

### Run a Single Simulation

Generate a `3x3` diagonal grid (`R = 1.5`) and execute the 800s fault-injection simulation:

```bash
python ./create_topology.py 3 1.5
python ./mySimulation.py 1 | tee output.txt
```

Or use [run_sim.sh](run_sim.sh) / [run_tests.sh](run_tests.sh) when deploying from the shared VM directory:

```bash
bash run_tests.sh
```

### Run the Automated Verification Suite

Execute all 20 isolated scenarios:

```bash
python ./test_suite.py
```

Execute a single scenario in worker mode (outputs trace to `test_worker.log`):

```bash
python ./test_suite.py --worker T20_DutyCycle_Lifecycle_30s_Epochs
```

---

## Automated Verification Suite (20 Scenarios, 8 Invariants)

[test_suite.py](test_suite.py) enforces **8 formal invariants** on every scenario:

1. **Bounded Random Walk:** All sensor samples remain in `[1..100]` and consecutive epochs stay within `±25%`.
2. **Minimal Payload Enforcement:** Every received data frame matches the exact `nx_struct` size (`1 B`, `2 B`, `3 B`, or `7 B`).
3. **Cycle Freedom & Depth Monotonicity:** No directed cycles ever occur in the parent graph; on settled snapshots, `depth[parent] < depth[child]` holds strictly within each `treeSeq`.
4. **Epoch Completeness & No Split-Brain Duplicates:** Zero missed or duplicate epoch outputs across the 800s simulation window.
5. **Exact Ground-Truth Aggregation:** Root `SUM`, `AVG`, `VARIANCE`, and `MAX` outputs match the ground-truth values across all alive nodes reachable from the active Root.
6. **Failover & Election Correctness:** Hub deaths trigger backup `FAILOVER`/`ORPHAN` recovery; Root deaths increment `treeSeq` (`mod 8`) cleanly and elect the lowest-ID surviving Depth-1 candidate.
7. **Partition Gracefulness:** Following a cut-vertex failure (`T19`), isolated nodes detach cleanly without orphan storms or false cross-cut attachments while the connected component stays `100%` accurate.
8. **Radio Duty-Cycling & SplitControl Lifecycle:** Zero `EOFF` transmit warnings; strict per-node start/stop parity (`0 <= starts - stops <= 1`, alternating `START`/`STOP` events); every connected node (and faulted/partitioned node completing `>= 2` epochs prior to fault) actively power-cycles (`stops >= len(epoch_results) - 1` in steady state; `>= 20` cycles in `T20`).

### Scenario Coverage (`T01–T20`)

- **Group 1 — Query Sweep (`T01–T04`):** `SUM`, `AVG`, `VARIANCE`, `MAX` on `3x3` (`R=1.5`)
- **Group 2 — Topologies & Scale (`T05–T10`):** `3x3` orthogonal, `4x4` (`R=1.5` & `R=1.0`), `5x5` (`R=1.5`), 13-node high-fanout star, 6-node linear chain
- **Group 3 — Intermediate Failures (`T11–T15`):** Leaf death, hub death with backup/orphan mix, orthogonal hub death without shallower backups, cascading hub deaths (`Phase 2`), dual hub failures on `4x4`
- **Group 4 — Root Elections & Partitions (`T16–T19`):** Single Root failure (`Node 0`), consecutive Root failures (`Node 0` then `Node 1`), combined hub + Root failure, and bridge cut-vertex network partition
- **Group 5 — Radio Duty-Cycling (`T20`):** `T20_DutyCycle_Lifecycle_30s_Epochs` on `3x3` (`R=1.5`) with 30s epochs (`~27` epochs, asserting `>= 20` radio start/stop cycles per node)

---

## Design Limits (Compact Packet Constraints)

Preserving `1–7 byte` payloads imposes well-defined protocol ceilings in [SimpleRoutingTree.h](SimpleRoutingTree.h):

- **Maximum Tree Depth:** `29` hops (`depth` field uses 5 bits, reserving `30` and `31` for control codes).
- **Network Size & Root IDs:** Up to `255` nodes (`count` and `rootID` are `nx_uint8_t`); 15-bit `sum` (`max 32,767`, because bit 15 encodes `txPktToggle`).
- **3-Bit Sequence Horizon:** `treeSeq` (`0..7`) tolerates up to `3` consecutive missed Root elections before modular comparison wraps.
