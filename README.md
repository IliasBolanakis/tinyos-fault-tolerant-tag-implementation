# Fault-Tolerant Tiny Aggregation (TAG) for TinyOS / TOSSIM

A fault-tolerant **Tiny Aggregation (TAG)** implementation in nesC for **TinyOS 2.x / TOSSIM** (`micaz sim`), engineered with ultra-compact packet formats (`1–7 bytes`), bottom-up slotted epoch aggregation, MAC-layer ACK monitoring, loop-free local repair, and slotted Root failover election.

Verified across **19 automated TOSSIM test scenarios (`19/19 PASSED`)** under realistic empirical RF noise (`meyer-heavy.txt`), covering all query types, diverse topologies, cascading intermediate hub deaths, consecutive Root failures, and network partitions.

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
| `RoutingMsg` | `22` | **3 B** | `depth` (`[7:5]` `treeSeq`, `[4:0]` `depth/code`), `queryAndEpoch` (`[7:4]` `epoch/10`, `[3:0]` `queryType`), `rootID` |
| `MaxMsg` | `23` | **1 B** | `maxVal` (`[7]` `txPktToggle`, `[6:0]` value) |
| `SumMsg` | `23` | **2 B** | `sum` (`[15]` `txPktToggle`, `[14:0]` partial sum) |
| `AvgMsg` | `23` | **3 B** | `sum` (`[15]` `txPktToggle`, `[14:0]` partial sum), `count` (`1 B`) |
| `VarMsg` | `23` | **7 B** | `sumSq` (`4 B`), `sum` (`2 B`, bit 15 = `txPktToggle`), `count` (`1 B`) |

*(Note: `NotifyParentMsg` remains in [SimpleRoutingTree.h](SimpleRoutingTree.h) solely for `mig` compatibility in [Makefile](Makefile).)*

---

## Repository Structure

| File | Description |
| :--- | :--- |
| [SRTreeAppC.nc](SRTreeAppC.nc) | Top-level nesC configuration wiring `SRTreeC`, `ActiveMessageC`, dual `AMSenderC`/`AMReceiverC` stacks (`AM 22` / `AM 23`), `PacketAcknowledgements`, timers, `RandomC`, and 4 `PacketQueueC(20)` queues. |
| [SRTreeC.nc](SRTreeC.nc) | Core protocol implementation: tree dissemination, query scheduling, duplicate-filtered in-network aggregation, MAC ACK retries, backup failover, two-phase orphan recovery, and slotted Root election. |
| [SimpleRoutingTree.h](SimpleRoutingTree.h) | AM IDs, timing/fault-tolerance constants, bit-packing macros, and minimal `nx_struct` packet definitions. |
| [PacketQueue.nc](PacketQueue.nc) / [PacketQueueC.nc](PacketQueueC.nc) | Generic FIFO `message_t` queue interface and ring-buffer component (`SENDER_QUEUE_SIZE = 20`, `RECEIVER_QUEUE_SIZE = 20`). |
| [Makefile](Makefile) | TinyOS build configuration targeting `SRTreeAppC` (`-DSERIAL_EN`). |
| [create_topology.py](create_topology.py) | Generates a `D x D` grid `topology.txt` for TOSSIM given diameter `D` and radio range `R` (`-50.0 dBm` symmetric links). |
| [mySimulation.py](mySimulation.py) | Standalone 800-second TOSSIM runner with `meyer-heavy.txt` noise and two fault injections (intermediate Node `4` at `t=250s`, Root `0` at `t=450s`). |
| [test_suite.py](test_suite.py) | Automated 19-scenario Python 2.7 verification suite running isolated TOSSIM subprocesses and asserting 7 routing and mathematical invariants. |
| [run_sim.sh](run_sim.sh) / [run_tests.sh](run_tests.sh) | Helper shell scripts for copying files into `/home/tinyos/local/src/tinyos-2x/apps/tinyOS`, compiling `micaz sim`, and running simulation/tests inside the Ubuntu TinyOS VM. |
| [test_results.txt](test_results.txt) | Latest recorded output from running `test_suite.py` (`19/19 PASSED`). |

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

Execute all 19 isolated scenarios:

```bash
python ./test_suite.py
```

Execute a single scenario in worker mode (outputs trace to `test_worker.log`):

```bash
python ./test_suite.py --worker T17_Consecutive_Root_Failures_0_Then_1
```

---

## Automated Verification Suite (19/19 Passing)

[test_suite.py](test_suite.py) enforces **7 formal invariants** on every scenario:

1. **Bounded Random Walk:** All sensor samples remain in `[1..100]` and consecutive epochs stay within `±25%`.
2. **Minimal Payload Enforcement:** Every received data frame matches the exact `nx_struct` size (`1 B`, `2 B`, `3 B`, or `7 B`).
3. **Cycle Freedom & Depth Monotonicity:** No directed cycles ever occur in the parent graph; on settled snapshots, `depth[parent] < depth[child]` holds strictly within each `treeSeq`.
4. **Epoch Completeness & No Split-Brain Duplicates:** Zero missed or duplicate epoch outputs across the 800s simulation window.
5. **Exact Ground-Truth Aggregation:** Root `SUM`, `AVG`, `VARIANCE`, and `MAX` outputs match the ground-truth values across all alive nodes reachable from the active Root.
6. **Failover & Election Correctness:** Hub deaths trigger backup `FAILOVER`/`ORPHAN` recovery; Root deaths increment `treeSeq` (`mod 8`) cleanly and elect the lowest-ID surviving Depth-1 candidate.
7. **Partition Gracefulness:** Following a cut-vertex failure (`T19`), isolated nodes detach cleanly without orphan storms or false cross-cut attachments while the connected component stays `100%` accurate.

### Latest Run Summary ([test_results.txt](test_results.txt))

- **Result:** `19/19 PASSED, 0 FAILED` (Total runtime: `111.00s`)
- **Group 1 — Query Sweep (`T01–T04`):** `SUM`, `AVG`, `VARIANCE`, `MAX` on `3x3` (`R=1.5`)
- **Group 2 — Topologies & Scale (`T05–T10`):** `3x3` orthogonal, `4x4` (`R=1.5` & `R=1.0`), `5x5` (`R=1.5`), 13-node high-fanout star, 6-node linear chain
- **Group 3 — Intermediate Failures (`T11–T15`):** Leaf death, hub death with backup/orphan mix, orthogonal hub death without shallower backups, cascading hub deaths (`Phase 2`), dual hub failures on `4x4`
- **Group 4 — Root Elections & Partitions (`T16–T19`):** Single Root failure (`Node 0`), consecutive Root failures (`Node 0` then `Node 1`), combined hub + Root failure, and bridge cut-vertex network partition

---

## Design Limits (Compact Packet Constraints)

Preserving `1–7 byte` payloads imposes well-defined protocol ceilings in [SimpleRoutingTree.h](SimpleRoutingTree.h):

- **Maximum Tree Depth:** `29` hops (`depth` field uses 5 bits, reserving `30` and `31` for control codes).
- **Network Size & Root IDs:** Up to `255` nodes (`count` and `rootID` are `nx_uint8_t`); 15-bit `sum` (`max 32,767`, because bit 15 encodes `txPktToggle`).
- **3-Bit Sequence Horizon:** `treeSeq` (`0..7`) tolerates up to `3` consecutive missed Root elections before modular comparison wraps.
