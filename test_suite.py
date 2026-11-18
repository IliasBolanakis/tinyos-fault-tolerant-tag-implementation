#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Exhaustive Automated Simulation and Verification Suite for TinyOS/TOSSIM TAG.

This script operates in two modes:
  1. Orchestrator mode (`python test_suite.py`):
     Iterates through all scenarios in SCENARIOS, spawning each scenario in an
     isolated child process (`--worker <scenario_id>`) so Linux/TOSSIM reclaims
     all C++ noise-model heap memory between runs. Summarizes results to stdout
     and writes `test_results.txt`, then restores the default 3x3 topology.
  2. Worker mode (`python test_suite.py --worker <scenario_id>`):
     Generates the scenario's topology (`topology.txt`), boots a TOSSIM instance
     with `meyer-heavy.txt` empirical noise, runs to 800 simulated seconds with
     scheduled fault injections, writes debug output to `test_worker.log`, and
     executes 8 automated protocol, radio-lifecycle, and mathematical invariant checks.

Strictly Python 2.7 compatible using built-in standard libraries only.
"""

import math
import os
import re
import shutil
import subprocess
import sys
import time


DEFAULT_TIMEOUT_SEC = 120
DEFAULT_SIM_SECONDS = 800
DEFAULT_TOPO_FILE = "topology.txt"
TEST_TOPO_FILE = "test_topo.txt"
WORKER_LOG_FILE = "test_worker.log"
RESULTS_FILE = "test_results.txt"

QUERY_ID_TO_NAME = {
    1: "SUM",
    2: "AVG",
    3: "VARIANCE",
    4: "MAX",
}

QUERY_NAME_TO_ID = {
    "SUM": 1,
    "AVG": 2,
    "VARIANCE": 3,
    "MAX": 4,
}

EXPECTED_PAYLOAD_MAP = {
    "SUM": ("SumMsg", 2),
    "AVG": ("AvgMsg", 3),
    "VARIANCE": ("VarMsg", 7),
    "MAX": ("MaxMsg", 1),
}


# ---------------------------------------------------------------------------
# TinyOS RandomMlcgP.nc Deterministic Seed Predictor
# ---------------------------------------------------------------------------

def _mlcg_step(seed):
    """Execute one step of TinyOS RandomMlcgP.nc 32-bit Park-Miller MLCG."""
    tmpseed = 33614 * (seed & 0xFFFFFFFF)
    q = (tmpseed & 0xFFFFFFFF) >> 1
    p = tmpseed >> 32
    mlcg = p + q
    if mlcg & 0x80000000:
        mlcg = (mlcg & 0x7FFFFFFF) + 1
    return mlcg & 0xFFFFFFFF


def predict_root_config(seed_offset):
    """
    Predict (query_name, query_id, epoch_duration) selected by Root Node 0
    in SRTreeC.nc for a given boot delay `seed_offset`.

    SRTreeC.nc boots Node 0 at `10 * ticksPerSecond() + seed_offset`, sets
    `seed = (sim_time() % 1024) + 1`, initializes TinyOS RandomMlcgP (`param + 1`),
    performs two warm-up `Random.rand16()` calls, and then draws `queryType`
    and `epochDuration`.
    """
    param = ((seed_offset % 1024) + 1) & 0xFFFF
    seed = (param + 1) & 0xFFFFFFFF
    seed = _mlcg_step(seed)  # warm-up rand16 #1
    seed = _mlcg_step(seed)  # warm-up rand16 #2
    seed = _mlcg_step(seed)
    q_id = ((seed & 0xFFFF) % 4) + 1
    seed = _mlcg_step(seed)
    e_dur = (((seed & 0xFFFF) % 8) + 3) * 10
    return (QUERY_ID_TO_NAME[q_id], q_id, e_dur)


def find_seed_for_query(target_query, preferred_epochs=(100, 90, 80, 70, 60, 50, 40, 30)):
    """Find the lowest non-negative seed_offset [0..1023] yielding `target_query`."""
    for target_dur in preferred_epochs:
        for offset in range(0, 1024):
            q_name, _, e_dur = predict_root_config(offset)
            if q_name == target_query and e_dur == target_dur:
                return offset
    for offset in range(0, 1024):
        q_name, _, _ = predict_root_config(offset)
        if q_name == target_query:
            return offset
    return 1


# ---------------------------------------------------------------------------
# Topology Generators & Adjacency Helpers
# ---------------------------------------------------------------------------

def generate_grid_topology(D, R, filepath=DEFAULT_TOPO_FILE):
    """Write a D x D Euclidean grid topology with radio range R and -50.0 dBm links."""
    adj = {}
    num_nodes = D * D
    for i in range(num_nodes):
        adj[i] = set()
    f = open(filepath, "w")
    try:
        for i in range(num_nodes):
            for j in range(num_nodes):
                if i != j and math.hypot((i // D) - (j // D), (i % D) - (j % D)) <= R:
                    f.write("%d %d -50.0\n" % (i, j))
                    adj[i].add(j)
    finally:
        f.close()
    return adj


def generate_star_topology(num_nodes, filepath=DEFAULT_TOPO_FILE):
    """Write a star topology with Node 0 connected bidirectionally to Nodes 1..N-1."""
    adj = {}
    for i in range(num_nodes):
        adj[i] = set()
    f = open(filepath, "w")
    try:
        for i in range(1, num_nodes):
            f.write("0 %d -50.0\n" % i)
            f.write("%d 0 -50.0\n" % i)
            adj[0].add(i)
            adj[i].add(0)
    finally:
        f.close()
    return adj


def generate_chain_topology(num_nodes, filepath=DEFAULT_TOPO_FILE):
    """Write a linear chain topology 0 - 1 - 2 - ... - (N-1)."""
    adj = {}
    for i in range(num_nodes):
        adj[i] = set()
    f = open(filepath, "w")
    try:
        for i in range(num_nodes - 1):
            f.write("%d %d -50.0\n" % (i, i + 1))
            f.write("%d %d -50.0\n" % (i + 1, i))
            adj[i].add(i + 1)
            adj[i + 1].add(i)
    finally:
        f.close()
    return adj


def generate_bridge_topology(filepath=DEFAULT_TOPO_FILE):
    """
    Write a 6-node dumbbell/bridge topology:
      - Root clique: {0, 1, 2} fully connected
      - Cut-vertex bridge Node 3 connected to {1, 2} and {4, 5}
      - Remote clique: {4, 5} connected to each other
    Killing Node 3 partitions {4, 5} cleanly from {0, 1, 2}.
    """
    edges = [
        (0, 1), (0, 2), (1, 2),
        (1, 3), (2, 3),
        (3, 4), (3, 5),
        (4, 5),
    ]
    adj = {}
    for i in range(6):
        adj[i] = set()
    f = open(filepath, "w")
    try:
        for u, v in edges:
            f.write("%d %d -50.0\n" % (u, v))
            f.write("%d %d -50.0\n" % (v, u))
            adj[u].add(v)
            adj[v].add(u)
    finally:
        f.close()
    return adj


def connected_component(adj, root_id, dead_nodes):
    """Return the set of surviving nodes reachable from `root_id` via alive links."""
    if root_id in dead_nodes or root_id not in adj:
        return set()
    visited = set([root_id])
    queue = [root_id]
    while queue:
        cur = queue.pop(0)
        for nxt in adj.get(cur, ()):
            if nxt not in dead_nodes and nxt not in visited:
                visited.add(nxt)
                queue.append(nxt)
    return visited


# ---------------------------------------------------------------------------
# Exhaustive Scenario Definitions (20 Scenarios Across 5 Groups)
# ---------------------------------------------------------------------------

def build_scenarios():
    """Return the ordered list of 20 automated test scenario dictionaries."""
    seed_sum = find_seed_for_query("SUM", (100, 90, 80, 70, 60))
    seed_avg = find_seed_for_query("AVG", (100, 90, 80, 70, 60))
    seed_var = find_seed_for_query("VARIANCE", (100, 90, 80, 70, 60))
    seed_max = find_seed_for_query("MAX", (100, 90, 80, 70, 60))
    seed_avg_30 = find_seed_for_query("AVG", (30,))

    return [
        # --- Group 1: Query Type & Epoch Duration Sweep ---
        {
            "id": "T01_Query_SUM_3x3_R1.5",
            "group": "Group 1: Query Sweep",
            "topo": ("grid", 3, 1.5),
            "seed": seed_sum,
            "expected_query": "SUM",
            "faults": [],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T02_Query_AVG_3x3_R1.5",
            "group": "Group 1: Query Sweep",
            "topo": ("grid", 3, 1.5),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T03_Query_VARIANCE_3x3_R1.5",
            "group": "Group 1: Query Sweep",
            "topo": ("grid", 3, 1.5),
            "seed": seed_var,
            "expected_query": "VARIANCE",
            "faults": [],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T04_Query_MAX_3x3_R1.5",
            "group": "Group 1: Query Sweep",
            "topo": ("grid", 3, 1.5),
            "seed": seed_max,
            "expected_query": "MAX",
            "faults": [],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
        },

        # --- Group 2: Diverse Grid Sizes & Radio Ranges ---
        {
            "id": "T05_Grid_3x3_Orthogonal_R1.0",
            "group": "Group 2: Topologies & Scale",
            "topo": ("grid", 3, 1.0),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T06_Grid_4x4_Diagonal_R1.5",
            "group": "Group 2: Topologies & Scale",
            "topo": ("grid", 4, 1.5),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T07_Grid_4x4_Orthogonal_R1.0",
            "group": "Group 2: Topologies & Scale",
            "topo": ("grid", 4, 1.0),
            "seed": seed_sum,
            "expected_query": "SUM",
            "faults": [],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T08_Grid_5x5_Diagonal_R1.5",
            "group": "Group 2: Topologies & Scale",
            "topo": ("grid", 5, 1.5),
            "seed": seed_var,
            "expected_query": "VARIANCE",
            "faults": [],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T09_Star_HighFanout_13Nodes",
            "group": "Group 2: Topologies & Scale",
            "topo": ("star", 13),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T10_Linear_Chain_6Nodes",
            "group": "Group 2: Topologies & Scale",
            "topo": ("chain", 6),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
        },

        # --- Group 3: Intermediate Node Failures & Orphan Edge Cases ---
        {
            "id": "T11_Leaf_Node_Death_3x3",
            "group": "Group 3: Intermediate Failures",
            "topo": ("grid", 3, 1.5),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [(250, 8)],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T12_Hub_Death_Backup_And_Orphan_3x3",
            "group": "Group 3: Intermediate Failures",
            "topo": ("grid", 3, 1.5),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [(250, 4)],
            "expect_hub_failover": True,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T13_Orthogonal_Hub_Death_No_Shallower_Backups",
            "group": "Group 3: Intermediate Failures",
            "topo": ("grid", 3, 1.0),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [(250, 1)],
            "expect_hub_failover": True,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T14_Cascading_Hub_Deaths_Phase2_Orphan",
            "group": "Group 3: Intermediate Failures",
            "topo": ("grid", 3, 1.5),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [(220, 4), (340, 3)],
            "expect_hub_failover": True,
            "expect_root_election": False,
            "partition_test": False,
        },
        {
            "id": "T15_Large_Grid_4x4_Dual_Hub_Failure",
            "group": "Group 3: Intermediate Failures",
            "topo": ("grid", 4, 1.5),
            "seed": seed_sum,
            "expected_query": "SUM",
            "faults": [(220, 5), (380, 10)],
            "expect_hub_failover": True,
            "expect_root_election": False,
            "partition_test": False,
        },

        # --- Group 4: Root Failures, Consecutive Elections & Partitions ---
        {
            "id": "T16_Root0_Failure_Election_3x3",
            "group": "Group 4: Root Elections & Partitions",
            "topo": ("grid", 3, 1.5),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [(350, 0)],
            "expect_hub_failover": False,
            "expect_root_election": True,
            "partition_test": False,
        },
        {
            "id": "T17_Consecutive_Root_Failures_0_Then_1",
            "group": "Group 4: Root Elections & Partitions",
            "topo": ("grid", 3, 1.5),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [(250, 0), (500, 1)],
            "expect_hub_failover": False,
            "expect_root_election": True,
            "partition_test": False,
        },
        {
            "id": "T18_Combined_Chaos_Hub_Plus_Root_Death",
            "group": "Group 4: Root Elections & Partitions",
            "topo": ("grid", 3, 1.5),
            "seed": 1,
            "expected_query": "AVG",
            "faults": [(250, 4), (450, 0)],
            "expect_hub_failover": True,
            "expect_root_election": True,
            "partition_test": False,
        },
        {
            "id": "T19_Bridge_CutVertex_Network_Partition",
            "group": "Group 4: Root Elections & Partitions",
            "topo": ("bridge",),
            "seed": seed_avg,
            "expected_query": "AVG",
            "faults": [(250, 3)],
            "expect_hub_failover": True,
            "expect_root_election": False,
            "partition_test": True,
        },

        # --- Group 5: Radio Duty-Cycling Lifecycle ---
        {
            "id": "T20_DutyCycle_Lifecycle_30s_Epochs",
            "group": "Group 5: Radio Duty-Cycling",
            "topo": ("grid", 3, 1.5),
            "seed": seed_avg_30,
            "expected_query": "AVG",
            "faults": [],
            "expect_hub_failover": False,
            "expect_root_election": False,
            "partition_test": False,
            "expected_epoch_dur": 30,
            "min_radio_cycles": 20,
        },
    ]


SCENARIOS = build_scenarios()
SCENARIO_BY_ID = dict((sc["id"], sc) for sc in SCENARIOS)


# ---------------------------------------------------------------------------
# Log Regexes (Matched to Exact SRTreeC.nc Debug Formats)
# ---------------------------------------------------------------------------

RE_TS = re.compile(r"^(\d+):(\d+):([0-9.]+)\s+DEBUG\s+\((\d+)\):")

RE_ROOT_BOOT = re.compile(
    r"DEBUG \(0\): ROOT \(Node 0\) booted: selected Query=(\d+) "
    r"\((SUM|AVG|VARIANCE|MAX)\), EpochDuration=(\d+) sec"
)

RE_RADIO_INIT = re.compile(r"DEBUG \((\d+)\): Radio initialized successfully!")
RE_RADIO_STOP = re.compile(r"DEBUG \((\d+)\): Radio stopped!")
RE_RADIO_EOFF = re.compile(r"EOFF")
RE_RADIO_ESIZE = re.compile(r"returned ESIZE")

RE_SELECT_PARENT = re.compile(
    r"DEBUG \((\d+)\): Node \1 selected parentID=(\d+), depth=(\d+) "
    r"\(treeSeq=(\d+), root=(\d+), Query=(SUM|AVG|VARIANCE|MAX), Epoch=(\d+)s\)"
)

RE_SAMPLE = re.compile(
    r"DEBUG \((\d+)\): Epoch (\d+) \| Node \1 \(depth (\d+), parent (\d+)\) "
    r"sampled local reading = (\d+)"
)

RE_FAILOVER = re.compile(
    r"DEBUG \((\d+)\): FAILOVER: Node \1 switched from parent (\d+) "
    r"to backup parent (\d+) \(depth (\d+) -> (\d+)\)"
)

RE_ORPHAN = re.compile(
    r"DEBUG \((\d+)\): ORPHAN: Node \1 lost parent (\d+) with no backups; sending local pull"
)

RE_ROOT_ELECT = re.compile(
    r"DEBUG \((\d+)\): ### ROOT FAILOVER: Node \1 ELECTED NEW ROOT "
    r"\(treeSeq=(\d+), Epoch=(\d+)\) ###"
)

RE_RES_SUM = re.compile(
    r"DEBUG \((\d+)\): >>> \[EPOCH (\d+) RESULT \(Root=\1\) \| "
    r"Query=SUM \| EpochDur=(\d+)s\] Final SUM = (\d+) \(from (\d+) nodes\) <<<"
)

RE_RES_AVG = re.compile(
    r"DEBUG \((\d+)\): >>> \[EPOCH (\d+) RESULT \(Root=\1\) \| "
    r"Query=AVG \| EpochDur=(\d+)s\] Final AVG = ([0-9.]+) \(Sum=(\d+), Count=(\d+)\) <<<"
)

RE_RES_VAR = re.compile(
    r"DEBUG \((\d+)\): >>> \[EPOCH (\d+) RESULT \(Root=\1\) \| "
    r"Query=VARIANCE \| EpochDur=(\d+)s\] Final VARIANCE = ([0-9.]+) "
    r"\(Sum=(\d+), SumSq=(\d+), Count=(\d+)\) <<<"
)

RE_RES_MAX = re.compile(
    r"DEBUG \((\d+)\): >>> \[EPOCH (\d+) RESULT \(Root=\1\) \| "
    r"Query=MAX \| EpochDur=(\d+)s\] Final MAX = (\d+) <<<"
)

RE_RX_PAYLOAD = re.compile(r"rx (MaxMsg|SumMsg|AvgMsg|VarMsg) \((\d+)B\)")

RE_FAULT = re.compile(
    r"^\*\*\* \[FAULT INJECTION\] Turning OFF Node (\d+) at t = (\d+)s \*\*\*"
)


def parse_timestamp_seconds(line):
    """Parse TOSSIM `H:M:S.frac` log prefix to floating-point seconds, or None."""
    m = RE_TS.match(line)
    if not m:
        return None
    hours = int(m.group(1))
    mins = int(m.group(2))
    secs = float(m.group(3))
    return hours * 3600.0 + mins * 60.0 + secs


# ---------------------------------------------------------------------------
# Simulation Runner & 8-Invariant Verification Engine (Worker Mode)
# ---------------------------------------------------------------------------

def build_scenario_topology(topo_spec, filepath=TEST_TOPO_FILE):
    """Create `filepath` (default test_topo.txt) for `topo_spec` and return its undirected adjacency dict."""
    kind = topo_spec[0]
    if kind == "grid":
        return generate_grid_topology(topo_spec[1], topo_spec[2], filepath)
    elif kind == "star":
        return generate_star_topology(topo_spec[1], filepath)
    elif kind == "chain":
        return generate_chain_topology(topo_spec[1], filepath)
    elif kind == "bridge":
        return generate_bridge_topology(filepath)
    else:
        raise ValueError("Unknown topology spec: %r" % (topo_spec,))


def run_tossim_scenario(scenario):
    """Execute a single scenario in TOSSIM and write channels + fault markers to WORKER_LOG_FILE."""
    from TOSSIM import Tossim  # Imported inside worker so orchestrator runs cleanly anywhere

    adj = build_scenario_topology(scenario["topo"], TEST_TOPO_FILE)
    num_nodes = len(adj)
    seed_offset = scenario["seed"]

    t = Tossim([])
    sim_end = DEFAULT_SIM_SECONDS * t.ticksPerSecond()

    log_f = open(WORKER_LOG_FILE, "w")
    try:
        t.addChannel("Boot", log_f)
        t.addChannel("Radio", log_f)
        t.addChannel("SRTreeC", log_f)
        t.addChannel("Lab1", log_f)

        r = t.radio()
        topo_f = open(TEST_TOPO_FILE, "r")
        try:
            for line in topo_f:
                parts = line.split()
                if len(parts) >= 3:
                    r.add(int(parts[0]), int(parts[1]), float(parts[2]))
        finally:
            topo_f.close()

        for i in range(num_nodes):
            t.getNode(i).bootAtTime(10 * t.ticksPerSecond() + i + seed_offset)

        tos_dir = os.getenv("TINYOS_ROOT_DIR")
        if not tos_dir:
            raise RuntimeError("TINYOS_ROOT_DIR environment variable is not set")
        noise_path = os.path.join(tos_dir, "tos", "lib", "tossim", "noise", "meyer-heavy.txt")
        noise_f = open(noise_path, "r")
        try:
            for line in noise_f:
                s = line.strip()
                if s:
                    val = int(s)
                    for i in range(num_nodes):
                        t.getNode(i).addNoiseTraceReading(val)
        finally:
            noise_f.close()

        for i in range(num_nodes):
            t.getNode(i).createNoiseModel()

        faults_sorted = sorted(scenario["faults"], key=lambda x: x[0])
        fault_idx = 0
        zero_events = 0

        while t.time() < sim_end:
            h = t.runNextEvent()
            if h <= 0:
                zero_events += 1
                if zero_events > 10000:
                    break
            else:
                zero_events = 0

            while fault_idx < len(faults_sorted):
                f_sec, f_node = faults_sorted[fault_idx]
                if t.time() >= f_sec * t.ticksPerSecond():
                    log_f.write("*** [FAULT INJECTION] Turning OFF Node %d at t = %ds ***\n" % (f_node, f_sec))
                    log_f.flush()
                    t.getNode(f_node).turnOff()
                    fault_idx += 1
                else:
                    break
    finally:
        log_f.close()

    return adj


def check_cycle_freedom(active_parent, active_depth, active_seq, active_root, dead_nodes, num_nodes, changed_node=None):
    """
    Walk parent pointers upward from every attached, alive node and verify:
      1. No directed cycle is ever formed (visited set check).
      2. A path terminates in <= num_nodes steps.
      3. Whenever `changed_node is None` (full settled snapshot at epoch readout) or
         `cur == changed_node` (the node that just updated its parent/depth), if child
         and parent share the same active `treeSeq` and both are attached,
         `depth[parent] < depth[child]` strictly holds. (Deferring non-`changed_node`
         edges on instantaneous FAILOVER avoids false positives during the ~8-35 ms
         1-hop radio propagation window before downstream children receive the beacon.)
    """
    for start in range(num_nodes):
        if start in dead_nodes:
            continue
        p = active_parent.get(start)
        d = active_depth.get(start)
        if p is None or d is None or d >= 30:
            continue

        visited = set([start])
        cur = start
        steps = 0
        while steps <= num_nodes:
            par = active_parent.get(cur)
            if par is None or active_depth.get(cur, 31) >= 30:
                break
            if par == cur and active_depth.get(cur) == 0:
                # Reached active Root
                break
            if par in dead_nodes:
                # Pending parent failure before child finishes retries
                break
            if par in visited:
                return (False, "Routing cycle detected involving Node %d -> Node %d" % (cur, par))
            par_d = active_depth.get(par)
            par_seq = active_seq.get(par)
            cur_seq = active_seq.get(cur)
            if (changed_node is None or cur == changed_node) and par_d is not None and par_d < 30 and par_seq == cur_seq:
                if not (par_d < active_depth[cur]):
                    return (
                        False,
                        "Depth monotonicity violation: child Node %d (depth %d) -> parent Node %d (depth %d) on treeSeq %s"
                        % (cur, active_depth[cur], par, par_d, str(cur_seq)),
                    )
            visited.add(par)
            cur = par
            steps += 1
        if steps > num_nodes:
            return (False, "Parent chain exceeded network size %d starting from Node %d" % (num_nodes, start))
    return (True, None)


def verify_log_invariants(scenario, adj, log_path=WORKER_LOG_FILE):
    """
    Parse `log_path` and verify all 8 protocol, routing, radio-lifecycle, and
    ground-truth aggregation invariants. Raises AssertionError on any violation.
    """
    num_nodes = len(adj)
    f = open(log_path, "r")
    try:
        lines = f.readlines()
    finally:
        f.close()

    # Parsed state containers
    root_query_name = None
    epoch_dur = None
    rx_payloads = []
    samples = {}             # samples[node][epoch] = (val, t_sec, depth, parent)
    epoch_results = []       # list of dicts in chronological emission order
    failover_events = []     # (node, old_p, new_p, old_d, new_d, t_sec)
    orphan_events = []       # (node, lost_p, t_sec)
    election_events = []     # (new_root, tree_seq, epoch_num, t_sec, eligible_d1_snapshot)
    fault_times = []         # (node_id, t_sec)
    radio_state_tracker = {} # node_id -> "ON" or "OFF"
    radio_starts = {}        # node_id -> start count
    radio_stops = {}         # node_id -> stop count
    eoff_lines = []          # any EOFF transmit warning lines
    esize_lines = []         # any ESIZE transmit drop warning lines

    # Dynamic routing state tracker for cycle/depth & election verification
    active_parent = {0: 0}
    active_depth = {0: 0}
    active_seq = {0: 0}
    active_root = {0: 0}
    dead_nodes = set()
    cur_tree_root = 0
    cur_tree_seq = 0
    last_time = 0.0

    for raw in lines:
        line = raw.strip()
        if not line:
            continue

        ts = parse_timestamp_seconds(line)
        if ts is not None:
            last_time = ts

        if RE_RADIO_EOFF.search(line):
            eoff_lines.append(line)
        if RE_RADIO_ESIZE.search(line):
            esize_lines.append(line)

        m_rinit = RE_RADIO_INIT.search(line)
        if m_rinit:
            rnid = int(m_rinit.group(1))
            if radio_state_tracker.get(rnid) == "ON":
                raise AssertionError(
                    "[Invariant 8: Radio Lifecycle] Node %d consecutive start without stop at t=%.3fs"
                    % (rnid, last_time)
                )
            radio_state_tracker[rnid] = "ON"
            radio_starts[rnid] = radio_starts.get(rnid, 0) + 1
            continue

        m_rstop = RE_RADIO_STOP.search(line)
        if m_rstop:
            rnid = int(m_rstop.group(1))
            if radio_state_tracker.get(rnid) != "ON":
                raise AssertionError(
                    "[Invariant 8: Radio Lifecycle] Node %d stop without active start at t=%.3fs"
                    % (rnid, last_time)
                )
            radio_state_tracker[rnid] = "OFF"
            radio_stops[rnid] = radio_stops.get(rnid, 0) + 1
            continue

        m_fault = RE_FAULT.match(line)
        if m_fault:
            fn = int(m_fault.group(1))
            ft = float(m_fault.group(2))
            dead_nodes.add(fn)
            active_parent.pop(fn, None)
            active_depth.pop(fn, None)
            fault_times.append((fn, ft))
            continue

        m_boot = RE_ROOT_BOOT.search(line)
        if m_boot:
            root_query_name = m_boot.group(2)
            epoch_dur = int(m_boot.group(3))
            continue

        m_rx = RE_RX_PAYLOAD.search(line)
        if m_rx:
            rx_payloads.append((m_rx.group(1), int(m_rx.group(2))))

        m_sel = RE_SELECT_PARENT.search(line)
        if m_sel:
            node = int(m_sel.group(1))
            par = int(m_sel.group(2))
            dep = int(m_sel.group(3))
            seq = int(m_sel.group(4))
            rt = int(m_sel.group(5))
            active_parent[node] = par
            active_depth[node] = dep
            active_seq[node] = seq
            active_root[node] = rt
            ok, err = check_cycle_freedom(active_parent, active_depth, active_seq, active_root, dead_nodes, num_nodes, changed_node=node)
            if not ok:
                raise AssertionError("[Invariant 3: Cycle Freedom] at t=%.3fs: %s" % (last_time, err))
            continue

        m_fail = RE_FAILOVER.search(line)
        if m_fail:
            node = int(m_fail.group(1))
            old_p = int(m_fail.group(2))
            new_p = int(m_fail.group(3))
            old_d = int(m_fail.group(4))
            new_d = int(m_fail.group(5))
            active_parent[node] = new_p
            active_depth[node] = new_d
            failover_events.append((node, old_p, new_p, old_d, new_d, last_time))
            ok, err = check_cycle_freedom(active_parent, active_depth, active_seq, active_root, dead_nodes, num_nodes, changed_node=node)
            if not ok:
                raise AssertionError("[Invariant 3: Cycle Freedom after FAILOVER] at t=%.3fs: %s" % (last_time, err))
            continue

        m_orp = RE_ORPHAN.search(line)
        if m_orp:
            node = int(m_orp.group(1))
            lost_p = int(m_orp.group(2))
            active_parent[node] = None
            active_depth[node] = 31
            orphan_events.append((node, lost_p, last_time))
            continue

        m_elec = RE_ROOT_ELECT.search(line)
        if m_elec:
            new_root = int(m_elec.group(1))
            seq = int(m_elec.group(2))
            ep = int(m_elec.group(3))
            # Snapshot alive depth-1 nodes attached under the failed root
            d1_candidates = sorted([
                nid for nid in range(num_nodes)
                if nid not in dead_nodes
                and active_depth.get(nid) == 1
                and active_parent.get(nid) == cur_tree_root
            ])
            election_events.append((new_root, seq, ep, last_time, cur_tree_seq, d1_candidates))
            cur_tree_root = new_root
            cur_tree_seq = seq
            active_parent[new_root] = new_root
            active_depth[new_root] = 0
            active_seq[new_root] = seq
            active_root[new_root] = new_root
            continue

        m_smp = RE_SAMPLE.search(line)
        if m_smp:
            node = int(m_smp.group(1))
            ep = int(m_smp.group(2))
            dep = int(m_smp.group(3))
            par = int(m_smp.group(4))
            val = int(m_smp.group(5))
            if node not in samples:
                samples[node] = {}
            samples[node][ep] = (val, last_time, dep, par)
            continue

        m_sum = RE_RES_SUM.search(line)
        if m_sum:
            ok, err = check_cycle_freedom(active_parent, active_depth, active_seq, active_root, dead_nodes, num_nodes, changed_node=None)
            if not ok:
                raise AssertionError("[Invariant 3: Cycle Freedom at epoch result] at t=%.3fs: %s" % (last_time, err))
            epoch_results.append({
                "root": int(m_sum.group(1)),
                "epoch": int(m_sum.group(2)),
                "dur": int(m_sum.group(3)),
                "query": "SUM",
                "sum": int(m_sum.group(4)),
                "count": int(m_sum.group(5)),
                "time": last_time,
                "dead_snapshot": set(dead_nodes),
            })
            continue

        m_avg = RE_RES_AVG.search(line)
        if m_avg:
            ok, err = check_cycle_freedom(active_parent, active_depth, active_seq, active_root, dead_nodes, num_nodes, changed_node=None)
            if not ok:
                raise AssertionError("[Invariant 3: Cycle Freedom at epoch result] at t=%.3fs: %s" % (last_time, err))
            epoch_results.append({
                "root": int(m_avg.group(1)),
                "epoch": int(m_avg.group(2)),
                "dur": int(m_avg.group(3)),
                "query": "AVG",
                "avg": float(m_avg.group(4)),
                "sum": int(m_avg.group(5)),
                "count": int(m_avg.group(6)),
                "time": last_time,
                "dead_snapshot": set(dead_nodes),
            })
            continue

        m_var = RE_RES_VAR.search(line)
        if m_var:
            ok, err = check_cycle_freedom(active_parent, active_depth, active_seq, active_root, dead_nodes, num_nodes, changed_node=None)
            if not ok:
                raise AssertionError("[Invariant 3: Cycle Freedom at epoch result] at t=%.3fs: %s" % (last_time, err))
            epoch_results.append({
                "root": int(m_var.group(1)),
                "epoch": int(m_var.group(2)),
                "dur": int(m_var.group(3)),
                "query": "VARIANCE",
                "var": float(m_var.group(4)),
                "sum": int(m_var.group(5)),
                "sumSq": int(m_var.group(6)),
                "count": int(m_var.group(7)),
                "time": last_time,
                "dead_snapshot": set(dead_nodes),
            })
            continue

        m_max = RE_RES_MAX.search(line)
        if m_max:
            ok, err = check_cycle_freedom(active_parent, active_depth, active_seq, active_root, dead_nodes, num_nodes, changed_node=None)
            if not ok:
                raise AssertionError("[Invariant 3: Cycle Freedom at epoch result] at t=%.3fs: %s" % (last_time, err))
            epoch_results.append({
                "root": int(m_max.group(1)),
                "epoch": int(m_max.group(2)),
                "dur": int(m_max.group(3)),
                "query": "MAX",
                "max": int(m_max.group(4)),
                "time": last_time,
                "dead_snapshot": set(dead_nodes),
            })
            continue

    if root_query_name is None or epoch_dur is None:
        raise AssertionError("Root Node 0 boot configuration line missing from log")

    if scenario.get("expected_query") and root_query_name != scenario["expected_query"]:
        raise AssertionError(
            "Query mismatch: expected %s (seed=%d), got %s"
            % (scenario["expected_query"], scenario["seed"], root_query_name)
        )
    if scenario.get("expected_epoch_dur") and epoch_dur != scenario["expected_epoch_dur"]:
        raise AssertionError(
            "Epoch duration mismatch: expected %ds (seed=%d), got %ds"
            % (scenario["expected_epoch_dur"], scenario["seed"], epoch_dur)
        )

    # --- Invariant 1: Bounded Random Walk & Sample Range Check ---
    if not samples:
        raise AssertionError("[Invariant 1] Zero sensor samples parsed")
    for nid, ep_map in samples.items():
        eps = sorted(ep_map.keys())
        for idx, ep in enumerate(eps):
            val = ep_map[ep][0]
            if not (1 <= val <= 100):
                raise AssertionError("[Invariant 1] Node %d Epoch %d sample %d out of [1..100]" % (nid, ep, val))
            if idx > 0 and eps[idx - 1] == ep - 1:
                prev_val = ep_map[ep - 1][0]
                delta = (prev_val * 25) // 100
                lo = max(1, prev_val - delta)
                hi = min(100, prev_val + delta)
                if not (lo <= val <= hi):
                    raise AssertionError(
                        "[Invariant 1] Node %d Epoch %d -> %d violated +/-25%% walk: prev=%d cur=%d allowed=[%d..%d]"
                        % (nid, ep - 1, ep, prev_val, val, lo, hi)
                    )

    # --- Invariant 2: Query-Specific Minimal Payload Size Check ---
    exp_msg_name, exp_bytes = EXPECTED_PAYLOAD_MAP[root_query_name]
    if not rx_payloads:
        raise AssertionError("[Invariant 2] No rx data packets observed")
    for msg_name, nbytes in rx_payloads:
        if msg_name != exp_msg_name or nbytes != exp_bytes:
            raise AssertionError(
                "[Invariant 2] Wrong payload: expected %s (%dB), got %s (%dB)"
                % (exp_msg_name, exp_bytes, msg_name, nbytes)
            )

    # --- Invariant 4: Zero Missed Epochs & Split-Brain Duplicate Check ---
    expected_num_epochs = int(math.floor((DEFAULT_SIM_SECONDS - 15.25) / float(epoch_dur))) + 1
    reported_epochs = [r["epoch"] for r in epoch_results]
    if len(reported_epochs) != len(set(reported_epochs)):
        raise AssertionError("[Invariant 4] Duplicate epoch result emitted (split-brain): %s" % (reported_epochs,))
    expected_seq = list(range(1, expected_num_epochs + 1))
    if reported_epochs != expected_seq:
        raise AssertionError(
            "[Invariant 4] Missed or out-of-order epochs (EpochDur=%ds): expected %s, got %s"
            % (epoch_dur, expected_seq, reported_epochs)
        )

    # --- Invariant 5: Exact Ground-Truth Aggregation Verification ---
    prev_res_time = 0.0
    for res in epoch_results:
        ep = res["epoch"]
        r_time = res["time"]
        r_root = res["root"]
        dead_snap = res["dead_snapshot"]

        # Connected alive component reachable from the active reporting Root
        alive_comp = connected_component(adj, r_root, dead_snap)

        # Determine whether any fault occurred in this epoch interval
        faults_in_epoch = [(fn, ft) for (fn, ft) in fault_times if prev_res_time < ft <= r_time]
        is_transition = len(faults_in_epoch) > 0
        # Check if any killed node already sampled during this epoch before dying
        mid_tx_death = any(
            (r_time - 3.0) <= ft <= (r_time + 0.5) and fn in samples and ep in samples[fn]
            for (fn, ft) in faults_in_epoch
        )

        # Gather ground-truth readings for alive_comp in epoch `ep`
        vals = []
        missing_samples = []
        for nid in sorted(alive_comp):
            if nid in samples and ep in samples[nid]:
                vals.append(samples[nid][ep][0])
            else:
                missing_samples.append(nid)

        if missing_samples:
            raise AssertionError(
                "[Invariant 5] Epoch %d missing sampled readings for alive connected nodes: %s"
                % (ep, missing_samples)
            )

        exp_cnt = len(vals)
        exp_sum = sum(vals)
        exp_sum_sq = sum(v * v for v in vals)
        exp_avg = round(float(exp_sum) / float(exp_cnt), 2)
        mean = float(exp_sum) / float(exp_cnt)
        mean_sq = float(exp_sum_sq) / float(exp_cnt)
        exp_var = round(max(0.0, mean_sq - mean * mean), 2)
        exp_max = max(vals)

        if not is_transition:
            # Steady-state epoch: strict 100% mathematical accuracy required
            if root_query_name == "SUM":
                # 2-byte SumMsg omits child count, so the Root only counts its local sample
                if res["sum"] != exp_sum:
                    raise AssertionError(
                        "[Invariant 5 Steady Epoch %d SUM] Expected Sum=%d, got Sum=%d"
                        % (ep, exp_sum, res["sum"])
                    )
            elif root_query_name == "AVG":
                if (res["count"] != exp_cnt or res["sum"] != exp_sum or
                        abs(res["avg"] - exp_avg) > 0.02):
                    raise AssertionError(
                        "[Invariant 5 Steady Epoch %d AVG] Expected (Avg=%.2f, Sum=%d, Count=%d), got (Avg=%.2f, Sum=%d, Count=%d)"
                        % (ep, exp_avg, exp_sum, exp_cnt, res["avg"], res["sum"], res["count"])
                    )
            elif root_query_name == "VARIANCE":
                if (res["count"] != exp_cnt or res["sum"] != exp_sum or
                        res["sumSq"] != exp_sum_sq or abs(res["var"] - exp_var) > 0.05):
                    raise AssertionError(
                        "[Invariant 5 Steady Epoch %d VAR] Expected (Var=%.2f, Sum=%d, SumSq=%d, Count=%d), got (Var=%.2f, Sum=%d, SumSq=%d, Count=%d)"
                        % (ep, exp_var, exp_sum, exp_sum_sq, exp_cnt, res["var"], res["sum"], res["sumSq"], res["count"])
                    )
            elif root_query_name == "MAX":
                if res["max"] != exp_max:
                    raise AssertionError(
                        "[Invariant 5 Steady Epoch %d MAX] Expected Max=%d, got Max=%d"
                        % (ep, exp_max, res["max"])
                    )
        else:
            # Transition epoch right after a fault injection
            if root_query_name == "SUM":
                if not mid_tx_death and res["sum"] != exp_sum:
                    raise AssertionError(
                        "[Invariant 5 Transition Epoch %d SUM] Expected Sum=%d across %s, got Sum=%d"
                        % (ep, exp_sum, sorted(alive_comp), res["sum"])
                    )
            elif root_query_name != "MAX":
                if not mid_tx_death and (res["count"] != exp_cnt or res["sum"] != exp_sum):
                    raise AssertionError(
                        "[Invariant 5 Transition Epoch %d %s] Expected (Sum=%d, Count=%d) across %s, got (Sum=%d, Count=%d)"
                        % (ep, root_query_name, exp_sum, exp_cnt, sorted(alive_comp), res["sum"], res["count"])
                    )
            else:
                if not mid_tx_death and res["max"] != exp_max:
                    raise AssertionError(
                        "[Invariant 5 Transition Epoch %d MAX] Expected Max=%d across %s, got Max=%d"
                        % (ep, exp_max, sorted(alive_comp), res["max"])
                    )

        prev_res_time = r_time

    # --- Invariant 6: Failover & Slotted Root Election Verification ---
    if scenario.get("expect_hub_failover"):
        if not failover_events and not orphan_events:
            raise AssertionError("[Invariant 6] Expected FAILOVER or ORPHAN recovery events after hub death, found none")

    if scenario.get("expect_root_election"):
        if not election_events:
            raise AssertionError("[Invariant 6] Expected ROOT FAILOVER election event, found none")
        for (new_root, seq, ep, e_time, prev_seq, d1_cands) in election_events:
            if seq != ((prev_seq + 1) & 0x07):
                raise AssertionError(
                    "[Invariant 6] Root election treeSeq did not increment mod 8: prev=%d, new=%d"
                    % (prev_seq, seq)
                )
            if d1_cands and new_root != d1_cands[0]:
                raise AssertionError(
                    "[Invariant 6] Elected Root Node %d was not the lowest-ID surviving Depth-1 candidate %s"
                    % (new_root, d1_cands)
                )

    # --- Invariant 7: Network Partition Gracefulness (Bridge Cut-Vertex) ---
    if scenario.get("partition_test"):
        # For T19: Bridge Node 3 killed at t=250s -> isolated {4, 5}, connected {0, 1, 2}
        post_fault_orphans = [ev for ev in orphan_events if ev[0] in (4, 5) and ev[2] >= 250.0]
        if not post_fault_orphans:
            raise AssertionError("[Invariant 7 Partition] Isolated nodes {4, 5} never detached into orphan state")
        # Ensure isolated nodes didn't attach back onto {0,1,2} after t=260s
        post_sel = [
            raw.strip() for raw in lines
            if RE_SELECT_PARENT.search(raw)
            and int(RE_SELECT_PARENT.search(raw).group(1)) in (4, 5)
            and parse_timestamp_seconds(raw) is not None
            and parse_timestamp_seconds(raw) >= 255.0
            and int(RE_SELECT_PARENT.search(raw).group(2)) in (0, 1, 2)
        ]
        if post_sel:
            raise AssertionError("[Invariant 7 Partition] Isolated node impossibly attached across cut: %s" % post_sel)
        # Ensure no runaway orphan pull storm on isolated nodes (> 15 pulls per minute)
        if len(post_fault_orphans) > 20:
            raise AssertionError(
                "[Invariant 7 Partition] Excessive orphan storm count (%d) after partition" % len(post_fault_orphans)
            )
        # Ensure all post-260s epochs reported exact count=3
        for res in epoch_results:
            if res["time"] >= 260.0 and res.get("count") != 3:
                raise AssertionError(
                    "[Invariant 7 Partition] Epoch %d expected 3 connected nodes {0,1,2}, got %s"
                    % (res["epoch"], res.get("count"))
                )

    # --- Invariant 8: Radio Duty-Cycling & SplitControl Lifecycle ---
    if eoff_lines:
        raise AssertionError(
            "[Invariant 8: Radio Lifecycle] Observed EOFF radio transmit warnings: %s"
            % (eoff_lines[:5],)
        )
    if esize_lines:
        raise AssertionError(
            "[Invariant 8: Radio Lifecycle] Observed ESIZE radio transmit drops: %s"
            % (esize_lines[:5],)
        )

    final_root = epoch_results[-1]["root"] if epoch_results else 0
    surviving_connected = connected_component(adj, final_root, dead_nodes)
    is_steady_state = (not scenario.get("faults") and not scenario.get("partition_test"))
    min_cycles_req = scenario.get("min_radio_cycles", 0)
    expect_final_off = (
        bool(epoch_results)
        and epoch_results[-1]["time"] < (DEFAULT_SIM_SECONDS - 3.0)
    )

    for nid in range(num_nodes):
        n_starts = radio_starts.get(nid, 0)
        n_stops = radio_stops.get(nid, 0)
        if n_starts < 1:
            raise AssertionError(
                "[Invariant 8: Radio Lifecycle] Node %d never started its radio" % nid
            )
        if not (0 <= (n_starts - n_stops) <= 1):
            raise AssertionError(
                "[Invariant 8: Radio Lifecycle] Node %d start/stop parity violation: starts=%d stops=%d"
                % (nid, n_starts, n_stops)
            )
        if expect_final_off and nid in surviving_connected and (n_starts != n_stops or radio_state_tracker.get(nid) != "OFF"):
            raise AssertionError(
                "[Invariant 8: Radio Lifecycle] Surviving Node %d failed to return to RADIO_OFF after final epoch (starts=%d, stops=%d, state=%s)"
                % (nid, n_starts, n_stops, radio_state_tracker.get(nid))
            )

        node_epochs = len(samples.get(nid, {}))
        if nid in surviving_connected and node_epochs >= 2:
            if n_starts < 2 or n_stops < 1:
                raise AssertionError(
                    "[Invariant 8: Radio Lifecycle] Surviving Node %d failed to power-cycle across %d epochs (starts=%d, stops=%d)"
                    % (nid, node_epochs, n_starts, n_stops)
                )
            if is_steady_state and epoch_dur <= 100 and len(epoch_results) >= 3:
                min_expected_stops = max(2, len(epoch_results) - 1)
                if n_stops < min_expected_stops:
                    raise AssertionError(
                        "[Invariant 8: Radio Lifecycle] Steady-state Node %d expected >= %d radio stops across %d epochs, got %d"
                        % (nid, min_expected_stops, len(epoch_results), n_stops)
                    )
            elif not is_steady_state and epoch_dur <= 100 and len(epoch_results) >= 4:
                min_fault_stops = max(2, len(epoch_results) - len(scenario.get("faults", [])))
                if n_stops < min_fault_stops:
                    raise AssertionError(
                        "[Invariant 8: Radio Lifecycle] Surviving Node %d expected >= %d radio stops across %d epochs (faults=%d), got %d"
                        % (nid, min_fault_stops, len(epoch_results), len(scenario.get("faults", [])), n_stops)
                    )
            if min_cycles_req > 0 and (n_starts < min_cycles_req or n_stops < min_cycles_req):
                raise AssertionError(
                    "[Invariant 8: Radio Lifecycle] Node %d expected >= %d radio start/stop cycles, got starts=%d stops=%d"
                    % (nid, min_cycles_req, n_starts, n_stops)
                )
        elif node_epochs >= 2:
            # Faulted/partitioned nodes that completed >= 2 epochs before death/partition
            # must have undergone radio duty-cycling prior to the fault/partition.
            if n_starts < 2 or n_stops < 1:
                raise AssertionError(
                    "[Invariant 8: Radio Lifecycle] Node %d completed %d epochs without duty-cycling (starts=%d, stops=%d)"
                    % (nid, node_epochs, n_starts, n_stops)
                )

    surv_stops = [radio_stops.get(nid, 0) for nid in surviving_connected] if surviving_connected else [0]

    return {
        "query": root_query_name,
        "epoch_dur": epoch_dur,
        "epochs": len(epoch_results),
        "failovers": len(failover_events),
        "orphans": len(orphan_events),
        "elections": len(election_events),
        "min_stops": min(surv_stops),
        "max_stops": max(surv_stops),
    }


def run_worker(scenario_id):
    """Worker entrypoint: simulate one scenario in isolation and assert all 8 invariants."""
    if scenario_id not in SCENARIO_BY_ID:
        sys.stderr.write("Unknown scenario ID: %s\n" % scenario_id)
        sys.exit(2)

    scenario = SCENARIO_BY_ID[scenario_id]
    adj = run_tossim_scenario(scenario)
    stats = verify_log_invariants(scenario, adj, WORKER_LOG_FILE)
    print(
        "WORKER_OK query=%s epoch_dur=%ds epochs=%d failovers=%d orphans=%d elections=%d stops=%d..%d"
        % (
            stats["query"],
            stats["epoch_dur"],
            stats["epochs"],
            stats["failovers"],
            stats["orphans"],
            stats["elections"],
            stats["min_stops"],
            stats["max_stops"],
        )
    )


# ---------------------------------------------------------------------------
# Subprocess Orchestrator Mode
# ---------------------------------------------------------------------------

def run_single_subprocess(scenario_id, timeout_sec):
    """Spawn `python test_suite.py --worker <scenario_id>` with a wall-clock timeout."""
    cmd = [sys.executable, os.path.abspath(__file__), "--worker", scenario_id]
    start_t = time.time()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    timed_out = False
    while proc.poll() is None:
        if (time.time() - start_t) > timeout_sec:
            timed_out = True
            try:
                proc.terminate()
                time.sleep(1.0)
                if proc.poll() is None:
                    proc.kill()
            except OSError:
                pass
            break
        time.sleep(0.2)

    out, _ = proc.communicate()
    elapsed = time.time() - start_t
    rc = proc.returncode
    return (timed_out, rc, out.strip(), elapsed)


def cleanup_and_restore_default_topology():
    """Restore topology.txt to default 3x3 (D=3, R=1.5) and remove worker temp files."""
    generate_grid_topology(3, 1.5, DEFAULT_TOPO_FILE)
    for tmp_name in (WORKER_LOG_FILE, TEST_TOPO_FILE):
        if os.path.exists(tmp_name):
            try:
                os.remove(tmp_name)
            except OSError:
                pass


def run_orchestrator():
    """Run all scenarios in isolated child processes, log report, and restore workspace."""
    passed = 0
    failed = 0
    total = len(SCENARIOS)
    report_lines = []

    header = (
        "============================================================================\n"
        "  TinyOS/TOSSIM TAG Automated Verification Suite (%d Scenarios)\n"
        "============================================================================" % total
    )
    print(header)
    report_lines.append(header)

    suite_start = time.time()
    try:
        current_group = None
        for idx, sc in enumerate(SCENARIOS):
            if sc["group"] != current_group:
                current_group = sc["group"]
                grp_banner = "\n--- %s ---" % current_group
                print(grp_banner)
                report_lines.append(grp_banner)

            timed_out, rc, output, elapsed = run_single_subprocess(sc["id"], DEFAULT_TIMEOUT_SEC)
            if not timed_out and rc == 0:
                passed += 1
                line = "[PASS] (%2d/%2d) %-44s (%.2fs) %s" % (idx + 1, total, sc["id"], elapsed, output)
                print(line)
                report_lines.append(line)
            else:
                failed += 1
                reason = "TIMEOUT (>%ds)" % DEFAULT_TIMEOUT_SEC if timed_out else ("EXIT %d" % rc)
                line = "[FAIL] (%2d/%2d) %-44s (%.2fs) [%s]" % (idx + 1, total, sc["id"], elapsed, reason)
                print(line)
                report_lines.append(line)
                if output:
                    for diag_line in output.splitlines():
                        indented = "       %s" % diag_line
                        print(indented)
                        report_lines.append(indented)
                # Preserve failing worker log excerpt for quick debugging
                if os.path.exists(WORKER_LOG_FILE):
                    fail_copy = "fail_%s.log" % sc["id"]
                    try:
                        shutil.copyfile(WORKER_LOG_FILE, fail_copy)
                        note = "       Saved full trace to %s" % fail_copy
                        print(note)
                        report_lines.append(note)
                    except IOError:
                        pass

        total_elapsed = time.time() - suite_start
        footer = (
            "\n============================================================================\n"
            "  SUMMARY: %d/%d PASSED, %d FAILED (Total time: %.2fs)\n"
            "============================================================================"
            % (passed, total, failed, total_elapsed)
        )
        print(footer)
        report_lines.append(footer)

        rf = open(RESULTS_FILE, "w")
        try:
            rf.write("\n".join(report_lines) + "\n")
        finally:
            rf.close()
    finally:
        cleanup_and_restore_default_topology()

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--worker":
        run_worker(sys.argv[2])
    elif len(sys.argv) == 1:
        sys.exit(run_orchestrator())
    else:
        sys.stderr.write("Usage: python %s [--worker <scenario_id>]\n" % sys.argv[0])
        sys.exit(2)
