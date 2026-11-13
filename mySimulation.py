#!/usr/bin/python
from TOSSIM import *
import sys, os, time
from random import *

# Initialize TOSSIM simulation instance
t = Tossim([])
f = sys.stdout

# Simulation termination set to 800 seconds as specified by the project requirements
SIM_END_TIME = 800 * t.ticksPerSecond()
print "TicksPerSecond : ", t.ticksPerSecond(), "\n"

# Register active debug channels for live console monitoring
t.addChannel("Boot", f)
t.addChannel("Radio", f)
t.addChannel("SRTreeC", f)
t.addChannel("Lab1", f)

# Determine PRNG seed offset from CLI argument or wall-clock epoch
randDelay = int(sys.argv[1]) % 10 if len(sys.argv) > 1 else int(time.time()) % 10

# Parse topology.txt and construct the radio connectivity graph
topo = open("topology.txt", "r")
if topo is None:
    print "Topology file not opened!!! \n"

r = t.radio()
lines = topo.readlines()
maxNodeID = 0
for line in lines:
    s = line.split()
    if len(s) > 0:
        n1, n2, gain = int(s[0]), int(s[1]), float(s[2])
        if n1 > maxNodeID: maxNodeID = n1
        if n2 > maxNodeID: maxNodeID = n2
        r.add(n1, n2, gain)
topo.close()

# Dynamically derive total network size from topology file
numNodes = maxNodeID + 1
print "Detected ", numNodes, " nodes in topology.txt (random seed offset =", randDelay, ")\n"

# Boot nodes with staggered micro-offsets to simulate asynchronous startup
for i in range(0, numNodes):
    t.getNode(i).bootAtTime(10 * t.ticksPerSecond() + i + randDelay)

# Populate TOSSIM Empirical Noise Model using the standard Meyer heavy trace
mTosdir = os.getenv("TINYOS_ROOT_DIR")
noiseF = open(mTosdir + "/tos/lib/tossim/noise/meyer-heavy.txt", "r")
for line in noiseF.readlines():
    str1 = line.strip()
    if str1:
        val = int(str1)
        for i in range(0, numNodes):
            t.getNode(i).addNoiseTraceReading(val)
noiseF.close()

for i in range(0, numNodes):
    t.getNode(i).createNoiseModel()

h = True
node4Killed = False
root0Killed = False

node4Killed = False
root0Killed = False
zeroEventCount = 0

while t.time() < SIM_END_TIME:
    try:
        h = t.runNextEvent()
    except:
        print sys.exc_info()
        break

    if h <= 0:
        zeroEventCount += 1
        if zeroEventCount > 10000:
            break
    else:
        zeroEventCount = 0

    # Fault Injection 1: Kill intermediate Node 4 at t = 250s
    if not node4Killed and t.time() >= 250 * t.ticksPerSecond():
        print "\n*** [FAULT INJECTION] Turning OFF Intermediate Node 4 at t = 250s ***\n"
        t.getNode(4).turnOff()
        node4Killed = True

    # Fault Injection 2: Kill Root Node 0 at t = 450s
    if not root0Killed and t.time() >= 450 * t.ticksPerSecond():
        print "\n*** [FAULT INJECTION] Turning OFF ROOT Node 0 at t = 450s ***\n"
        t.getNode(0).turnOff()
        root0Killed = True
