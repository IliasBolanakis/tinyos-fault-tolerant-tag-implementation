#!/bin/bash
# Automated TOSSIM verification wrapper for the Ubuntu TinyOS VM.
# Copies project sources and test_suite.py into the TinyOS app directory,
# builds _TOSSIMmodule.so if missing, executes all isolated test scenarios,
# and copies the resulting report + restored topology.txt back to the shared folder.

cd /home/tinyos/local/src/tinyos-2x/apps/tinyOS
cp ~/Desktop/tinyos_SHARED/project1/PacketQueue.nc .
cp ~/Desktop/tinyos_SHARED/project1/PacketQueueC.nc .
cp ~/Desktop/tinyos_SHARED/project1/SimpleRoutingTree.h .
cp ~/Desktop/tinyos_SHARED/project1/BackupTable.nc .
cp ~/Desktop/tinyos_SHARED/project1/BackupTableP.nc .
cp ~/Desktop/tinyos_SHARED/project1/DupFilter.nc .
cp ~/Desktop/tinyos_SHARED/project1/DupFilterP.nc .
cp ~/Desktop/tinyos_SHARED/project1/TreeTopology.nc .
cp ~/Desktop/tinyos_SHARED/project1/TreeRoutingP.nc .
cp ~/Desktop/tinyos_SHARED/project1/SRTreeAppC.nc .
cp ~/Desktop/tinyos_SHARED/project1/SRTreeC.nc .
cp ~/Desktop/tinyos_SHARED/project1/create_topology.py .
cp ~/Desktop/tinyos_SHARED/project1/mySimulation.py .
cp ~/Desktop/tinyos_SHARED/project1/test_suite.py .

rm -f fail_*.log ~/Desktop/tinyos_SHARED/project1/fail_*.log
make micaz sim || exit 1

python ./test_suite.py | tee test_results.txt
cp test_results.txt topology.txt ~/Desktop/tinyos_SHARED/project1/
cp -f fail_*.log ~/Desktop/tinyos_SHARED/project1/ 2>/dev/null || true

