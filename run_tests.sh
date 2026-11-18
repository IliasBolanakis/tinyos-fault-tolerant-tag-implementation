#!/bin/bash
# Automated TOSSIM verification wrapper for the Ubuntu TinyOS VM.
# Copies project sources and test_suite.py into the TinyOS app directory,
# builds _TOSSIMmodule.so if missing, executes all isolated test scenarios,
# and copies the resulting report + restored topology.txt back to the shared folder.

# Path to the shared project folder on your machine (update to match your setup)
SHARED_DIR="$HOME/Desktop/tinyos_SHARED/project1"

cd /home/tinyos/local/src/tinyos-2x/apps/tinyOS
cp "$SHARED_DIR/PacketQueue.nc" .
cp "$SHARED_DIR/PacketQueueC.nc" .
cp "$SHARED_DIR/SimpleRoutingTree.h" .
cp "$SHARED_DIR/BackupTable.nc" .
cp "$SHARED_DIR/BackupTableP.nc" .
cp "$SHARED_DIR/DupFilter.nc" .
cp "$SHARED_DIR/DupFilterP.nc" .
cp "$SHARED_DIR/TreeTopology.nc" .
cp "$SHARED_DIR/TreeRoutingP.nc" .
cp "$SHARED_DIR/SRTreeAppC.nc" .
cp "$SHARED_DIR/SRTreeC.nc" .
cp "$SHARED_DIR/create_topology.py" .
cp "$SHARED_DIR/mySimulation.py" .
cp "$SHARED_DIR/test_suite.py" .

rm -f fail_*.log "$SHARED_DIR"/fail_*.log
make micaz sim || exit 1

python ./test_suite.py | tee test_results.txt
cp test_results.txt topology.txt "$SHARED_DIR/"
cp -f fail_*.log "$SHARED_DIR/" 2>/dev/null || true

