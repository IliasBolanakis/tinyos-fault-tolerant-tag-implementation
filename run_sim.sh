#!/bin/bash
# Path to the shared project folder on your machine (update to match your setup)
SHARED_DIR="$HOME/Desktop/tinyos_SHARED/project1"

cd /home/tinyos/local/src/tinyos-2x/apps/tinyOS

# 1. Copy the restored queue files into the VM workspace
cp "$SHARED_DIR/PacketQueue.nc" .
cp "$SHARED_DIR/PacketQueueC.nc" .

# 2. Copy the refactored project files into the VM workspace
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

# 3. Clean and compile the TOSSIM simulation binary
make clean
make micaz sim

# 4. Generate the 3x3 topology (D=3, R=1.5) and run the simulation with seed 1
python ./create_topology.py 3 1.5
python ./mySimulation.py 1 | tee output.txt

# 5. Copy the newly generated topology.txt and output.txt back to the shared folder
cp topology.txt output.txt "$SHARED_DIR/"