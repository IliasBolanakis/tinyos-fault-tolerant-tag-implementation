cd /home/tinyos/local/src/tinyos-2x/apps/tinyOS

# 1. Copy the restored queue files into the VM workspace
cp ~/Desktop/tinyos_SHARED/project1/PacketQueue.nc .
cp ~/Desktop/tinyos_SHARED/project1/PacketQueueC.nc .

# 2. Copy only the 5 modified project files into the VM workspace
cp ~/Desktop/tinyos_SHARED/project1/SimpleRoutingTree.h .
cp ~/Desktop/tinyos_SHARED/project1/SRTreeAppC.nc .
cp ~/Desktop/tinyos_SHARED/project1/SRTreeC.nc .
cp ~/Desktop/tinyos_SHARED/project1/create_topology.py .
cp ~/Desktop/tinyos_SHARED/project1/mySimulation.py .

# 3. Clean and compile the TOSSIM simulation binary
make clean
make micaz sim

# 4. Generate the 3x3 topology (D=3, R=1.5) and run the simulation with seed 1
python ./create_topology.py 3 1.5
python ./mySimulation.py 1 | tee output.txt

# 5. Copy the newly generated topology.txt and output.txt back to the shared folder
cp topology.txt output.txt ~/Desktop/tinyos_SHARED/project1/