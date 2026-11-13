#!/usr/bin/env python
"""Generates a DxD grid topology file for TOSSIM given diameter D and range R."""
import sys, math

D, R = int(sys.argv[1]), float(sys.argv[2])
with open("topology.txt", "w") as f:
    for i in range(D * D):
        # Map linear ID to 2D grid coordinates: row = i // D, col = i % D
        for j in range(D * D):
            # Check Euclidean distance threshold (dist <= R) and exclude self-loops
            if i != j and math.hypot(i // D - j // D, i % D - j % D) <= R:
                # Format: <src> <dest> <gain_dBm> for the TOSSIM radio model
                f.write("%d %d -50.0\n" % (i, j))