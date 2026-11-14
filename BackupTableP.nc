/**
 * @file BackupTableP.nc
 * @brief Private TEP-3 module implementing the passive backup parent table.
 */
#include "SimpleRoutingTree.h"

module BackupTableP @safe() {
    provides interface BackupTable;
}
implementation {
    uint16_t backupID[MAX_BACKUP_PARENTS];
    uint8_t backupDepth[MAX_BACKUP_PARENTS];

    command void BackupTable.clear() {
        uint8_t i;
        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            backupID[i] = INVALID_NODE_ID;
            backupDepth[i] = DEPTH_UNATTACHED;
        }
    }

    command void BackupTable.updateCandidate(uint16_t nodeID, uint8_t nDepth,
                                             uint16_t activeParentID, uint8_t curDepth) {
        uint8_t i;
        uint8_t worstIdx = 0;
        uint8_t worstDepth = 0;
        bool invalidCand = (nodeID == activeParentID) ||
                           (nDepth >= DEPTH_ROOT_PROBE) ||
                           (curDepth < DEPTH_ROOT_PROBE && nDepth >= curDepth);

        if (nodeID == TOS_NODE_ID) {
            return;
        }

        // Update or evict existing entry if neighbor is already in the table
        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            if (backupID[i] == nodeID) {
                if (invalidCand) {
                    backupID[i] = INVALID_NODE_ID;
                    backupDepth[i] = DEPTH_UNATTACHED;
                } else {
                    backupDepth[i] = nDepth;
                }
                return;
            }
        }

        if (invalidCand) {
            return;
        }

        // Insert into first available slot while tracking worst candidate for eviction
        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            if (backupID[i] == INVALID_NODE_ID) {
                backupID[i] = nodeID;
                backupDepth[i] = nDepth;
                return;
            }
            if (backupDepth[i] > worstDepth) {
                worstDepth = backupDepth[i];
                worstIdx = i;
            }
        }

        // Replace worst entry if new candidate is strictly closer to the Root
        if (nDepth < worstDepth) {
            backupID[worstIdx] = nodeID;
            backupDepth[worstIdx] = nDepth;
        }
    }

    command void BackupTable.pruneDeeper(uint8_t curDepth) {
        uint8_t i;
        if (curDepth >= DEPTH_UNATTACHED) {
            return;
        }
        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            if (backupID[i] != INVALID_NODE_ID && backupDepth[i] >= curDepth) {
                backupID[i] = INVALID_NODE_ID;
                backupDepth[i] = DEPTH_UNATTACHED;
            }
        }
    }

    command void BackupTable.remove(uint16_t deadNodeID) {
        uint8_t i;
        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            if (backupID[i] == deadNodeID) {
                backupID[i] = INVALID_NODE_ID;
                backupDepth[i] = DEPTH_UNATTACHED;
            }
        }
    }

    command uint16_t BackupTable.selectBest(uint8_t curDepth, uint8_t* outNeighborDepth) {
        uint8_t i;
        uint16_t bestID = INVALID_NODE_ID;
        uint8_t bestD = DEPTH_UNATTACHED;

        for (i = 0; i < MAX_BACKUP_PARENTS; i++) {
            if (backupID[i] != INVALID_NODE_ID &&
                backupDepth[i] < DEPTH_ROOT_PROBE &&
                (curDepth >= DEPTH_ROOT_PROBE || backupDepth[i] < curDepth)) {
                if (backupDepth[i] < bestD || (backupDepth[i] == bestD && backupID[i] < bestID)) {
                    bestID = backupID[i];
                    bestD = backupDepth[i];
                }
            }
        }
        if (outNeighborDepth != NULL) {
            *outNeighborDepth = bestD;
        }
        return bestID;
    }
}

