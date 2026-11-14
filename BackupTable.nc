/**
 * @file BackupTable.nc
 * @brief Interface for passive backup parent table management in TAG routing.
 *
 * Maintains up to `MAX_BACKUP_PARENTS` strictly shallower upstream neighbor
 * candidates (`neighborDepth < curDepth`) discovered passively from routing
 * advertisements, allowing rapid local failover without forming loops.
 */
#include "SimpleRoutingTree.h"

interface BackupTable {
    /**
     * @brief Clears all entries in the backup parent table.
     */
    command void clear();

    /**
     * @brief Records, updates, or evicts a neighbor candidate based on its
     *        advertised depth relative to the local node's current parent and depth.
     *
     * @param nodeID         Neighbor node ID.
     * @param nDepth         Neighbor's advertised tree depth (or control code).
     * @param activeParentID Current primary parent ID (excluded from backup slots).
     * @param curDepth       Local node's current tree depth.
     */
    command void updateCandidate(uint16_t nodeID, uint8_t nDepth,
                                 uint16_t activeParentID, uint8_t curDepth);

    /**
     * @brief Evicts any backup candidates at or deeper than `curDepth`.
     *
     * @param curDepth Local node's current tree depth.
     */
    command void pruneDeeper(uint8_t curDepth);

    /**
     * @brief Removes a dead or invalidated neighbor from the backup table.
     *
     * @param deadNodeID Node ID to evict.
     */
    command void remove(uint16_t deadNodeID);

    /**
     * @brief Selects the best strictly shallower backup candidate
     *        (minimum depth `< curDepth`, lowest node ID tie-break).
     *
     * @param curDepth         Local node's current tree depth.
     * @param outNeighborDepth Output pointer receiving the chosen neighbor's depth
     *                         (`DEPTH_UNATTACHED` if no valid candidate exists).
     * @return Selected backup parent node ID, or `INVALID_NODE_ID` if none.
     */
    command uint16_t selectBest(uint8_t curDepth, uint8_t* outNeighborDepth);
}

