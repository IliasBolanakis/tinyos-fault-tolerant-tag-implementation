/**
 * @file DupFilter.nc
 * @brief Interface for per-epoch MAC retry duplicate packet filtering.
 *
 * Distinguishes legitimate post-fire child delta updates (which flip the
 * MSB `txPktToggle` bit) from link-layer retransmissions caused by lost ACKs
 * within `DUP_RETRY_WINDOW_MILLIS`.
 */
#include "SimpleRoutingTree.h"

interface DupFilter {
    /**
     * @brief Clears all tracked child packet signatures at epoch boundaries.
     */
    command void reset();

    /**
     * @brief Checks whether a packet from `childID` matches the last observed
     *        signature within `DUP_RETRY_WINDOW_MILLIS`, updating the entry
     *        timestamp/signature if not.
     *
     * @param childID Source node ID of the child packet.
     * @param sig     Compact payload signature (including MSB `txPktToggle`).
     * @param now     Current local millisecond timestamp.
     * @return TRUE if the packet is a duplicate MAC retry to drop; FALSE otherwise.
     */
    command bool isDuplicate(uint16_t childID, uint32_t sig, uint32_t now);
}

