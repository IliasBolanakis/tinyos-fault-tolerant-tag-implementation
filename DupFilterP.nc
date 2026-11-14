/**
 * @file DupFilterP.nc
 * @brief Private TEP-3 module implementing per-epoch child duplicate filtering.
 */
#include "SimpleRoutingTree.h"

module DupFilterP @safe() {
    provides interface DupFilter;
}
implementation {
    uint16_t rxChildID[MAX_TRACKED_CHILDREN];
    uint32_t rxChildSig[MAX_TRACKED_CHILDREN];
    uint32_t rxChildTime[MAX_TRACKED_CHILDREN];
    uint8_t rxChildUsed = 0;

    command void DupFilter.reset() {
        rxChildUsed = 0;
    }

    command bool DupFilter.isDuplicate(uint16_t childID, uint32_t sig, uint32_t now) {
        uint8_t i;
        for (i = 0; i < rxChildUsed; i++) {
            if (rxChildID[i] == childID) {
                if (rxChildSig[i] == sig && (now - rxChildTime[i]) < DUP_RETRY_WINDOW_MILLIS) {
                    rxChildTime[i] = now;
                    return TRUE;
                }
                rxChildSig[i] = sig;
                rxChildTime[i] = now;
                return FALSE;
            }
        }
        if (rxChildUsed < MAX_TRACKED_CHILDREN) {
            rxChildID[rxChildUsed] = childID;
            rxChildSig[rxChildUsed] = sig;
            rxChildTime[rxChildUsed] = now;
            rxChildUsed++;
        }
        return FALSE;
    }
}

