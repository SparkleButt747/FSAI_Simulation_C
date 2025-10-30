#include "types.h"

/**
 * Translates the control command structure to a CAN message using the API
 * specification in docs/ADS-DV_Software_Interface_Specification_v4.0.pdf
 */
void getCANMessage(FsaiControlCmd controlCmd);