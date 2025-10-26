#include "types.h"

int Control_Init(const char* config_yaml);
int Control_Tick(const VehicleState* s, const ConeDet* dets, ControlCmd* u); // 1=OK, 0=HOLD
void Control_Shutdown(void);