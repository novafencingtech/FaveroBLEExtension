#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include "FA05_BLE_Library.h"

extern machineStatus_BLE fa05Status;
extern lightData_BLE fa05Lights;
extern matchData_BLE fa05Score;

extern bool scoreChanged;
extern bool lightsChanged;
extern bool heartbeatChanged;

extern bool isConnected;

#endif

