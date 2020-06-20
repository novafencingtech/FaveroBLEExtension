// BLE Service UUID 1a93facf-a8c0-40d4-9327-e17f0d4d7d1a
/* UUID List
  94488c25-5726-4627-9efc-f69b8217ce05
  e81f8bfb-cd5e-4d45-90c7-d0ee47b3c91f
  c6b676d8-5c3d-4049-8230-c325261bb385
  cc708eab-8a31-4316-a6bb-d0763b374db1
  cd6929e2-62b3-4e51-84e8-8c6170892e35
  53a81b3e-b1ca-449e-8822-39ef709c2c74
  926a3a69-515a-452e-a019-75b986b39a9c
  d0d1922b-bdf0-484c-b54e-c3492065b98e
  b3f258e9-2202-4fb0-bc17-06e96ea5ce62
  8a6021ec-6e42-4d65-9978-8b89ac8cd11e
  358964a8-8441-464e-99c8-7e0888b92e6f
  580917e0-496a-4d5f-86ad-43eb0e8792b5
  6d1bb9e5-0940-444a-a2a1-013b8b427e6a
  45328de2-6d2b-49fc-8e12-0c8e8028d309
*/

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#ifndef FA05_BLE_H
#define FA05_BLE_H

#include <Arduino.h>

#define TO_HEX(i) (i <= 9 ? '0' + i : 'a' - 10 + i)

#define FA05_DISCOVERY_UUID "1a93fac0-a8c0-40d4-9327-e17f0d4d7d1a"
#define FA05_SERVICE_CHAR_UUID "1a93fac1-a8c0-40d4-9327-e17f0d4d7d1a"
#define FA05_DISCOVERY_TIME_UUID "1a93fac2-a8c0-40d4-9327-e17f0d4d7d1a"
#define BASE_FA05_UUID "1a93facf-a8c0-40d4-9327-e17f0d" //Needs to have 6-digit hex from BLE MAC appended
#define RAW_DATA_UUID "732a4ccc-cd60-4a08-821e-302d5868b189"
#define LIGHT_STATE_UUID "cc81c2b6-b01d-4b18-9db9-f4193b5efb1e"
#define MATCH_UUID "94488c25-5726-4627-9efc-f69b8217ce05"
#define HEARTBEAT_UUID "e81f8bfb-cd5e-4d45-90c7-d0ee47b3c91f"
#define SCROLL_TXT_UUID "732a4ccc-cd60-4a08-821e-302d5868b189"
#define SCROLL_SETTINGS_UUID "cc81c2b6-b01d-4b18-9db9-f4193b5efb1e"

typedef enum FA05ErrorCodes {fa05err_NoError=0x0,fa05err_BadPacket=0x1A,fa05err_NoData=0xAB};

const int FA05_PAIRING_TIMEOUT=45*1000; //ms, Pairing time-out

const byte heartBeatPacketSize = 2*4+4*1;
typedef struct machineStatus_BLE {
  union {
    uint8_t rawBytes[heartBeatPacketSize];
    struct {
      uint32_t packetCount;
      uint32_t upTimeSec;
      uint8_t isActive;
	 uint8_t stripNum;
      FA05ErrorCodes errCode;
      uint8_t errCount;
    };
  };
};

const byte lightsBLEPacketSize = 1;
typedef struct lightData_BLE {
  union {
    uint8_t value;
    struct {
      _Bool touchL: 1;
      _Bool offTargetL: 1;
      _Bool gndFaultL: 1;
      _Bool spacerL: 1;
      _Bool touchR: 1;
      _Bool offTargetR: 1;
      _Bool gndFaultR: 1;
      //_Bool startBit: 1;
      _Bool lightsOnly: 1;
    };
  };
};

const byte matchDataBLEPacketSize = 6;
typedef struct matchData_BLE {
  union {
    uint8_t rawBytes[matchDataBLEPacketSize];
    struct {
      uint8_t scoreL;
      uint8_t scoreR;
      uint8_t timeRemainMin;
      uint8_t timeRemainSec;
      uint8_t matchCount;
      unsigned yellowL: 1;
      unsigned redL: 1;
      unsigned yellowR: 1;
      unsigned redR: 1;
      unsigned priorityL: 1;
      unsigned priorityR: 1;
      unsigned clockRunning: 1;
      unsigned clockIdle: 1;
    };
  };
};

typedef struct scrollSettings_BLE {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t scrollSpeed;
  uint8_t messageOn;
};

const uint32_t UUID_seed = 0x4cf0;

#endif