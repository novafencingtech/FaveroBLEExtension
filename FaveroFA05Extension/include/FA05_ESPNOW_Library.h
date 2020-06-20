/* Library for sending Favero FA05 Scoring Machine Data over ESP-NOW to ESP32/ESP8266
Graham Allen
gsallen@gmail.com
June 2020

*/

#ifndef FA05_ESPNOW_H
#define FA05_ESPNOW_H

#include <Arduino.h>
#include <esp_now.h>

#define TO_HEX(i) (i <= 9 ? '0' + i : 'a' - 10 + i)

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA

#define FA05_MSG_NULL 0x0
#define FA05_SERVER_BROADCAST 0x9F
#define FA05_PAIR_BROADCAST_SERVER 0xA0
#define FA05_PAIR_BROADCAST_CLIENT 0xA1
#define FA05_PAIR_REQUEST 0xA2
#define FA05_PAIR_AWK 0xA3
#define FA05_UNPAIR_REQ 0xA2
#define FA05_CONNECT_REQ 0xB0
#define FA05_CONNECT_AWK 0xB1
#define FA05_DATA_UPDATE 0xC0
#define FA05_DATA_AWK 0xC2
#define FA05_KEEP_ALIVE 0xD0
#define FA05_CMD_REQ 0xE0
#define FA05_CMD_AWK 0xE1
#define FA05_CMD_NEG 0xE2
#define FA05_SCROLL_UPDATE 0xF0
#define FA05_SCROLL_AWK 0xF1

const byte PACKET_START_BYTE = 0xFF;
const int PACKET_SIZE = 10;

const uint8_t fa05HeaderPkt = 0;
const uint8_t fa05ScoreRightPkt = 1;
const uint8_t fa05ScoreLeftPkt = 2;
const uint8_t fa05TimeSecPkt = 3;
const uint8_t fa05TimeMinPkt = 4;
const uint8_t fa05LightPkt = 5;
const uint8_t fa05PriorityPkt = 6;
const uint8_t fa05FlashPkt = 7;
const uint8_t fa05CardPkt = 8;
const uint8_t fa05ChkSumPkt = 9;



enum FA05ErrorCodes {fa05err_NoError=0x0,fa05err_BadPacket=0x1A,fa05err_NoData=0xAB};

const uint8_t FA05_PASSWORD_LEN=18;
const char FA05_WIFI_PASSWORD[FA05_PASSWORD_LEN]="7HkBZ+T,Cc@MA-DkE";
const uint8_t FA05_WIFI_CHANNEL=13;

const int FA05_LIGHTS_INTERVAL = 200; //Update the score at least this often, even if no new data (limits dropped packets)
const int FA05_SCORE_INTERVAL = 1200;
const int FA05_HEART_BEAT_INTERVAL = 5 * 1000;
const uint32_t SCORE_IDLE_TIMEOUT = 8 * 60 * 1000;
const uint32_t FA05_OFF_TIMEOUT = 20 * 1000;
const uint32_t FA05_IDLE_TIMEOUT = 10 * 60 * 1000;

const unsigned long FA05_PAIRING_TIMEOUT=45*1000; //ms, Pairing time-out
const unsigned long FA05_PAIR_AWK_TIMEOUT=1500; //ms, Pairing must be awk'ed within this interval
const unsigned long FA05_PAIR_BROADCAST_INTERVAL=200;
const unsigned long FA05_BEACON_BROADCAST_INTERVAL=1500;
const unsigned long FA05_CLIENT_KEEPALIVE_INTERVAL=15*1000;
const unsigned long FA05_CLIENT_TIMEOUT=4*FA05_CLIENT_KEEPALIVE_INTERVAL;
const unsigned long FA05_SERVER_TIMEOUT=6*FA05_HEART_BEAT_INTERVAL;

const uint8_t heartBeatPacketSize = 2*4+4*1;
struct machineStatus_BLE {
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

const uint8_t lightsBLEPacketSize = 1;
struct lightData_BLE {
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
      _Bool lightsOnly: 1;
    };
  };
};

const uint8_t matchDataBLEPacketSize = 6;
struct matchData_BLE {
  union {
    uint8_t rawBytes[matchDataBLEPacketSize];
    struct {
      uint8_t scoreL;
      uint8_t scoreR;
      uint8_t timeRemainMin;
      uint8_t timeRemainSec;
      uint8_t matchCount;
      union {
          uint8_t cardState;
          struct {      
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
  };
};

struct scrollSettings_BLE {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t scrollSpeed;
  uint8_t messageOn;
};

struct fa05PairMsg {
    uint8_t msgType=FA05_PAIR_BROADCAST_SERVER;
    uint8_t targetMAC[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    uint8_t senderMAC[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
};

struct fa05EspData
{
    uint8_t msgType=FA05_DATA_UPDATE;
    uint32_t msgID=0;
    machineStatus_BLE status;
    matchData_BLE score;
    lightData_BLE lights;
};

const uint8_t MAX_SCROLL_LENGTH=100;
struct fa05ScrollMessage {
    uint8_t msgType=FA05_SCROLL_UPDATE;
    scrollSettings_BLE settings;
    char text[MAX_SCROLL_LENGTH];
};

#endif