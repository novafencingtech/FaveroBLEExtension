#include <Arduino.h>

#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "esp_event.h"

#include <string.h>
#include <Preferences.h>
#include <list>

//#include <SoftwareSerial.h>
#include "FA05_ESPNOW_Library.h"
#include <GSA_timer.h>
#include <espNowDevice.h>

const uint8_t rxPin = 21;
const uint8_t txPin = 23;

const gpio_num_t BUTTON_PIN = GPIO_NUM_33;
const gpio_num_t LED_PIN = GPIO_NUM_2;

uint8_t buffer1[2 * PACKET_SIZE];
uint8_t lastPacket[PACKET_SIZE];
uint8_t currentState[PACKET_SIZE];
uint32_t packetCount = 0; //Data should be a 30 data frames/sec, so 10sec heart beat

// set up a new serial object
//SoftwareSerial fa05SerialIn(rxPin, txPin);
HardwareSerial fa05SerialIn(1);

Preferences kvStore;

const int LED_FAST_BLINK = 350;
const int LED_SLOW_BLINK = 1200;

const unsigned long tShortPress = 100;   //ms-Short button press
const unsigned long tLongPress = 1000;   //ms - Long button press
const unsigned long tDoubleClick = 1000; //ms -- for two clicks

const uint8_t MAX_NUM_STRIPS = 25;
uint8_t activeStrip = 00;
bool stripChanged = false;

fa05EspData espData;

unsigned long fa05LastPacket = 0;
unsigned long fa05LastActive = 0;
bool fa05SendingData = false;
uint32_t tLastScoreChange = 0;
unsigned long tPairingActive = 0;

const uint8_t maxClients = 6;
uint8_t numConnected = 0;
const uint8_t broadcast_all_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const char fa05DeviceMfr[8] = "GSAllen";
char txDeviceName[12] = "NoVA_";
bool updatePairedDevices = false;
uint8_t myMACAddress[ESP_NOW_ETH_ALEN];
fa05PairMsg beaconMsg;
fa05PairMsg pairBroadcastMsg;
Timer pairingTimer(FA05_PAIRING_TIMEOUT);
Timer heartBeatTimer(FA05_HEART_BEAT_INTERVAL);
Timer fa05OffTimer(FA05_OFF_TIMEOUT);
Timer fa05IdleTimer(FA05_IDLE_TIMEOUT);

wifi_config_t apConfig = {
    .ap = {.ssid = {},
           .password = {},
           .ssid_len = 12,
           .channel = FA05_WIFI_CHANNEL,
           .authmode = WIFI_AUTH_WPA2_PSK,
           .ssid_hidden = 0,
           .max_connection = maxClients,
           .beacon_interval = 1000}};

bool deviceConnected = false;
bool pairingEnabled = false;
bool rxFound = false;
bool rxRequest = false;
bool pairRequest = false;
unsigned long tRxFound = 0;
uint8_t rxMacAddr[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t rxPeerInfo = {
    .peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    .lmk = {},
    .channel = FA05_WIFI_CHANNEL,
    .ifidx = ESPNOW_WIFI_IF,
    .encrypt = false,
    .priv = nullptr};

std::list<espNowDevice *> connectedDevices;
//std::list<espNowDevice *>::iterator deviceIterator;

char txtBuf[128]="";

static uint8_t hexToUint8(uint8_t hexValue) {
  char buf[4];
  uint8_t decValue=0;

  //Serial.print("Hex value: "); Serial.println(hexValue,HEX);
  sprintf(buf,"%x",hexValue);
  //Serial.print("String value: "); Serial.println(buf);  
  sscanf(buf,"%u",&decValue);
  //Serial.print("Dec value: "); Serial.println(decValue);
  return decValue;
}

bool macEquals(const uint8_t *mac1, const uint8_t *mac2)
{
  for (int k = 0; k < ESP_NOW_ETH_ALEN; k++)
  {
    if (mac1[k] == mac2[k])
    {
      //do nothing
    }
    else
    {
      return false;
    }
  }
  return true;
}

void macAddressToChar(char *buf, const uint8_t *addr)
{
  snprintf(buf, (ESP_NOW_ETH_ALEN)*3, "%02x:%02x:%02x:%02x:%02x:%02x",
           addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

bool deviceIsConnected(const uint8_t *addr)
{
  std::list<espNowDevice *>::iterator it;
  for (it = connectedDevices.begin(); it != connectedDevices.end(); it++)
  {
    if (macEquals((*it)->mac, addr))
    {
      return true;
    }
  }
  return false;
}

espNowDevice *getConnectedDevice(const uint8_t *addr)
{
  std::list<espNowDevice *>::iterator it;
  for (it = connectedDevices.begin(); it != connectedDevices.end(); it++)
  {
    if (macEquals((*it)->mac, addr))
    {
      //sprintf(txtBuf,"Device found @ %p",*it);      
      //Serial.println(txtBuf);
      return (*it);
    }
  }
  return NULL;
}

void incomingDataCB(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  fa05PairMsg *pairMsg;
  espNowDevice *sender=NULL;
  char buf[32];

  uint8_t msgType = FA05_MSG_NULL;

  Serial.println("Message incoming");
  if (data_len > 0)
  {
    msgType = data[0];
  }

  sender = getConnectedDevice(mac_addr);
  //sprintf(txtBuf,"sender  @ %p, NULL=%p",sender,NULL);      
  //Serial.println(txtBuf);
  if (sender == NULL)
  {
    //Serial.println("Message from unconnected sender");
  }

  if (msgType == FA05_KEEP_ALIVE)
  {
    if (sender != NULL)
    {
      macAddressToChar(buf,mac_addr);
      sprintf(txtBuf,"[%lu] Client alive: %s",millis(),buf);
      Serial.println(txtBuf);
      if (macEquals(sender->mac,mac_addr)) {
        sender->lastSeen(millis());
      } else {
        Serial.println("Error finding MAC address");
      }

    }
    else
    {
      macAddressToChar(buf,mac_addr);
      sprintf(txtBuf,"[%lu] Device found, adding: %s",millis(),buf);
      Serial.println(txtBuf);
      espNowDevice *sender = new espNowDevice(mac_addr);
      sender->lastSeen(millis());
      connectedDevices.push_back(sender);
    }
  }

  if (msgType == FA05_UNPAIR_REQ)
  {
    pairMsg = (fa05PairMsg *)data;
    esp_now_del_peer(mac_addr);
    if (sender != NULL)
    {
      delete sender;
      connectedDevices.remove(sender);
    }
    //Serial.println("Drop request");
  }

  if ((msgType == FA05_CONNECT_REQ) || (msgType == FA05_PAIR_REQUEST))
  {
    if (msgType==FA05_PAIR_REQUEST) {
      pairRequest=true;
    }
    pairMsg = (fa05PairMsg *)data;
    //Serial.println("Connection request");
    macAddressToChar(buf,mac_addr);
    sprintf(txtBuf,"[%lu] Connection request from: %s",millis(),buf);
    Serial.println(txtBuf);
    if (sender == NULL)
    {
      //Serial.println("Adding connected device");
      espNowDevice *newDevice = new espNowDevice(pairMsg->senderMAC);
      sprintf(txtBuf,"Device created @ %p",newDevice);      
      Serial.println(txtBuf);
      newDevice->lastSeen(millis());
      connectedDevices.push_front(newDevice);
      //Serial.print("Adding device, count = ");
      //Serial.println(connectedDevices.size());
    } else {
      sender->lastSeen(millis());
    }
    memcpy(rxMacAddr, mac_addr, ESP_NOW_ETH_ALEN);
    rxRequest = true;
  }
}

void doConnect()
{
  fa05PairMsg awkMsg;
  bool success = false;
  espNowDevice *device;
  esp_err_t err;
  char buf[64];
  std::list<espNowDevice *>::iterator it;

  for (it = connectedDevices.begin(); it != connectedDevices.end(); it++)
  {
    device = *it;
    if (millis() - device->getLastSeen() > FA05_CLIENT_TIMEOUT)
    {
      Serial.println("Client timed out -- dropping");
      if (!macEquals(device->mac, broadcast_all_mac))
      {
        esp_now_del_peer(device->mac);
      }
      delete device;
      connectedDevices.remove(device);
    }
    esp_now_peer_info_t info;
    if (esp_now_get_peer(device->mac, &info) == ESP_ERR_NOT_FOUND)
    {
      //Serial.println("Not peer device found -- adding to peers");
      memcpy(device->peerInfo.peer_addr, device->mac, ESP_NOW_ETH_ALEN);
      esp_now_add_peer(&(device->peerInfo));
    }
  }

  if (rxRequest)
  {
    if (esp_now_is_peer_exist(rxMacAddr))
    {
      success = true;
    }
    else
    {
      memcpy(rxPeerInfo.peer_addr, rxMacAddr, ESP_NOW_ETH_ALEN);
      esp_err_t err=esp_now_add_peer(&rxPeerInfo);
      if ( (err == ESP_OK) || (err==ESP_ERR_ESPNOW_EXIST) )
      {       
        success = true;
      }
      else
      {        
        success = false;
      }
    }
  }
  if (success)
  {
    rxRequest = false;
    if (pairRequest && pairingEnabled)
    {
      awkMsg.msgType = FA05_PAIR_AWK;
      pairRequest = false;
      pairingEnabled=false;
      pairingTimer.stop();
    }
    else
    {
      awkMsg.msgType = FA05_CONNECT_AWK;
    }
    //Serial.println("Sending connection AWK");
    memcpy(awkMsg.senderMAC, myMACAddress, ESP_NOW_ETH_ALEN);
    memcpy(awkMsg.targetMAC, rxMacAddr, ESP_NOW_ETH_ALEN);
    err=esp_now_send(rxMacAddr, (uint8_t *)&awkMsg, sizeof(fa05PairMsg));
    if (err==ESP_OK) {
      //Serial.println("AWK sent");
    } else {
      //Serial.println("AWk failed");
    }
  }
}

void sendCallBack(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

void espNowInit()
{
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
  esp_err_t err = esp_wifi_start();
  switch (err)
  {
  case ESP_OK:
    Serial.println("WiFi started");
    break;
  case ESP_ERR_WIFI_NOT_INIT:
    Serial.println("Wifi not init");
    break;
  case ESP_ERR_WIFI_CONN:
    Serial.println("Wifi fail, bad control block");
    break;
  default:
    Serial.println("Wifi fail");
    break;
  }

  ESP_ERROR_CHECK(esp_wifi_get_mac(ESPNOW_WIFI_IF, myMACAddress));

  esp_wifi_set_ps(WIFI_PS_NONE);
  //esp_wifi_set_channel(FA05_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  //strcpy((char *)apConfig.ap.ssid, txDeviceName);
  //strcpy((char *)apConfig.ap.password, FA05_WIFI_PASSWORD);

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
  ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif

  //esp_wifi_set_config(ESP_IF_WIFI_AP, &apConfig);
  ESP_ERROR_CHECK(esp_now_init());

  esp_now_register_recv_cb(incomingDataCB);
  esp_now_register_send_cb(sendCallBack);

  pairBroadcastMsg.msgType = FA05_PAIR_BROADCAST_SERVER;
  memcpy(pairBroadcastMsg.senderMAC, myMACAddress, ESP_NOW_ETH_ALEN);
  beaconMsg.msgType = FA05_SERVER_BROADCAST;
  memcpy(beaconMsg.senderMAC, myMACAddress, ESP_NOW_ETH_ALEN);

  esp_now_add_peer(&rxPeerInfo);
}

void setup()
{
  Serial.begin(115200);
  pairingTimer.stop();

  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);
  gpio_pullup_en(BUTTON_PIN);

  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  kvStore.begin("FA05-Tx");
  activeStrip = kvStore.getUChar("StripNum");
  sprintf(txDeviceName, "NoVA_%u", activeStrip);

  Serial.println("Initializing Wifi");
  espNowInit();

  fa05SerialIn.begin(2400,SERIAL_8N1,rxPin,txPin);
}

void checkButtonState()
{
  const unsigned long LOCK_OUT = 1000; //Don't allow button pushes faster than X.
  const int DEBOUNCE_TIME = 20;
  static bool lastState = HIGH;
  static bool newState = HIGH;
  bool valid = false;
  static unsigned long tLastPress = 0;
  static unsigned long tPressed = 0;
  static unsigned long tSwitch = 0;
  static unsigned long tLockout = 0;
  static unsigned long tLow = 0;
  static unsigned long tChange = 0;
  unsigned long tNow = millis();

  bool pinState = gpio_get_level(BUTTON_PIN);

  if (newState != pinState)
  {
    if ((tNow - tChange) > DEBOUNCE_TIME)
    {
      valid = true;
      newState = pinState;
    }
    else
    {
      tChange = tNow;
      return;
    }
  }

  if ((newState != lastState) && (valid))
  {
    if ((tNow - tLastPress) > LOCK_OUT)
    {
    }
    if (newState == HIGH)
    { //Button release
      if ((tNow - tSwitch) > tShortPress)
      {
        tLastPress = tNow;
        activeStrip++;
        if (activeStrip > MAX_NUM_STRIPS)
        {
          activeStrip = 0;
        }
        sprintf(txDeviceName, "NoVA_%u", activeStrip);
        stripChanged = true;

        //updateHeartBeat();
        //Serial.println("Short press");
        //Short button press
      }
    }
    tSwitch = tNow;
    lastState = newState;
  }

  if ((newState == LOW) && ((tNow - tSwitch) > tLongPress))
  {
    //Long button push action
    gpio_set_level(LED_PIN, HIGH);
    while (gpio_get_level(BUTTON_PIN) == LOW)
    {
      delay(50); //Loop until released
    }
    gpio_set_level(LED_PIN, LOW);
    tLastPress = millis();
    lastState = HIGH;
    tSwitch = millis();
    pairingTimer.reset(); //Activates pairing mode
    //delay(200);
    //Serial.println("Long press");
    //tPairingActive=millis();
  }
}

void lightsUpdate() {
  espData.lights.offTargetL = bitRead(currentState[fa05LightPkt], 0);
  espData.lights.offTargetR = bitRead(currentState[fa05LightPkt], 1);
  espData.lights.touchL = bitRead(currentState[fa05LightPkt], 2);
  espData.lights.touchR = bitRead(currentState[fa05LightPkt], 3);
  espData.lights.gndFaultR = bitRead(currentState[fa05LightPkt], 4);
  espData.lights.gndFaultL = bitRead(currentState[fa05LightPkt], 5);
  espData.lights.lightsOnly = ((millis() - tLastScoreChange) > SCORE_IDLE_TIMEOUT) ? 1 : 0;
}

void scoreUpdate() {
  static uint8_t oldPacket=0;
  const uint8_t id=fa05FlashPkt;
  char buf[64];
  #define getBit(val, bit) (val & (bit<<1))==0 ? 0 : 1
  
  espData.score.scoreL = hexToUint8(currentState[fa05ScoreLeftPkt]);
  espData.score.scoreR = hexToUint8(currentState[fa05ScoreRightPkt]);
  espData.score.timeRemainMin = hexToUint8(currentState[fa05TimeMinPkt]);
  espData.score.timeRemainSec = hexToUint8(currentState[fa05TimeSecPkt]);
  //Serial.println(currentState[fa05TimeSecPkt]);
  espData.score.matchCount = hexToUint8(currentState[fa05PriorityPkt] & (0x3)); //Only the first two bits are the match count
  espData.score.redR = bitRead(currentState[fa05CardPkt], 0);
  espData.score.redL = bitRead(currentState[fa05CardPkt], 1);
  espData.score.yellowR = bitRead(currentState[fa05CardPkt], 2);
  espData.score.yellowL = bitRead(currentState[fa05CardPkt], 3);
  espData.score.clockRunning = bitRead(currentState[fa05FlashPkt], 0);
  //espData.score.clockRunning = getBit(currentState[fa05FlashPkt], 0);
  //espData.score.clockRunning = 1;
  espData.score.clockIdle = ((millis() - tLastScoreChange) > SCORE_IDLE_TIMEOUT) ? 1 : 0;
  espData.score.priorityR = bitRead(currentState[fa05PriorityPkt], 2);
  espData.score.priorityL = bitRead(currentState[fa05PriorityPkt], 3);
  //espData.score.rawBytes[5]=0xFF;

  /*if (currentState[id]!=oldPacket) {
    Serial.print("Packet changed:  old = "); Serial.print(oldPacket,BIN); Serial.print(" new = "); 
    Serial.println(currentState[id],BIN);
    for (uint8_t k = 0; k<8; k++) {
      sprintf(buf,"Bit #%u = %u",k,bitRead(currentState[id],k));
      Serial.println(buf);
    }
    oldPacket=currentState[id];

    Serial.print("matchBLE = ");
    for (uint8_t k = 0; k<matchDataBLEPacketSize+2; k++) {
      Serial.print(espData.score.rawBytes[k],HEX);Serial.print("-");
    }
    Serial.println("");
  }*/    
}

void heartBeatUpdate() {
  //static unsigned long lastUpdate=0;
  
  espData.status.upTimeSec = millis() / 1000UL;
  espData.status.isActive = !(fa05IdleTimer.isElapsed() || fa05OffTimer.isElapsed());
  espData.status.stripNum = activeStrip;
  //lastUpdate=millis();
  //Serial.print("Strip #"); Serial.println(activeStrip);
  //Serial.println("Heart-beat sent");
}

void updateClients()
{
  esp_err_t err;
  static uint32_t outgoingCount = 0;
  esp_now_peer_num espStats;
  esp_now_peer_info_t peerDev;
  espNowDevice *rxClient;
  std::list<espNowDevice *>::iterator it;
  char buf[24];

  esp_now_get_peer_num(&espStats);
  numConnected = connectedDevices.size(); //Broadcast all should always be a peer
  if (numConnected != (espStats.total_num - 1))
  {
    //Serial.println("Error peer devices mis-match");
    //Serial.print("List of devices=");
    //Serial.print(connectedDevices.size());
    //Serial.print("\t Num peers = ");
    //Serial.println(espStats.total_num);
  }

  if (connectedDevices.size() == 0)
  {
    return;
  }

  outgoingCount++;
  espData.msgID = outgoingCount;

  for (it = connectedDevices.begin(); it != connectedDevices.end(); it++)
  {
    rxClient = *it;
    //Serial.print("Sending to:");
    macAddressToChar(buf, rxClient->mac);
    //Serial.println(buf);
    err = esp_now_send(rxClient->mac, (uint8_t *)&espData, sizeof(espData)); //Sends to all peers
    if (err == ESP_OK)
    {
      //Serial.println("Send success");
    }
    else
    {
      //Serial.println("Send Error");
    }
  }
}

void updateData() {
  unsigned long tNow = millis();
  static unsigned long tLights = 0;
  static unsigned long tScore = 0;
  bool sendUpdate = false;
  //static unsigned long tic, toc;
  static Timer lightsTimer(FA05_LIGHTS_INTERVAL);

  for (int k = 0; k < PACKET_SIZE; k++) {
    if (currentState[k] != lastPacket[k]) {
      //packetChange = true;
      //fa05LastActive = tNow;
      fa05IdleTimer.reset();
      if (espData.status.isActive==false) {
        espData.status.isActive=true;
        //updateHeartBeat();
      }
      switch (k) {
        case fa05LightPkt:
          sendUpdate = true;
          break;
        case fa05ScoreRightPkt:
        case fa05ScoreLeftPkt:
        case fa05TimeSecPkt:
        case fa05TimeMinPkt:
        case fa05PriorityPkt:
        case fa05FlashPkt:
        case fa05CardPkt:
          tLastScoreChange = millis();
          sendUpdate = true;          
          break;
        case fa05ChkSumPkt:
          if (fa05SendingData) {
            sendUpdate=true;
            //sendRawData();
          }
          break;
      }
      /*Serial.println("Change");
        Serial.print("Old Byte #"); Serial.print(k); Serial.print("\t"); Serial.print(lastPacket[k], HEX); Serial.print("\t"); Serial.println(lastPacket[k], BIN);
        Serial.print("New Byte #"); Serial.print(k); Serial.print("\t"); Serial.print(newPacket[k], HEX); Serial.print("\t"); Serial.println(newPacket[k], BIN);
        Serial.print("Check Sum:"); Serial.print("\t"); Serial.print(chkSum, HEX); Serial.print("\t"); Serial.print(chkSum, BIN);
      */
    }
  }

  if (lightsTimer.isElapsed()) {
    sendUpdate = true;
  }

  if (heartBeatTimer.isElapsed()) {
    //tic=millis();
    //updateHeartBeat();
    //sendRawData();
    sendUpdate=true;
    heartBeatTimer.reset();
    //uint8_t oldStrip=kvStore.getUChar("StripNum");
    if (stripChanged) {
      //fa05SerialIn.enableRx(false);
      kvStore.putUChar("StripNum",activeStrip);
      stripChanged=false;
      //fa05SerialIn.enableRx(true);
    }
    //toc=millis();
    //Serial.print("BLE Update time = "); Serial.print(toc-tic); Serial.println(" ms");
  }
  if (sendUpdate) {
    //Serial.println("Sending updates");
    lightsUpdate();
    scoreUpdate();
    heartBeatUpdate();
    updateClients();
    lightsTimer.reset();
  }
}

void checkForDataPacket() {
  bool packetDone = false;
  bool packetChange = false;
  static uint8_t indx = 0;
  static bool startFound = false;
  //static uint8_t packetCount = 0;
  uint8_t incData, chkSum;

  /*if (fa05SerialIn.overflow()) {
    //Serial.println("Serial port overflow detected");
  }*/

  if (fa05SerialIn.available() > 0) {
    if (startFound == false) {
      if (fa05SerialIn.peek() == PACKET_START_BYTE) {
        //Serial.println("Start found");
        indx = 0;
        buffer1[indx] = fa05SerialIn.read();
        indx++;
        startFound = true;
        packetDone = false;
        //chkSum = buffer1[indx];
      } else {
        incData = fa05SerialIn.read();
      }
    }
    if (startFound == true) {
      while (fa05SerialIn.available() > 0) {
        if (fa05SerialIn.peek() == PACKET_START_BYTE) {
          //Serial.println("Packet done");
          //chkSum=chkSum-buffer1[indx-1];
          packetDone = true;
          startFound = false;
          /*for (int k = 0; k < PACKET_SIZE; k++) {
            Serial.print("Byte #"); Serial.print(k); Serial.print("\t"); Serial.print(buffer1[k], HEX); Serial.print("\t"); Serial.println(buffer1[k], BIN);
            }*/
          break;
        }
        incData = fa05SerialIn.read();
        //chkSum = chkSum + incData;
        buffer1[indx] = incData;
        indx++;
      }
    }
  }

  if (packetDone) {
    chkSum = 0;
    for (int k = 0; k < PACKET_SIZE - 1; k++) {
      chkSum += buffer1[k];
    }
    if (chkSum == buffer1[PACKET_SIZE - 1]) {
      for (int k = 0; k < PACKET_SIZE; k++) {
        currentState[k] = buffer1[k];
      }
      //packetChange = false;
      fa05SendingData = true;
      fa05OffTimer.reset();
      fa05LastPacket = millis();
      espData.status.packetCount++;
      //statusBLE.errCode = fa05err_NoError;
      updateData();
      for (int k = 0; k < PACKET_SIZE; k++) {
        lastPacket[k] = currentState[k];
      }
      packetDone = false;
    } else { //check sum failed, assume that we're lost in mid-stream
      packetDone = false;
      espData.status.errCount++;
      espData.status.errCode = fa05err_BadPacket;
      /*Serial.println("Bad check sum");
        Serial.print("ChkSum expt #"); Serial.print("\t"); Serial.print(chkSum, HEX); Serial.print("\t"); Serial.println(chkSum, BIN);
        Serial.print("ChkSum act #"); Serial.print("\t"); Serial.print(newPacket[PACKET_SIZE - 1], HEX); Serial.print("\t"); Serial.println(newPacket[PACKET_SIZE - 1], BIN);
        startFound = false;
        //endFound = false;
        for (int k = 0; k < PACKET_SIZE; k++) {
        Serial.print("Byte #"); Serial.print(k); Serial.print("\t"); Serial.print(newPacket[k], HEX); Serial.print("\t"); Serial.println(newPacket[k], BIN);
        }*/
    } //end ChkSum loop
  }

  if (fa05OffTimer.isElapsed()) {
    fa05SendingData = false;
    espData.status.errCode = fa05err_NoData;
    espData.status.isActive = false;
  }

  if (heartBeatTimer.isElapsed()) {
    updateData();
  }
}

/*void updateData()
{
  static unsigned long tLast = 0;

  if (millis() - tLast > 1000)
  {
    espData.status.upTimeSec = millis() / 1000;
    updateClients();
    tLast = millis();
  }
}*/

void updatePairing()
{
  static Timer pairTimer(FA05_PAIR_BROADCAST_INTERVAL);
  static Timer bcastTimer(FA05_BEACON_BROADCAST_INTERVAL);

  if (bcastTimer.isElapsed())
  {
    //Serial.print("Num devices=");
    //Serial.println(connectedDevices.size());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_now_send(broadcast_all_mac, (uint8_t *)&beaconMsg, sizeof(beaconMsg)));
    bcastTimer.reset();
  }

  //To start pairing reset the timer
  pairingEnabled = !(pairingTimer.isElapsed());

  if (pairingEnabled)
  {
    if (pairTimer.isElapsed())
    {
      esp_now_send(broadcast_all_mac, (uint8_t *)&pairBroadcastMsg, sizeof(pairBroadcastMsg));
      pairTimer.reset();
    }
  }
}

void loop()
{
  static Blinker blinkLED(LED_FAST_BLINK, 30);
  static Blinker blinkLEDslow(LED_SLOW_BLINK, 30);
  // put your main code here, to run repeatedly:

  checkButtonState();

  checkForDataPacket();

  updatePairing();

  doConnect();

  if (pairingEnabled)
  {
    gpio_set_level(LED_PIN, blinkLED.isOn());
  }
  else
  {
    gpio_set_level(LED_PIN, ((connectedDevices.size() > 0) ? 1 : 0));
  }
}