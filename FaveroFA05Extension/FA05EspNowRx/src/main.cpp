#include <Arduino.h>
#include "FA05_ESPNOW_Library.h"

#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "esp_event.h"
#include <queue>

#include <string.h>
#include <Preferences.h>

#include "FA05_ESPNOW_Library.h"
#include "GSA_timer.h"
#include "espNowDevice.h"
#include "FA05Display.h"
#include "eventCounter.h"

#define BUTTON_PIN GPIO_NUM_33
#define LED_PIN GPIO_NUM_2

const unsigned long tSleepTimeout = 2 * 60 * 1000; //Need to be disconnected for 15min before switching to sleep
const unsigned long tSleepInterval = 60 * 1000;    //Wakeup ever 60s to scan for devices if not connected.
const unsigned long idleTimeOut = 10 * 60 * 1000;  //Switch to idle mode if inactive for 15min
const unsigned long tShortPress = 100;             //ms-Short button press
const unsigned long tLongPress = 1000;             //ms - Long button press

unsigned long tLastSeen = 0;   //timer for the last time a heart-beat packet was recieved
unsigned long tLastActive = 0; //timer for the last time the scoring machine changed state
unsigned long tLastScoreChange = 0;

Timer pairConfirm(FA05_PAIR_AWK_TIMEOUT);
Timer pairTimer(FA05_PAIRING_TIMEOUT);
Timer connectionTimeOut(FA05_SERVER_TIMEOUT);
Timer keepAliveTimer(FA05_CLIENT_KEEPALIVE_INTERVAL);
Timer sleepTimer(tSleepTimeout);

char txtBuf[128] = "";
uint8_t myMACAddress[ESP_NOW_ETH_ALEN];
//uint8_t txMAC[ESP_NOW_ETH_ALEN] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
espNowDevice *txDevice = NULL;
fa05PairMsg pairMsg;      //Message for pairing
fa05PairMsg keepAliveMsg; //Message for pairing
fa05EspData *lastPacket=NULL;
fa05EspData *newPacket=NULL;
std::queue<fa05EspData *> packetQueue;
std::queue<uint8_t> incomingMessages;

eventCounter droppedPackets;
eventCounter missedPackets;
eventCounter totalPackets;

volatile bool updateComplete = true;
volatile bool lightsChanged = false;
volatile bool scoreChanged = false;

Preferences kvStore;

FA05Display RGBMatrix;

enum RX_STATE
{
  Sleep,
  Idle,
  Active,
  LightsOnly,
  Unpaired
};
RX_STATE rxModuleStatus = Sleep;

esp_now_peer_info_t txPeerInfo = {
    .peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    .lmk = {},
    .channel = FA05_WIFI_CHANNEL,
    .ifidx = ESPNOW_WIFI_IF,
    .encrypt = false,
    .priv = nullptr};
volatile bool txFound = false;
volatile bool isPaired = false;
volatile bool isConnected = false;
volatile bool needsAwk = false;
volatile bool pairingEnabled = false;
volatile bool updatePrefs = false;

bool checkScoreChanged()
{
  if (lastPacket==NULL) {
    return true;
  }
  if (lastPacket->score.scoreL != newPacket->score.scoreL)
  {
    return true;
  }
  if (lastPacket->score.scoreR != newPacket->score.scoreR)
  {
    return true;
  }
  if (lastPacket->score.timeRemainMin != newPacket->score.timeRemainMin)
  {
    return true;
  }
  if (lastPacket->score.timeRemainSec != newPacket->score.timeRemainSec)
  {
    return true;
  }
  if (lastPacket->score.matchCount != newPacket->score.matchCount)
  {
    return true;
  }
  if (lastPacket->score.cardState != newPacket->score.cardState)
  {
    return true;
  }
  return false;
}

bool checkLightsChanged()
{
  if (lastPacket==NULL) {
    return true;
  }
  if (lastPacket->lights.value != newPacket->lights.value)
  {
    Serial.println("Lights changed");
    return true;
  }
  return false;
}

bool checkMachineStateChanged()
{
  if (lastPacket==NULL) {
    return true;
  }
  if (newPacket->status.isActive != lastPacket->status.isActive)
  {
    return true;
  }
  if (newPacket->status.stripNum != lastPacket->status.stripNum)
  {
    return true;
  }
  return false;
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

void incomingDataCB(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  char buf[20];
  uint8_t msgType = FA05_MSG_NULL;
  static uint32_t lastMsgId = 0;

  //Serial.println("Data recieved.");

  if (data_len > 0)
  {
    msgType = data[0];
  }

  if (msgType == FA05_SERVER_BROADCAST)
  {
    //macAddressToChar(buf, mac_addr);
    //sprintf(txtBuf, "[%lu] Broadcast message from: %s", millis(), buf);
    //macAddressToChar(buf, mac_addr);
    //Serial.println(txtBuf);
    if (isPaired && txDevice != NULL)
    {
      if (macEquals(txDevice->mac, mac_addr))
      {
        //Serial.println("Server found: ");
        txFound = true;
        sleepTimer.reset();
      }
      incomingMessages.push(msgType);
    }
  }

  if (msgType == FA05_PAIR_BROADCAST_SERVER)
  {
    incomingMessages.push(msgType);
    if (pairingEnabled && !txFound)
    {
      //Serial.println("TX Found");
      //macAddressToChar(buf, mac_addr);
      //Serial.println(buf);
      if (txDevice != NULL)
      {
        delete txDevice;
      }
      txDevice = new espNowDevice(mac_addr);
      txFound = true;
      sleepTimer.reset();
    }
  }

  if ((msgType == FA05_DATA_UPDATE) && (txDevice != NULL))
  {
    fa05EspData *packet;
    //char buffer1[20];
    //char buffer2[20];
    
    //macAddressToChar(buffer1,mac_addr);
    //macAddressToChar(buffer2,txDevice->mac);
    //sprintf(txtBuf,"[%lu] Data Packet from %s expecting %s",millis(),buffer1,buffer2);
    //Serial.println(txtBuf);
    if (macEquals(txDevice->mac, mac_addr) && (data_len == sizeof(fa05EspData)))
    {
      //Serial.println("Data message");
      totalPackets.addItem();
      while (!packetQueue.empty())
      {
        Serial.println("Dropping packet");
        packet = packetQueue.front();
        packetQueue.pop();
        free(packet);
        droppedPackets.addItem();
      }
      connectionTimeOut.reset();
      packet = (fa05EspData *)malloc(sizeof(fa05EspData));
      memcpy(packet, data, data_len);
      packetQueue.push(packet);
      sleepTimer.reset();
      if (packet->msgID - lastMsgId > 1)
      {
        Serial.println("Warning: Missed packet");
        missedPackets.addItem();
        incomingMessages.push(FA05_MSG_NULL);
      }
      lastMsgId = packet->msgID;
      incomingMessages.push(msgType);
      //sprintf(txtBuf, "[%lu] Data packet read:  Sec = %u", millis(), packet->score.timeRemainSec);
      //Serial.println(txtBuf);
    }
  }

  if ((msgType == FA05_CONNECT_AWK) || (msgType == FA05_PAIR_AWK))
  {
    Serial.println("Awk message");
    if (macEquals(txDevice->mac, mac_addr) && (!pairConfirm.isElapsed()))
    {
      Serial.println("Connection confirmed");
      memcpy(keepAliveMsg.targetMAC, txDevice->mac, ESP_NOW_ETH_ALEN);
      needsAwk = false;
      isPaired = true;
      isConnected = true;
      pairingEnabled = false;
      pairTimer.stop();
      pairConfirm.stop();
      updatePrefs = true;
      incomingMessages.push(msgType);
      connectionTimeOut.reset();
      keepAliveTimer.reset();
      sleepTimer.reset();
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
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
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

  ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_STA, myMACAddress));

  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(FA05_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
  ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif

  ESP_ERROR_CHECK(esp_now_init());

  esp_now_register_recv_cb(incomingDataCB);
  esp_now_register_send_cb(sendCallBack);

  pairMsg.msgType = FA05_PAIR_REQUEST;
  memcpy(pairMsg.senderMAC, myMACAddress, ESP_NOW_ETH_ALEN);

  keepAliveMsg.msgType = FA05_KEEP_ALIVE;
  memcpy(keepAliveMsg.senderMAC, myMACAddress, ESP_NOW_ETH_ALEN);
  //esp_now_add_peer(&txPeerInfo); //Add broadcast all
}

void setPairingMode()
{
  fa05PairMsg delMsg;

  Serial.println("Pairing mode");
  pairTimer.reset();

  if (isConnected)
  {
    delMsg.msgType = FA05_UNPAIR_REQ;
    memcpy(delMsg.targetMAC, txDevice->mac, ESP_NOW_ETH_ALEN);
    //esp_now_send(txDevice->mac, (uint8_t *)&delMsg, sizeof(delMsg));
    esp_now_del_peer(txDevice->mac);
    delete txDevice;
    txDevice = NULL;
  }
  updatePrefs = true;
  isConnected = false;
  txFound = false;
  isPaired = false;
  needsAwk = false;
}

void connectToServer()
{
  char buf[20];
  if (isConnected)
  {
    if (connectionTimeOut.isElapsed())
    {
      Serial.println("Connection dropped");
      isConnected = false; //Timer expired and we dropped the connection
      txFound = false;
    }
    if (keepAliveTimer.isElapsed())
    {
      macAddressToChar(buf, txDevice->mac);
      sprintf(txtBuf, "[%lu] Sending keep-alive to: %s", millis(), buf);
      Serial.println(txtBuf);
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_now_send(txDevice->mac, (uint8_t *)&keepAliveMsg, sizeof(keepAliveMsg)));
      keepAliveTimer.reset();
    }
    return;
  }
  //To start pairing reset the timer
  pairingEnabled = !(pairTimer.isElapsed());

  if (pairingEnabled && !isPaired)
  {
    pairMsg.msgType = FA05_PAIR_REQUEST;
  }
  else
  {
    pairMsg.msgType = FA05_CONNECT_REQ;
  }

  if (txFound && (needsAwk == false))
  {
    uint8_t broadcast_all[ESP_NOW_ETH_ALEN]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
      macAddressToChar(buf, txDevice->mac);
      sprintf(txtBuf, "[%lu] Sending connect req to: %s", millis(), buf);
      Serial.println(txtBuf);
    esp_now_add_peer(&(txDevice->peerInfo));
    memcpy(pairMsg.targetMAC, txDevice->mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_now_send((uint8_t *)txDevice->mac, (uint8_t *)&pairMsg, sizeof(pairMsg)));
    //ESP_ERROR_CHECK_WITHOUT_ABORT(esp_now_send(broadcast_all, (uint8_t *)&pairMsg, sizeof(pairMsg)));
    pairConfirm.reset();
    needsAwk = true;
  }
  if ((needsAwk == true) && (pairConfirm.isElapsed()))
  {
    sprintf(txtBuf,"[%lu] Timed out",millis());
    Serial.println(txtBuf);
    txFound = false;
    needsAwk = false;
    esp_now_del_peer(txDevice->mac);
    //delete txDevice;
    //txDevice = NULL;
  }
}

bool checkDataChanged()
{
  bool retVal = false;
  scoreChanged = false;
  lightsChanged = false;

  //sprintf(txtBuf, "[%u] Processing data packet", millis());
  //Serial.println(txtBuf);
  if (checkScoreChanged())
  {
    //sprintf(txtBuf, "[%u] Score Changed:", millis());
    //Serial.println(txtBuf);
    retVal = true;
    scoreChanged = true;
    tLastActive = millis();
  }
  if (checkLightsChanged())
  {
    //sprintf(txtBuf, "[%u] lights Changed:", millis());
    //Serial.println(txtBuf);
    retVal = true;
    lightsChanged = true;
    tLastActive = millis();
  }
  return retVal;
}

void doSleep()
{
  static Timer keepAwake(FA05_SERVER_TIMEOUT);

  if (keepAwake.isElapsed() && sleepTimer.isElapsed())
  {
    Serial.println("Sleep");
    gpio_set_level(LED_PIN, LOW);
    esp_now_deinit();
    esp_wifi_stop();
    RGBMatrix.clearScreen();
    delay(50); //Let events complete before continuing
    gpio_intr_enable(BUTTON_PIN);
    gpio_wakeup_enable(BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_sleep_enable_timer_wakeup(tSleepInterval * 1000); //Set sleep timer for 1s
    esp_light_sleep_start();
    //Wake up happens here
    esp_wifi_start();
    delay(50);
    esp_now_init();
    esp_now_register_recv_cb(incomingDataCB);
    esp_now_register_send_cb(sendCallBack);
    Serial.println("Awake");
    keepAwake.reset();
    //esp_bluedroid_enable();
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO)
    {
      sleepTimer.reset();
    }
    delay(500);
    RGBMatrix.setStatusText("Scan", RGBMatrix.LED_BLUE_MED);
    //Serial.println("Scan start");
    gpio_set_level(LED_PIN, HIGH);
    //scanForDevices();
    //Serial.println("Function return start");
  }
}

//enum RX_STATE {Sleep, Idle, Active, LightsOnly, Unpaired};
void changeSystemState(RX_STATE newState)
{
  switch (newState)
  {
  case (Sleep):
    RGBMatrix.clearScreen();
    RGBMatrix.displayLogo();
    Serial.println("Sleep mode");
    break;
  case (Idle):
    RGBMatrix.clearScreen();
    RGBMatrix.updateIdleScreen(&newPacket->status);
    Serial.println("Idle mode");
    break;
  case (Active):
    RGBMatrix.clearScreen();
    RGBMatrix.updateScore(&(newPacket->score));
    RGBMatrix.updateLights(newPacket);
    Serial.println("Active mode");
    break;
  case (LightsOnly):
    RGBMatrix.clearScreen();
    Serial.println("Lights-only mode");
    RGBMatrix.updateLights(newPacket);
    break;
  case (Unpaired):
    //BLEClient*  pClient  = BLEDevice::createClient();
    RGBMatrix.clearScreen();
    RGBMatrix.displayLogo();
    Serial.println("Unpaired mode");
    break;
  }
  rxModuleStatus = newState;
}

//{Sleep, Idle, Active, LightsOnly, Unpaired};
void updateState()
{
  unsigned long tNow = millis();
  char buf[8];

  switch (rxModuleStatus)
  {
  case (Idle):
    //setStatusText(buf, LED_GREEN_MED);
    RGBMatrix.updateIncomingPackets(&incomingMessages,true);
    RGBMatrix.updateIdleScreen(&newPacket->status);
    break;
  case (Active):
    //setStatusText(buf, LED_GREEN_MED);
    RGBMatrix.updateIncomingPackets(&incomingMessages,false);
    if (scoreChanged)
    {
      RGBMatrix.updateScore(&newPacket->score);
    }
    if (lightsChanged)
    {
      RGBMatrix.updateLights(newPacket);
    }    
    break;
  case (LightsOnly):
  {
    RGBMatrix.updateIncomingPackets(&incomingMessages,false);
    if (lightsChanged)
    {
      RGBMatrix.updateLights(newPacket);
    }    
  }
  break;
  case (Unpaired):
    RGBMatrix.updateIncomingPackets(&incomingMessages,true);
    if (pairingEnabled)
    {
      RGBMatrix.setStatusText("Pair", RGBMatrix.LED_BLUE_MED);
    }
    else
    {
      RGBMatrix.setStatusText("N/C", RGBMatrix.LED_RED_MED);
    }
    break;
  case (Sleep):
    doSleep();
    break;
  }
}

void checkSystemState()
{
  RX_STATE newState = rxModuleStatus;

  if ((!isConnected) || (newPacket==NULL))
  {
    newState = Sleep;
  }
  else
  {
    if (newPacket->status.isActive == false)
    {
      newState = Idle;
    }
    else
    {
      if (newPacket->lights.lightsOnly)
      {
        newState = LightsOnly;
      }
      else
      {
        newState = Active;
      }
    }
  }

  if (isPaired == false)
  {
    newState = Unpaired;
  }
  if (newState != rxModuleStatus)
  {
    changeSystemState(newState);
  }
}

void checkButtonState()
{
  static bool lastState = HIGH;
  static unsigned long tLastPress = 0;
  static unsigned long tPressed = 0;
  static unsigned long tSwitch = 0;
  unsigned long tNow = millis();

  int newState = gpio_get_level(BUTTON_PIN);

  if (newState != lastState)
  {
    if (newState == HIGH)
    { //Button release
      if ((tNow - tSwitch) > tShortPress)
      {
        tLastPress = tNow;
        char buf[24]="N/C";
        if (txDevice!=NULL) {
          macAddressToChar(buf,txDevice->mac);
        }
        RGBMatrix.displayStatusScreen(5000,buf);
        changeSystemState(rxModuleStatus);
        Serial.println("Short press");
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
      delay(50);
      if ((millis() - tSwitch) > 10000)
      { //10s press
        RGBMatrix.playEasterEgg();
        lastState = HIGH;
        changeSystemState(rxModuleStatus);
        return;
      }
    }
    tLastPress = millis();
    lastState = HIGH;
    setPairingMode();
    Serial.println("Long press");
    //tPairingActive=millis();
  }
}

void saveSettings()
{

  if (isPaired)
  {
    Serial.println("Saving paired device");
    kvStore.putBool("txPaired", true);
    kvStore.putBytes("lastMAC", txDevice->mac, ESP_NOW_ETH_ALEN);
  }
  else
  {
    Serial.println("Saving unpaired mode");
    kvStore.putBool("txPaired", false);
    //kvStore.putBytes("lastMAC", {0, 0, 0, 0, 0, 0}, ESP_NOW_ETH_ALEN);
  }
}

void updateSystemState()
{
  checkButtonState();

  if (updatePrefs)
  {
    saveSettings();
    updatePrefs = false;
  }

  //If there's data waiting, free the old one, and pop an element from the queue.
  if (!packetQueue.empty())
  {
    free(lastPacket);
    lastPacket = newPacket;
    newPacket = packetQueue.front();
    packetQueue.pop();
    checkDataChanged();
  }
  checkSystemState();
  updateState();

  lightsChanged = false;
  scoreChanged = false;
}

void loadSavedConfig()
{
  uint8_t buf[ESP_NOW_ETH_ALEN];
  char buffer[20];

  txFound = false;
  isConnected = false;
  pairingEnabled = false;

  kvStore.begin("FA05-Rx");
  isPaired = kvStore.getBool("txPaired");
  if (isPaired == false)
  {
    txDevice = NULL;
    //changeSystemState(Unpaired);
    changeSystemState(Unpaired);
  }
  else
  {
    kvStore.getBytes("lastMAC", buf, ESP_NOW_ETH_ALEN);
    txDevice = new espNowDevice(buf);
    RGBMatrix.displayLogo();
    macAddressToChar(buffer, txDevice->mac);
    sprintf(txtBuf, "Searching for : %s", buffer);
    Serial.println(txtBuf);
    for (int k = 0; k < 64; k++)
    {
      RGBMatrix.drawPixel(k, 31, RGBMatrix.LED_BLUE_HIGH);
      delay(500);
      if (txFound)
      {
        break;
      }
    }
    changeSystemState(Sleep);
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println("Started...");
  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);
  gpio_pullup_en(BUTTON_PIN);

  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 0);

  pairTimer.stop();

  espNowInit();

  RGBMatrix.initDisplay();

  loadSavedConfig();
}

void loop()
{
  unsigned long tic, toc;
  static Blinker blinkLED(400, 30);

  connectToServer();

  tic = millis();
  updateSystemState();
  toc = millis();
  if (toc - tic > 500)
  {
    Serial.println("Warning update exceeded 500ms");
  }

  if (pairingEnabled)
  {
    gpio_set_level(LED_PIN, blinkLED.isOn());
  }
  else
  {
    gpio_set_level(LED_PIN, isConnected);
  }
  // put your main code here, to run repeatedly:
}