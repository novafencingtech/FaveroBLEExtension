/* Favero FA-05 Bluetooth Low Energy Extension Lights
Graham Allen
May 2020

Uses a standard 64x32 RGB matrix display as an extension light for a Favero Scoring machine.

*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <esp_bt_main.h>
#include <Preferences.h>
#include "FA05_BLE_Library.h"

#define GPIOPINOUT ESP32_FORUM_PINOUT
//#include <MatrixHardware_ESP32_V0.h>
#include "RGB_ControllerHardware_ESP32.h"
//#define ESP32_I2S_CLOCK_SPEED (10000000UL)
//#define ESP32_I2S_CLOCK_SPEED (100000UL)
//#define ESP32_I2S_CLOCK_SPEED (50000UL)
//#define R1_PIN  GPIO_NUM_13

#define BUTTON_PIN GPIO_NUM_33
#define LED_PIN GPIO_NUM_2


#include "RGB_Display_Functions.h"
#include "globals.h"

//ESP_GATTC_NOTIFY_EVT

#define UUID_STR_LEN 37

//const unsigned long BLE_SEARCH_TIMEOUT=45*1000; //Time

// The remote service we wish to connect to.
//static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
//static BLEUUID    charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

//static boolean doConnect = false;
//static boolean connected = false;
//static boolean doScan = false;

boolean txPaired = false;
boolean txFound = false;
boolean isConnected = false;
boolean isConnecting = false;
boolean pairingEnabled = false;
float aveRSSI = 0;
const uint16_t RSSI_NUM_AVE = 128;

const unsigned long tSleepInterval = 60 * 1000; //Wakeup ever 60s to scan for devices if not connected.
const unsigned long idleTimeOut = 10 * 60 * 1000; //Switch to idle mode if inactive for 15min
const unsigned long tShortPress = 100; //ms-Short button press
const unsigned long tLongPress = 1000; //ms - Long button press
const unsigned long tLEDBlinkFast = 200; //ms -- Fast Blink LED
const unsigned long tLEDBlinkSlow = 800; //ms -- Slow Blink LED
const unsigned long BLEScanDuration_t = 5; //Scan duration in seconds
const unsigned long BLEIdleDuration_t = 30 * 1000; //Scan duration in ms

unsigned long tLastSeen = 0; //timer for the last time a heart-beat packet was recieved
unsigned long tLastActive = 0; //timer for the last time the scoring machine changed state
unsigned long tLastScoreChange = 0;
unsigned long tPairingActive = 0; //
unsigned long tLastScan = 0;

// Sleep = Paired but not connected to Tx module, Idle = connected waiting for scoring machine state change,
// Active = Running full repeat,  LightsOnly = only showing hit indications
// Unpaired = Not paired with the BLE server
//
enum RX_STATE {Sleep, Idle, Active, LightsOnly, Unpaired};

RX_STATE rxModuleStatus = Sleep;

enum BLE_ERROR_CODE {BLE_SUCCESS = 0, DeviceNotFound, ServiceNotFound, CharNotFound, RegFailure, ConnectFail};

static BLERemoteCharacteristic* fa05TxDiscChar;
static BLERemoteCharacteristic* fa05TxTimeChar;
static BLERemoteCharacteristic* heartBeatChar;
static BLERemoteCharacteristic* rawDataChar;
static BLERemoteCharacteristic* lightsChar;
static BLERemoteCharacteristic* scoreChar;

static BLEAdvertisedDevice* searchDevice;
static BLEAdvertisedDevice* fa05Tx;

static BLEClient* RxBLEClient;

char fa05ServiceUUID[UUID_STR_LEN];
//char[UUID_STR_LEN] fa05ServiceUUID="";
BLEUUID discoveryUUID{FA05_DISCOVERY_UUID};
BLEUUID fa05TxService;
BLEAddress lastConnectBLE("");
uint8_t fa05ServiceUUID128[ESP_UUID_LEN_128] = {0};

machineStatus_BLE fa05Status;
lightData_BLE fa05Lights;
matchData_BLE fa05Score;

bool scoreChanged = false;
bool lightsChanged = false;
bool heartbeatChanged = false;

Preferences kvStore;

void updateHeartBeat();


BLE_ERROR_CODE connectToFA05Service();

static void heartbeatCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t packet_size,
  bool isNotify) {
  //Serial.println("Heart-beat updated");
  if (pBLERemoteCharacteristic->getUUID().equals(BLEUUID(HEARTBEAT_UUID))) {
    memcpy(fa05Status.rawBytes, pData, packet_size);
    updateHeartBeat();
  }
  lightsChanged = true;
  //Serial.println("New heartbeat");
  //Serial.println(fa05Status.packetCount);
  //Serial.println(fa05Status.upTimeSec);
  //Serial.println(fa05Status.isActive);
  //Serial.print("Active strip # "); Serial.println(fa05Status.stripNum);
  //Serial.println("Callback finished successfully");
}

void updateHeartBeat() {
  static unsigned long tUp = 0;
  static bool state = true;

  tLastSeen = millis();
  if (fa05Status.isActive != state) {
    heartbeatChanged = true;
    //changeSystemState(Idle);
  }
  if (fa05Status.upTimeSec < tUp) {
    Serial.println("Tx has rebooted!");
    //displayStatusScreen(10000);
  }
  state = fa05Status.isActive;
  tUp = fa05Status.upTimeSec;
}

static void lightsCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t packet_size,
  bool isNotify) {
  if (pBLERemoteCharacteristic->getUUID().equals(BLEUUID(LIGHT_STATE_UUID ))) {
    if (*pData != fa05Lights.value) {
      memcpy(&(fa05Lights.value), pData, packet_size);
      lightsChanged = true;
      //updateLights();
      //Serial.println("Lights changed");
    }
  }
}

static void scoreCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t packet_size,
  bool isNotify) {
  matchData_BLE* newPacket = (matchData_BLE*) pData;
  bool updateValue = false;

  //Serial.println("Score callback");
  if (pBLERemoteCharacteristic->getUUID().equals(BLEUUID(MATCH_UUID)))
  {
    if (newPacket->scoreL != fa05Score.scoreL) {
      updateValue = true;
    }
    if (newPacket->scoreR != fa05Score.scoreR) {
      updateValue = true;
    }
    if (newPacket->timeRemainMin != fa05Score.timeRemainMin) {
      updateValue = true;
    }
    if (newPacket->timeRemainSec != fa05Score.timeRemainSec) {
      updateValue = true;
    }
    if (newPacket->matchCount != fa05Score.matchCount) {
      updateValue = true;
    }
    if (newPacket->rawBytes[matchDataBLEPacketSize - 1] != fa05Score.rawBytes[matchDataBLEPacketSize - 1]) {
      updateValue = true;
    }
    if (updateValue) {
      memcpy(fa05Score.rawBytes, newPacket->rawBytes, packet_size);
      //updateScore();
      scoreChanged = true;
      tLastScoreChange = millis();
      tLastActive = millis();
    }
  }
  //Serial.println("Score callback completed successfully");
  //Serial.println(fa05Score.timeRemainSec);
}

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
      isConnecting=false;
      Serial.println("onConnect Callback");
      if (txPaired) {
        //connectToFA05Service(pclient);
      } else {
        //pairService(pclient);
      }
    }

    void onDisconnect(BLEClient* pclient) {
      //connected = false;
      isConnecting=false;
      txFound = false;
      isConnected = false;
      Serial.println("onDisconnect");
    }
};

void setPairing(bool active) {
  pairingEnabled = active;
  if (active) {
    tPairingActive = millis();
    changeSystemState(Unpaired);
    pairingEnabled = true;
  } else {
    tPairingActive = 0;
    pairingEnabled = false;
    gpio_set_level(LED_PIN, LOW);
  }
}

void pairService() {
  Serial.println(" - Created client");

  BLERemoteService* pRemoteService = RxBLEClient->getService(discoveryUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(discoveryUUID.toString().c_str());
    RxBLEClient->disconnect();
    return;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(FA05_SERVICE_CHAR_UUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(FA05_SERVICE_CHAR_UUID);
    RxBLEClient->disconnect();
    return;
  }
  //Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    fa05TxService = BLEUUID(value);
    Serial.println(fa05TxService.toString().c_str());

    if (connectToFA05Service() == BLE_SUCCESS) {
      //kvStore.remove("lastUUID");
    };
    txFound = true;
    //Serial.print("The characteristic value was: ");
    //Serial.println(value.c_str());
  }

}

BLE_ERROR_CODE connectToFA05Service() {
  //BLE_ERROR_CODE errVal = BLE_SUCCESS;
  BLERemoteService* pRemoteService = RxBLEClient->getService(fa05TxService);
  uint8_t* blePacketData;
  char buf[64];

  if (pRemoteService == nullptr) {
    Serial.println("Service not found");
    return (ServiceNotFound);
  }

  BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(HEARTBEAT_UUID);
  blePacketData = pRemoteCharacteristic->readRawData();
  //Serial.println("Heart beat packet reports: ");
  //Serial.println(pRemoteCharacteristic->readValue().c_str());
  for (int k = 0; k < heartBeatPacketSize; k++) {
    //sprintf(buf, "Packet Count: %l Time: %l IsActive: %u Errors:  %u %u",blePacketData[0],blePacketData[4],
    //  blePacketData[8], blePacketData[9],blePacketData[10]);
  }
  //memcpy(fa05Status.rawBytes,pRemoteCharacteristic->readRawData(),heartBeatPacketSize);
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(heartbeatCallback, true);
    //Serial.println("Registering for heart-beat notifications");
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(MATCH_UUID);
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(scoreCallback, true);
    //Serial.println("Registering for Score notifications");
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(LIGHT_STATE_UUID);
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(lightsCallback, true);
    //Serial.println("Registering for lights notifications");
  }
  //Serial.println("Registered for Notification");
  strncpy(buf, pRemoteService->toString().c_str(), UUID_STR_LEN + 15);
  //int totalLen=strlen(buf);
  //Serial.println(buf);
  //fa05ServiceUUID[UUID_STR_LEN];
  strncpy(fa05ServiceUUID, &(buf[15]), UUID_STR_LEN - 1);
  //memcpy(fa05ServiceUUID128,pRemoteService->getUUID().getNative()->uuid.uuid128,ESP_UUID_LEN_128);
  //Serial.println(fa05ServiceUUID);
  return (BLE_SUCCESS);
}

bool connectToServer() {
  char buf[64];
  uint8_t retryCount=10;
  unsigned long tic;
  

  //Serial.print("Forming a connection to ");
  //Serial.println(searchDevice->getAddress().toString().c_str());

  //Serial.println(" - Created client");

  RxBLEClient->setClientCallbacks(new MyClientCallback());

  //BLEDevice::getScan()->stop(); //Stop any scans before attempting to connect
  if (esp_ble_gap_stop_scanning()!=ESP_OK) {
    Serial.println("Error stopping scan");
  }
  delay(200); 
  // Connect to the remove BLE Server.
  Serial.print("Forming a connection to ");
  Serial.println(searchDevice->getAddress().toString().c_str());
  while (!RxBLEClient->isConnected()) {
    RxBLEClient->connect(searchDevice);
    //esp_ble_gattc_open(ESP_GATT_IF_NONE,(uint8_t*) searchDevice->getAddress().getNative(),searchDevice->getAddressType(),true);
    //while (isConnecting) {
    //  delay(10);
    //  if (millis()-tic>1000) {
    //    isConnecting=false;
    //  }
    //}
    retryCount--;
    if (retryCount==0) {
      Serial.println("Failed to connect after re-tries");
      //BLEDevice::init();
      return false;
    }
    delay(50);
  }
  //gatt_client_update_connection_params(searchDevice->getAddress());
  Serial.println(" - Connected to server");


  // Obtain a reference to the service we are after in the remote BLE server.
  if ((txPaired == false) && pairingEnabled) {
    pairService();
  } else {
    if (txPaired) {

    }
  }

  if (isConnected == false) {
    if (connectToFA05Service() == BLE_SUCCESS) {
      txPaired = true;
      txFound = true;
      isConnected = true;
      setPairing(false);
      kvStore.putString("lastBLEAdd", searchDevice->getAddress().toString().c_str());
      kvStore.putString("lastUUID", fa05ServiceUUID);
      kvStore.putBool("txPaired", true);

      //kvStore.putBytes("lastUUID",fa05ServiceUUID128,ESP_UUID_LEN_128);
      //Serial.print("Connected - saving Device UUID: ");
      //Serial.println(fa05ServiceUUID);
      //delay(500);
      //Serial.println(kvStore.getString("lastUUID", buf, UUID_STR_LEN));
    }
  }
}

//void gatt_client_update_connection_params(uint8_t channel, uint8_t* remote_bda)
void gatt_client_update_connection_params(BLEAddress remoteDevice)
{
  Serial.println("Attempting configuration update");
  const uint8_t BLE_MAC_ADDR_LEN = 6;
  //uint8_t *remote_bda=&((remoteDevice.getNative())[0]);
  esp_ble_conn_update_params_t conn_params;
  memcpy(conn_params.bda, remoteDevice.getNative(), BLE_MAC_ADDR_LEN);
  conn_params.min_int = 0x06; // x 1.25ms
  conn_params.max_int = 0x20; // x 1.25ms
  conn_params.latency = 0x03; //number of skippable connection events
  conn_params.timeout = 0x50; // x 6.25ms, time before peripheral will assume connection is dropped.

  if (esp_ble_gap_update_conn_params(&conn_params)==ESP_OK) {
    Serial.println("BLE connection configured");
  } else {
    Serial.println("Error configuring BLE connection");
  }
}

/**
   Scan for BLE servers and find the first one that advertises the service we are looking for.
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      //Serial.print("BLE Advertised Device found: ");
      //Serial.println(advertisedDevice.toString().c_str());

      // We have found a device, let us now see if it contains the service we are looking for.
      if (txPaired) {
        if (lastConnectBLE.equals(advertisedDevice.getAddress())) {  
        //if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(fa05TxService)) {
          Serial.println("Found last device");
          BLEDevice::getScan()->stop();
          delete searchDevice;
          searchDevice = new BLEAdvertisedDevice(advertisedDevice);
          //searchDevice=advertisedDevice;
          txFound = true;
          //doConnect = true;
          //doScan = true;
        }
      } else {
        if (pairingEnabled) {
          if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(FA05_DISCOVERY_UUID))) {
            BLEDevice::getScan()->stop();
            delete searchDevice;
            searchDevice = new BLEAdvertisedDevice(advertisedDevice);
            txFound = true;
          } // Found our server
        } // onResult
      }
    }
}; // MyAdvertisedDeviceCallbacks

void scanForDevices() {
  static MyAdvertisedDeviceCallbacks* advertisingCallbacks=new MyAdvertisedDeviceCallbacks();
  
  BLEScanResults bleDeviceList;
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(advertisingCallbacks);
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  bleDeviceList = pBLEScan->start(BLEScanDuration_t, false);
  tLastScan = millis();
}

//enum RX_STATE {Sleep, Idle, Active, LightsOnly, Unpaired};
void changeSystemState(RX_STATE newState) {
  switch (newState) {
    case (Sleep):
      clearScreen();
      Serial.println("Sleep mode");
      break;
    case (Idle):
      clearScreen();
      updateIdleScreen();
      Serial.println("Idle mode");
      break;
    case (Active):
      clearScreen();
      updateScore();
      updateLights();
      Serial.println("Active mode");
      break;
    case (LightsOnly):
      clearScreen();
      Serial.println("Lights-only mode");
      updateLights();
      break;
    case (Unpaired):
      //BLEClient*  pClient  = BLEDevice::createClient();
      Serial.println("Unpaired mode");
      clearScreen();
      /*if (RxBLEClient->isConnected()) {
        RxBLEClient->disconnect();
        delay(200);
      }*/
      kvStore.remove("lastUUID");
      kvStore.putBool("txPaired", false);
      //Serial.println("Removing stored UUID");
      fa05ServiceUUID[0] = '\0';
      txFound = false;
      isConnected = false;
      txPaired = false;
      break;
  }
  rxModuleStatus = newState;
}

void setPairing(bool active) {
  pairingEnabled = active;
  if (active) {
    tPairingActive = millis();
    changeSystemState(Unpaired);
    pairingEnabled = true;
  } else {
    tPairingActive = 0;
    pairingEnabled = false;
    gpio_set_level(LED_PIN, LOW);
  }
}

void checkButtonState() {
  static bool lastState = HIGH;
  static unsigned long tLastPress = 0;
  static unsigned long tPressed = 0;
  static unsigned long tSwitch = 0;
  unsigned long tNow = millis();

  int newState = gpio_get_level(BUTTON_PIN);

  if (newState != lastState) {
    if (newState == HIGH) { //Button release
      if ((tNow - tSwitch) > tShortPress) {
        tLastPress = tNow;
        displayStatusScreen(5000,aveRSSI);
        //Serial.println("Short press");
        //Short button press
      }
    }
    tSwitch = tNow;
    lastState = newState;
  }

  if ((newState == LOW) && ((tNow - tSwitch) > tLongPress)) {
    //Long button push action
    gpio_set_level(LED_PIN, HIGH);
    while (gpio_get_level(BUTTON_PIN) == LOW) {
      delay(50);      
      if ((millis()-tSwitch)>10000) { //10s press
        playEasterEgg();
        lastState=HIGH;
        return;
      }
    }
    tLastPress = millis();
    lastState = HIGH;
    setPairing(true);
    //Serial.println("Long press");
    //tPairingActive=millis();
  }
}

//{Sleep, Idle, Active, LightsOnly, Unpaired};
void updateSystemState() {
  unsigned long tNow = millis();
  static unsigned long tLED = 0;
  static int rssi = 0;
  char buf[8];

  checkButtonState();

  if (isConnected) {
    rssi = RxBLEClient->getRssi();
    aveRSSI = ((RSSI_NUM_AVE - 1) * aveRSSI + rssi) / RSSI_NUM_AVE;
    sprintf(buf, "%3.0f", aveRSSI);
    //Serial.print("RSSI: last= "); Serial.print(rssi);Serial.print("   Ave = ");Serial.println(buf);
  }

  switch (rxModuleStatus) {
    case (Unpaired):
      if (txPaired) {
        setPairing(false);
        if (isConnected) {
          changeSystemState(Active);
        }
      }
      if (pairingEnabled) {
        if ((tNow - tPairingActive) > FA05_PAIRING_TIMEOUT) {
          setPairing(false);
          setStatusText(" N/C ", LED_RED_MED);
          esp_ble_gap_stop_scanning();
        } else {
          if ((tNow - tLastScan) > BLEScanDuration_t) {
            scanForDevices();
          }
          setStatusText("Pair", LED_BLUE_MED);
        }
      } else {
        setStatusText(" N/C ", LED_RED_MED);
      }
      break;
    case (Sleep):
      if (isConnected) {
        changeSystemState(Active);
      }
      if (isConnected == false) {
        Serial.println("Sleep");
        delay(100);
        esp_bluedroid_disable();
        gpio_intr_enable(BUTTON_PIN);
        gpio_wakeup_enable(BUTTON_PIN,GPIO_INTR_LOW_LEVEL);
        esp_sleep_enable_gpio_wakeup();
        esp_sleep_enable_timer_wakeup(tSleepInterval * 1000); //Set sleep timer for 1s
        esp_light_sleep_start();
        Serial.println("Awake");
        esp_bluedroid_enable();
        delay(500);
        setStatusText("Scan",LED_BLUE_MED);
        //Serial.println("Scan start");
        gpio_set_level(LED_PIN, HIGH);
        scanForDevices();
        gpio_set_level(LED_PIN, LOW);
        //Serial.println("Function return start");
        
      }
      if (txPaired == false) {
        changeSystemState(Unpaired);
      }
      break;
    case (Idle):
      //setStatusText(buf, LED_GREEN_MED);
      updateIdleScreen();
      if (isConnected == false) {
        changeSystemState(Sleep);
      }
      if (fa05Status.isActive) {
        if (fa05Lights.lightsOnly) {
          changeSystemState(LightsOnly);
        } else {
          changeSystemState(Active);
        }
      }
      break;
    case (Active):
      //setStatusText(buf, LED_GREEN_MED);
      if (fa05Lights.lightsOnly) {
        changeSystemState(LightsOnly);
      }
      if (lightsChanged) {
        updateLights();
      }
      if (scoreChanged) {
        updateScore();
      }
      if (isConnected == false) {
        changeSystemState(Sleep);
      }      
      if (fa05Status.isActive==false) {
        changeSystemState(Idle);
      }
      break;
    case (LightsOnly):
      //setStatusText(buf, LED_GREEN_MED);
      if (fa05Lights.lightsOnly == false) {
        changeSystemState(Active);
      }
      if (isConnected == false) {
        changeSystemState(Sleep);
      }
      if (fa05Status.isActive==false) {
        changeSystemState(Idle);
      }
      if (lightsChanged) {
        updateLights();
      }
      break;
  }
}

void initBLEStructs() {
  //fa05Lights.value = 0;
  //fa05Score.rawBytes{0, 0, 0, 1, 0, 0};

}


void setup() {
  Serial.begin(115200);
  //Serial.println("Starting Arduino BLE Client application...");
  char buf[64];

  kvStore.begin("FA05-Rx");
  BLEDevice::init("NoVA_FC");
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
  
  gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);
  gpio_pullup_en(BUTTON_PIN);
  
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  initBLEStructs();
  RxBLEClient  = BLEDevice::createClient();

  // setup matrix
  matrix.addLayer(&backgroundLayer);
  matrix.addLayer(&scrollingLayer1);
  matrix.addLayer(&scrollingLayer2);
  matrix.addLayer(&scrollingLayer3);
  matrix.begin();

  backgroundLayer.fillScreen(LED_BLACK);
  scrollingLayer1.setMode(bounceForward);
  displayLogo();
  //delay(5000);

  //int val=kvStore.getString("lastUUID",fa05ServiceUUID,UUID_STR_LEN);
  //Serial.print("Pre-stored value : ");Serial.println(val);

  //val=kvStore.getString("Test",buf,UUID_STR_LEN);
  //Serial.print("Pre-stored value : ");Serial.println(val);

  if (kvStore.getBool("txPaired") == false) {
    //if (kvStore.getString("lastUUID", fa05ServiceUUID, UUID_STR_LEN) == 0) {
    //if (kvStore.getBytesLength("lastUUID") != ESP_UUID_LEN_128) {
    //Serial.println("No devices saved");
    //Serial.println(kvStore.getBytesLength("lastUUID"));
    fa05ServiceUUID[0] = '\0';
    txPaired = false;
    txFound = false;
    isConnected = false;
    pairingEnabled = false;
    changeSystemState(Unpaired);
  } else {
    kvStore.getString("lastUUID", fa05ServiceUUID, UUID_STR_LEN);
    kvStore.getString("lastBLEAdd", buf, 32);
    //Serial.println(buf);
    lastConnectBLE = BLEAddress(buf);
    //kvStore.getBytes("lastUUID", fa05ServiceUUID128, ESP_UUID_LEN_128);
    //fa05TxService = BLEUUID(fa05ServiceUUID128,ESP_UUID_LEN_128,true);
    fa05TxService = BLEUUID(fa05ServiceUUID);
    //Serial.println(fa05TxService.toString().c_str());
    //Serial.println(lastConnectBLE.toString().c_str());
    txPaired = true;
    txFound = false;
    isConnected = false;
    pairingEnabled = false;
    //scanForDevices();
    //delay(5000);
    for (int i = 0; i < 64; i = i + 5) {
      scanForDevices();
      if (txFound) {
        break;
      }
      backgroundLayer.drawPixel(i, 31, LED_BLUE_HIGH);
      backgroundLayer.drawPixel(i + 1, 31, LED_BLUE_HIGH);
      backgroundLayer.drawPixel(i + 2, 31, LED_BLUE_HIGH);
      backgroundLayer.drawPixel(i + 3, 31, LED_BLUE_HIGH);
      backgroundLayer.drawPixel(i + 4, 31, LED_BLUE_HIGH);
      backgroundLayer.swapBuffers();
      //delay(1000);
    }
    if (txFound) {
      connectToServer();
      changeSystemState(Active);
    } else {
      changeSystemState(Sleep);
      //changeSystemState(Active);
    }
  }



  //kvStore.putString("Test","This is a Test");

  //scanForDiscoveryServices();
} // End of setup.


// This is the Arduino main loop function.
void loop() {
  static unsigned long tLED = 0;
  static bool LED_on = false;

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if ((isConnected == false) && (txFound)) {
    Serial.println("Tx found - Connecting to server");
    connectToServer();
  }

  checkButtonState();
  //runDemo();
  updateSystemState();

  if (pairingEnabled) {
    if ((millis() - tLED) > tLEDBlinkFast) {
      LED_on = !LED_on;
      gpio_set_level(LED_PIN, LED_on);
      tLED = millis();
    }
  } else {
    if (isConnected) {
      gpio_set_level(LED_PIN, true);
    } else {
      if ((millis() - tLED) > tLEDBlinkSlow) {
        LED_on = !LED_on;
        gpio_set_level(LED_PIN, LED_on);
        tLED = millis();
      }
    }
  }
} // End of loop
