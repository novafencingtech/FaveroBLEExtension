#include <Arduino.h>
#include <BLEDevice.h>
#include <esp_bt_main.h>
#include <Preferences.h>
#include <FA05_BLE_Library.h>

#define BUTTON_PIN GPIO_NUM_33
#define LED_PIN GPIO_NUM_2

#define UUID_STR_LEN 37

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


byte statusTextLocX = 25;
byte statusTextLocY = 31 - 5;

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

BLE_ERROR_CODE connectToFA05Service();

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());

      // We have found a device, let us now see if it contains the service we are looking for.
      if (txPaired) {
        //if (lastConnectBLE.equals(advertisedDevice.getAddress())) {  
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(fa05TxService)) {
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

static void heartbeatCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t packet_size,
  bool isNotify) {
  //Serial.println("Heart-beat updated");
  if (pBLERemoteCharacteristic->getUUID().equals(BLEUUID(HEARTBEAT_UUID))) {
    memcpy(fa05Status.rawBytes, pData, packet_size);    
  }
  lightsChanged = true;
  Serial.println("New heartbeat");
  Serial.println(fa05Status.packetCount);
  Serial.println(fa05Status.upTimeSec);
  Serial.println(fa05Status.isActive);
  Serial.print("Active strip # "); Serial.println(fa05Status.stripNum);
  Serial.println("Callback finished successfully");
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
      scoreChanged = true;
      tLastScoreChange = millis();
      tLastActive = millis();
    }
  }
  //Serial.println("Score callback completed successfully");
  //Serial.println(fa05Score.timeRemainSec);
}


void pairService() { 

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
    Serial.print("Tx Service = ");
    Serial.println(fa05TxService.toString().c_str());
  }
}

bool connectToServer() {
  char buf[64];
  uint8_t retryCount=10;
  unsigned long tic;
  

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
      pairingEnabled=false;
      //kvStore.putString("lastBLEAdd", searchDevice->getAddress().toString().c_str());
      //kvStore.putString("lastUUID", fa05ServiceUUID);
      //kvStore.putBool("txPaired", true);

      //kvStore.putBytes("lastUUID",fa05ServiceUUID128,ESP_UUID_LEN_128);
      Serial.print("Connected - saving Device UUID: ");
      Serial.println(fa05ServiceUUID);
      delay(500);
      //Serial.println(kvStore.getString("lastUUID", buf, UUID_STR_LEN));
    } else {
      Serial.println("Error connecting to services");
    }
  }
}



BLE_ERROR_CODE connectToFA05Service() {
  BLE_ERROR_CODE errVal = BLE_SUCCESS;
  BLERemoteService* pRemoteService = RxBLEClient->getService(fa05TxService);
  uint8_t* blePacketData;
  char buf[64];

  Serial.println("Attempting to register for services...");

  if (pRemoteService == nullptr) {
    Serial.println("Service not found");
    return (ServiceNotFound);
  }

  BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(HEARTBEAT_UUID);
  //blePacketData = pRemoteCharacteristic->readRawData();
  /*Serial.println("Heart beat packet reports: ");
  Serial.println(pRemoteCharacteristic->readValue().c_str());
  for (int k = 0; k < heartBeatPacketSize; k++) {
    sprintf(buf, "Packet Count: %l Time: %l IsActive: %u Errors:  %u %u",blePacketData[0],blePacketData[4],
      blePacketData[8], blePacketData[9],blePacketData[10]);
  }*/
  //memcpy(fa05Status.rawBytes,pRemoteCharacteristic->readRawData(),heartBeatPacketSize);
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(heartbeatCallback, true);
    Serial.println("Registering for heart-beat notifications");
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(MATCH_UUID);
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(scoreCallback, true);
    Serial.println("Registering for Score notifications");
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(LIGHT_STATE_UUID);
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(lightsCallback, true);
    Serial.println("Registering for lights notifications");
  }
  Serial.println("Registered for Notification");
  strncpy(buf, pRemoteService->toString().c_str(), UUID_STR_LEN + 15);
  //int totalLen=strlen(buf);
  Serial.println(buf);
  //fa05ServiceUUID[UUID_STR_LEN];
  strncpy(fa05ServiceUUID, &(buf[15]), UUID_STR_LEN - 1);
  //memcpy(fa05ServiceUUID128,pRemoteService->getUUID().getNative()->uuid.uuid128,ESP_UUID_LEN_128);
  Serial.println(fa05ServiceUUID);
  return (BLE_SUCCESS);
}

void scanForDevices() {
  //static BLEAdvertisedDeviceCallbacks* advertisingCallbacks=new MyAdvertisedDeviceCallbacks();
  
  BLEScanResults bleDeviceList;
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  Serial.println("Starting scan...");
  bleDeviceList = pBLEScan->start(BLEScanDuration_t, false);
  tLastScan = millis();
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
  
  RxBLEClient  = BLEDevice::createClient();

  txPaired=false;
  txFound=false;
  pairingEnabled=true;
  isConnected=false;
  Serial.println("Starting BLE Client");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (txFound) {    
    if (isConnected==false){
      connectToServer();    
    }  
  } else {
    scanForDevices();        
  }
  delay(5000);
  Serial.println("Loop completed");
}
