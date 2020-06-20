#include "Arduino.h"
#include "ESP-NOWModule.h"
//#include "Preferences.h"

espNOWModule::espNOWModule()
{
}

espNOWModule::init(bool isMaster)
{

    /*if (isMaster) {
        wifiMode=WIFI_MODE_AP;
    } else
    {
        wifiMode=STA;
    }*/
    wifiMode = WIFI_MODE_STA;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(wifiMode));
    ESP_ERROR_CHECK(esp_wifi_start());

    //ESP_ERROR_CHECK( esp_wifi_set_config());

    esp_wifi_set_ps(WIFI_PS_NONE);

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(incomingData);
    ESP_ERROR_CHECK(sendCallBack);
}

espNOWModule::setPairing(bool active)
{
    _pairingEnabled = active;

    if (isMaster==false)
    {
        if (_pairingEnabled) {
            broadcastAllMessage(&(espMsg_PairBroadcast));
        }
    }
}

espNOWModule::macAddressToChar(char *buf, uint8_t *addr)
{
    snprintf(buf, 6 * 3, "%02x:%02x:%02x:%02x:%02x:%02x",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

bool espNOWModule::macEquals(uint8_t *mac1, uint8_t *mac2)
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

void espNOWModule::incomingData(uint8_t *mac_addr, uint8_t *data, int data_len)
{
    char buf[20];
    esoNOWMessage incMsg = espNOWMessage(data,data_len);

    Serial.print("Message recieved from:");
    macAddressToChar(buf,*mac_addr);
    Serial.println(buf);
    
    if (isMaster == false)
    {
        if (macEquals(mac_addr, serverMAC))
        {
            //valid message from server goes to callback
        }
    }
}

void espNOWModule::sendCallBack(uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (ESP_NOW_SEND_SUCCESS)
    {
        Serial.println("Message sent to");
    } else
    {
        Serial.println("Error sending message");
    }
    
}

void espNOWModule

espNOWMessage::espNOWMessage(espNOWMessage msg)
{
    _msgData = malloc(msg->_msgSize);
    memcpy(msgData, msg->_msgData, msg->_msgSize);
    _isMalloc=true;
}

espNOWMessage::espNOWMessage(uint8_t *data, int size)
{
    _msgSize = size;
    _msgData = data;
    _type = data[0];
    _isMalloc=false;
}

espNOWMessage::espNOWMessage(espNowMessageType type, uint8_t *data, int size)
{
    _msgSize = size + 1;
    _msgData = malloc(_msgSize);
    _msgData[0] = _type;
    memcpy(&(_msgData[1]), data, size);
    _isMalloc=true;
}

espNOWMessage::~espNOWMessage()
{
    if (_isMalloc) {
        free(_msgData);
    }
}

espNOWMessage::copyData(uint8_t *buffer)
{
    memcpy(buffer, &(_msgData[1]), _msgSize - 1);
}

espNOWMessage::getRawMessage(uint8_t *payload, uint8_t *size)
{
    *size = _msgSize;
    payload = _msgData;
}

int espNOWMessage::getRawMessageSize()
{
    if (_msgSize > 0)
    {
        return (_msgSize - 1);
    }
    else
    {
        return 0;
    }
}

int espNOWMessage::getPayloadSize()
{
    if (_msgSize > 1)
    {
        return (_msgSize - 1);
    }
    else
    {
        return 0;
    }
}
