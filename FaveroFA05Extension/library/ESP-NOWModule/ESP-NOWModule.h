#ifndef ESP_NOW_GSA_H
#define ESP_NOW_GSA_H

#include <esp_wifi.h>
#include <esp_now.h>
#include <semphr.h>

enum espNowMessageType = {espMsg_PairBroadcast, espMsg_PairRequest, espMsg_PairAwk, espMsg_Data, espMsg_DataAwk, espMsg_Command, espMsg_CommandAwk,
                          espMsg_ConnectReq, espMsg_ConnectAwk, espMsg_KeepAlive, espMsg_KeepAliveAwk};

class espNOWModule
{
public:

    enum ESPNOW_MESSAGE_EVENT = {ESPNOW_MSG_ONCONNECT,ESPNOW_MSG_DISCONNECT,ESPNOW_MSG_DATA_READY}
    static uint8_t broadcast_all_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static const unsigned long PairingTimeout = 45 * 1000; //ms
    static void macAddressToChar(char *buf, uint8_t *addr);
    static init();        
    void updateClients(uint8_t *data, int size);
    void updateServer(uint8_t *data, int size);
    void broadcastAllMessage(uint8_t *data, int size);
    void setPairing(bool active);
    bool isPairing();
    bool isPaired();
    bool isConnected();
    int getNumActiveClients();
    void setClientMode();
    void setServerMode();
    bool addClient(uint8_t *clientAddr);
    bool addServer(uint8_t *serverAddr);
    void update(); 
    static bool macEquals(uint8_t *mac1, uint8_t *mac2);

private:
    static const uint8_t _maxNumClients;
    uint8_t numActiveClients=0;
    wifi_mode_t _wifiMode;
    bool _isMaster = false;
    uint8_t _serverMAC[ESP_NOW_ETH_ALEN];
    uint8_t _clientMAC[_maxNumClients][ESP_NOW_ETH_ALEN];
    uint8_t _clientListIndex=0;
    uint32_t _lastMessageID;
    bool _pairingEnabled;
    bool _isPaired = false;
    
    void incomingData(uint8_t *mac_addr, uint8_t *data, int data_len);
    void sendCallBack(uint8_t *mac_addr, esp_now_send_status_t status);

}

class espNOWMessage
{
public:
    espNOWMessage(uint8_t* data, int data_len); //Creates a message without allocating memory
    espNOWMessage(espNowMessageType type, uint8_t *data, int size); //Creates a message and allocates memory for it
    void copyPayload(uint8_t *buffer); //Warning buffer must be large enough to store the message data
    void getRawMessage(uint8_t *payload, uint8_t *size);
    int getRawMessageSize();
    int getPayloadSize();
    espNowMessageType getMessageType();
    ~espNOWMessage();

private:
    espNowMessageType _type;
    uint8_t *_msgData;
    int _msgSize;
    bool _isMalloc;
}

#endif
