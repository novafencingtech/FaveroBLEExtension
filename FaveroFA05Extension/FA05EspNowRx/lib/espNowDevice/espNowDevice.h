#include <Arduino.h>
#include <esp_now.h>
#include "FA05_ESPNOW_Library.h"


class espNowDevice {
  public:    
    static const uint64_t BIT_MASK_48=0xFFFFFFFFFFFF0000;
    struct {
      union {
        uint64_t mac64;
        uint8_t mac[8];
      };
    };
    espNowDevice(const uint8_t *addr);
    void lastSeen(unsigned long time);
    unsigned long getLastSeen();
    esp_now_peer_info_t peerInfo = {
      .peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
      .lmk = {},
      .channel = FA05_WIFI_CHANNEL,
      .ifidx = ESPNOW_WIFI_IF,
      .encrypt = false,
      .priv = nullptr};
    friend bool operator==(const espNowDevice &dev1, const espNowDevice &dev2);
    friend bool operator<(const espNowDevice &dev1, const espNowDevice &dev2);
    friend bool operator>(const espNowDevice &dev1, const espNowDevice &dev2);

  private:
    unsigned long _seen=0;
};