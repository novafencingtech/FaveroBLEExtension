#include "espNowDevice.h"

espNowDevice::espNowDevice(const uint8_t *addr) {
  mac64=0;
  memcpy(mac,addr,ESP_NOW_ETH_ALEN);
  memcpy(peerInfo.peer_addr,addr,ESP_NOW_ETH_ALEN);
  _seen=0;
  //mac64=(mac64 & BIT_MASK_48);


}

void espNowDevice::lastSeen(unsigned long time) {
    _seen=time;
}

unsigned long espNowDevice::getLastSeen() {
    return _seen;
}

bool operator==(const espNowDevice &dev1, const espNowDevice &dev2) {
    return (dev1.mac64==dev2.mac64);
}
bool operator<(const espNowDevice &dev1, const espNowDevice &dev2) {
    return (dev1.mac64<dev2.mac64);
}
bool operator>(const espNowDevice &dev1, const espNowDevice &dev2) {
    return (dev1.mac64>dev2.mac64);
}