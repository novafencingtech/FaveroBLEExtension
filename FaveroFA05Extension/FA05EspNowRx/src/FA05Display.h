#include <Arduino.h>

#include "RGB_ControllerHardware_ESP32.h"
#include "FA05_ESPNOW_Library.h"
#define GPIOPINOUT ESP32_FORUM_PINOUT
#include <SmartMatrix3.h>
#include <queue>

#include "NOVA-FENCING-CLUB-LOGO-tiny.c"
#include "cake.c"

#define SM_RGB rgb24

class FA05Display
{
public:
    const SM_RGB LED_BLACK = {0x0, 0x0, 0x0};
    const SM_RGB LED_WHITE_LOW = {50, 50, 50};
    const SM_RGB LED_WHITE_MED = {210, 210, 210};
    const SM_RGB LED_WHITE_HIGH = {0xff, 0xff, 0xff};

    const SM_RGB LED_RED_LOW = {50, 0x0, 0x0};
    const SM_RGB LED_RED_MED = {170, 0x0, 0x0};
    const SM_RGB LED_RED_HIGH = {0xff, 0x0, 0x0};

    const SM_RGB LED_GREEN_LOW = {0x0, 50, 0x0};
    const SM_RGB LED_GREEN_MED = {0x0, 150, 0x0};
    const SM_RGB LED_GREEN_HIGH = {0x0, 0xff, 0x0};

    const SM_RGB LED_BLUE_LOW = {0x0, 0, 50};
    const SM_RGB LED_BLUE_MED = {0x0, 0, 150};
    const SM_RGB LED_BLUE_HIGH = {0x0, 0x0, 0xff};

    const SM_RGB LED_PURPLE_HIGH = {0xff, 0, 0xff};

    const SM_RGB LED_YELLOW_MED = {160, 160, 0};
    const SM_RGB LED_YELLOW_HIGH = {0xff, 0xff, 0};

    const SM_RGB LED_ORANGE_HIGH = {0xff, 150, 0};

    void initDisplay();
    void updateIncomingPackets(std::queue<uint8_t> *msgQueue,bool fullWidth);
    void setStatusText(char *msg, SM_RGB color);
    void clearScreen();
    void idleMessage(char *msg, SM_RGB color);
    void displayLogo();
    void updateIdleScreen(const machineStatus_BLE *fa05Status);
    void displayStatusScreen(unsigned long duration, char *msg);
    void updateLights(fa05EspData *systemData);
    void updateScore(const matchData_BLE *scorePtr);
    void updateCards(const matchData_BLE *scorePtr);
    void drawPixel(uint16_t x,uint16_t y, SM_RGB color);
    void runDemo();
    void playEasterEgg();


    private:
        uint8_t _statusTextLocX=25;
        uint8_t _statusTextLocY=31 - 5;;
};