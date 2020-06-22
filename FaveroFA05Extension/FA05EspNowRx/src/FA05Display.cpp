#include "FA05Display.h"
#include "SmartMatrix3.h"
#include "eventCounter.h"

// ========================== CONFIG START ===================================================
/// SmartMatrix Defines
#define COLOR_DEPTH 24                                        // known working: 24, 48 - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24
const uint8_t kMatrixWidth = 64;                              // known working: 32, 64, 96, 128
const uint8_t kMatrixHeight = 32;                             // known working: 16, 32, 48, 64
const uint8_t kRefreshDepth = 36;                             // known working: 24, 36, 48
const uint8_t kDmaBufferRows = 4;                             // known working: 2-4, use 2 to save memory, more to keep from dropping frames and automatically lowering refresh rate
const uint8_t kPanelType = SMARTMATRIX_HUB75_32ROW_MOD16SCAN; // use SMARTMATRIX_HUB75_16ROW_MOD8SCAN for common 16x32 panels
const uint8_t kMatrixOptions = (SMARTMATRIX_OPTIONS_NONE);    // see http://docs.pixelmatix.com/SmartMatrix for options
const uint8_t kIndexedLayerOptions = (SM_INDEXED_OPTIONS_NONE);
const uint8_t kBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);
const uint8_t kScrollingLayerOptions = (SM_SCROLLING_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);
SMARTMATRIX_ALLOCATE_SCROLLING_LAYER(scrollingLayer1, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kScrollingLayerOptions);
SMARTMATRIX_ALLOCATE_SCROLLING_LAYER(scrollingLayer2, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kScrollingLayerOptions);
SMARTMATRIX_ALLOCATE_SCROLLING_LAYER(scrollingLayer3, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kScrollingLayerOptions);
//SMARTMATRIX_ALLOCATE_INDEXED_LAYER(indexedLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kIndexedLayerOptions);

const int defaultBrightness = (100 * 255) / 100; // full (100%) brightness
const int dimBrightness = (20 * 255) / 100;      // dim (20%) brightness
// ========================== CONFIG END ======================================================

void FA05Display::initDisplay()
{
    // setup matrix
    matrix.addLayer(&backgroundLayer);
    matrix.addLayer(&scrollingLayer1);
    matrix.addLayer(&scrollingLayer2);
    matrix.addLayer(&scrollingLayer3);
    matrix.begin();

    backgroundLayer.fillScreen(LED_BLACK);
    scrollingLayer1.setMode(bounceForward);
}

void FA05Display::setStatusText(char *msg, SM_RGB color)
{
    backgroundLayer.fillRectangle(_statusTextLocX, _statusTextLocY, _statusTextLocX + 6 * 3, _statusTextLocY + 5, LED_BLACK);
    backgroundLayer.setFont(font3x5);
    //backgroundLayer.drawString(statusTextLocX, statusTextLocY, LED_BLACK, "    ");
    backgroundLayer.drawString(_statusTextLocX, _statusTextLocY, color, msg);
    backgroundLayer.swapBuffers();
}

void FA05Display::updateIncomingPackets(std::queue<uint8_t> *msgQueue,bool fullWidth)
{
    uint8_t msgBarL = 25;
    uint8_t msgBarR = 64 - 25;
    const uint8_t locY = 31;
    bool updated=false;
    static bool full=false;

    if (full!=fullWidth) {
        if (full) {
            backgroundLayer.drawLine(0,locY,kMatrixWidth,locY,LED_BLACK);
            updated=true;
        } else {
            backgroundLayer.drawLine(msgBarL, locY, msgBarR, locY, LED_BLACK);
            updated=true;
        }
        full=fullWidth;
    }
    
    
    if (fullWidth) {
        msgBarL=0;
        msgBarR=kMatrixWidth-1;
    } 

    static uint8_t nextPixel = msgBarL;

    uint8_t msg;
    SM_RGB color;

    while (!msgQueue->empty())
    {
        updated=true;
        if (nextPixel == msgBarL)
        {
            backgroundLayer.drawLine(msgBarL, locY, msgBarR, locY, LED_BLACK);
        }
        msg = msgQueue->front();
        msgQueue->pop();
        switch (msg)
        {
        case (FA05_MSG_NULL):
            color = LED_RED_MED;
            break;
        case (FA05_PAIR_AWK):
            color = LED_PURPLE_HIGH;
            break;
        case (FA05_SERVER_BROADCAST):
            color = LED_GREEN_MED;
            break;
        case (FA05_DATA_UPDATE):
            color = LED_BLUE_HIGH;
            break;
        default:
            color = LED_BLACK;
        }
        backgroundLayer.drawPixel(nextPixel, 31, color);
        nextPixel++;
        if (nextPixel >= msgBarR)
        {
            nextPixel = msgBarL;
        }        
    }
    if (updated) {
        backgroundLayer.swapBuffers();
    }
}

void FA05Display::clearScreen()
{
    backgroundLayer.fillScreen(LED_BLACK);
    backgroundLayer.swapBuffers();
}

void FA05Display::idleMessage(char *msg, SM_RGB color)
{
    //scrollingLayer.drawString()
}

void FA05Display::displayLogo()
{
    uint16_t indx = 0;
    rgb24 pixelVal;

    for (int j = 0; j < NoVAlogo.height; j++)
    {
        for (int i = 0; i < NoVAlogo.width; i++)
        {
            pixelVal.red = NoVAlogo.pixel_data[NoVAlogo.bytes_per_pixel * indx + 0];
            pixelVal.green = NoVAlogo.pixel_data[NoVAlogo.bytes_per_pixel * indx + 1];
            pixelVal.blue = NoVAlogo.pixel_data[NoVAlogo.bytes_per_pixel * indx + 2];
            backgroundLayer.drawPixel(i, j, pixelVal);
            indx++;
        }
    }
    backgroundLayer.swapBuffers();
}

void FA05Display::updateIdleScreen(const machineStatus_BLE *fa05Status)
{
    char buf[4];
    static uint8_t lastStrip = 0;

    //clearScreen();
    displayLogo();

    backgroundLayer.setFont(gohufont11b);
    backgroundLayer.fillRectangle(24, 18, 40, 18 + 9, LED_BLACK);
    if (fa05Status->stripNum > 0)
    {
        sprintf(buf, "%u", fa05Status->stripNum);
        if (fa05Status->stripNum < 10)
        {
            backgroundLayer.drawString(30, 18, LED_PURPLE_HIGH, buf);
        }
        else
        {
            backgroundLayer.drawString(27, 18, LED_PURPLE_HIGH, buf);
        }
    }
    backgroundLayer.swapBuffers();
    lastStrip = fa05Status->stripNum;
}

void FA05Display::displayStatusScreen(unsigned long duration, char *msg)
{
    char buf[24];
    SM_RGB txtColor;
    extern eventCounter droppedPackets;
    extern eventCounter missedPackets;
    extern eventCounter totalPackets;
    extern bool isConnected;
    extern bool isPaired;

    float packetLoss;
    uint16_t missed = missedPackets.getCount();
    uint16_t dropped = droppedPackets.getCount();
    uint32_t total = totalPackets.getCount();

    backgroundLayer.fillScreen(LED_BLACK);
    backgroundLayer.setFont(font6x10);
    if (isConnected)
    {
        scrollingLayer1.setRefreshRate(120);
        scrollingLayer1.setSpeed(10);
        scrollingLayer1.setColor(LED_BLUE_MED);
        scrollingLayer1.setMode(wrapForwardFromLeft);
        scrollingLayer1.setStartOffsetFromLeft(15);
        scrollingLayer1.start(msg, -1);
        //backgroundLayer.drawString(0, 0, LED_BLUE_MED, msg);
        //backgroundLayer.drawString(0, 0, LED_BLUE_MED, msg);
        backgroundLayer.drawString(0, 10, LED_GREEN_MED, "30s =");
        packetLoss = ((float)missed / total);

        sprintf(buf, "%u,%2.1f,%u", total, packetLoss, dropped);

        if (missed >= 3)
        {
            txtColor = LED_RED_MED;
        }
        if (missed >= 2)
        {
            txtColor = LED_ORANGE_HIGH;
        }
        if (missed >= 1)
        {
            txtColor = LED_GREEN_MED;
        }
        if (missed == 0)
        {
            txtColor = LED_GREEN_HIGH;
        }
        backgroundLayer.drawString(0, 20, txtColor, buf);
    }
    else
    {
        backgroundLayer.drawString(0, 0, LED_RED_MED, "Not connected");
        if (isPaired)
        {
            backgroundLayer.drawString(0, 10, LED_BLUE_MED, "Paired");
        }
        else
        {
            backgroundLayer.drawString(0, 10, LED_RED_MED, "Un-paired");
        }
    }
    backgroundLayer.swapBuffers();
    delay(duration);
    scrollingLayer1.stop();
    backgroundLayer.fillScreen(LED_BLACK);
    backgroundLayer.swapBuffers();
}

void FA05Display::updateLights(fa05EspData *systemData)
{
    char buf[4];

    machineStatus_BLE *status = &(systemData->status);
    lightData_BLE *lights = &(systemData->lights);

    static SM_RGB lampLColor = LED_BLACK;
    static SM_RGB lampRColor = LED_BLACK;
    static SM_RGB faultLColor = LED_BLACK;
    static SM_RGB faultRColor = LED_BLACK;

    backgroundLayer.setFont(gohufont11b);
    backgroundLayer.fillRectangle(24, 0, 40, 0 + 9, LED_BLACK);
    if (status->stripNum > 0)
    {
        sprintf(buf, "%u", status->stripNum);
        if (status->stripNum < 10)
        {
            backgroundLayer.drawString(30, 0, LED_PURPLE_HIGH, buf);
        }
        else
        {
            backgroundLayer.drawString(27, 0, LED_PURPLE_HIGH, buf);
        }
    }

    lampLColor = LED_BLACK;
    if (lights->offTargetL == true)
    {
        //Serial.println("Off-target L");
        lampLColor = LED_WHITE_MED;
    }
    if (lights->touchL == true)
    {
        //Serial.println("Touch L");
        lampLColor = LED_RED_HIGH;
    }
    faultLColor = LED_BLACK;
    if (lights->gndFaultL)
    {
        faultLColor = LED_ORANGE_HIGH;
    }
    if (lights->lightsOnly)
    {
        backgroundLayer.fillRectangle(0, 0, 25, 14, LED_BLACK);
        backgroundLayer.fillRectangle(0, 18, 25, 31, LED_BLACK);
        if (lights->offTargetL == true)
        {
            backgroundLayer.fillRectangle(0, 18, 25, 31, LED_WHITE_MED);
        }
        if (lights->touchL == true)
        {
            backgroundLayer.fillRectangle(0, 0, 25, 14, LED_RED_HIGH);
        }
        backgroundLayer.fillRectangle(0, 15, 25, 17, faultLColor);
    }
    else
    {
        backgroundLayer.fillRectangle(0, 0, 23, 9, lampLColor);
        backgroundLayer.fillRectangle(0, 10, 23, 11, faultLColor);
    }

    lampRColor = LED_BLACK;
    if (lights->offTargetR == true)
    {
        //Serial.println("Off-target R");
        lampRColor = LED_WHITE_MED;
    }
    if (lights->touchR == true)
    {
        //Serial.println("Touch R");
        lampRColor = LED_GREEN_HIGH;
    }
    faultRColor = LED_BLACK;
    if (lights->gndFaultR)
    {
        faultRColor = LED_ORANGE_HIGH;
    }
    if (lights->lightsOnly)
    {
        backgroundLayer.fillRectangle(63 - 25, 0, 63, 14, LED_BLACK);
        backgroundLayer.fillRectangle(63 - 25, 18, 63, 31, LED_BLACK);
        if (lights->offTargetR == true)
        {
            backgroundLayer.fillRectangle(63 - 25, 18, 63, 31, LED_WHITE_MED);
        }
        if (lights->touchR == true)
        {
            backgroundLayer.fillRectangle(63 - 25, 0, 63, 14, LED_GREEN_HIGH);
        }
        backgroundLayer.fillRectangle(63 - 25, 15, 63, 17, faultRColor);
    }
    else
    {
        backgroundLayer.fillRectangle(63 - 23, 0, 63, 9, lampRColor);
        backgroundLayer.fillRectangle(63 - 23, 10, 63, 11, faultRColor);
        //backgroundLayer.swapBuffers();
        //Serial.println("Update R Lights");
    }
    //Serial.println("Lights updated");
    backgroundLayer.swapBuffers();
    //lightsChanged = false;
}

void FA05Display::updateScore(const matchData_BLE *scorePtr)
{
    static uint8_t cards = 0xff;
    static uint16_t timeRemainSec = 0;
    static uint8_t period = 1;
    static uint8_t scoreL = 0;
    static uint8_t scoreR = 0;
    SM_RGB clockColor = LED_GREEN_MED;
    char buf[5];

    //Process cards first
    if (cards != scorePtr->cardState)
    {
        updateCards(scorePtr);
        cards = scorePtr->cardState;
    }
    // Process clock next
    clockColor = LED_ORANGE_HIGH;
    if (scorePtr->clockRunning)
    {
        clockColor = LED_GREEN_HIGH;
    }

    backgroundLayer.setFont(font5x7);
    backgroundLayer.fillRectangle(27, 26, 24 + 2 * 6, 29, LED_BLACK);
    if (scorePtr->matchCount > 0)
    {
        sprintf(buf, "P%1.1u", scorePtr->matchCount);
        backgroundLayer.drawString(27, 24, LED_BLUE_HIGH, buf);
    }

    //backgroundLayer.swapBuffers();

    backgroundLayer.setFont(gohufont11);
    backgroundLayer.fillRectangle(21, 12, 21 + 4 * 6, 20, LED_BLACK);
    sprintf(buf, "%0.1u:%0.2u", scorePtr->timeRemainMin, scorePtr->timeRemainSec);
    backgroundLayer.drawString(21, 12, clockColor, buf);

    // Process score changes
    backgroundLayer.fillRectangle(5, 31 - 8, 5 + 16, 31, LED_BLACK);
    //backgroundLayer.setFont(gohufont11b);
    backgroundLayer.setFont(font8x13);
    sprintf(buf, "%-2u", scorePtr->scoreL);
    backgroundLayer.drawString(5, 31 - 10, LED_WHITE_MED, buf);

    backgroundLayer.fillRectangle(63 - (16 + 4), 23, 58, 31, LED_BLACK);
    backgroundLayer.setFont(font8x13);
    sprintf(buf, "%2u", scorePtr->scoreR);
    backgroundLayer.drawString(63 - (16 + 4), 31 - 10, LED_WHITE_MED, buf);
    backgroundLayer.swapBuffers();
    //scoreChanged = false;
}

void FA05Display::updateCards(const matchData_BLE *scorePtr)
{
    SM_RGB cardColor = LED_BLACK;

    backgroundLayer.setFont(font5x7);
    if (scorePtr->priorityL)
        backgroundLayer.drawString(4, 16, LED_GREEN_MED, "Pr");
    else
    {
        backgroundLayer.fillRectangle(4, 16, 4 + 2 * 5, 16 + 7, LED_BLACK);
    }
    if (scorePtr->priorityR)
        backgroundLayer.drawString(63 - 2 - 2 * 5, 16, LED_GREEN_MED, "Pr");
    else
    {
        backgroundLayer.fillRectangle(63 - 2 - 2 * 5, 16, 63 - 4, 16 + 7, LED_BLACK);
    }

    cardColor = (scorePtr->yellowL) ? LED_YELLOW_MED : LED_BLACK;
    backgroundLayer.fillRectangle(0, 31 - 8, 3, 31 - 5, cardColor);

    cardColor = (scorePtr->redL) ? LED_RED_MED : LED_BLACK;
    backgroundLayer.fillRectangle(0, 31 - 3, 3, 31, cardColor);

    cardColor = (scorePtr->yellowR) ? LED_YELLOW_MED : LED_BLACK;
    backgroundLayer.fillRectangle(63 - 3, 31 - 8, 63, 31 - 5, cardColor);

    cardColor = (scorePtr->redR) ? LED_RED_MED : LED_BLACK;
    backgroundLayer.fillRectangle(63 - 3, 31 - 3, 63, 31, cardColor);
    //backgroundLayer.swapBuffers();
}

void FA05Display::drawPixel(uint16_t x, uint16_t y, SM_RGB color)
{
    backgroundLayer.drawPixel(x, y, color);
    backgroundLayer.swapBuffers();
}

void FA05Display::runDemo()
{
    /*backgroundLayer.fillScreen(LED_WHITE_LOW);
    backgroundLayer.swapBuffers();
    delay(500);
    backgroundLayer.fillScreen(LED_BLACK);
    backgroundLayer.swapBuffers();

    setStatusText("Demo", LED_BLUE_MED);

    tLastActive = millis();
    fa05Lights.lightsOnly = 0;
    fa05Lights.touchL = 1;
    fa05Score.scoreL = 1;
    fa05Score.priorityL = 0;
    fa05Score.priorityR = 0;
    fa05Lights.gndFaultL = 0;
    fa05Lights.gndFaultR = 0;
    updateLights();
    updateScore();
    delay(5000);

    fa05Lights.offTargetR = 1;
    fa05Lights.touchL = 0;
    fa05Score.scoreR = 88;
    fa05Score.scoreL = 88;
    fa05Score.matchCount = 1;
    fa05Score.timeRemainMin = 2;
    fa05Score.timeRemainSec = 58;
    updateLights();
    updateScore();
    delay(2000);

    fa05Lights.offTargetR = 0;
    fa05Lights.offTargetL = 0;
    fa05Lights.touchL = 1;
    fa05Lights.touchR = 1;
    fa05Score.scoreR = 0;
    fa05Score.scoreL = 0;
    fa05Score.matchCount = 2;
    updateLights();
    updateScore();
    delay(1000);

    fa05Score.timeRemainMin = 0;
    fa05Score.clockRunning = 1;
    fa05Score.timeRemainSec = 07;
    updateScore();
    for (int k = 99; k >= 0; k--)
    {
        fa05Score.timeRemainSec = k;
        updateScore();
        delay(50);
    }
    fa05Score.clockRunning = 0;

    fa05Lights.touchL = 0;
    fa05Lights.touchR = 0;
    fa05Lights.offTargetR = 1;
    fa05Lights.offTargetL = 1;
    fa05Lights.gndFaultL = 1;
    fa05Lights.gndFaultR = 1;
    fa05Score.scoreL = 0;
    fa05Score.scoreR = 0;
    fa05Score.yellowL = 1;
    fa05Score.yellowR = 1;
    fa05Score.matchCount = 3;
    fa05Score.priorityL = 1;
    fa05Score.priorityR = 1;
    updateLights();
    updateScore();
    delay(500);

    fa05Score.redL = 1;
    fa05Score.redR = 1;
    updateLights();
    updateScore();
    delay(5000);

    backgroundLayer.fillScreen(LED_BLACK);
    backgroundLayer.swapBuffers();
    fa05Lights.lightsOnly = 1;
    fa05Lights.touchL = 1;
    fa05Lights.touchR = 1;
    fa05Lights.offTargetR = 1;
    fa05Lights.offTargetL = 1;
    fa05Lights.gndFaultL = 1;
    fa05Lights.gndFaultR = 1;
    updateLights();
    delay(5000);
    //updateScore();*/
}

void FA05Display::playEasterEgg()
{
    char buf[8];

    backgroundLayer.fillScreen(LED_BLACK);
    backgroundLayer.swapBuffers();
    backgroundLayer.setFont(gohufont11);

    for (uint8_t k = 10; k > 0; k--)
    {
        backgroundLayer.fillScreen(LED_BLACK);
        sprintf(buf, "%u", k);
        backgroundLayer.drawString(31 - 4, 11, LED_PURPLE_HIGH, buf);
        backgroundLayer.swapBuffers();
        delay(1000);
    }
    backgroundLayer.fillScreen(LED_BLACK);
    backgroundLayer.swapBuffers();

    scrollingLayer1.setRefreshRate(50);

    scrollingLayer2.setMode(bounceReverse);
    scrollingLayer2.setRefreshRate(50);
    scrollingLayer2.setSpeed(10);

    scrollingLayer3.setRefreshRate(50);
    scrollingLayer3.setMode(bounceReverse);
    scrollingLayer3.setSpeed(1);

    scrollingLayer1.setMode(bounceForward);
    scrollingLayer1.setStartOffsetFromLeft(17);
    scrollingLayer1.setFont(gohufont11b);
    scrollingLayer1.setColor(LED_PURPLE_HIGH);
    scrollingLayer1.setSpeed(10);
    scrollingLayer1.start("Happy", 1);

    delay(3300);
    scrollingLayer1.setSpeed(0);

    scrollingLayer2.setMode(bounceReverse);
    scrollingLayer2.setOffsetFromTop(16);
    scrollingLayer2.setStartOffsetFromLeft(8);
    scrollingLayer2.setFont(gohufont11b);
    scrollingLayer2.setColor({0, 205, 255});
    scrollingLayer2.setSpeed(15);
    scrollingLayer2.start("Birthday", 1);
    delay(2700);
    //scrollingLayer2.setMode(stopped);
    scrollingLayer2.setSpeed(0);
    delay(1000);

    scrollingLayer3.setFont(gohufont11b);
    scrollingLayer3.setOffsetFromTop(12);
    scrollingLayer2.setMode(bounceReverse);
    scrollingLayer3.setSpeed(5);
    scrollingLayer3.setColor(LED_GREEN_HIGH);
    scrollingLayer3.start("Greg!", 1);
    scrollingLayer1.setMode(bounceReverse);
    scrollingLayer2.setMode(bounceReverse);
    delay(500);
    scrollingLayer1.setSpeed(10);
    scrollingLayer2.setSpeed(10);

    delay(3000);

    unsigned long tic = millis();

    int x0 = 8;
    uint8_t y0 = 0;
    uint16_t indx = 0;

    for (x0 = -31; x0 < 64; x0++)
    {
        indx = 0;
        for (int i = 0; i < cake.height; i++)
        {
            for (int j = 0; j < cake.width; j++)
            {
                if ((x0 + j > 0) && (x0 + j < 64))
                {
                    backgroundLayer.drawPixel(x0 + j, y0 + i, rgb24(cake.pixel_data[indx + 0], cake.pixel_data[indx + 1], cake.pixel_data[indx + 2]));
                }
                indx += cake.bytes_per_pixel;
            }
        }
        backgroundLayer.swapBuffers();
        delay(150);
    }
    scrollingLayer3.stop();
}