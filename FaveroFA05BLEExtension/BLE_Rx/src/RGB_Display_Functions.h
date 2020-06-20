#include "globals.h"

#ifndef RGB_FCNS_H
#define RGB_FCNS_H

#include "globals.h"
#include <SmartMatrix3.h>

#include "NOVA-FENCING-CLUB-LOGO-tiny.c"
#include "cake.c"


// ========================== CONFIG START ===================================================

/// SmartMatrix Defines
#define COLOR_DEPTH 24                  // known working: 24, 48 - If the sketch uses type `rgb24` directly, COLOR_DEPTH must be 24
const uint8_t kMatrixWidth = 64;        // known working: 32, 64, 96, 128
const uint8_t kMatrixHeight = 32;       // known working: 16, 32, 48, 64
const uint8_t kRefreshDepth = 36;       // known working: 24, 36, 48
const uint8_t kDmaBufferRows = 4;       // known working: 2-4, use 2 to save memory, more to keep from dropping frames and automatically lowering refresh rate
const uint8_t kPanelType = SMARTMATRIX_HUB75_32ROW_MOD16SCAN;   // use SMARTMATRIX_HUB75_16ROW_MOD8SCAN for common 16x32 panels
const uint8_t kMatrixOptions = (SMARTMATRIX_OPTIONS_NONE);      // see http://docs.pixelmatix.com/SmartMatrix for options
const uint8_t kIndexedLayerOptions = (SM_INDEXED_OPTIONS_NONE);
const uint8_t kBackgroundLayerOptions = (SM_BACKGROUND_OPTIONS_NONE);
const uint8_t kScrollingLayerOptions = (SM_SCROLLING_OPTIONS_NONE);

SMARTMATRIX_ALLOCATE_BUFFERS(matrix, kMatrixWidth, kMatrixHeight, kRefreshDepth, kDmaBufferRows, kPanelType, kMatrixOptions);
SMARTMATRIX_ALLOCATE_BACKGROUND_LAYER(backgroundLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kBackgroundLayerOptions);
SMARTMATRIX_ALLOCATE_SCROLLING_LAYER(scrollingLayer1, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kScrollingLayerOptions);
SMARTMATRIX_ALLOCATE_SCROLLING_LAYER(scrollingLayer2, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kScrollingLayerOptions);
SMARTMATRIX_ALLOCATE_SCROLLING_LAYER(scrollingLayer3, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kScrollingLayerOptions);
//SMARTMATRIX_ALLOCATE_INDEXED_LAYER(indexedLayer, kMatrixWidth, kMatrixHeight, COLOR_DEPTH, kIndexedLayerOptions);

const int defaultBrightness = (100 * 255) / 100;  // full (100%) brightness
const int dimBrightness = (20 * 255) / 100;  // dim (20%) brightness
// ========================== CONFIG END ======================================================

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

byte statusTextLocX = 25;
byte statusTextLocY = 31 - 5;


void setStatusText(char *msg, SM_RGB color);
void clearScreen();
void idleMessage(char *msg, SM_RGB color);
void displayLogo();
void updateIdleScreen();
void displayStatusScreen(unsigned long duration, float RSSI);
void updateLights();
void updateScore();
void updateCards();
void runDemo();
void playEasterEgg();


#endif