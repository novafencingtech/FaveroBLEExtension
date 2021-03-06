#include "RGB_Display_Functions.h"

void setStatusText(char *msg, SM_RGB color) {
  backgroundLayer.fillRectangle(20, 23, 43, 31, LED_BLACK);
  backgroundLayer.setFont(font3x5);
  //backgroundLayer.drawString(statusTextLocX, statusTextLocY, LED_BLACK, "    ");
  backgroundLayer.drawString(statusTextLocX, statusTextLocY, color, msg);
  backgroundLayer.swapBuffers();
}

void clearScreen() {
  backgroundLayer.fillScreen(LED_BLACK);
  backgroundLayer.swapBuffers();
}

void idleMessage(char *msg, SM_RGB color) {
  //scrollingLayer.drawString()
}

void displayLogo() {
  uint16_t indx = 0;
  rgb24 pixelVal;

  for (int j = 0; j < NoVAlogo.height; j++) {
    for (int i = 0; i < NoVAlogo.width; i++) {
      pixelVal.red = NoVAlogo.pixel_data[NoVAlogo.bytes_per_pixel * indx + 0];
      pixelVal.green = NoVAlogo.pixel_data[NoVAlogo.bytes_per_pixel * indx + 1];
      pixelVal.blue = NoVAlogo.pixel_data[NoVAlogo.bytes_per_pixel * indx + 2];
      backgroundLayer.drawPixel(i, j, pixelVal);
      indx++;
    }
  }
  backgroundLayer.swapBuffers();
}
void updateIdleScreen() {
  char buf[4];
  static uint8_t lastStrip = 0;

  //clearScreen();
  displayLogo();

  backgroundLayer.setFont(gohufont11b);
  backgroundLayer.fillRectangle(24, 18, 40, 18 + 9, LED_BLACK);
  if (fa05Status.stripNum > 0) {
    sprintf(buf, "%u", fa05Status.stripNum);
    if (fa05Status.stripNum < 10) {
      backgroundLayer.drawString(30, 18, LED_PURPLE_HIGH, buf);
    } else {
      backgroundLayer.drawString(27, 18, LED_PURPLE_HIGH, buf);
    }
  }
  backgroundLayer.swapBuffers();
  lastStrip = fa05Status.stripNum;
}

void displayStatusScreen(unsigned long duration, float RSSI) {
  char buf[24];
  SM_RGB txtColor;

  backgroundLayer.fillScreen(LED_BLACK);
  backgroundLayer.setFont(font6x10);
  if (isConnected) {
    backgroundLayer.drawString(0, 0, LED_GREEN_HIGH, "Connected");
    //backgroundLayer.drawString(0, 10, LED_BLUE_HIGH, RxBLEClient->getPeerAddress().toString().c_str());
    sprintf(buf, "RS=%3.0f", RSSI);

    if (RSSI > -97) {
      txtColor = LED_RED_MED;
    }
    if (RSSI > -92) {
      txtColor = LED_ORANGE_HIGH;
    }
    if (RSSI > -90) {
      txtColor = LED_GREEN_MED;
    }
    if (RSSI > -85) {
      txtColor = LED_GREEN_HIGH;
    }
    backgroundLayer.drawString(0, 20, txtColor, buf);
  } else {
    backgroundLayer.drawString(0, 0, LED_RED_MED, "Not connected");
    /*if (txPaired) {
      backgroundLayer.drawString(0, 10, LED_BLUE_MED, "Paired");
    } else {
      backgroundLayer.drawString(0, 10, LED_RED_MED, "Un-paired");
    }*/

  }
  backgroundLayer.swapBuffers();
  delay(duration);
  backgroundLayer.fillScreen(LED_BLACK);
  backgroundLayer.swapBuffers();
}

void updateLights() {
  char buf[4];

  static SM_RGB lampLColor = LED_BLACK;
  static SM_RGB lampRColor = LED_BLACK;
  static SM_RGB faultLColor = LED_BLACK;
  static SM_RGB faultRColor = LED_BLACK;

  backgroundLayer.setFont(gohufont11b);
  backgroundLayer.fillRectangle(24, 0, 40, 0 + 9, LED_BLACK);
  if (fa05Status.stripNum > 0) {
    sprintf(buf, "%u", fa05Status.stripNum);
    if (fa05Status.stripNum < 10) {
      backgroundLayer.drawString(30, 0, LED_PURPLE_HIGH, buf);
    } else {
      backgroundLayer.drawString(27, 0, LED_PURPLE_HIGH, buf);
    }
  }

  lampLColor = LED_BLACK;
  if (fa05Lights.offTargetL == true) {
    //Serial.println("Off-target L");
    lampLColor = LED_WHITE_MED;
  }
  if (fa05Lights.touchL == true) {
    //Serial.println("Touch L");
    lampLColor = LED_RED_HIGH;
  }
  faultLColor = LED_BLACK;
  if (fa05Lights.gndFaultL) {
    faultLColor = LED_ORANGE_HIGH;
  }
  if (fa05Lights.lightsOnly) {
    backgroundLayer.fillRectangle(0, 0, 25, 14, LED_BLACK);
    backgroundLayer.fillRectangle(0, 18, 25, 31, LED_BLACK);
    if (fa05Lights.offTargetL == true) {
      backgroundLayer.fillRectangle(0, 18, 25, 31, LED_WHITE_MED);
    }
    if (fa05Lights.touchL == true) {
      backgroundLayer.fillRectangle(0, 0, 25, 14, LED_RED_HIGH);
    }
    backgroundLayer.fillRectangle(0, 15, 25, 17, faultLColor);
  } else {
    backgroundLayer.fillRectangle(0, 0, 23, 9, lampLColor);
    backgroundLayer.fillRectangle(0, 10, 23, 11, faultLColor);
  }

  lampRColor = LED_BLACK;
  if (fa05Lights.offTargetR == true) {
    //Serial.println("Off-target R");
    lampRColor = LED_WHITE_MED;
  }
  if (fa05Lights.touchR == true) {
    //Serial.println("Touch R");
    lampRColor = LED_GREEN_HIGH;
  }
  faultRColor = LED_BLACK;
  if (fa05Lights.gndFaultR) {
    faultRColor = LED_ORANGE_HIGH;
  }
  if (fa05Lights.lightsOnly) {
    backgroundLayer.fillRectangle(63 - 25, 0, 63, 14, LED_BLACK);
    backgroundLayer.fillRectangle(63 - 25, 18, 63, 31, LED_BLACK);
    if (fa05Lights.offTargetR == true) {
      backgroundLayer.fillRectangle(63 - 25, 18, 63, 31, LED_WHITE_MED);
    }
    if (fa05Lights.touchR == true) {
      backgroundLayer.fillRectangle(63 - 25, 0, 63, 14, LED_GREEN_HIGH);
    }
    backgroundLayer.fillRectangle(63 - 25, 15, 63, 17, faultRColor);
  } else {
    backgroundLayer.fillRectangle(63 - 23, 0, 63, 9, lampRColor);
    backgroundLayer.fillRectangle(63 - 23, 10, 63, 11, faultRColor);
    //backgroundLayer.swapBuffers();
    //Serial.println("Update R Lights");
  }
  //Serial.println("Lights updated");
  backgroundLayer.swapBuffers();
  lightsChanged = false;
}

void updateScore() {
  static uint8_t cardState = 0xff;
  static uint16_t timeRemainSec = 0;
  static uint8_t period = 1;
  static uint8_t scoreL = 0;
  static uint8_t scoreR = 0;
  SM_RGB clockColor = LED_GREEN_MED;
  char buf[5];

  //Process cards first
  if (cardState != fa05Score.rawBytes[matchDataBLEPacketSize - 1]) {
    updateCards();
    cardState = fa05Score.rawBytes[matchDataBLEPacketSize - 1];
  }
  // Process clock next
  clockColor = LED_ORANGE_HIGH;
  if (fa05Score.clockRunning) {
    clockColor = LED_GREEN_HIGH;
  }

  backgroundLayer.setFont(font5x7);
  backgroundLayer.fillRectangle(27, 26, 26 + 2 * 6, 31, LED_BLACK);
  if (fa05Score.matchCount > 0) {
    sprintf(buf, "P%1.1u", fa05Score.matchCount);
    backgroundLayer.drawString(27, 26, LED_BLUE_HIGH, buf);
  }

  //backgroundLayer.swapBuffers();

  backgroundLayer.setFont(gohufont11);
  backgroundLayer.fillRectangle(21, 12, 21 + 4 * 6, 20, LED_BLACK);
  sprintf(buf, "%0.1u:%0.2u", fa05Score.timeRemainMin, fa05Score.timeRemainSec);
  backgroundLayer.drawString(21, 12, clockColor, buf);

  // Process score changes
  backgroundLayer.fillRectangle(5, 31 - 8, 5 + 16, 31, LED_BLACK);
  //backgroundLayer.setFont(gohufont11b);
  backgroundLayer.setFont(font8x13);
  sprintf(buf, "%-2u", fa05Score.scoreL);
  backgroundLayer.drawString(5, 31 - 10, LED_WHITE_MED, buf);

  backgroundLayer.fillRectangle(63 - (16 + 4), 23, 58, 31, LED_BLACK);
  backgroundLayer.setFont(font8x13);
  sprintf(buf, "%2u", fa05Score.scoreR);
  backgroundLayer.drawString( 63 - (16 + 4), 31 - 10, LED_WHITE_MED, buf);
  backgroundLayer.swapBuffers();
  scoreChanged = false;
}
void updateCards() {
  SM_RGB cardColor = LED_BLACK;

  backgroundLayer.setFont(font5x7);
  if (fa05Score.priorityL)
    backgroundLayer.drawString(4, 16, LED_GREEN_MED, "Pr");
  else {
    backgroundLayer.fillRectangle(4, 16, 4 + 2 * 5, 16 + 7, LED_BLACK);
  }
  if (fa05Score.priorityR)
    backgroundLayer.drawString(63 - 2 - 2 * 5, 16, LED_GREEN_MED, "Pr");
  else {
    backgroundLayer.fillRectangle(63 - 2 - 2 * 5, 16, 63 - 4, 16 + 7, LED_BLACK);
  }

  cardColor = (fa05Score.yellowL) ? LED_YELLOW_MED : LED_BLACK;
  backgroundLayer.fillRectangle(0, 31 - 8, 3, 31 - 5, cardColor);

  cardColor = (fa05Score.redL) ? LED_RED_MED : LED_BLACK;
  backgroundLayer.fillRectangle(0, 31 - 3, 3, 31, cardColor);

  cardColor = (fa05Score.yellowR) ? LED_YELLOW_MED : LED_BLACK;
  backgroundLayer.fillRectangle(63 - 3, 31 - 8, 63, 31 - 5, cardColor);

  cardColor = (fa05Score.redR) ? LED_RED_MED : LED_BLACK;
  backgroundLayer.fillRectangle(63 - 3, 31 - 3, 63, 31, cardColor);
  //backgroundLayer.swapBuffers();
}

void runDemo() {
  backgroundLayer.fillScreen(LED_WHITE_LOW);
  backgroundLayer.swapBuffers();
  delay(500);
  backgroundLayer.fillScreen(LED_BLACK);
  backgroundLayer.swapBuffers();

  setStatusText("Demo", LED_BLUE_MED);

  //tLastActive = millis();
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
  for (int k = 99; k >= 0; k--) {
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
  //updateScore();
}

void playEasterEgg() {
  char buf[8];
  
  backgroundLayer.fillScreen(LED_BLACK);
  backgroundLayer.swapBuffers();
  backgroundLayer.setFont(gohufont11);

  for (uint8_t k=10;k>0;k--) {
    backgroundLayer.fillScreen(LED_BLACK);    
    sprintf(buf,"%u",k);
    backgroundLayer.drawString(31-4,11,LED_PURPLE_HIGH,buf);
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
  scrollingLayer1.start("Happy",1);
  
  delay(3300);
  scrollingLayer1.setSpeed(0);    

  scrollingLayer2.setMode(bounceReverse);
  scrollingLayer2.setOffsetFromTop(16);
  scrollingLayer2.setStartOffsetFromLeft(8);
  scrollingLayer2.setFont(gohufont11b);
  scrollingLayer2.setColor({0,205,255});
  scrollingLayer2.setSpeed(15);
  scrollingLayer2.start("Birthday",1);
  delay(2700);
  //scrollingLayer2.setMode(stopped);
  scrollingLayer2.setSpeed(0);
  delay(1000);

  scrollingLayer3.setFont(gohufont11b);
  scrollingLayer3.setOffsetFromTop(12);
  scrollingLayer2.setMode(bounceReverse);
  scrollingLayer3.setSpeed(5);
  scrollingLayer3.setColor(LED_GREEN_HIGH);
  scrollingLayer3.start("Greg!",1);
  scrollingLayer1.setMode(bounceReverse);
  scrollingLayer2.setMode(bounceReverse);
  delay(500);
  scrollingLayer1.setSpeed(10);
  scrollingLayer2.setSpeed(10);
     
  delay(3000);

  unsigned long tic=millis();

  int x0=8;
  uint8_t y0=0;
  uint16_t indx=0;

  for (x0=-31;x0<64;x0++) {
    indx=0;
    for (int i=0;i<cake.height;i++) {
    for (int j=0;j<cake.width;j++){
      if ((x0+j>0) && (x0+j<64)) {
        backgroundLayer.drawPixel(x0+j,y0+i,rgb24(cake.pixel_data[indx+0],cake.pixel_data[indx+1],cake.pixel_data[indx+2]));
      }
      indx+=cake.bytes_per_pixel;
    }
  }
  backgroundLayer.swapBuffers();
  delay(150);
  }
  scrollingLayer3.stop();  
}
