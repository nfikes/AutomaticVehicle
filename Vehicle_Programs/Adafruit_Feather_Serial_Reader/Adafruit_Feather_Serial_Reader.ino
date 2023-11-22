// Last Update 11/22/23 
//
// TX pin reading on RX, replicates serial monitor output from print sequence.
// Created by Nathan Fikes for use of displaying serial from [Arduino Nano] to [Adafruit Feather TFT ESP32-S2]

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int RX_PIN = 2;
int TX_PIN = 1;

void setup() {
  Serial.begin(115200); // ope`ns serial port, sets data rate to x bps
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT
  tft.init(135, 240); // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  Serial.println(F("Initialized"));
}

void loop() {
  // reply only when you receive data:
  if (Serial1.available()) {
    char receivedChar = Serial1.read();
    // Process the received data
    Serial.print(receivedChar);
    char buf[2] = {receivedChar, 0};
    tft.setTextColor(ST77XX_WHITE);
    testdrawtext(buf, ST77XX_WHITE);
  }
}

void testdrawtext(char *text, uint16_t color) {
  static int x = 0;
  static int y = 0;
  tft.setCursor(x, y);
  x += 5;

  if (text[0] == '\n'){
    x = 0;
    y += 10;
  } else if (x == 200){
    x = 0;
    y += 10;
  }

  if ((y == 80) || (text[0] == '\u0001')){
    tft.fillScreen(ST77XX_BLACK);
    x = y = 0;
  }

  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}