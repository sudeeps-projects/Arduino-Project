#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  8

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  tft.init(240, 240);
  tft.setRotation(0);

  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(35, 20);
  tft.println("SilentSleep");

  tft.drawLine(10, 50, 230, 50, ST77XX_WHITE);

  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.setCursor(45, 90);
  tft.println("Listening");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(30, 145);
  tft.println("Waiting for");
  tft.setCursor(60, 175);
  tft.println("sound...");
}

void loop() {
}
