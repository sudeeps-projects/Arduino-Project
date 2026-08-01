#include <Adafruit_DRV2605.h>

Adafruit_DRV2605 drv;
int drvFlag = 0;
void setup() {
  Serial.begin(9600);
  delay(5000);
  Serial.println("Starting DRV2605L Test...");

  if (!drv.begin()) {
    Serial.println("ERROR: DRV2605L NOT FOUND");
    drvFlag = 0;
    while (1)
    {
      Serial.println("ERROR: DRV2605L NOT FOUND");
      delay(5000);
    }
  }
  drvFlag = 1;
  Serial.println("DRV2605L FOUND");
  delay(5000);
  drv.selectLibrary(1);
  drv.useERM();          // Use for coin vibration motors

  Serial.println("Ready");
}

void loop() {

  Serial.println("Vibrating...");
  if (drvFlag == 1 )
  {
      Serial.println("DRV2605L FOUND");
  }
  else 
  {
      Serial.println("DRV2605L NOT FOUND");
  }
  drv.setWaveform(0, 1);   // Strong click
  drv.setWaveform(1, 1);   // Strong click
  drv.setWaveform(2, 1); 
  drv.setWaveform(3, 0);    // End sequence

  drv.go();

  delay(3000);
}