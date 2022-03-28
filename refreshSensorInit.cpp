// 
// 
// 

/* call all dependencies*/
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF69.h>
#include "BMI088.h"
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>
#include "refreshSensorInit.h"
//Adafruit_GPS GPS(&GPSSerial);

  long time = millis();
  double SQUARE = 0; // acceleration
  double LINEAR = 0; // speed

double integrate(double IN){
  double newTime = millis();
  double deltaTime = newTime - time;
  double avgSQ = (SQUARE - IN);
  double newLINEAR = SQUARE + (avgSQ  * (deltaTime / 1000));
  LINEAR = newLINEAR;
  time = newTime;
  SQUARE = IN;
  return LINEAR;
}

void Licc(byte PIN, byte O){ // had to do it to em
  tone(PIN, (440 * pow(2,O)), 75);
  delay(150);
  tone(PIN, (493 * pow(2,O)), 75);
  delay(150);
  tone(PIN, (523 * pow(2,O)), 75);
  delay(150);
  tone(PIN, (587 * pow(2,O)), 75);
  delay(150);
  tone(PIN, (493 * pow(2,O)), 275);
  delay(300);
  tone(PIN, (391 * pow(2,O)), 75);
  delay(150);
  tone(PIN, (440 * pow(2,O)), 75);
  delay(150); // ya like jazz?
}
