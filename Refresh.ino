/* REFRESH FLIGHT COMPUTER SOFTWARE
 *  V0.11 - DECEMBER 2021
 *  LAVIE OHANA (@lavie154) / CALIFORNIA MODEL AEROSPACE
 *  
 *  It's okay, I have no idea what this code does either.
 *  
 *  Refresh is a 54mm form-factor flight computer designed for
 *  datalogging on high-power rockets. It's my first attempt at
 *  such a device, and definitely my first attempt at programming
 *  something this complex, so view this code at your own risk!
 *  
 *  Refresh's major subsystems:
 *  MCU   - Teensy 4.1
 *  IMU   - Bosch BMI088
 *  BARO  - Bosch BMP388
 *  GNSS  - CDtop PA1616S (Adafruit Ultimate GPS V3)
 *  RADIO - HopeRF RFM69HCW @ 915 MHz
 *  
 *  Pyrotechnic deployments and backup altimeter data are provided
 *  by a PerfectFlite Stratologger CF connected via UART.
 *  
 *  FYI - This code is INCOMPLETE! This is largely bashed together
 *  example code and will slowly approach something more flight-worthy
 *  as time goes on. Refresh itself is at about TRL 5 right now.
 *  
 *  If I were you, I wouldn't trust this code to work! Yet...
 */

#include "refreshSensorInit.h"
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF69.h>
#include "BMI088.h"

// what's the name of the hardware serial port?


// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// Single instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// *** BARO SETUP ***
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

// *** IMU SETUP ***

/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);

char Pstr[10];
char Cstr[10];
char Astr[10];
char LTstr[10];
char LNstr[10];
char AXstr[10];
char AYstr[10];
char AZstr[10];
char GXstr[10];
char GYstr[10];
char GZstr[10];

double C,P,A,LT,LN,AX,AY,AZ,GX,GY,GZ;
char buffer[50];

uint32_t timer = millis();


void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Refresh v0.11 - Booting Up");
  Serial.println("California Model Aerospace - 2021");
  Licc(41, 2);
  GNSSinit();

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  // ** RADIO SETUP ** 
    pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  // ** BARO SETUP **
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}
  int status;
  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
}
}

void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 50) {
    timer = millis(); // reset the timer
    /*
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
    */
     if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  accel.readSensor();
  gyro.readSensor();
   Serial.println(bmp.pressure / 100);
   P = bmp.readPressure() * 0.001;
   C = bmp.readTemperature();
   A = bmp.readAltitude(SEALEVELPRESSURE_HPA);
   LT = GPS.latitude;
   LN = GPS.longitude;
   AX = accel.getAccelX_mss();
   AY = accel.getAccelY_mss();
   AZ = accel.getAccelZ_mss();
   GX = gyro.getGyroX_rads();
   GY = gyro.getGyroY_rads();
   GZ = gyro.getGyroZ_rads();
   
   dtostrf (P, 3,2, Pstr);
   dtostrf (C, 3,2, Cstr);
   dtostrf (A, 3,2, Astr);
   dtostrf (LT, 3,2, LTstr);
   dtostrf (LN, 3,2, LNstr);
   dtostrf (AX, 3,2, AXstr);
   dtostrf (AY, 3,2, AYstr);
   dtostrf (AZ, 3,2, AZstr);
   dtostrf (GX, 3,2, GXstr);
   dtostrf (GY, 3,2, GYstr);
   dtostrf (GZ, 3,2, GZstr);
   
   sprintf(buffer, "%s %s %s %s %s %s %s %s", Pstr, Cstr, Astr, LTstr, LNstr, AXstr, AYstr, AZstr);

   //sendLen = strlen(buffer);
   uint8_t radiopacket[] = {bmp.pressure / 100.0};
   Serial.print("Sending "); Serial.println(buffer);
   rf69.send((uint8_t *)buffer, strlen(buffer));
   rf69.waitPacketSent();
  }
}
/* INITIALIZATION FUNCTIONS */
// These will likely get their own file someday, I just need to learn how to make that happen.
// These functions should hopefully come with 30% less screaming and crying.
void GNSSinit() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
}
void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
