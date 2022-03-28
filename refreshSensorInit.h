// refreshSensorInit.h

#ifndef _REFRESHSENSORINIT_h
#define _REFRESHSENSORINIT_h

/* RF Setup definitions */
#define RF69_FREQ 915.0
#define RFM69_RST 38
#define RFM69_CS 10
#define RFM69_INT 37
#define LED 36
#define RFM69_IRQN digitalPinToInterrupt(RFM69_INT )

#define GPSSerial Serial1 // define GPS UART port

//void GNSSinit();

void Licc(byte PIN, byte O);
double integrate(double IN);
#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif
