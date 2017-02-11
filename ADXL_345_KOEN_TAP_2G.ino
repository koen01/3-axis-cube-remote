/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************
*/
#define MY_DEBUG
#define MY_RF24_PA_LEVEL RF24_PA_MIN
#define MY_RADIO_NRF24
#include <MySensors.h>
#include <SparkFun_ADXL345.h>
//#include <avr/sleep.h>
#include <avr/power.h>
#define CHILD_ID 1   // Id of the sensor child
#define MY_NODE_ID 39
#define BATT_SENSOR 199
#define TAP_SENSOR 2
#define ACTIVITY_SENSOR 3

// custom Orientation type
#define V_ORIENTATION V_TEXT
#define S_ORIENTATION S_INFO

// standard messages
MyMessage orientationMsg(CHILD_ID, V_ORIENTATION);
MyMessage msgBatt(BATT_SENSOR, V_VOLTAGE);
MyMessage msgTap(TAP_SENSOR, V_STATUS);
MyMessage msgActivity(ACTIVITY_SENSOR, V_TRIPPED);

/*********** COMMUNICATION SELECTION ***********/
ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

/****************** INTERRUPT ******************/
/*      Uncomment If Attaching Interrupt       */
const byte interruptPin = 2;               // Setup pin 2 to be the interrupt pin (for most Arduino Boards)pinMode(interruptPin, INPUT_PULLUP);
boolean transmission_occured = false;
boolean tapStatus = false;
long lastBattery = -100;

/****************** AXIS ******************/
int MAX_X = 265;
int MIN_X = -265;
int MAX_Y = 265;
int MIN_Y = -265;
int MAX_Z = 265;
int MIN_Z = -265;
int range = 25;


/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  Serial.begin(9600);                 // Start the serial terminal
  Serial.println("3-axis cube sensor");
  Serial.println();

  adxl.powerOn();                     // Power on the ADXL345
  adxl.setActivityAc(1);             // This is important for getting the right kind of activiy detection for the cube.
  adxl.setLowPower(1);
  adxl.setRangeSetting(2);           // Give the range settings
  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1

  adxl.setActivityXYZ(1, 1, 1);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(20);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
  adxl.setInactivityXYZ(1, 1, 1);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(40);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(5);         // How many seconds of no activity is inactive?
  adxl.setTapDetectionOnXYZ(1, 1, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)

  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(100);           // 62.5 mg per increment
  adxl.setTapDuration(25);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment

  // Setting all interupts to take place on INT1 pin
  adxl.setImportantInterruptMapping(1, 1, 1, 1, 2);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);"
  // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
  // This library may have a problem using INT2 pin. Default to INT1 pin.

  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.doubleTapINT(1);
  adxl.singleTapINT(1);
  adxl.InactivityINT(0);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(0);
}


void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("3-Axis Sensor", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID, S_ORIENTATION);
  present(BATT_SENSOR, S_POWER);
  present(TAP_SENSOR, S_BINARY);
  present(ACTIVITY_SENSOR, S_MOTION);
}

void loop() {
  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  byte interrupts = adxl.getInterruptSource();
  // Tap
  if (adxl.triggered(interrupts, ADXL345_SINGLE_TAP)) {
    tapStatus = !tapStatus;
    Serial.println("*** TAP ***");
    send(msgTap.set(tapStatus ? 1 : 0)) ;
  }
  // Activity
  else if (adxl.triggered(interrupts, ADXL345_ACTIVITY)) {
    Serial.println("*** ACTIVITY ***");
    send(msgActivity.set(1)) ;
    Serial.println("Wait for it.....");
    delay(1500);
    int x, y, z;
    adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
    if (z >= (MAX_Z - range) ) {
      Serial.println("Z+");
      send(orientationMsg.set("Z+")) ;
    }
    else if (z <= (MIN_Z + range) ) {
      Serial.println("Z-");
      send(orientationMsg.set("Z-")) ;
    }
    else if (x <= (MIN_X + range) ) {
      Serial.println("X-");
      send(orientationMsg.set("X-")) ;
    }
    else if (x >= (MAX_X - range) ) {
      Serial.println("X+");
      send(orientationMsg.set("X+")) ;
    }
    else if (y <= (MIN_Y + range) ) {
      Serial.println("Y-");
      send(orientationMsg.set("Y-")) ;
    }
    else if (y >= (MAX_Y - range) ) {
      Serial.println("Y+");
      send(orientationMsg.set("Y+")) ;
    }
    else {
      Serial.println("Error");
      char payload[30];
      sprintf(payload, "%d;%d;%d", x, y, z);
      send(orientationMsg.set(payload)) ;        // Send to controller
    }
  }

  sendBattLevel(true);
  send(msgActivity.set(0)) ;
  sleep(digitalPinToInterrupt(interruptPin), RISING, 0);
}

/********************************************

   Sends battery information (battery percentage)

   Parameters
   - force : Forces transmission of a value

 *******************************************/
void sendBattLevel(bool force)
{
  if (force) lastBattery = -1;
  long vcc = readVcc();
  if (vcc != lastBattery) {
    lastBattery = vcc;

#ifdef BATT_SENSOR
    float send_voltage = float(vcc) / 1000.0f;
    send(msgBatt.set(send_voltage, 3));
#endif

    // Calculate percentage

    vcc = vcc - 1900; // subtract 1.9V from vcc, as this is the lowest voltage we will operate at

    long percent = vcc / 14.0;
    sendBatteryLevel(percent);
    transmission_occured = true;
  }
}


/*******************************************

   Internal battery ADC measuring

 *******************************************/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADcdMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts

}
