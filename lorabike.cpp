#include <Arduino.h>
#include <Wire.h>
#include "Sodaq_RN2483.h"
#include <Sodaq_UBlox_GPS.h>

#define DEBUG_SERIAL SerialUSB
#define LORA_SERIAL Serial1
/* Accel Interrupts */
#define ACCEL_ADR 0b0011110
volatile bool int1_flag = false;
volatile bool int2_flag = false;


uint8_t DevEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t AppEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t AppKey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

void setupLoRaOTAA(){
  if (LoRaBee.initOTA(LORA_SERIAL, DevEUI, AppEUI, AppKey, false))
  {
    DEBUG_SERIAL.println("Communication to LoRaBEE successful.");
  }
  else
  {
    DEBUG_SERIAL.println("OTAA Setup failed!");
  }
}

void setupGPS(){
  sodaq_gps.init(GPS_ENABLE);
  sodaq_gps.setDiag(DEBUG_SERIAL);
}

void setup(){
  /* Enable GPS */
  /* Accelerometer Interrupt */
  Wire.begin();
  pinMode(ACCEL_INT1, INPUT_PULLUP);
  pinMode(ACCEL_INT2, INPUT_PULLUP);
  attachInterrupt(ACCEL_INT1, ISR1, FALLING);
  attachInterrupt(ACCEL_INT2, ISR2, FALLING);

  writeReg(0x1F, 0b10000000); /* Reboot */
  writeReg(0x20, 0b01010111); /* Set to 50z all axes active */

  // Interrupt source 1
  // Set to be Y sensitive
  // IG_SRC1
  writeReg(0x30, 0b10001000); // Axes mask
  writeReg(0x32, 0b00111111); // Threshold
  writeReg(0x33, 0b00000000); // Duration

  // Interrupt source 2
  // Set to be X sensitive
  // IG_SRC2
  writeReg(0x34, 0b10000010); // Axes mask
  writeReg(0x36, 0b00111111); // Threshold
  writeReg(0x37, 0b00000000); // Duration

  writeReg(0x22, 0b00100000); // INT1
  writeReg(0x23, 0b00100000); // INT2
}

void loop(){
  if (int1_flag || int2_flag) {
    int2_flag = false;
    int1_flag = false;
    DEBUG_SERIAL.println("Interrupt!");
  }

    int16_t x_val = (readReg(0x29) << 8) | readReg(0x28);
    int16_t y_val = (readReg(0x2B) << 8) | readReg(0x2A);
    int16_t z_val = (readReg(0x2D) << 8) | readReg(0x2C);

    DEBUG_SERIAL.println(String("Accelerometer Readings: ") + x_val + ", " + y_val + ", " + z_val);

    DEBUG_SERIAL.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
    DEBUG_SERIAL.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
    DEBUG_SERIAL.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));


  for (int i=0; i<1000; i++) {
    delayMicroseconds(1000);
  }
}


uint8_t readReg(uint8_t reg)
{
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ACCEL_ADR, 0x01);

  uint8_t val = Wire.read();
  Wire.endTransmission();

  return val;
}

uint8_t writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delayMicroseconds(10000);
}

void ISR1()
{
  int1_flag = true;
}

void ISR2()
{
  int2_flag = true;
}
