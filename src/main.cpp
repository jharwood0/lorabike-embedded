#include <Arduino.h>
#include <Wire.h>
#include <rn2xx3.h>
#include <Sodaq_UBlox_GPS.h>

#define DEBUG_SERIAL SerialUSB
#define LORA_SERIAL Serial1

#define WAKE_DURATION 300000 //(5 minutes in millis)

/* Accel Interrupts */
#define ACCEL_ADR 0b0011110
volatile bool int1_flag = false;
volatile bool int2_flag = false;

unsigned long wake_time;
bool wake;

String AppEUI = "70B3D57EF00041F8";
String AppKey = "6F5A76EE295FB19C6E51095DD606498D";
String hweui = "";
bool join_result;

rn2xx3 LoRaWAN(LORA_SERIAL);


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

void init_radio(){
  delay(100);
  LORA_SERIAL.flush();
  LoRaWAN.autobaud();

  /* Wait for RN2483 to turn on */
  DEBUG_SERIAL.println("Getting hweui");
  while(hweui.length() != 16){
    hweui = LoRaWAN.hweui();
    DEBUG_SERIAL.println("Can't get HWEUI");
  }
  DEBUG_SERIAL.println("Register device using: ");
  DEBUG_SERIAL.println(hweui);
  DEBUG_SERIAL.println("RN2xx3 firmware version:");
  DEBUG_SERIAL.println(LoRaWAN.sysver());
  join_result = LoRaWAN.initOTAA(AppEUI, AppKey);
  if(join_result){
    DEBUG_SERIAL.println("Connected to LoRaWAN successfully!");
  }else{
    DEBUG_SERIAL.println("Failed to connect to LoRaWAN");
  }
}

struct lora_pkt {
  long epoch;
  uint8_t bat;
  int8_t temp;
  int32_t lat;
  int32_t lng;
  int16_t altitude;
  int16_t speed;
  uint8_t course;
  uint8_t sat;
  uint8_t ttf;
};

void init_gps(){
  sodaq_gps.init(GPS_ENABLE);
  sodaq_gps.setDiag(DEBUG_SERIAL);
}

void init_accel(){
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


void setup(){
  while ((!SerialUSB) && (millis() < 10000)) {
    // Wait for SerialUSB or start after 30 seconds
  }
  /* Open serial ports */
  LORA_SERIAL.begin(9600);
  DEBUG_SERIAL.begin(9600);

  /* Accelerometer Interrupt */
  init_accel();

  /* LoRaWAN */
  init_radio();

  /* GPS */
  init_gps();

  wake = false;
}

void loop(){
  if (int1_flag || int2_flag) {
    wake_time = millis();
    wake = true;
    int2_flag = false;
    int1_flag = false;
    DEBUG_SERIAL.println("Interrupt!");
  }
  if(wake){
    DEBUG_SERIAL.println("I AM AWAKE");

    DEBUG_SERIAL.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
    DEBUG_SERIAL.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
    DEBUG_SERIAL.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));

    lora_pkt data = { 0, 12, 0, sodaq_gps.getLat(), sodaq_gps.getLon(), 0, 0, 0, sodaq_gps.getNumberOfSatellites() ,0};
    LoRaWAN.txBytes((byte*)&data, (uint8_t)sizeof(data));

    if(millis() - wake_time >= WAKE_DURATION){
      DEBUG_SERIAL.println("GOING TO SLEEP......");
      wake = false;
    }
  }else{
    DEBUG_SERIAL.println("I AM ASLEEP");
  }


    int16_t x_val = (readReg(0x29) << 8) | readReg(0x28);
    int16_t y_val = (readReg(0x2B) << 8) | readReg(0x2A);
    int16_t z_val = (readReg(0x2D) << 8) | readReg(0x2C);

    DEBUG_SERIAL.println(String("Accelerometer Readings: ") + x_val + ", " + y_val + ", " + z_val);



  for (int i=0; i<1000; i++) {
    delayMicroseconds(1000);
  }
}
