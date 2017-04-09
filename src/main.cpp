#include <Arduino.h>
#include <Wire.h>
#include <rn2xx3.h>

#define DEBUG_SERIAL SerialUSB
#define LORA_SERIAL Serial1

#define WAKE_DURATION 300000 //(5 minutes in millis)

#define ACCEL_ADR 0x1E

unsigned long wake_time;

String AppEUI = "70B3D57EF00041F8";
String AppKey = "6F5A76EE295FB19C6E51095DD606498D";
String hweui = "";
bool join_result;

rn2xx3 LoRaWAN(LORA_SERIAL);


uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ACCEL_ADR, 0x01);

  uint8_t val = Wire.read();
  Wire.endTransmission();

  return val;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delayMicroseconds(10000);
}

void ISR1() {
  wake_time = millis();
}

/* Attempts to connect to LoRaWAN gateway */
void lora_connect() {
  DEBUG_SERIAL.println("[LoRaWAN] Initialising OTAA");
  join_result = LoRaWAN.initOTAA(AppEUI, AppKey);
  if (join_result) {
    DEBUG_SERIAL.println("[LoRaWAN] Connected to LoRaWAN successfully!");
  } else {
    DEBUG_SERIAL.println("[LoRaWAN] Failed to connect to LoRaWAN");
  }
}

/* Initialises RN2483 */
void init_radio() {
  delay(100);
  DEBUG_SERIAL.println("[LoRaWAN] autobauding...");
  LoRaWAN.autobaud();

  /* Wait for RN2483 to turn on */
  DEBUG_SERIAL.println("[LoRaWAN] getting hweui");
  while (hweui.length() != 16) {
    hweui = LoRaWAN.hweui();
  }
  DEBUG_SERIAL.println("[LoRaWAN] devEUI = " + hweui);
  DEBUG_SERIAL.println("[LoRaWAN] " + LoRaWAN.sysver());
  lora_connect();
}

/* Struct for lorawan packet */
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

/* Initialises accel */
void init_accel(){
  Wire.begin();

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; /* Set sleep mode */

  pinMode(ACCEL_INT1, INPUT_PULLUP);
  attachInterrupt(ACCEL_INT1, ISR1, FALLING);
  SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;  /* tell XOSC32K to run on standby */
  // Configure EIC to use GCLK1 which uses XOSC32K
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;

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

  LORA_SERIAL.begin(9600);
  DEBUG_SERIAL.begin(9600);
  DEBUG_SERIAL.println("[SYS] Hello World!");

  /* Accelerometer Interrupt */
  init_accel();
  init_radio();

  delay(10000);
  DEBUG_SERIAL.println("[SYS] Going to sleep mode");
}

void loop() {
  //Disable USB
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
  __WFI();              //Enter sleep mode
  //Enable USB
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;

  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_GREEN, LOW);
    delayMicroseconds(200000);
    digitalWrite(LED_GREEN, HIGH);
    delayMicroseconds(200000);
  }
  /* Stay awake for 5 mins and send data */
  while(millis() - wake_time < WAKE_DURATION){
    if(join_result){
      lora_pkt data = { 0, 12, 0, 0, 0, 0, 0, 0, 4 ,0};
      LoRaWAN.txBytes((byte*)&data, (uint8_t)sizeof(data));
      for (int i = 0; i < 10; i++) {
        digitalWrite(LED_BLUE, LOW);
        delayMicroseconds(200000);
        digitalWrite(LED_BLUE, HIGH);
        delayMicroseconds(200000);
      }
      //lora_pkt data = { 0, 12, 0, sodaq_gps.getLat(), sodaq_gps.getLon(), 0, 0, 0, sodaq_gps.getNumberOfSatellites() ,0};
    }else{
      lora_connect();
    }
    delay(5000);
  }
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_RED, LOW);
    delayMicroseconds(200000);
    digitalWrite(LED_RED, HIGH);
    delayMicroseconds(200000);
  }

}
