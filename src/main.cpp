#include <TinyGPS.h> /* GPS Lib */
#include <rn2xx3.h> /* RN2483 Lib */

/*
STM32 port mappings
UART1 -> Serial  -> DEBUG
UART2 -> Serial1 -> GPS
UART3 -> Serial2 -> LoRaWAN
*/

String APPEUI = "70B3D57EF00041F8";
String APPKEY = "6F5A76EE295FB19C6E51095DD606498D";

#define DEBUG 1
#define ENABLE_GPS 1
#define ENABLE_LORAWAN 1

const int debug_baud = 9600;
const int gps_baud = 9600;

bool LoRaWAN_connection = false;

float flat = 0;
float flon = 0;
float falt = 0;
uint8_t satellites = 0;
unsigned long age = 0;
bool new_gps_data = false;

TinyGPS gps;
rn2xx3 LoRaWAN(Serial2);

/* Struct for lorawan packet */
struct lora_pkt {
  int32_t lat;
  int32_t lng;
  int16_t altitude;
  uint8_t sat;
};


void initialise_LoRaWAN(){
  if(DEBUG) Serial.print("[LoRaWAN] Initialising UART...");
  Serial2.begin(9600); //serial port to radio
  Serial2.flush();
  LoRaWAN.autobaud();
  if(DEBUG) Serial.print("Done\n");
  /* Join LoRaWAN */
  if(DEBUG) Serial.print("[LoRaWAN] Connecting...");
  LoRaWAN_connection = LoRaWAN.initOTAA(APPEUI, APPKEY);
  if(DEBUG){
    if(LoRaWAN_connection){
      Serial.print("Success!\n");
    }else{
      Serial.print("Failed\n");
    }
  }
}

void initialise_GPS(){
  if(DEBUG) Serial.print("[GPS] Initialising UART...");
  Serial1.begin(gps_baud);
  if(DEBUG) Serial.print("Done\n");
}

void setup(){
  /* give time to attach debug monitor */
  delay(5000);

  /* Initialise uart communication to Debug */
  if(DEBUG) Serial.begin(debug_baud);

  /* Initialise uart communication to GPS module */
  if(ENABLE_GPS) initialise_GPS();

  /* Initialise uart communication to LoRaWAN module */
  if(ENABLE_LORAWAN) initialise_LoRaWAN();
}

void loop(){
  if(ENABLE_GPS){
    if(DEBUG) Serial.print("[GPS] Reading UART...\n");
    if(DEBUG) delay(1000); /* Wait for Serial to clear for debug */
    while (!Serial1.available()){}
    String out = "";
    while (Serial1.available()){
      uint8_t c = Serial1.read();
      out += char(c);
      if (gps.encode(c))
      {
        new_gps_data = true;
      }
    }
    //if(DEBUG) Serial.println(out);
    if(DEBUG) delay(1000); /* Wait for Serial to clear for debug */
    if(new_gps_data){
      if(DEBUG) Serial.print("[GPS] New data\n");
      gps.f_get_position(&flat, &flon, &age);
      satellites = gps.satellites();
      falt = gps.f_altitude();
      if(DEBUG){
        Serial.print("LAT: ");
        Serial.print(flat);
        Serial.print("| LON: ");
        Serial.print(flon);
        Serial.print("| AGE: ");
        Serial.print(age);
        Serial.print("| SAT: ");
        Serial.print(satellites);
        Serial.print("| ALT: ");
        Serial.println(falt);
      }
    }
  }
  if(ENABLE_LORAWAN){
    if(!LoRaWAN_connection){
      initialise_LoRaWAN();
    }else{
      lora_pkt data = {(flat * 10000000), (flon * 10000000), falt, satellites};
      LoRaWAN.txBytes((byte*)&data, (uint8_t)sizeof(data));
    }
    if(DEBUG) Serial.println(LoRaWAN.deveui());
  }
  delay(2000);
}
