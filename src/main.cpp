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

#define DEBUG
#define ENABLE_GPS 1
#define ENABLE_LORAWAN 1

const int debug_baud = 9600;
const int gps_baud = 9600;
const int rn2483_baud = 9600;

uint8_t tx_result;

bool LoRaWAN_connection = false;

float flat = 0;
float flon = 0;
float falt = 0;
unsigned long chars;
unsigned short sentences, fchecksum;
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

void read_GPS(){
  //data is sent from gps once a second, we'll gather data for 3 seconds to make sure
  unsigned long currentMillis = millis();
  #ifdef DEBUG
    Serial.print("[GPS] Reading UART...");
  #endif
  while(millis() - currentMillis < 3000){
    while (Serial1.available()){
      gps.encode(Serial1.read());
    }
  }
  #ifdef DEBUG
    Serial.print("Done\n");
  #endif
}

void initialise_LoRaWAN(){
  #ifdef DEBUG
    Serial.print("[LoRaWAN] Initialising UART...");
  #endif
  Serial2.begin(rn2483_baud); //serial port to radio
  Serial2.flush();
  LoRaWAN.autobaud();
  #ifdef DEBUG
    Serial.print("Done\n");
    Serial.print("[LoRaWAN] Connecting...");
  #endif
  LoRaWAN_connection = LoRaWAN.initOTAA(APPEUI, APPKEY);
  #ifdef DEBUG
    if(LoRaWAN_connection){
      Serial.print("Success!\n");
    }else{
      Serial.print("Failed\n");
    }
  #endif
}

void initialise_GPS(){
  #ifdef DEBUG
    Serial.print("[GPS] Initialising UART...");
  #endif

  Serial1.begin(gps_baud);

  #ifdef DEBUG
    Serial.print("Done\n");
  #endif
}

void setup(){
  /* give time to attach debug monitor */
  delay(5000);

  /* Initialise uart communication to Debug */
  #ifdef DEBUG
    Serial.begin(debug_baud);
  #endif

  /* Initialise uart communication to GPS module */
  if(ENABLE_GPS) initialise_GPS();

  /* Initialise uart communication to LoRaWAN module */
  if(ENABLE_LORAWAN) initialise_LoRaWAN();
}

void loop(){
  if(ENABLE_GPS){

    read_GPS(); // Reads uart for 3 seconds and feeds to tinygps

    /* grab new gps values */
    gps.f_get_position(&flat, &flon, &age);
    gps.stats(&chars, &sentences, &fchecksum);
    satellites = gps.satellites();
    falt = gps.f_altitude();

    #ifdef DEBUG
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
      Serial.print("CHARS: ");
      Serial.print(chars);
      Serial.print(" SENTENCES: ");
      Serial.print(sentences);
      Serial.print(" FAILED: ");
      Serial.print(fchecksum);
    #endif

  }
  if(ENABLE_LORAWAN){
    if(!LoRaWAN_connection){
      initialise_LoRaWAN();
    }else{
      #ifdef DEBUG
        Serial.print("[LoRaWAN] Constructing packet...");
      #endif

      lora_pkt data = {(flat * 10000000), (flon * 10000000), falt, satellites};

      #ifdef DEBUG
        Serial.print("Done\n");
        Serial.print("[LoRaWAN] Transmitting packet...");
      #endif

      tx_result = LoRaWAN.txBytes((byte*)&data, (uint8_t)sizeof(data));

      #ifdef DEBUG
        Serial.print("Done\n");
        switch(tx_result){
          case 0:
            Serial.print("Failed");
            break;
          case 1:
            Serial.print("Success");
            break;
          case 2:
            Serial.print("Downlink");
            break;
        }
      #endif
    }
  }

  delay(5000); // TODO: Replace this with sleep with MPU6050 wake
}
