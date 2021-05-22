#define GPS_RX 4
#define GPS_TX 3
#define GPS_Serial_Baud 9600
#include <SoftwareSerial.h>
#include "TinyGPS.h"
 
TinyGPS gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

float original_posi_gps_N;
float original_posi_gps_E;

void setup()
{
  Serial.begin(GPS_Serial_Baud);
  gpsSerial.begin(GPS_Serial_Baud);

  // Armazena a Latitude e Longitude iniciais
  // Fina nesse loop até que seja possível receber algum dado do GPS  
  bool newData = false;
  while(!newData) {
    for (unsigned long start = millis(); millis() - start < 0.025;) {
      while (gpsSerial.available()) {
        char c = gpsSerial.read();
        if (gps.encode(c)) // Atribui true para newData caso novos dados sejam recebidos
          newData = true;
      }
    }
  }
  float posi_gps_E, posi_gps_N;
  unsigned long age;
  gps.f_get_position(&posi_gps_E, &posi_gps_N, &age);
  // Armazena as posições no eixo North e East no momento que é ligado o dispositivo 
  original_posi_gps_N = (posi_gps_N == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : posi_gps_N);
  original_posi_gps_E = (posi_gps_E == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : posi_gps_E);
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 0.025;) {
    Serial.println("Teste");
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      if (gps.encode(c)) // Atribui true para newData caso novos dados sejam recebidos
        newData = true;
    }
  }
  if (newData) {
    float posi_gps_E, posi_gps_N, speed_gps;
    unsigned long age;
    gps.f_get_position(&posi_gps_E, &posi_gps_N, &age);
    posi_gps_E = posi_gps_E - original_posi_gps_E;
    posi_gps_N = posi_gps_N - original_posi_gps_N;
    speed_gps = gps.f_speed_mps();
    if (TinyGPS::GPS_INVALID_F_SPEED && TinyGPS::GPS_INVALID_F_ANGLE) {
      Serial.print("POSI GPS EAST =");
      Serial.print(posi_gps_E, 6);
      Serial.print(" POSI GPS NORTH=");
      Serial.print(posi_gps_N, 6);
      Serial.print(" SPEED GPS=");
      Serial.println(speed_gps, 6);
      Serial.println("");
    } else {
      // Tem algum dado inválido
      Serial.println("Dado inválido");
    }
  }
}
