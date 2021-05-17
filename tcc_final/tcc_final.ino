/*
 * Nome do arquivo: tcc_final
 * Autor: Arhur Novaes
 * Última atuliazação: 16/05/2021
 * Descrição: Programa final para aplicação completa do filtro de Kalman nos
 * nos sensores de acelerômetro e GPS
 */

#include "conversion.h"


#define CALIB_PENDING 13
#define CALIB_DONE 8

mpu_conv_class mpu_new;

void setup() {
  pinMode(CALIB_PENDING, OUTPUT);
  pinMode(CALIB_DONE, OUTPUT);
  digitalWrite(CALIB_PENDING, HIGH);
  digitalWrite(CALIB_DONE, LOW);

  Serial.begin(115200);

  mpu_new.config_mpu();
  digitalWrite(CALIB_PENDING, LOW);
  digitalWrite(CALIB_DONE, HIGH);
}

void loop() {
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + 25) {
    prev_ms = millis();
    if (mpu_new.update_data()) {
      mpu_new.make_conversion();      
      Serial.print(mpu_new.return_N());
      Serial.print(" ");
      Serial.print(mpu_new.return_E());
      Serial.print(" ");
      Serial.println(mpu_new.return_D());
    }
  }
}
