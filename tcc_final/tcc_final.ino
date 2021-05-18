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

mpu_conv_class mpu_new(CALIB_PENDING, CALIB_DONE);

float speed_acc_N = 0.0;
float speed_acc_E = 0.0;

void setup() {
  pinMode(CALIB_PENDING, OUTPUT);
  pinMode(CALIB_DONE, OUTPUT);
  digitalWrite(CALIB_PENDING, LOW);
  digitalWrite(CALIB_DONE, LOW);

  Serial.begin(115200);

  digitalWrite(CALIB_PENDING, HIGH);
  digitalWrite(CALIB_DONE, LOW);
  if (mpu_new.config_mpu()) {
    digitalWrite(CALIB_PENDING, LOW);
    digitalWrite(CALIB_DONE, HIGH); 
  } else {
    Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
    digitalWrite(CALIB_PENDING, HIGH);
    digitalWrite(CALIB_DONE, HIGH);
    // while(1);
  }
}


// matriz "A", de estado (0.025 = período de amostragem)
double A[2][2] = {
  { 1,  0.025 },
  { 0,  1     }
};

// matriz "B", de entrada
float B[2][1] = {
  { 0.000625 },
  { 0.025    }
};


double xk_E[2][1] = {
  { 0 },
  { 0 }
};
double xk_N[2][1] = {
  { 0 },
  { 0 }
};

void loop() {
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + 25) {
    prev_ms = millis();
    if (mpu_new.update_data()) {
      mpu_new.make_conversion(); // Faz a leitura dos acelerometro e a conversão das coordenadas 

      // EVOLUÇÃO DOS ESTADOS - ACELERÔMETRO
      // calculo da velocidade por meio da somatória das acelerações
      speed_acc_N += mpu_new.return_acc_NED('N') * 0.025;
      speed_acc_E += mpu_new.return_acc_NED('E') * 0.025;

      // Estados eixo "North"
      float x_N[2][1];
      x_N[0][0] = xk_N[0][0]; // posição
      x_N[1][0] = xk_N[1][0]; // velocidade
      // Estados eixo "East"
      float x_E[2][1];
      x_E[0][0] = xk_E[0][0]; // posição
      x_E[1][0] = xk_E[1][0]; // velocidade


      // Matriz "u"
      float u_N[1] = { mpu_new.return_acc_NED('N')};
      float u_E[1] = { mpu_new.return_acc_NED('E')};

      // Determinação de x_k - evolução dos estados
      xk_N[0][0] = ((A[0][0] * x_N[0][0]) + (A[0][1] * x_N[1][0])) + (B[0][0] * u_N[0]);
      xk_N[1][0] = ((A[1][0] * x_N[0][0]) + (A[1][1] * x_N[1][0])) + (B[1][0] * u_N[0]);
      xk_E[0][0] = ((A[0][0] * x_E[0][0]) + (A[0][1] * x_E[1][0])) + (B[0][0] * u_E[0]);
      xk_E[1][0] = ((A[1][0] * x_E[0][0]) + (A[1][1] * x_E[1][0])) + (B[1][0] * u_E[0]);

      double speedTotal_state = sqrt((xk_E[1][0]*xk_E[1][0]) + (xk_N[1][0]*xk_N[1][0]));
      double posiTotal_state = sqrt((xk_E[0][0]*xk_E[0][0]) + (xk_N[0][0]*xk_N[0][0]));

      Serial.print(0);
      Serial.print(" ");
      Serial.print(mpu_new.return_acc_NED('N'));
      Serial.print(" ");
      Serial.println(mpu_new.return_acc_NED('E'));
    }
  }
}
