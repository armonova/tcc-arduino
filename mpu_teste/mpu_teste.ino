/*
 * Nome do arquivo: tcc_final
 * Autor: Arhur Novaes
 * Última atuliazação: 16/05/2021
 * Descrição: Programa final para aplicação completa do filtro de Kalman nos
 * nos sensores de acelerômetro e GPS
 * 
 * Calibração: Quando inicia a placa
 * Estado 1: LED VERMELHO aceso: calibrando Acelerômetro e Giroscópio - Deixar a placa parada
 * Estado 2: LED VERMELHO E VERDE acesos: Calibrando o Magnetómetro - Fazer movimento em forma de um 8
 * Estado 3: LED AMARELO E VERDE acesos: Calculando matrize de covariância
 * Estado 4: LED VERDE aceso: Calibração finalizada - Já pode utilizar o dispositivo
 */

#include "conversion.h"

#define CALIB_PENDING 13  // Led Vermelho
#define CALIB_DONE 8      // Led Verde
#define CALIB_COV 7  // Led Amarelo - Calibração Matrizes de covariância

// Classe para conversão da biblioteca do MPU
mpu_conv_class mpu_new(CALIB_PENDING, CALIB_DONE);

// Velocidade inicial nas coordenadas North e East
float speed_acc_N = 0.0;
float speed_acc_E = 0.0;

// matriz "Q", de covariância
double Q_N[2][2] = {
  {0, 0},
  {0, 0}
};
double Q_E[2][2] = {
  {0, 0},
  {0, 0}
};

void setup() {
  // Seta os pinos do led como output
  pinMode(CALIB_PENDING, OUTPUT);
  pinMode(CALIB_DONE, OUTPUT);
  pinMode(CALIB_COV, OUTPUT);

  // Inicia os leds como desligados  
  digitalWrite(CALIB_PENDING, LOW);
  digitalWrite(CALIB_DONE, LOW);
  digitalWrite(CALIB_COV, LOW);

  Serial.begin(9600); // Inicializa a porta serial


  // Inicializa a calibração da MPU
  if (mpu_new.config_mpu()) {
    // Se a calibração foi bem sucedida
    digitalWrite(CALIB_PENDING, LOW);
    digitalWrite(CALIB_DONE, HIGH); 
  } else {
    // Calibração mal sucedida
    Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
    digitalWrite(CALIB_PENDING, LOW);
    digitalWrite(CALIB_DONE, LOW);
    while(1); // traba o programa caso a calibração não funcione e desliga os LEDs
  }
  

  // Inicializa o cáluclo das matrizes de COVARIÂNCIA
  digitalWrite(CALIB_COV, HIGH);
  delay(3000); // espera 3 segundos para parar a placa
  mpu_new.standard_deviation();
  double DP_posi_N = 0.0003125 * mpu_new.return_DP_NED('N'); // 0.0003125 = (0.025 * 0.025)/2
  double DP_vel_N = 0.025 * mpu_new.return_DP_NED('N');

  double DP_posi_E = 0.0003125 * mpu_new.return_DP_NED('E'); // 0.0003125 = (0.025 * 0.025)/2
  double DP_vel_E = 0.025 * mpu_new.return_DP_NED('E');
  
  // Determinação da matriz Q de covariância
  Q_N[0][0] = DP_posi_N * DP_posi_N;
  Q_N[1][1] = DP_vel_N * DP_vel_N;
  
  Q_E[0][0] = DP_posi_E * DP_posi_E;
  Q_E[1][1] = DP_vel_E * DP_vel_E;

  Serial.println("Covariances matrix calculated");
  
  digitalWrite(CALIB_COV, LOW);
}


// matriz "A", de estado (0.025 = período de amostragem)
double A[2][2] = {
  { 1,  0.025 },
  { 0,  1     }
};

// matriz "B", de entrada
double B[2][1] = {
  { 0.000625 },
  { 0.025    }
};


// Matriz de estados do eixo North
double xk_N[2][1] = {
  { 0 },
  { 0 }
};
// Matriz de estados do eixo East
double xk_E[2][1] = {
  { 0 },
  { 0 }
};

void loop() {
  static uint32_t prev_ms = millis();
  // Faz uma medição a cada 25 ms = 40Hz~ 45Hz
  // @TODO: Conferir esse período
  if (millis() > prev_ms + 25) {
    prev_ms = millis();
    // Faz a aferição dos sensores  
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
