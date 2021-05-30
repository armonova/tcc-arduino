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
#include <SoftwareSerial.h>
#include "TinyGPS.h"

#define CALIB_PENDING 13  // Led Vermelho
#define CALIB_DONE 8      // Led Verde
#define CALIB_COV 7  // Led Amarelo - Calibração Matrizes de covariância
#define GPS_RX 4
#define GPS_TX 3
#define Serial_Baud 9600

// Classe para conversão da biblioteca do MPU
mpu_conv_class mpu_new(CALIB_PENDING, CALIB_DONE);

TinyGPS gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// Velocidade inicial nas coordenadas North e East
float speed_acc_N = 0.0;
float speed_acc_E = 0.0;

// posição inicial do GPS
float original_posi_gps_N;
float original_posi_gps_E;

// matriz "Q", de covariância
float Q_MOD[2][2] = {
  {0.0, 0.0},
  {0.0, 0.0}
};

float R_MOD[2][2] = {
  {0.0, 0.0},
  {0.0, 0.0}
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

  gpsSerial.begin(Serial_Baud); // inicializa o GPS

  /*
   * MPU
   */
  // Inicializa a calibração da MPU
  if (mpu_new.config_mpu() || true) {
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
  delay(5000); // espera 3 segundos para parar a placa
  standard_deviation_acc();
  standard_deviation_gps(); 

  Serial.println("Matriz Q");
  Serial.print("Q[0][0]: ");
  Serial.println(Q_MOD[0][0], 10);
  Serial.print("Q[1][1]: ");
  Serial.println(Q_MOD[1][1], 10);
  Serial.println("");
  Serial.print("R[0][0]: ");
  Serial.println(R_MOD[0][0], 10);
  Serial.print("R[1][1]: ");
  Serial.println(R_MOD[1][1], 10);
  
  // Matriz de covariância calculada

  /*
   * GPS
   */
  digitalWrite(CALIB_PENDING, HIGH);
  // Armazena a Latitude e Longitude iniciais
  // Fica nesse loop até que seja possível receber algum dado do GPS  
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

  // Nesse ponto já foram armazenadas as posições inicias do GPS
  
  digitalWrite(CALIB_PENDING, LOW);
  digitalWrite(CALIB_COV, LOW);
}

// [linhas][colunas]
float x_k_ant[2][1] = {
  { 0.0 },
  { 0.0 }
};

float new_P_ant[2][2] ={
  {1, 0},
  {0, 1}
};

void loop() {
  // Variáveis
  bool newGpsData = false;
  float posi_gps_E, posi_gps_N, speed_gps;
  
  // matriz "A", de estado (0.025 = período de amostragem)
  float A[2][2] = {
    { 1,  0.025 },
    { 0,  1     }
  };
  
  // matriz "B", de entrada
  float B[2][1] = {
    { 0.000625 },
    { 0.025    }
  };
  
  // Matriz de estados do eixo North
  float xk_N[2][1] = {
    { 0 },
    { 0 }
  };
  // Matriz de estados do eixo East
  float xk_E[2][1] = {
    { 0 },
    { 0 }
  };
  
  static uint32_t prev_ms = millis();
  // Faz uma medição a cada 25 ms = 40Hz ~ 45Hz
  // @TODO: Conferir esse período
  while(millis() < prev_ms + 25);
  prev_ms = millis();
  
  // Leitura do GPS
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (gps.encode(c)) // Atribui true para newData caso novos dados sejam recebidos
      newGpsData = true;
  }
  if (newGpsData) {
    // Atualiza os dados do GPS
    unsigned long age;
    gps.f_get_position(&posi_gps_E, &posi_gps_N, &age);
    if (TinyGPS::GPS_INVALID_F_SPEED && TinyGPS::GPS_INVALID_F_ANGLE) {
      posi_gps_E = posi_gps_E - original_posi_gps_E;
      posi_gps_N = posi_gps_N - original_posi_gps_N;
      speed_gps = gps.f_speed_mps();
    } else {
      // Tem algum dado inválido
      // Serial.println("Dado inválido");
    }
  }

  // Leitura da MPU
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
    // @TODO: parar de usar matriz aqui
    float u_N[1] = { (mpu_new.return_acc_NED('N')) };
    float u_E[1] = { (mpu_new.return_acc_NED('E')) };
    float u = sqrt( (mpu_new.return_acc_NED('N') * mpu_new.return_acc_NED('N')) + (mpu_new.return_acc_NED('E') * mpu_new.return_acc_NED('E')) );

    // Determinação de x_k - evolução dos estados
    xk_N[0][0] = ((A[0][0] * x_N[0][0]) + (A[0][1] * x_N[1][0])) + (B[0][0] * u_N[0]);
    xk_N[1][0] = ((A[1][0] * x_N[0][0]) + (A[1][1] * x_N[1][0])) + (B[1][0] * u_N[0]);
    xk_E[0][0] = ((A[0][0] * x_E[0][0]) + (A[0][1] * x_E[1][0])) + (B[0][0] * u_E[0]);
    xk_E[1][0] = ((A[1][0] * x_E[0][0]) + (A[1][1] * x_E[1][0])) + (B[1][0] * u_E[0]);

    float speedTotal_state = sqrt((xk_E[1][0]*xk_E[1][0]) + (xk_N[1][0]*xk_N[1][0]));
    float posiTotal_state = sqrt((xk_E[0][0]*xk_E[0][0]) + (xk_N[0][0]*xk_N[0][0]));

    Serial.print(0);
    Serial.print(" ");
    Serial.print(speedTotal_state);
    Serial.print(" ");
    Serial.println(speed_gps);

    /* Nesse ponto do código eu tenho a evolução dos estados do acelerômetro e GPS para 
     * a determinação da velocidade e da posição.
     * O próximo passo é a realização do filtro de Kalmn
     */

    /***************** F I L T R O   D E   K A L M N *****************/
    // P R E D I Ç  Ã O
    // passo 1 - @TODO: Refazer utilizando as funções de multiplicação de matrizes

    // [MATLAB] x_aux = A * x_k_ant + B * u;  
    float x_aux[2][1] = {
      { ((A[0][0] * x_k_ant[0][0]) + (A[0][1] * x_k_ant[1][0])) },
      { ((A[1][0] * x_k_ant[0][0]) + (A[1][1] * x_k_ant[1][0])) }
    };

    x_aux[0][0] += B[0][0] * u;
    x_aux[1][0] += B[1][0] * u;
    
    // passo 2 - @TODO: Refazer utilizando as funções de multiplicação de matrizes
    // [MATLAB] P = A * new_P_ant * A' + Q;
    float P[2][2] = {
      { (A[0][0] * new_P_ant[0][0] + A[0][1] * new_P_ant[1][0]), (A[0][0] * new_P_ant[0][1] + A[0][1] * new_P_ant[1][1]) },
      { (A[1][0] * new_P_ant[0][0] + A[1][1] * new_P_ant[1][0]), (A[1][0] * new_P_ant[0][1] + A[1][1] * new_P_ant[1][1]) }
    }; // A * new_P_ant

    P[0][0] = (P[0][0] * A[0][0] + P[0][1] * A[0][1]) + Q_MOD[0][0];
    P[0][1] = (P[0][0] * A[1][0] + P[0][1] * A[1][1]) + Q_MOD[0][1];

    P[1][0] = (P[1][0] * A[0][0] + P[1][1] * A[0][1]) + Q_MOD[1][0];
    P[1][1] = (P[1][0] * A[1][0] + P[1][1] * A[1][1]) + Q_MOD[1][1];
    // (P * A') + Q - Obs atenção para a multiplicação da matriz transposta

    // C O R R E Ç Ã O
    // passo 3
    // K = P * C' * inv(C * P * C' + R);
    float C[2][2] = {
      {1, 0},
      {0, 1}
    };
    float K[2][2];
    float aux[2][2];
    multiplyMatrix_2x2_2x2(C, P, aux);
    multiplyMatrix_2x2_2x2(aux, C, aux);
    aux[0][0] += R_MOD[0][0];
    aux[0][1] += R_MOD[0][1]; 

    aux[1][0] += R_MOD[1][0];
    aux[1][1] += R_MOD[1][1];

    // @TODO falta calulcar a inversa 
    // inverseMatrix()
//    
//    % passo 4
//    z = C * x_aux;
//    y = y_k(:,i);
//    new_x_k(:,i) = x_aux + K * (C * y - z);
//    
//    % passo 5
//    new_P_ant = (eye(size(Q)) - K * C) * P;
  }
}


void standard_deviation_gps() {
  char _acquires = 50; // número de aquisição para cálculo do desvio padrão
  
  float _posi_gps_DP_MOD[_acquires];
  float _vel_gps_DP_MOD[_acquires];

  float _sum_posi_gps_MOD = 0.0;
  float _sum_vel_gps_MOD = 0.0;

  for (char i = 0; i < _acquires; i++) {
    // Armazena a Latitude e Longitude iniciais
    // Fica nesse loop até que seja possível receber algum dado do GPS  
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
    float _posi_gps_DP_N = (posi_gps_N == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : posi_gps_N);
    float _posi_gps_DP_E = (posi_gps_E == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : posi_gps_E);
    _posi_gps_DP_MOD[i] = sqrt((_posi_gps_DP_N * _posi_gps_DP_N) + (_posi_gps_DP_E * _posi_gps_DP_E));
    _vel_gps_DP_MOD[i] = gps.f_speed_mps();
  
    _sum_posi_gps_MOD += _posi_gps_DP_MOD[i];
    _sum_vel_gps_MOD += _vel_gps_DP_MOD[i];
    
    delay(25); // frequencia de 40Hz
  }

  float _avg_posi_gps_MOD = _sum_posi_gps_MOD / (float)_acquires;
  float _avg_vel_gps_MOD = _sum_vel_gps_MOD / (float)_acquires;

  
  float _aux_DP_posi_gps_MOD = 0.0;
  float _aux_DP_vel_gps_MOD = 0.0;

  for (int i = 0; i < _acquires; i++) {
    _aux_DP_posi_gps_MOD += (_posi_gps_DP_MOD[i] - _avg_posi_gps_MOD) * (_posi_gps_DP_MOD[i] - _avg_posi_gps_MOD);
    _aux_DP_vel_gps_MOD += (_vel_gps_DP_MOD[i] - _avg_vel_gps_MOD) * (_vel_gps_DP_MOD[i] - _avg_vel_gps_MOD);
  }

  Serial.print("_aux_DP_posi_gps_MOD: ");
  Serial.println(_aux_DP_posi_gps_MOD, 8);

  // Determina a matriz de covariância R (GPS)
  R_MOD[0][0] = _aux_DP_posi_gps_MOD * _aux_DP_posi_gps_MOD;
  R_MOD[1][1] = _aux_DP_vel_gps_MOD * _aux_DP_vel_gps_MOD;
}


void standard_deviation_acc() {
  char _acquires = 50; // número de aquisição para cálculo do desvio padrão
  
  float _acc_acc_DP_MOD[_acquires];

  float _sum_acc_acc_MOD = 0.0;
  
  for (char i = 0; i < _acquires;) {
    if (mpu_new.update_data()) {
      mpu_new.make_conversion();
      float _acc_acc_DP_N = mpu_new.return_acc_NED('N');
      float _acc_acc_DP_E = mpu_new.return_acc_NED('E');
      _acc_acc_DP_MOD[i] = sqrt((_acc_acc_DP_N * _acc_acc_DP_N) + (_acc_acc_DP_E * _acc_acc_DP_E));
    
      _sum_acc_acc_MOD += _acc_acc_DP_MOD[i];
      i++;
    }
    delay(25); // frequencia de 40Hz
  }

  delay(2000);

  float _avg_acc_acc_MOD = _sum_acc_acc_MOD / (float)_acquires;
  
  float _auxE_DP_acc_acc_MOD = 0.0;

  for (int i = 0; i < _acquires; i++) {
    _auxE_DP_acc_acc_MOD += (_acc_acc_DP_MOD[i] - _avg_acc_acc_MOD) * (_acc_acc_DP_MOD[i] - _avg_acc_acc_MOD);
  }
  
  float _DP_acc_acc_MOD = sqrt(_auxE_DP_acc_acc_MOD / (float)_acquires); // @TODO: verificar remoção da raiz 

  float DP_posi_MOD = 0.0003125 * _DP_acc_acc_MOD; // 0.0003125 = (0.025 * 0.025)/2
  float DP_vel_MOD = 0.025 * _DP_acc_acc_MOD;
    
  // Determinação da matriz Q de covariância
  Q_MOD[0][0] = DP_posi_MOD * DP_posi_MOD;
  Q_MOD[1][1] = DP_vel_MOD * DP_vel_MOD;
}

void multiplyMatrix_2x2_2x2(float M1[2][2], float M2[2][2], float returnMatriz[2][2]) {
    returnMatriz[0][0] = (M1[0][0] * M2[0][0]) + (M1[0][1] * M2[1][0]);
    returnMatriz[0][1] = (M1[0][0] * M2[0][1]) + (M1[0][1] * M2[1][1]);
    
    returnMatriz[1][0] = (M1[1][0] * M2[0][0]) + (M1[1][1] * M2[1][0]);
    returnMatriz[1][1] = (M1[1][0] * M2[0][1]) + (M1[1][1] * M2[1][1]);
    return;
}

void multiplyMatrix_2x2_2x1(float M1[2][2], float M2[2][1], float returnMatriz[2][1]) {
    returnMatriz[0][0] = (M1[0][0] * M2[0][0]) + (M1[0][1] * M2[1][0]);
    returnMatriz[1][0] = (M1[1][0] * M2[0][0]) + (M1[1][1] * M2[1][0]);
    return;
}

void inverseMatrix(float M[2][2], float retunInv[2][2]) {
    retunInv[0][0] = (-M[1][1]) / ((M[1][0] * M[0][1]) - (M[0][0] * M[1][1]));  // a
    retunInv[0][1] = (M[0][1]) / ((M[1][0] * M[0][1]) - (M[0][0] * M[1][1]));   // b
    
    retunInv[1][0] = (-M[1][0]) / ((M[1][1] * M[0][0]) - (M[0][1] * M[1][0]));  // c
    retunInv[1][1] = (M[0][0]) / ((M[1][1] * M[0][0]) - (M[0][1] * M[1][0]));   // d
    return;
}

void transposed(float M[2][2], float retunTransp[2][2]) {
    retunTransp[0][0] = M[0][0];
    retunTransp[0][1] = M[1][0];
    
    retunTransp[1][0] = M[0][1];
    retunTransp[1][1] = M[1][1];
}
