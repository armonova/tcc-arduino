/*
   Nome do arquivo: tcc_final
   Autor: Arhur Novaes
   Última atuliazação: 16/05/2021
   Descrição: Programa final para aplicação completa do filtro de Kalman nos
   nos sensores de acelerômetro e GPS

   Calibração: Quando inicia a placa
   Estado 1: LED VERMELHO aceso: calibrando Acelerômetro e Giroscópio - Deixar a placa parada
   Estado 2: LED VERMELHO E VERDE acesos: Calibrando o Magnetómetro - Fazer movimento em forma de um 8
   Estado 3: LED AMARELO E VERDE acesos: Calculando matrize de covariância
   Estado 4: LED VERDE aceso: Calibração finalizada - Já pode utilizar o dispositivo
*/

/*
 * TODO list:
 * Diminuir a frequência de amostragem
 *  - Lembrar de atualizar os parâmetros que estão sendo utilizados para o cálculo da matriz Q - Acelerômetro
 */

#include "conversion.h"
#include <SoftwareSerial.h>
#include "TinyGPS.h"

#define CALIB_PENDING 2  // Led Vermelho
#define CALIB_DONE 8      // Led Verde
#define CALIB_COV 13       // Led Amarelo - Calibração Matrizes de covariância
#define STOPPED 2        // LED vermelho - indica que a pessoa parou
#define MOVING 8          // LED Verde - Pessoa em movimento
#define OFFSET_MOVING 0.5
#define OFFSET_STOPPED 0.1
#define GPS_RX 4
#define GPS_TX 3
#define Serial_Baud 9600

#define GRAPH_VISUALIZATION
//#define REAL_TEST
//#define GRAPH_AXIS_N_VISUALIZATION
//#define GRAPH_AXIS_E_VISUALIZATION

#define CALC_SD

// matriz "B", de entrada
#define B_0 0.005     // (0.1)² / 2 => (delta t)²/2
#define B_1 0.1  // delta t

// Classe para conversão da biblioteca do MPU
mpu_conv_class mpu_new(CALIB_PENDING, CALIB_DONE);

TinyGPS gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// posição inicial do GPS
float original_posi_gps_N;
float original_posi_gps_E;

// matriz "Q", de covariância
float Q_N[2] = {0.1, 0.1};
float R_N[2] = {0.1, 0.1};

// matriz "Q", de covariância
float Q_E[2] = {0.1, 0.1};
float R_E[2] = {0.1, 0.1};

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
     MPU
  */
  // Inicializa a calibração da MPU

  if (mpu_new.config_mpu()) {
    // Se a calibração foi bem sucedida
    digitalWrite(CALIB_PENDING, LOW);
    digitalWrite(CALIB_DONE, HIGH);
  } else {
    // Calibração mal sucedida
    //Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
    digitalWrite(CALIB_PENDING, LOW);
    digitalWrite(CALIB_DONE, LOW);
    while (1); // trava o programa caso a calibração não funcione e desliga os LEDs
  }

  // Inicializa o cáluclo das matrizes de COVARIÂNCIA
  digitalWrite(CALIB_COV, HIGH);
  delay(2000); // espera 3 segundos para parar a placa
  #ifdef CALC_SD
  standard_deviation_acc('N');
  standard_deviation_acc('E');
  standard_deviation_gps();
  #else
  delay(5000);
  #endif
  
  digitalWrite(CALIB_PENDING, HIGH);
  // Armazena a Latitude e Longitude iniciais
  // Fica nesse loop até que seja possível receber algum dado do GPS
  bool newData = false;
  while (!newData) {
    delay(100);
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      if (gps.encode(c)) // Atribui true para newData caso novos dados sejam recebidos
        newData = true;
    }
  }
  gps.get_position(&original_posi_gps_E, &original_posi_gps_N);
  // Armazena as posições no eixo North e East no momento que é ligado o dispositivo
  // Cuidado: removi a verificação de erro

  // Nesse ponto já foram armazenadas as posições inicias do GPS
  digitalWrite(CALIB_PENDING, HIGH);
  digitalWrite(CALIB_DONE, HIGH);
  digitalWrite(CALIB_COV, HIGH);
}

// [linhas][colunas]
float new_P_ant_N[2][2] = {
  {1.0, 0.0},
  {0.0, 1.0}
};
float new_P_ant_E[2][2] = {
  {1.0, 0.0},
  {0.0, 1.0}
};


float xk_kalman_N[2] = { 0.0, 0.0 };
float xk_kalman_E[2] = { 0.0, 0.0 };

float xk_N[2] = { 0.0, 0.0 };
float xk_E[2] = { 0.0, 0.0 };

// Matriz de estados
// gps
float yk_N[2] = { 0.0 , 0.0 };
float yk_E[2] = { 0.0 , 0.0 };

int calibration_measurements = 10;
char loop_count = 0;
float threshold_array[10];
float threshold;
bool calibration_pending = true;

void loop() {
  /*
   * TESTE: MATRIZ Q e R
   * para deixar as matriz com os mesmos pesos
   * Dessa maneira o resuldado do filtro deve ser uma combinação entre as duas medições
   */
  // Variáveis
  bool newGpsData = false;
  float posi_gps_E, posi_gps_N;

  // matriz "A", de estado (0.1 = período de amostragem)
  float A[2][2] = {
    { 1.0,  0.1 },
    { 0.0,  1.0 }
  };
  float A_trasnp[2][2] = {
    { 1.0,  0.0 },
    { 0.1,  1.0 }
  };

  // Faz uma medição a cada 100 ms = 100Hz
  // Precisei fazer essa alteração pois estava havendo delay na leitura do GPS e da MPU
  delay(100); // @TODO Fazer utilizando o millis IMPORTANTE

  // Leitura do GPS
  // TODO: arrumar um jeito de não travar o programa aqui (interrupção ?)
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (gps.encode(c)) // Atribui true para newData caso novos dados sejam recebidos
      newGpsData = true;
  }
  if (newGpsData) {
    // Atualiza os dados do GPS
    gps.get_position(&posi_gps_E, &posi_gps_N);
    if (TinyGPS::GPS_INVALID_F_SPEED && TinyGPS::GPS_INVALID_F_ANGLE) {

      yk_N[0] = posi_gps_N - original_posi_gps_N;
      yk_E[0] = posi_gps_E - original_posi_gps_E;

      // TODO: usar um angulo correto instead of yk[0] - posição - Precisa ser ou o ângulo do mag. ou a velocidade que vem do acelerômetro
      // TODO: Fazer baseado na velocidade do acelerômetro
      mpu_new.returnCordCart(gps.f_speed_mps(), &yk_N[1], &yk_E[1], xk_kalman_N[1], xk_kalman_E[1]);
    } else {
      // Tem algum dado inválido
      // Serial.println("Dado inválido");
    }
  }
  // remoçao do segurador
  /*
  else {
    yk_N[1] = 0;
    yk_E[1] = 0;
  }*/

  // Leitura da MPU
  // Faz a aferição dos sensores
  float acc_N, acc_E;
  if (mpu_new.update_data()) {
    mpu_new.make_conversion(&acc_N, &acc_E); // Faz a leitura dos acelerometro e a conversão das coordenadas
  }

  // EVOLUÇÃO DOS ESTADOS - ACELERÔMETRO
  float xk_ant_N[2];
  float xk_ant_E[2];
  xk_ant_N[0] = xk_N[0]; // posição
  xk_ant_N[1] = xk_N[1]; // velocidade
  xk_ant_E[0] = xk_E[0]; // posição
  xk_ant_E[1] = xk_E[1]; // velocidade

  // Matriz "u"
  /*
    float acc_N = acc_N;
    float u_E = acc_E;
  */

  // Determinação de x_k - evolução dos estados
  xk_N[0] = ((A[0][0] * xk_ant_N[0]) + (A[0][1] * xk_ant_N[1])) + (B_0 * acc_N); // posição
  xk_N[1] = ((A[1][0] * xk_ant_N[0]) + (A[1][1] * xk_ant_N[1])) + (B_1 * acc_N); // velocidade
  xk_E[0] = ((A[0][0] * xk_ant_E[0]) + (A[0][1] * xk_ant_E[1])) + (B_0 * acc_E); // posição
  xk_E[1] = ((A[1][0] * xk_ant_E[0]) + (A[1][1] * xk_ant_E[1])) + (B_1 * acc_E); // velocidade


  /* Nesse ponto do código eu tenho a evolução dos estados do acelerômetro e GPS para
     a determinação da velocidade e da posição.
     O próximo passo é a realização do filtro de Kalmn
  */

  /*****************************************************************
     F I L T R O   D E   K A L M N
   *****************************************************************/

  /************************ P R E D I Ç Ã O ************************/
  // passo 1

  // [MATLAB] xk_aux = A * xk_ant + B * u;
  xk_ant_N[0] = xk_kalman_N[0];
  xk_ant_E[0] = xk_kalman_E[0];
  xk_ant_N[1] = xk_kalman_N[1];
  xk_ant_E[1] = xk_kalman_E[1];
  float xk_aux_N[2];
  float xk_aux_E[2];
  multiplyMatrix_2x2_2x1(A, xk_ant_N, xk_aux_N);
  multiplyMatrix_2x2_2x1(A, xk_ant_E, xk_aux_E);

  xk_aux_N[0] += B_0 * acc_N;
  xk_aux_N[1] += B_1 * acc_N;
  xk_aux_E[0] += B_0 * acc_E;
  xk_aux_E[1] += B_1 * acc_E;

  // passo 2
  // [MATLAB] P = A * new_P_ant * A' + Q;
  float P_N[2][2];
  float P_aux_aux_N[2][2];
  float P_E[2][2];
  float P_aux_aux_E[2][2];

  multiplyMatrix_2x2_2x2(A, new_P_ant_N, P_aux_aux_N); // A * new_P_ant
  multiplyMatrix_2x2_2x2(A, new_P_ant_E, P_aux_aux_E); // A * new_P_ant
  multiplyMatrix_2x2_2x2(P_aux_aux_N, A_trasnp, P_N); // (A * new_P_ant) * A'
  multiplyMatrix_2x2_2x2(P_aux_aux_E, A_trasnp, P_E); // (A * new_P_ant) * A'

  // (A * new_P_ant * A') + Q
  P_N[0][0] += Q_N[0];
  //P_N[0][1] += Q_N[0][1]; = 0
  //P_N[1][0] += Q_N[1][0]; = 0
  P_N[1][1] += Q_N[1];
  //--
  P_E[0][0] += Q_E[0];
  //P_E[0][1] += Q_E[0][1]; = 0
  //P_E[1][0] += Q_E[1][0]; = 0
  P_E[1][1] += Q_E[1];


  /************************ C O R R E Ç Ã O ************************/
  // passo 3
  // K = P * C' * inv(C * P * C' + R);
  float C[2][2] = {
    {1.0, 0.0},
    {0.0, 1.0}
  };

  float K_N[2][2];
  float K_E[2][2];
  float K_aux_N[2][2];
  float K_aux_E[2][2];
  float aux_N[2][2];
  float aux_E[2][2];
  float aux_aux_N[2][2];
  float aux_aux_E[2][2];
  multiplyMatrix_2x2_2x2(C, P_N, aux_aux_N);
  multiplyMatrix_2x2_2x2(C, P_E, aux_aux_E);
  multiplyMatrix_2x2_2x2(aux_aux_N, C, aux_N);
  multiplyMatrix_2x2_2x2(aux_aux_E, C, aux_E);
  aux_N[0][0] += R_N[0];
  //aux_N[0][1] += R_N[0][1];
  //aux_N[1][0] += R_N[1][0];
  aux_N[1][1] += R_N[1];

  aux_E[0][0] += R_E[0];
  //aux_E[0][1] += R_E[0][1]; = 0
  //aux_E[1][0] += R_E[1][0]; = 0
  aux_E[1][1] += R_E[1];

  float aux_inverse_N[2][2];
  float aux_inverse_E[2][2];
  inverseMatrix(aux_N, aux_inverse_N); // = inv(C * P * C' + R)
  inverseMatrix(aux_E, aux_inverse_E); // = inv(C * P * C' + R)

  multiplyMatrix_2x2_2x2(P_N, C, K_aux_N);
  multiplyMatrix_2x2_2x2(K_aux_N, aux_inverse_N, K_N);
  multiplyMatrix_2x2_2x2(P_E, C, K_aux_E);
  multiplyMatrix_2x2_2x2(K_aux_E, aux_inverse_E, K_E);

  // % passo 4
  // [MATLAB]
  // z = C * xk_aux;
  // y = y_k(:,i);
  // new_x_k(:,i) = xk_aux + K * (C * y - z);
  float z_N[2];
  float z_E[2];
  multiplyMatrix_2x2_2x1(C, xk_aux_N, z_N);
  multiplyMatrix_2x2_2x1(C, xk_aux_E, z_E);
  float cXy_N[2];
  float cXy_aux_N[2];
  float cXy_E[2];
  float cXy_aux_E[2];
  multiplyMatrix_2x2_2x1(C, yk_N, cXy_aux_N);
  multiplyMatrix_2x2_2x1(C, yk_E, cXy_aux_E);
  cXy_aux_N[0] -= z_N[0];
  cXy_aux_N[1] -= z_N[1];
  cXy_aux_E[0] -= z_E[0];
  cXy_aux_E[1] -= z_E[1];
  multiplyMatrix_2x2_2x1(K_N, cXy_aux_N, cXy_N);
  multiplyMatrix_2x2_2x1(K_E, cXy_aux_E, cXy_E);

  xk_kalman_N[0] = xk_aux_N[0] + cXy_N[0]; // posição com o filtro de kalman
  xk_kalman_N[1] = xk_aux_N[1] + cXy_N[1]; // velocidade com o filtro de kalman
  xk_kalman_E[0] = xk_aux_E[0] + cXy_E[0]; // posição com o filtro de kalman
  xk_kalman_E[1] = xk_aux_E[1] + cXy_E[1]; // velocidade com o filtro de kal

  // passo 5
  // new_P_ant = (eye(size(Q)) - K * C) * P;
  float P_aux_N[2][2];
  float P_aux_E[2][2];
  multiplyMatrix_2x2_2x2(K_N, C, P_aux_N);
  multiplyMatrix_2x2_2x2(K_E, C, P_aux_E);
  P_aux_N[0][0] = 1 - P_aux_N[0][0];
  P_aux_N[0][1] = 0 - P_aux_N[0][1];
  P_aux_N[1][0] = 0 - P_aux_N[1][0];
  P_aux_N[1][1] = 1 - P_aux_N[1][1];

  P_aux_E[0][0] = 1 - P_aux_E[0][0];
  P_aux_E[0][1] = 0 - P_aux_E[0][1];
  P_aux_E[1][0] = 0 - P_aux_E[1][0];
  P_aux_E[1][1] = 1 - P_aux_E[1][1];
  multiplyMatrix_2x2_2x2(P_aux_N, P_N, new_P_ant_N);
  multiplyMatrix_2x2_2x2(P_aux_E, P_E, new_P_ant_E);

#ifdef GRAPH_VISUALIZATION
  digitalWrite(CALIB_PENDING, LOW);
  digitalWrite(CALIB_DONE, LOW);
  digitalWrite(CALIB_COV, LOW);
  
  float mod_vel_kalman = sqrt((xk_kalman_N[1] * xk_kalman_N[1]) + (xk_kalman_E[1] * xk_kalman_E[1]));
  float mod_acc_vel = sqrt((xk_N[1] * xk_N[1]) + (xk_E[1] * xk_E[1])); // PREDIÇÃO
  float mod_gps_vel = sqrt((yk_N[1] * yk_N[1]) + (yk_E[1] * yk_E[1])); // SENSOR
  
  Serial.print(mod_vel_kalman, 6);     // kalman | azul
  Serial.print(" ");
  Serial.print(mod_acc_vel, 6);        // acelerometro | vermelho
  Serial.print(" ");
  Serial.println(mod_gps_vel, 6);      // gps | verde
#endif

#ifdef GRAPH_AXIS_N_VISUALIZATION
  digitalWrite(CALIB_PENDING, LOW);
  digitalWrite(CALIB_DONE, LOW);
  digitalWrite(CALIB_COV, LOW);
  
  Serial.print(xk_kalman_N[1], 6);     // kalman | azul
  Serial.print(" ");
  Serial.print(xk_N[1], 6);        // acelerometro | vermelho
  Serial.print(" ");
  Serial.println(yk_N[1], 6);      // gps | verde  
#endif

#ifdef GRAPH_AXIS_E_VISUALIZATION
  digitalWrite(CALIB_PENDING, LOW);
  digitalWrite(CALIB_DONE, LOW);
  digitalWrite(CALIB_COV, LOW);
  
  Serial.print(xk_kalman_E[1], 6);     // kalman | azul
  Serial.print(" ");
  Serial.print(xk_E[1], 6);        // acelerometro | vermelho
  Serial.print(" ");
  Serial.println(yk_E[1], 6);      // gps | verde  
#endif

#ifdef REAL_TEST
  if (calibration_pending) {
    if (loop_count >= calibration_measurements) {
      float sum_aux = 0.0;
      for (char a = 0; a < calibration_measurements; a++) {
        sum_aux += threshold_array[a];
      }
      threshold = sum_aux / calibration_measurements;

      calibration_pending = false;
      digitalWrite(CALIB_PENDING, LOW);
      digitalWrite(CALIB_DONE, LOW);
      digitalWrite(CALIB_COV, LOW);

    } else {
      threshold_array[loop_count] = sqrt((xk_kalman_N[1] * xk_kalman_N[1]) + (xk_kalman_E[1] * xk_kalman_E[1])); // TODO: verificar se essa é a melhor maneira de fazer e se tem memória para mais uma variável
      loop_count++;
    }
  } else {
    Serial.println(sqrt((xk_kalman_N[1] * xk_kalman_N[1]) + (xk_kalman_E[1] * xk_kalman_E[1])), 5);
    //Serial.println(threshold);
    if (sqrt((xk_kalman_N[1] * xk_kalman_N[1]) + (xk_kalman_E[1] * xk_kalman_E[1])) > (threshold + OFFSET_MOVING)) {
      digitalWrite(STOPPED, LOW);
      digitalWrite(MOVING, HIGH);
    } else if (sqrt((xk_kalman_N[1] * xk_kalman_N[1]) + (xk_kalman_E[1] * xk_kalman_E[1])) < (threshold + OFFSET_STOPPED)) {
      digitalWrite(STOPPED, HIGH);
      digitalWrite(MOVING, LOW);
    }
  }
#endif
}

#ifdef CALC_SD
void standard_deviation_gps() {
  char _acquires = 15; // número de aquisição para cálculo do desvio padrão

  float _posi_gps_DP_N[_acquires];
  float _posi_gps_DP_E[_acquires];
  float _vel_gps_DP_N[_acquires];
  float _vel_gps_DP_E[_acquires];

  float _sum_posi_gps_N = 0.0;
  float _sum_vel_gps_N = 0.0;
  float _sum_posi_gps_E = 0.0;
  float _sum_vel_gps_E = 0.0;

  for (char i = 0; i < _acquires;) {
    // Armazena a Latitude e Longitude iniciais
    // Fica nesse loop até que seja possível receber algum dado do GPS
    bool newData = false;
    while (!newData) {
      for (unsigned long start = millis(); millis() - start < 100;) {
        while (gpsSerial.available()) {
          char c = gpsSerial.read();
          if (gps.encode(c)) // Atribui true para newData caso novos dados sejam recebidos
            newData = true;
        }
      }
    }
    if (newData) {
      newData = false;
      float posi_gps_E, posi_gps_N;
      gps.get_position(&posi_gps_E, &posi_gps_N);
      // Armazena as posições no eixo North e East no momento que é ligado o dispositivo
      _posi_gps_DP_N[i] = (posi_gps_N == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : posi_gps_N) - original_posi_gps_N;
      _posi_gps_DP_E[i] = (posi_gps_E == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : posi_gps_E) - original_posi_gps_E;
      
      float vel_instant_N, vel_instant_E;
      if (i > 0) {
        vel_instant_N = (_posi_gps_DP_N[i] - _posi_gps_DP_N[i - 1]) * 0.1;
        vel_instant_E = (_posi_gps_DP_E[i] - _posi_gps_DP_E[i - 1]) * 0.1;
      } else {
        vel_instant_N = _posi_gps_DP_N[i] * 0.1;
        vel_instant_E = _posi_gps_DP_E[i] * 0.1;
      }

      // TODO: pegar o ângulo do Magnetômetro
      // Fazer a evolução junto com o acelerometro? - Lembrando que nesse ponto a plaquinha deveria ser considerada como parada
      mpu_new.returnCordCart(gps.f_speed_mps(), &_vel_gps_DP_N[i], &_vel_gps_DP_E[i], vel_instant_N, vel_instant_E);

      _sum_posi_gps_N += _posi_gps_DP_N[i];
      _sum_vel_gps_N += _vel_gps_DP_N[i];
      _sum_posi_gps_E += _posi_gps_DP_E[i];
      _sum_vel_gps_E += _vel_gps_DP_E[i];
      i++;
    }
  }

  float _avg_posi_gps_N = _sum_posi_gps_N / _acquires;
  float _avg_vel_gps_N = _sum_vel_gps_N / _acquires;
  float _avg_posi_gps_E = _sum_posi_gps_E / _acquires;
  float _avg_vel_gps_E = _sum_vel_gps_E / _acquires;


  float _aux_DP_posi_gps_N = 0.0;
  float _aux_DP_vel_gps_N = 0.0;
  float _aux_DP_posi_gps_E = 0.0;
  float _aux_DP_vel_gps_E = 0.0;

  for (char i = 0; i < _acquires; i++) {
    _aux_DP_posi_gps_N += (_posi_gps_DP_N[i] - _avg_posi_gps_N) * (_posi_gps_DP_N[i] - _avg_posi_gps_N);
    _aux_DP_vel_gps_N += (_vel_gps_DP_N[i] - _avg_vel_gps_N) * (_vel_gps_DP_N[i] - _avg_vel_gps_N);
    _aux_DP_posi_gps_E += (_posi_gps_DP_E[i] - _avg_posi_gps_E) * (_posi_gps_DP_E[i] - _avg_posi_gps_E);
    _aux_DP_vel_gps_E += (_vel_gps_DP_E[i] - _avg_vel_gps_E) * (_vel_gps_DP_E[i] - _avg_vel_gps_E);
  }

  // Determina a matriz de covariância R (GPS)
  R_N[0] = _aux_DP_posi_gps_N / _acquires;  // posição
  R_N[1] = _aux_DP_vel_gps_N / _acquires;  // posição
  R_E[0] = _aux_DP_posi_gps_E / _acquires;  // posição
  R_E[1] = _aux_DP_vel_gps_E / _acquires;  // posição
}


void standard_deviation_acc(char axis) {
  char _acquires = 15; // número de aquisição para cálculo do desvio padrão

  float _acc_acc_DP[_acquires];

  float _sum_acc_acc = 0.0;
  float acc_N, acc_E;

  for (char i = 0; i < _acquires;) {
    if (mpu_new.update_data()) {
      mpu_new.make_conversion(&acc_N, &acc_E);
      if (axis == 'N') _acc_acc_DP[i] = acc_N;
      else if (axis == 'E') _acc_acc_DP[i] = acc_E;
      

      _sum_acc_acc += _acc_acc_DP[i];
      i++;
    }
    delay(100); // frequencia de 10Hz
  }

  delay(2000);

  float _aux_DP_acc_acc = 0.0;

  for (char i = 0; i < _acquires; i++) {
    _aux_DP_acc_acc += (_acc_acc_DP[i] - (_sum_acc_acc / _acquires)) * (_acc_acc_DP[i] - (_sum_acc_acc / _acquires));
  }

  float _DP_acc_acc = sqrt(_aux_DP_acc_acc / _acquires);

  float DP_posi = 0.005 * _DP_acc_acc;    // 0.005 = (0.1 * 0.1)/2 => (delta t)²/2
  float DP_vel = 0.1 * _DP_acc_acc;       // 0.1 => delta t

  // Determinação da matriz Q de covariância
  if (axis == 'N') {
    Q_N[0] = DP_posi * DP_posi;  // variancia posição
    Q_N[1] = DP_vel * DP_vel;    // variancia velocidade
  } else {
    Q_E[0] = DP_posi * DP_posi;  // variancia posição
    Q_E[1] = DP_vel * DP_vel;    // variancia velocidade 
  }
}
#endif

void multiplyMatrix_2x2_2x2(float M1[2][2], float M2[2][2], float returnMatriz[2][2]) {
  returnMatriz[0][0] = (M1[0][0] * M2[0][0]) + (M1[0][1] * M2[1][0]);
  returnMatriz[0][1] = (M1[0][0] * M2[0][1]) + (M1[0][1] * M2[1][1]);

  returnMatriz[1][0] = (M1[1][0] * M2[0][0]) + (M1[1][1] * M2[1][0]);
  returnMatriz[1][1] = (M1[1][0] * M2[0][1]) + (M1[1][1] * M2[1][1]);
  return;
}

void multiplyMatrix_2x2_2x1(float M1[2][2], float M2[2], float returnMatriz[2]) {
  returnMatriz[0] = ((M1[0][0] * M2[0])) + ((M1[0][1] * M2[1]));
  returnMatriz[1] = ((M1[1][0] * M2[0])) + ((M1[1][1] * M2[1]));
  return;
}

void inverseMatrix(float M[2][2], float retunInv[2][2]) {
  retunInv[0][0] = (-M[1][1]) / ((M[1][0] * M[0][1]) - (M[0][0] * M[1][1]));  // a
  retunInv[0][1] = (M[0][1]) / ((M[1][0] * M[0][1]) - (M[0][0] * M[1][1]));   // b

  retunInv[1][0] = (-M[1][0]) / ((M[1][1] * M[0][0]) - (M[0][1] * M[1][0]));  // c
  retunInv[1][1] = (M[0][0]) / ((M[1][1] * M[0][0]) - (M[0][1] * M[1][0]));   // d
  return;
}
