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

#define AXIS_N

// matriz "B", de entrada
// agora é definida em tempo de execução

// Classe para conversão da biblioteca do MPU
mpu_conv_class mpu_new(CALIB_PENDING, CALIB_DONE);

TinyGPS gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// posição inicial do GPS
float original_posi_gps_N, original_posi_gps_E, original_posi_gps;

// matriz "Q", de covariância
float Q[2] = {0.1, 0.1};
float R[2] = {0.1, 0.1};

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
  #ifdef AXIS_N
  standard_deviation_acc('N');
  #else
  standard_deviation_acc('E');
  #endif
  
  
  standard_deviation_gps();
  
  digitalWrite(CALIB_PENDING, HIGH);
  // Armazena a Latitude e Longitude iniciais
  // Fica nesse loop até que seja possível receber algum dado do GPS
  bool newData = false;
  while (!newData) {
    delay(100);
    while (gpsSerial.available()) {
      if (gps.encode(gpsSerial.read())) // Atribui true para newData caso novos dados sejam recebidos
        newData = true;
    }
  }
  gps.get_position(&original_posi_gps_N, &original_posi_gps_N);
  #ifdef AXIS_N
  original_posi_gps = original_posi_gps_N;
  #else
  original_posi_gps = original_posi_gps_E;
  #endif
  // Armazena as posições no eixo North e East no momento que é ligado o dispositivo
  // Cuidado: removi a verificação de erro

  // Nesse ponto já foram armazenadas as posições inicias do GPS
  digitalWrite(CALIB_PENDING, HIGH);
  digitalWrite(CALIB_DONE, HIGH);
  digitalWrite(CALIB_COV, HIGH);
}

#ifdef REAL_TEST
int calibration_measurements = 10;
char loop_count = 0;
float threshold_array[10];
float threshold;
bool calibration_pending = true;a
#endif

void loop() {
  // [linhas][colunas]
  // Variáveis estáticas TODO testar para ver se usar static variables atrapalha em algo
  static float new_P_ant[2][2] = {
    {1.0, 0.0},
    {0.0, 1.0}
  };
  
  static long millisAux = 0.0;
  static float xk_kalman[2] = { 0.0, 0.0 };
  
  static float xk[2] = { 0.0, 0.0 };
  
  // Matriz de estados
  // gps
  static float yk[2] = { 0.0 , 0.0 };
  
  /*
   * TESTE: MATRIZ Q e R
   * para deixar as matriz com os mesmos pesos
   * Dessa maneira o resuldado do filtro deve ser uma combinação entre as duas medições
   */
  // Variáveis
  bool newGpsData = false;
  float posi_gps_N, posi_gps_E, posi_gps;

  // Faz uma medição a cada 100 ms = 100Hz
  // Precisei fazer essa alteração pois estava havendo delay na leitura do GPS e da MPU
  delay(100); // @TODO Fazer utilizando o millis IMPORTANTE

  // Leitura do GPS
  // TODO: arrumar um jeito de não travar o programa aqui (interrupção ?)
  while (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) // Atribui true para newData caso novos dados sejam recebidos
      newGpsData = true;
  }
  if (newGpsData) {
    // Atualiza os dados do GPS
    gps.get_position(&posi_gps_N, &posi_gps_E);
    if (TinyGPS::GPS_INVALID_F_SPEED && TinyGPS::GPS_INVALID_F_ANGLE) {
      #ifdef AXIS_N
      posi_gps = posi_gps_N;
      #else
      posi_gps = posi_gps_E;
      #endif

      yk[0] = posi_gps - original_posi_gps;
      yk[1] = gps.f_speed_mps();
      // mpu_new.returnCordCart(gps.f_speed_mps(), &yk_N[1], &yk_E[1], xk_kalman_N[1], xk_kalman_E[1]);
    } else {
      // Tem algum dado inválido
      // Serial.println("Dado inválido");
    }
  }

  // Leitura da MPU
  // Faz a aferição dos sensores
  float acc_N = 0.0, acc_E = 0.0, acc = 0.0;
  if (mpu_new.update_data()) {
    mpu_new.make_conversion(&acc_N, &acc_E); // Faz a leitura dos acelerometro e a conversão das coordenadas
    #ifdef AXIS_N
    acc = acc_N; //TODO escolher o eixo
    #else
    acc = acc_E; //TODO escolher o eixo
    #endif
  }

  // EVOLUÇÃO DOS ESTADOS - ACELERÔMETRO
  float xk_ant[2];
  xk_ant[0] = xk[0]; // posição
  xk_ant[1] = xk[1]; // velocidade

  // Matriz "u"
  float B_1 = (millisAux == 0.0 ? 0.0 : (millis() - millisAux)) / 1000.0;
  float B_0 = pow(B_1, 2)/ 2; // => (delta t)²/2
  millisAux = millis();

    // matriz "A", de estado (0.1 = período de amostragem)
  float A[2][2] = {
    { 1.0,  B_1 },
    { 0.0,  1.0 }
  };
  float A_trasnp[2][2] = {
    { 1.0,  0.0 },
    { B_1,  1.0 }
  };

  // Determinação de x_k - evolução dos estados
  xk[0] = (xk_ant[0]) + (B_1 * xk_ant[1]) + (B_0 * acc); // posição
  xk[1] = (xk_ant[1]) + (B_1 * acc); // velocidade


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
  xk_ant[0] = xk_kalman[0];
  xk_ant[1] = xk_kalman[1];
  float xk_aux[2];
  multiplyMatrix_2x2_2x1(A, xk_ant, xk_aux);

  xk_aux[0] += B_0 * acc;
  xk_aux[1] += B_1 * acc;

  // passo 2
  // [MATLAB] P = A * new_P_ant * A' + Q;
  float P[2][2];
  float P_aux_aux[2][2];

  multiplyMatrix_2x2_2x2(A, new_P_ant, P_aux_aux); // A * new_P_ant
  multiplyMatrix_2x2_2x2(P_aux_aux, A_trasnp, P); // (A * new_P_ant) * A'

  // (A * new_P_ant * A') + Q
  P[0][0] += Q[0];
  P[1][1] += Q[1];


  /************************ C O R R E Ç Ã O ************************/
  // passo 3
  // K = P * C' * inv(C * P * C' + R); 
  // inv(C * P * C' + R) => inv(P + R)

  float K[2][2];
  float aux[2][2];
  aux[0][0] = P[0][0] + R[0];
  aux[1][0] = P[1][0];
  aux[1][1] = P[1][1] + R[1];
  aux[0][1] = P[0][1];

  float aux_inverse[2][2];
  inverseMatrix(aux, aux_inverse); // = inv(C * P * C' + R)

  multiplyMatrix_2x2_2x2(P, aux_inverse, K);

  // % passo 4
  // [MATLAB]
  // z = C * xk_aux;
  // y = y_k(:,i);
  // new_x_k(:,i) = xk_aux + K * (C * y - z);
  float z[2];
  z[0] = xk_aux[0];
  z[1] = xk_aux[1];

  float cXy[2];
  float cXy_aux[2];
  
  
  cXy_aux[0] = yk[0] - z[0];
  cXy_aux[1] = yk[1] - z[1];
  
  multiplyMatrix_2x2_2x1(K, cXy_aux, cXy);

  xk_kalman[0] = xk_aux[0] + cXy[0]; // posição com o filtro de kalman
  xk_kalman[1] = xk_aux[1] + cXy[1]; // velocidade com o filtro de kalman

  // passo 5
  // new_P_ant = (eye(size(Q)) - K * C) * P;
  float P_aux[2][2];
  
  P_aux[0][0] = 1 - K[0][0];
  P_aux[0][1] = 0 - K[0][1];
  P_aux[1][0] = 0 - K[1][0];
  P_aux[1][1] = 1 - K[1][1];

  multiplyMatrix_2x2_2x2(P_aux, P, new_P_ant);

#ifdef GRAPH_VISUALIZATION
  digitalWrite(CALIB_PENDING, LOW);
  digitalWrite(CALIB_DONE, LOW);
  digitalWrite(CALIB_COV, LOW);
  
  Serial.print(xk_kalman[1], 6);     // kalman | azul
  Serial.print(" ");
  Serial.print(xk[1], 6);        // acelerometro | vermelho
  Serial.print(" ");
  Serial.print(yk[1], 6);      // gps | verde
  Serial.print(" ");
  Serial.println(acc, 6);
#endif

#ifdef GRAPH_AXIS_N_VISUALIZATION
  digitalWrite(CALIB_PENDING, LOW);
  digitalWrite(CALIB_DONE, LOW);
  digitalWrite(CALIB_COV, LOW);
  
  Serial.print(xk_kalman_N[1], 6);     // kalman | azul
  Serial.print(" ");
  Serial.print(xk_N[1], 6);        // acelerometro | vermelho
  Serial.print(" ");
  Serial.println(yk_N[1], 6); // gps | verde  
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


void standard_deviation_gps() {
  char _acquires = 15; // número de aquisição para cálculo do desvio padrão

  float _posi_gps_DP[_acquires];
  float _vel_gps_DP[_acquires];

  float _sum_posi_gps = 0.0;
  float _sum_vel_gps = 0.0;

  for (char i = 0; i < _acquires;) {
    // Armazena a Latitude e Longitude iniciais
    // Fica nesse loop até que seja possível receber algum dado do GPS
    bool newData = false;
    while (!newData) {
      for (unsigned long start = millis(); millis() - start < 100;) {
        while (gpsSerial.available()) {
          if (gps.encode(gpsSerial.read())) // Atribui true para newData caso novos dados sejam recebidos
            newData = true;
        }
      }
    }
    if (newData) {
      newData = false;
      float posi_gps_E, posi_gps_N, posi_gps;
      gps.get_position(&posi_gps_E, &posi_gps_N);
      // Armazena as posições no eixo North e East no momento que é ligado o dispositivo
      #ifdef AXIS_N
      _posi_gps_DP[i] = (posi_gps_N == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : posi_gps_N) - original_posi_gps;
      #else
      _posi_gps_DP[i] = (posi_gps_E == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : posi_gps_E) - original_posi_gps;
      #endif
      
      float vel_instant;
      if (i > 0) {
        vel_instant = (_posi_gps_DP[i] - _posi_gps_DP[i - 1]) * 0.1;
      } else {
        vel_instant = _posi_gps_DP[i] * 0.1;
      }

      _vel_gps_DP[i] = gps.f_speed_mps();

      _sum_posi_gps += _posi_gps_DP[i];
      _sum_vel_gps += _vel_gps_DP[i];
      i++;
    }
  }

  float _avg_posi_gps = _sum_posi_gps / _acquires;
  float _avg_vel_gps = _sum_vel_gps / _acquires;


  float _aux_DP_posi_gps = 0.0;
  float _aux_DP_vel_gps = 0.0;

  for (char i = 0; i < _acquires; i++) {
    _aux_DP_posi_gps += (_posi_gps_DP[i] - _avg_posi_gps) * (_posi_gps_DP[i] - _avg_posi_gps);
    _aux_DP_vel_gps += (_vel_gps_DP[i] - _avg_vel_gps) * (_vel_gps_DP[i] - _avg_vel_gps);
  }

  // Determina a matriz de covariância R (GPS)
  R[0] = _aux_DP_posi_gps / _acquires;  // posição
  R[1] = _aux_DP_vel_gps / _acquires;  // posição
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
    Q[0] = DP_posi * DP_posi;  // variancia posição
  } else {
    Q[0] = DP_posi * DP_posi;  // variancia posição
  }
}


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
