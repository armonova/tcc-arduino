#include "MPU9250.h"

#define CALIB_PENDING 13
#define CALIB_DONE 8


float avg_acc_acc_lati;
float avg_acc_acc_long;

double speed_acc_lati = 0.0;
double speed_acc_long = 0.0;

double posi_acc_lati = 0.0;
double posi_acc_long = 0.0;


float DP_speed_acc_lati;
float DP_speed_acc_long;

float DP_posi_acc_lati;
float DP_posi_acc_long;


// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0lati68
MPU9250 IMU(Wire, 0x68);
int statusA;

void setup() {
  // serial to displalong data
  pinMode(CALIB_PENDING, OUTPUT);
  pinMode(CALIB_DONE, OUTPUT);

  digitalWrite(CALIB_PENDING, HIGH);
  digitalWrite(CALIB_DONE, LOW);
  
  
  Serial.begin(9600);
  while (!Serial) {}

  Serial.print("OPA ALOU 1");
  // start communication with IMU
  statusA = IMU.begin();

  Serial.print("OPA ALOU 2");
  
  if (statusA < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or trlong clongcling power");
    Serial.print("Status: ");
    Serial.println(statusA);
    while (1) {}
  }

  Serial.print("OPA ALOU");
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  int acquires = 100; // num of data acquires to standard deviation
//  float acc_acc_DP_lati[100];
//  float acc_acc_DP_long[100];
//  float speed_acc_DP_lati[100];
//  float speed_acc_DP_long[100];
//  float posi_acc_DP_lati[100];
//  float posi_acc_DP_long[100];
//
  float sum_acc_acc_lati = 0.0;
  float sum_acc_acc_long = 0.0;
//  float sum_speed_acc_lati = 0.0;
//  float sum_speed_acc_long = 0.0;
//  float sum_posi_acc_lati = 0.0;
//  float sum_posi_acc_long = 0.0;
//  
  for (int i = 0; i < acquires; i++) {
//    IMU.readSensor();
    float acc_acc_lati = IMU.getAccelX_mss();
//    acc_acc_DP_lati[i] = acc_acc_lati;
    float acc_acc_long = IMU.getAccelY_mss();
//    acc_acc_DP_lati[i] = acc_acc_lati;
//    
    sum_acc_acc_lati += acc_acc_lati;
    sum_acc_acc_long += acc_acc_long;
//   
//    
//    speed_acc_lati += acc_acc_lati; // TODO:  precisaria multiplicar algum tempo aqui? mesmo não tendo o delalong
//    speed_acc_DP_lati[i] = speed_acc_lati;
//    speed_acc_long += acc_acc_long;
//    speed_acc_DP_long[i] = speed_acc_long;
//
//    sum_speed_acc_lati += speed_acc_lati;
//    sum_speed_acc_long += speed_acc_long;
//
//
//    posi_acc_lati += speed_acc_lati; // TODO:  precisaria multiplicar algum tempo aqui? mesmo não tendo o delalong
//    posi_acc_DP_lati[i] = posi_acc_lati;
//    posi_acc_long += speed_acc_long;
//    posi_acc_DP_long[i] = posi_acc_long;
//
//    sum_posi_acc_lati += posi_acc_lati;
//    sum_posi_acc_long += posi_acc_long;
  }
//  
  avg_acc_acc_lati = sum_acc_acc_lati / acquires;
  avg_acc_acc_long = sum_acc_acc_long / acquires;
//  float avg_speed_acc_lati = sum_speed_acc_lati / acquires;
//  float avg_speed_acc_long = sum_speed_acc_long / acquires;
//  float avg_posi_acc_lati = sum_posi_acc_lati / acquires;
//  float avg_posi_acc_long = sum_posi_acc_long / acquires;
//  
//
//  float aulati_DP_acc_acc_lati = 0;
//  float aulati_DP_acc_acc_long = 0;
//  float aulati_DP_speed_acc_lati = 0;
//  float aulati_DP_speed_acc_long = 0;
//  float aulati_DP_posi_acc_lati = 0;
//  float aulati_DP_posi_acc_long = 0;
//  for (int i = 0; i < acquires; i++) {
//    aulati_DP_acc_acc_lati += (acc_acc_DP_lati[i] - avg_acc_acc_lati) * (acc_acc_DP_lati[i] - avg_acc_acc_lati);
//    aulati_DP_acc_acc_long += (acc_acc_DP_long[i] - avg_acc_acc_long) * (acc_acc_DP_long[i] - avg_acc_acc_long);
//    
//    aulati_DP_speed_acc_lati += (speed_acc_DP_lati[i] - avg_speed_acc_lati) * (speed_acc_DP_lati[i] - avg_speed_acc_lati);
//    aulati_DP_speed_acc_long += (speed_acc_DP_long[i] - avg_speed_acc_long) * (speed_acc_DP_long[i] - avg_speed_acc_long);
//
//    aulati_DP_posi_acc_lati += (posi_acc_DP_lati[i] - avg_posi_acc_lati) * (posi_acc_DP_lati[i] - avg_posi_acc_lati);
//    aulati_DP_posi_acc_long += (posi_acc_DP_long[i] - avg_posi_acc_long) * (posi_acc_DP_long[i] - avg_posi_acc_long);
//  }
//
//  float DP_acc_acc_lati = sqrt(aulati_DP_acc_acc_lati / acquires);
//  float DP_acc_acc_long = sqrt(aulati_DP_acc_acc_lati / acquires);
//
//  DP_speed_acc_lati = sqrt(aulati_DP_speed_acc_lati / acquires);
//  DP_speed_acc_long = sqrt(aulati_DP_speed_acc_long / acquires);
//
//  DP_posi_acc_lati = sqrt(aulati_DP_posi_acc_lati / acquires);
//  DP_posi_acc_long = sqrt(aulati_DP_posi_acc_long / acquires);
//  
//
//
//  // zera novamente a velocidade e a posição
//  speed_acc_lati = 0.0;
//  speed_acc_long = 0.0;
//
//  posi_acc_lati = 0.0;
//  posi_acc_long = 0.0;
//
//  // Acabou a aquisição de dados para a matriz de covariancia
  digitalWrite(CALIB_PENDING, LOW);
  digitalWrite(CALIB_DONE, HIGH);
}

float acc_acc_lati;
float acc_acc_long;
float A[2][2] = {
  { 1,  0.02 },
  { 0,  1    }
};
float B[2][1] = {
  { 0.0002 },
  { 0.02   }
};

float sigma_posi_acc_lati_2 = DP_posi_acc_lati * DP_posi_acc_lati;
float sigma_posi_acc_long_2 = DP_posi_acc_long * DP_posi_acc_long;
float sigma_speed_acc_lati_2 = DP_speed_acc_lati * DP_speed_acc_lati;
float sigma_speed_acc_long_2 = DP_speed_acc_long * DP_speed_acc_long;

float Q_lati[2][2] = {
  { sigma_posi_acc_lati_2,  0                    },
  { 0,                    sigma_speed_acc_lati_2 }
};

float Q_long[2][2] = {
  { sigma_posi_acc_long_2,  0                    },
  { 0,                    sigma_speed_acc_long_2 }
};

float C[2][2] = {
  {1, 0},
  {0, 1}
};

float xk_lati[2][1] = {
  { 0 },
  { 0 }
};
float xk_long[2][1] = {
  { 0 },
  { 0 }
};
int teste = 1;

void loop() {  
  // read the sensor
  IMU.readSensor();

  // aquisição dos dados do acelerômetro
  // Está sendo subtraído a média por causa dos 9m/s² da gravidade da terra
  acc_acc_lati = IMU.getAccelX_mss() - avg_acc_acc_lati;
  acc_acc_long = IMU.getAccelY_mss() - avg_acc_acc_long;
   
  
  speed_acc_lati += acc_acc_lati * 0.02;
  speed_acc_long += acc_acc_long * 0.02;


  float x_lati[2][1];
  x_lati[0][0] = xk_lati[0][0];
  x_lati[1][0] = xk_lati[1][0];
  float x_long[2][1];
  x_long[0][0] = xk_long[0][0];
  x_long[1][0] = xk_long[1][0];

  float u_lati[1] = { acc_acc_lati };
  float u_long[1] = { acc_acc_long };

  xk_lati[0][0] = ((A[0][0] * x_lati[0][0]) + (A[0][1] * x_lati[1][0])) + (B[0][0] * u_lati[0]);
  xk_lati[1][0] = ((A[1][0] * x_lati[0][0]) + (A[1][1] * x_lati[1][0])) + (B[1][0] * u_lati[0]);
  xk_long[0][0] = ((A[0][0] * x_long[0][0]) + (A[0][1] * x_long[1][0])) + (B[0][0] * u_long[0]);
  xk_long[1][0] = ((A[1][0] * x_long[0][0]) + (A[1][1] * x_long[1][0])) + (B[1][0] * u_long[0]);

  double speedTotal_state = sqrt((xk_lati[1][0]*xk_lati[1][0]) + (xk_long[1][0]*xk_long[1][0]));
  double speedTotal_int = sqrt((speed_acc_lati*speed_acc_lati) + (speed_acc_long*speed_acc_long));

  Serial.print(speedTotal_int + 0.1);
  Serial.print(" ");
  Serial.print(speedTotal_state);
  Serial.print(" ");
  Serial.print(acc_acc_long);
  Serial.print(" ");
  Serial.println(acc_acc_lati);
  
  delay(20);
}
