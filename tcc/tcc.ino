#include "MPU9250.h"

#define CALIB_PENDING 13
#define CALIB_DONE 8


float avg_acc_acc_x;
float avg_acc_acc_y;

double speed_acc_x = 0.0;
double speed_acc_y = 0.0;

double posi_acc_x = 0.0;
double posi_acc_y = 0.0;


float DP_speed_acc_x;
float DP_speed_acc_y;

float DP_posi_acc_x;
float DP_posi_acc_y;


// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;

void setup() {
  // serial to display data
  pinMode(CALIB_PENDING, OUTPUT);
  pinMode(CALIB_DONE, OUTPUT);

  digitalWrite(CALIB_PENDING, HIGH);
  digitalWrite(CALIB_DONE, LOW);
  
  Serial.begin(9600);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  IMU.setSrd(19);

  int acquires = 100; // num of data acquires to standard deviation
  float acc_acc_DP_x[100];
  float acc_acc_DP_y[100];
  float speed_acc_DP_x[100];
  float speed_acc_DP_y[100];
  float posi_acc_DP_x[100];
  float posi_acc_DP_y[100];

  float sum_acc_acc_x = 0.0;
  float sum_acc_acc_y = 0.0;
  float sum_speed_acc_x = 0.0;
  float sum_speed_acc_y = 0.0;
  float sum_posi_acc_x = 0.0;
  float sum_posi_acc_y = 0.0;
  
  for (int i = 0; i < acquires; i++) {
    IMU.readSensor();
    float acc_acc_x = IMU.getAccelX_mss() - avg_acc_acc_x;
    acc_acc_DP_x[i] = acc_acc_x;
    float acc_acc_y = IMU.getAccelY_mss() - avg_acc_acc_y;
    acc_acc_DP_x[i] = acc_acc_x;
    
    sum_acc_acc_x += acc_acc_x;
    sum_acc_acc_y += acc_acc_y;
   
    
    speed_acc_x += acc_acc_x; // TODO:  precisaria multiplicar algum tempo aqui? mesmo não tendo o delay
    speed_acc_DP_x[i] = speed_acc_x;
    speed_acc_y += acc_acc_y;
    speed_acc_DP_y[i] = speed_acc_y;

    sum_speed_acc_x += speed_acc_x;
    sum_speed_acc_y += speed_acc_y;


    posi_acc_x += speed_acc_x; // TODO:  precisaria multiplicar algum tempo aqui? mesmo não tendo o delay
    posi_acc_DP_x[i] = posi_acc_x;
    posi_acc_y += speed_acc_y;
    posi_acc_DP_y[i] = posi_acc_y;

    sum_posi_acc_x += posi_acc_x;
    sum_posi_acc_y += posi_acc_y;
  }
  
  avg_acc_acc_x = sum_acc_acc_x / acquires;
  avg_acc_acc_y = sum_acc_acc_y / acquires;
  float avg_speed_acc_x = sum_speed_acc_x / acquires;
  float avg_speed_acc_y = sum_speed_acc_y / acquires;
  float avg_posi_acc_x = sum_posi_acc_x / acquires;
  float avg_posi_acc_y = sum_posi_acc_y / acquires;
  

  float aux_DP_acc_acc_x = 0;
  float aux_DP_acc_acc_y = 0;
  float aux_DP_speed_acc_x = 0;
  float aux_DP_speed_acc_y = 0;
  float aux_DP_posi_acc_x = 0;
  float aux_DP_posi_acc_y = 0;
  for (int i = 0; i < acquires; i++) {
    aux_DP_acc_acc_x += (acc_acc_DP_x[i] - avg_acc_acc_x) * (acc_acc_DP_x[i] - avg_acc_acc_x);
    aux_DP_acc_acc_y += (acc_acc_DP_y[i] - avg_acc_acc_y) * (acc_acc_DP_y[i] - avg_acc_acc_y);
    
    aux_DP_speed_acc_x += (speed_acc_DP_x[i] - avg_speed_acc_x) * (speed_acc_DP_x[i] - avg_speed_acc_x);
    aux_DP_speed_acc_y += (speed_acc_DP_y[i] - avg_speed_acc_y) * (speed_acc_DP_y[i] - avg_speed_acc_y);

    aux_DP_posi_acc_x += (posi_acc_DP_x[i] - avg_posi_acc_x) * (posi_acc_DP_x[i] - avg_posi_acc_x);
    aux_DP_posi_acc_y += (posi_acc_DP_y[i] - avg_posi_acc_y) * (posi_acc_DP_y[i] - avg_posi_acc_y);
  }

  float DP_acc_acc_x = sqrt(aux_DP_acc_acc_x / acquires);
  float DP_acc_acc_y = sqrt(aux_DP_acc_acc_x / acquires);

  DP_speed_acc_x = sqrt(aux_DP_speed_acc_x / acquires);
  DP_speed_acc_y = sqrt(aux_DP_speed_acc_y / acquires);

  DP_posi_acc_x = sqrt(aux_DP_posi_acc_x / acquires);
  DP_posi_acc_y = sqrt(aux_DP_posi_acc_y / acquires);
  


  // zera novamente a velocidade e a posição
  speed_acc_x = 0.0;
  speed_acc_y = 0.0;

  posi_acc_x = 0.0;
  posi_acc_y = 0.0;

  // Acabou a aquisição de dados para a matriz de covariancia
  digitalWrite(CALIB_PENDING, LOW);
  digitalWrite(CALIB_DONE, HIGH);
}

float acc_acc_x;
float acc_acc_y;
float A[2][2] = {
  { 1,  0.02 },
  { 0,  1    }
};
float B[2][1] = {
  { 0.0002 },
  { 0.02   }
};

float sigma_posi_acc_x_2 = DP_posi_acc_x * DP_posi_acc_x;
float sigma_posi_acc_y_2 = DP_posi_acc_y * DP_posi_acc_y;
float sigma_speed_acc_x_2 = DP_speed_acc_x * DP_speed_acc_x;
float sigma_speed_acc_y_2 = DP_speed_acc_y * DP_speed_acc_y;

float Q_x[2][2] = {
  { sigma_posi_acc_x_2,  0                    },
  { 0,                    sigma_speed_acc_x_2 }
};

float Q_y[2][2] = {
  { sigma_posi_acc_y_2,  0                    },
  { 0,                    sigma_speed_acc_y_2 }
};

float C[2][2] = {
  {1, 0},
  {0, 1}
};

void loop() {
  // read the sensor
  IMU.readSensor();
  
  // calcular a velocidade através do acelerometro  
  acc_acc_x = IMU.getAccelX_mss() - avg_acc_acc_x;
  acc_acc_y = IMU.getAccelY_mss() - avg_acc_acc_y;
   
  
  speed_acc_x += acc_acc_x * 0.02;
  speed_acc_y += acc_acc_y * 0.02;

  double speedTotal = sqrt((speed_acc_x*speed_acc_x) + (speed_acc_y*speed_acc_y));

  Serial.print(speedTotal);
  Serial.print(" ");
  Serial.println(acc_acc_x);
  
  delay(20);
}
