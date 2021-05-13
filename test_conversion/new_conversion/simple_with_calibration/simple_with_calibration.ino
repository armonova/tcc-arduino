#include "MPU9250.h"

#define CALIB_PENDING 13
#define CALIB_DONE 8
#define PI 3.1415926535897932384626433832795

MPU9250 mpu;

double C[3][3] = { { 0.0, 0.0, 0.0 },
                   { 0.0, 0.0, 0.0 },
                   { 0.0, 0.0, 0.0 } };
double theta, phi, psi;

double acc_x, acc_y, acc_z;
double acc_N, acc_E, acc_D;

void setup() {
    pinMode(CALIB_PENDING, OUTPUT);
    pinMode(CALIB_DONE, OUTPUT);
    digitalWrite(CALIB_PENDING, LOW);
    digitalWrite(CALIB_DONE, LOW);
    
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
//    Serial.println("Accel Gyro calibration will start in 5sec.");
//    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

//    Serial.println("Mag calibration will start in 5sec.");
//    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();

//    print_calibration();
    mpu.verbose(false);
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
}

void print_roll_pitch_yaw() {
//    Serial.print("Euler X, Euler Y, Euler Z: ");
//    Serial.print(mpu.getEulerX(), 2); 
    
    phi = mpu.getEulerX() * (PI/180); // angulo entre o eixo X e a reta nodal
    theta = mpu.getEulerY() * (PI/180); // angulo entre o eixo X' e a reta nodal
    psi = mpu.getEulerZ() * (PI/180); // anugulo entre o vetor Z e o vetor Z'

    // primeira linha da matriz de rotação
    C[0][0] = cos(theta) * cos(psi);
    C[0][1] = -(cos(phi) * sin(psi)) + (sin(phi) * sin(theta) * cos(psi));
    C[0][2] = (sin(phi) * sin(psi)) + (cos(phi) * sin(theta) * cos(psi));

    // segunda linha da matriz de rotação
    C[1][0] = cos(theta) * sin(psi);
    C[1][1] = (cos(phi) * cos(psi)) + (sin(phi) * sin(theta) * sin(psi));
    C[1][2] = -(sin(phi) * cos(psi)) + (cos(phi) * sin(theta) * sin(psi));

    // terceira linha da matriz de rotação
    C[2][0] = -sin(theta);
    C[2][1] = sin(phi) * cos(theta);
    C[2][2] = cos(phi) * cos(theta);

    acc_x = mpu.getAccX();
    acc_y = mpu.getAccY();
    acc_z = mpu.getAccZ();

    // multiplicação de matrizes
    acc_N = (acc_x * C[0][0]) + (acc_y * C[0][1]) + (acc_z * C[0][2]);
    acc_E = (acc_x * C[1][0]) + (acc_y * C[1][1]) + (acc_z * C[1][2]);
    acc_D = (acc_x * C[2][0]) + (acc_y * C[2][1]) + (acc_z * C[2][2]);

  Serial.print(acc_N);
  Serial.print(" ");
  Serial.print(acc_E);
  Serial.print(" ");
  Serial.println(acc_D);
    
    

    
    
}

void print_calibration() {
//    Serial.println("< calibration parameters >");
//    Serial.println("accel bias [g]: ");
//    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
//    Serial.print(", ");
//    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
//    Serial.print(", ");
//    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
//    Serial.println();
//    Serial.println("gyro bias [deg/s]: ");
//    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
//    Serial.print(", ");
//    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
//    Serial.print(", ");
//    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
//    Serial.println();
//    Serial.println("mag bias [mG]: ");
//    Serial.print(mpu.getMagBiasX());
//    Serial.print(", ");
//    Serial.print(mpu.getMagBiasY());
//    Serial.print(", ");
//    Serial.print(mpu.getMagBiasZ());
//    Serial.println();
//    Serial.println("mag scale []: ");
//    Serial.print(mpu.getMagScaleX());
//    Serial.print(", ");
//    Serial.print(mpu.getMagScaleY());
//    Serial.print(", ");
//    Serial.print(mpu.getMagScaleZ());
//    Serial.println();

    digitalWrite(CALIB_PENDING, HIGH);
    digitalWrite(CALIB_DONE, LOW);
    mpu.getAccBiasX();
    mpu.getAccBiasY();
    mpu.getAccBiasZ();
    mpu.getGyroBiasX();
    mpu.getGyroBiasY();
    mpu.getGyroBiasZ();
    mpu.getMagBiasX();
    mpu.getMagBiasY();
    mpu.getMagBiasZ();
    mpu.getMagScaleX();
    mpu.getMagScaleY();
    mpu.getMagScaleZ();
    digitalWrite(CALIB_PENDING, LOW);
    digitalWrite(CALIB_DONE, HIGH);
}
