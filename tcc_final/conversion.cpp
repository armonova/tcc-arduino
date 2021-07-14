/*
 * Nome do arquivo: conversion.cpp
 * Autor: Arhur Novaes
 * Última atuliazação: 16/05/2021
 * Descrição: Bibliotca para conversão das medições do acelerômetro para o sistema NED
 */

#include "Arduino.h"
#include "conversion.h"


#define PIdiv180 0.01745329251994329576923690768489


MPU9250 mpu;

mpu_conv_class::mpu_conv_class(int calib_pending, int calib_done) {
  _calib_pending = calib_pending;
  _calib_done = calib_done;

  pinMode(_calib_pending, OUTPUT);
  pinMode(_calib_done, OUTPUT);
  digitalWrite(_calib_pending, LOW);
  digitalWrite(_calib_done, LOW);
}

bool mpu_conv_class::config_mpu() {
  Wire.begin();
  delay(2000);
  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      return false;
      delay(3000);
    }
  }
    // calibrate anytime you want to
    // Serial.println("Accel Gyro calibration will start in 5sec.");
    // Serial.println("Please leave the device still on the flat plane.");
    digitalWrite(_calib_pending, HIGH);
    digitalWrite(_calib_done, LOW);
    delay(5000);
    mpu.calibrateAccelGyro();

    // Serial.println("Mag calibration will start in 5sec.");
    // Serial.println("Please Wave device in a figure eight unt1il done.");
    digitalWrite(_calib_pending, HIGH);
    digitalWrite(_calib_done, HIGH);
    delay(5000);
    mpu.calibrateMag();
        
    return true;
}

void mpu_conv_class::make_conversion(float *acc_N, float *acc_E){
  float _phi, _theta, _psi; // angulos de euler
  /*
   float _rot[3][3] = { { 0.0, 0.0, 0.0 },
                       { 0.0, 0.0, 0.0 },
                       { 0.0, 0.0, 0.0 } };
  */
  // converte para angulos em radianos
  // angulos de Euler
  // angulos da medida do magnetómetro - teste
  _phi = mpu.getRoll() * PIdiv180; // angulo entre o eixo X e a reta nodal      // getEulerX
  _theta = mpu.getPitch() * PIdiv180; // angulo entre o eixo X' e a reta nodal  // getEulerY
  _psi = mpu.getYaw() * PIdiv180; // anugulo entre o vetor Z e o vetor Z'       // getEulerZ

  /*
  // primeira linha da matriz de _rotação
  _rot[0][0] = cos(_theta) * cos(_psi);
  _rot[0][1] = -(cos(_phi) * sin(_psi)) + (sin(_phi) * sin(_theta) * cos(_psi));
  _rot[0][2] = (sin(_phi) * sin(_psi)) + (cos(_phi) * sin(_theta) * cos(_psi));
  
  // segunda linha da matriz de _rotação
  _rot[1][0] = cos(_theta) * sin(_psi);
  _rot[1][1] = (cos(_phi) * cos(_psi)) + (sin(_phi) * sin(_theta) * sin(_psi));
  _rot[1][2] = -(sin(_phi) * cos(_psi)) + (cos(_phi) * sin(_theta) * sin(_psi));
  
  // terceira linha da matriz de _rotação
  _rot[2][0] = -sin(_theta);
  _rot[2][1] = sin(_phi) * cos(_theta);
  _rot[2][2] = cos(_phi) * cos(_theta);
  

  // multiplicação de matrizes
  _acc_N = (mpu.getAccX() * _rot[0][0]) + (mpu.getAccY() * _rot[0][1]) + (mpu.getAccZ() * _rot[0][2]);
  _acc_E = (mpu.getAccX() * _rot[1][0]) + (mpu.getAccY() * _rot[1][1]) + (mpu.getAccZ() * _rot[1][2]);
  _acc_D = (mpu.getAccX() * _rot[2][0]) + (mpu.getAccY() * _rot[2][1]) + (mpu.getAccZ() * _rot[2][2]); 
  */
  
  *acc_N = (mpu.getAccX() * cos(_theta) * cos(_psi)) + (mpu.getAccY() * -(cos(_phi) * sin(_psi)) + (sin(_phi) * sin(_theta) * cos(_psi))) + (mpu.getAccZ() * (sin(_phi) * sin(_psi)) + (cos(_phi) * sin(_theta) * cos(_psi)));
  *acc_E = (mpu.getAccX() * cos(_theta) * sin(_psi)) + (mpu.getAccY() * (cos(_phi) * cos(_psi)) + (sin(_phi) * sin(_theta) * sin(_psi))) + (mpu.getAccZ() * -(sin(_phi) * cos(_psi)) + (cos(_phi) * sin(_theta) * sin(_psi)));
  //*acc_D = (mpu.getAccX() * -sin(_theta)) + (mpu.getAccY() * sin(_phi) * cos(_theta)) + (mpu.getAccZ() * cos(_phi) * cos(_theta)); 
}

bool mpu_conv_class::update_data() {
  return mpu.update();
}

void mpu_conv_class::returnCordCart(float mod, float *outN, float *outE, float refN, float refE) {
  if (refN != 0 && refE != 0) {
    float angle = atan2(refN, refE);
    *outN = sin(angle) * mod;
    *outE = cos(angle) * mod;
  } else {
    *outN = 0;
    *outE = 0;
  }
}
