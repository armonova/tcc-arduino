/*
 * Nome do arquivo: conversion.cpp
 * Autor: Arhur Novaes
 * Última atuliazação: 16/05/2021
 * Descrição: Bibliotca para conversão das medições do acelerômetro para o sistema NED
 */

#include "Arduino.h"
#include "conversion.h"


#define PI 3.1415926535897932384626433832795


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
      delay(5000);
    }
  }
    // calibrate anytime you want to
    // Serial.println("Accel Gyro calibration will start in 5sec.");
    // Serial.println("Please leave the device still on the flat plane.");
    digitalWrite(_calib_pending, HIGH);
    digitalWrite(_calib_done, LOW);
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    // Serial.println("Mag calibration will start in 5sec.");
    // Serial.println("Please Wave device in a figure eight unt1il done.");
    digitalWrite(_calib_pending, HIGH);
    digitalWrite(_calib_done, HIGH);
    delay(5000);
    mpu.calibrateMag();

    mpu.verbose(false);
    
    return true;
}

void mpu_conv_class::make_conversion(){
  // converte para angulos em radianos
  _phi = mpu.getEulerX() * (PI/180.0); // angulo entre o eixo X e a reta nodal
  _theta = mpu.getEulerY() * (PI/180.0); // angulo entre o eixo X' e a reta nodal
  _psi = mpu.getEulerZ() * (PI/180.0); // anugulo entre o vetor Z e o vetor Z'
  
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
  
  _acc_x = mpu.getAccX();
  _acc_y = mpu.getAccY();
  _acc_z = mpu.getAccZ();
  
  // multiplicação de matrizes
  _acc_N = (_acc_x * _rot[0][0]) + (_acc_y * _rot[0][1]) + (_acc_z * _rot[0][2]);
  _acc_E = (_acc_x * _rot[1][0]) + (_acc_y * _rot[1][1]) + (_acc_z * _rot[1][2]);
  _acc_D = (_acc_x * _rot[2][0]) + (_acc_y * _rot[2][1]) + (_acc_z * _rot[2][2]); 
}

bool mpu_conv_class::update_data() {
  return mpu.update();
}

double mpu_conv_class::return_acc_NED(char select) {
  if (select == 'N') return _acc_N;
  if (select == 'E') return _acc_E;
  if (select == 'D') return _acc_D;
}

double mpu_conv_class::return_acc_XYZ(char select) {
  if (select == 'x') return _acc_x;
  if (select == 'y') return _acc_y;
  if (select == 'z') return _acc_z;
}

void mpu_conv_class::standard_deviation() {
  char _acquires = 100; // número de aquisição para cálculo do desvio padrão
  
  float _acc_acc_DP_N[_acquires];
  float _acc_acc_DP_E[_acquires];

  float _sum_acc_acc_N = 0.0;
  float _sum_acc_acc_E = 0.0;
  
  for (char i = 0; i < _acquires; i++) {
    make_conversion();
    _acc_acc_DP_N[i] = _acc_N;
    _acc_acc_DP_E[i] = _acc_E;
  
    _sum_acc_acc_N += _acc_acc_DP_N[i];
    _sum_acc_acc_E += _acc_acc_DP_E[i];
    delay(25); // frequencia de 40Hz
  }

  delay(2000);

  float _avg_acc_acc_N = _sum_acc_acc_N / _acquires;  
  float _avg_acc_acc_E = _sum_acc_acc_E / _acquires;
  
  float _auxN_DP_acc_acc_N = 0.0;
  float _auxE_DP_acc_acc_E = 0.0;

  for (int i = 0; i < _acquires; i++) {
    _auxN_DP_acc_acc_N += (_acc_acc_DP_N[i] - _avg_acc_acc_N) * (_acc_acc_DP_N[i] - _avg_acc_acc_N);
    _auxE_DP_acc_acc_E += (_acc_acc_DP_E[i] - _avg_acc_acc_E) * (_acc_acc_DP_E[i] - _avg_acc_acc_E);
  }

  _DP_acc_acc_N = sqrt(_auxN_DP_acc_acc_N / _acquires);  
  _DP_acc_acc_E = sqrt(_auxE_DP_acc_acc_E / _acquires);
}

double mpu_conv_class::return_DP_NED(char select) {
  if (select == 'N') return _DP_acc_acc_N;
  if (select == 'E') return _DP_acc_acc_E;
}
