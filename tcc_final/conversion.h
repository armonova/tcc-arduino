/*
 * Nome do arquivo: conversion.h
 * Autor: Arhur Novaes
 * Última atuliazação: 16/05/2021
 * Descrição: Bibliotca para conversão das medições do acelerômetro para o sistema NED
 */

#ifndef conversion
#define conversion

#include "Arduino.h"
#include "MPU9250.h"

class mpu_conv_class {
  public:
    mpu_conv_class(int calib_pending, int calib_done);
    bool config_mpu();
    void make_conversion();
    float return_acc_NED(char select);
    // float return_acc_XYZ(char select);
    bool update_data();
    void returnCordCart(float mod, float *outN, float *outE, float refN, float refE);
    
  private:
    float _acc_N, _acc_E, _acc_D;

    char _calib_pending, _calib_done;
};

#endif
