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
    double return_acc_NED(char select);
    double return_acc_XYZ(char select);
    bool update_data();
    void standard_deviation();
    double return_DP_NED(char select);
    
    
  private:
    double _phi, _theta, _psi; // angulos de euler
    double _acc_N, _acc_E, _acc_D;
    double _acc_x, _acc_y, _acc_z;

    int _calib_pending, _calib_done;

    double _DP_acc_acc_N, _DP_acc_acc_E; // desvio padrão da aceleração
    
    double _rot[3][3] = { { 0.0, 0.0, 0.0 },
                         { 0.0, 0.0, 0.0 },
                         { 0.0, 0.0, 0.0 } };
};

#endif
