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
    mpu_conv_class();
    void config_mpu();
    void make_conversion();
    double return_N();
    double return_E();
    double return_D();
    bool update_data();
    
  private:
    double phi;
    double theta;
    double psi;
    
    double acc_N, acc_E, acc_D;
    double acc_x, acc_y, acc_z;
    
    double rot[3][3] = { { 0.0, 0.0, 0.0 },
                         { 0.0, 0.0, 0.0 },
                         { 0.0, 0.0, 0.0 } };
};

#endif
