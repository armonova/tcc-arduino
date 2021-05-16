/*
 * Nome do arquivo: conversion.cpp
 * Autor: Arhur Novaes
 * Última atuliazação: 16/05/2021
 * Descrição: Bibliotca para conversão das medições do acelerômetro para o sistema NED
 */

#include "Arduino.h"
#include "conversion.h"

MPU9250 mpu;

conv_class::conv_class(){
  phi = mpu.getEulerX() * (PI/180); // angulo entre o eixo X e a reta nodal
  theta = mpu.getEulerY() * (PI/180); // angulo entre o eixo X' e a reta nodal
  psi = mpu.getEulerZ() * (PI/180); // anugulo entre o vetor Z e o vetor Z'
  
  // primeira linha da matriz de rotação
  rot[0][0] = cos(theta) * cos(psi);
  rot[0][1] = -(cos(phi) * sin(psi)) + (sin(phi) * sin(theta) * cos(psi));
  rot[0][2] = (sin(phi) * sin(psi)) + (cos(phi) * sin(theta) * cos(psi));
  
  // segunda linha da matriz de rotação
  rot[1][0] = cos(theta) * sin(psi);
  rot[1][1] = (cos(phi) * cos(psi)) + (sin(phi) * sin(theta) * sin(psi));
  rot[1][2] = -(sin(phi) * cos(psi)) + (cos(phi) * sin(theta) * sin(psi));
  
  // terceira linha da matriz de rotação
  rot[2][0] = -sin(theta);
  rot[2][1] = sin(phi) * cos(theta);
  rot[2][2] = cos(phi) * cos(theta);
  
  acc_x = mpu.getAccX();
  acc_y = mpu.getAccY();
  acc_z = mpu.getAccZ();
  
  // multiplicação de matrizes
  acc_N = (acc_x * rot[0][0]) + (acc_y * rot[0][1]) + (acc_z * rot[0][2]);
  acc_E = (acc_x * rot[1][0]) + (acc_y * rot[1][1]) + (acc_z * rot[1][2]);
  acc_D = (acc_x * rot[2][0]) + (acc_y * rot[2][1]) + (acc_z * rot[2][2]); 
}

double conv_class::return_N() {
  return acc_N;
}

double conv_class::return_E() {
  return acc_E;
}

double conv_class::return_D() {
  return acc_D;
}
