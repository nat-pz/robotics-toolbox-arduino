#ifndef COMPLEMENTOS_H
#define COMPLEMENTOS_H
#define NUMERO_DE_ESLABONES 6
#define NUMERO_DE_ACTUADORES 5


#include <Arduino.h>
#include <ArduinoEigen.h>

void printMatriz(Eigen::Matrix<float, 3, 1> matriz);

void printMatriz(Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> matriz);

void printMatriz(Eigen::Matrix<float, 2, 1> matriz);

void printMatriz(Eigen::Matrix4f matriz);

void printMatriz(Eigen::Matrix3f matriz);

void printMatriz(Eigen::Matrix<float, 6, NUMERO_DE_ESLABONES> matriz);

float radianes_a_grados(float rad);
float grados_a_radianes(float grados);


#endif  // COMPLEMENTOS_H
