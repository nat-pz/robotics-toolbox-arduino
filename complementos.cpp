#include "complementos.h"

void printMatriz(Eigen::Matrix<float, 3, 1> matriz) {
  String salida = String(matriz(0)) + "\t" + String(matriz(1)) + "\t" + String(matriz(2));
  Serial.println(salida);
}

void printMatriz(Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> matriz) {
  String salida = "";
  for (int j = 0; j < NUMERO_DE_ESLABONES; ++j) {
    salida += String(matriz(j),4) + " ";
  }
  Serial.println(salida);
}

void printMatriz(Eigen::Matrix<float, 2, 1> matriz) {
  String salida = "";
  for (int j = 0; j < 2; ++j) {
    salida += String(matriz(j)) + "\n";
  }
  Serial.println(salida);
}

void printMatriz(Eigen::Matrix4f matriz) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      Serial.print(matriz(i, j),4);
      Serial.print(F("\t"));
    }
    Serial.println();
  }
}

void printMatriz(Eigen::Matrix3f matriz) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Serial.print(matriz(i, j),4);
      Serial.print(F("\t"));
    }
    Serial.println();
  }
}

void printMatriz(Eigen::Matrix<float, 6, NUMERO_DE_ESLABONES> matriz) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < NUMERO_DE_ESLABONES; j++) {
      Serial.print(matriz(i, j), 4);
      Serial.print(F("\t"));
    }
    Serial.println();
  }
}

float radianes_a_grados(float rad){
  float resultado = rad * 180.0 / M_PI;
  return resultado;
}

float grados_a_radianes(float grados){
 float resultado = grados * M_PI / 180.0;
 return resultado;
}

