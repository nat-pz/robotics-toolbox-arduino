#ifndef MATESPAC_H
#define MATESPAC_H

#include <Arduino.h>
#include <ArduinoEigen.h>

float norm(float v);
Eigen::Matrix<float, 6, 1> tr2delta(Eigen::Matrix4f T0, Eigen::Matrix4f T1);
Eigen::Matrix<float, 6, 1> tr2delta(Eigen::Matrix4f argumento);
Eigen::Vector3f transl(Eigen::Matrix4f T);
Eigen::Vector3f vex(Eigen::Matrix3f S);
float vex(Eigen::Matrix2f S);
Eigen::Matrix3f t2r(Eigen::Matrix4f T);
Eigen::Matrix3f trnorm(Eigen::Matrix4f T);

#endif