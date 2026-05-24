#ifndef LINK_H
#define LINK_H

#include <Arduino.h>
#include <ArduinoEigen.h>

class Link {
public:
  Link();
  Link(float theta, float d, float alpha, float a);

  float get_theta();
  void set_theta(float rad);

  float get_d();
  void set_d(float valor);

  float get_alpha();
  void set_alpha(float rad);

  float get_a();
  void set_a(float valor);

  char get_jointtype();
  void set_jointtype(char valor);

  bool get_mdh();
  void set_mdh(bool valor);

  float get_offset();
  void set_offset(float valor);

  bool get_flip();
  void set_flip(bool valor);

  float get_m();
  void set_m(float valor);

  Eigen::Matrix<float, 3, 1> get_r();
  void set_r(float valor1, float valor2, float valor3);

  Eigen::Matrix<float, 2, 1> get_qlim();
  void set_qlim(float min, float max);

  Eigen::Matrix<float, 3, 3> get_I();
  void set_I(float a11, float a12, float a13,
             float a21, float a22, float a23,
             float a31, float a32, float a33);

  float get_B();
  void set_B(float valor);

  Eigen::Matrix<float, 2, 1> get_Tc();
  void set_Tc(float valor);
  void set_Tc(float valor1, float valor2);

  float get_G();
  void set_G(float valor);

  float get_Jm();
  void set_Jm(float valor);

  String get_name();
  void set_name(String nombre);

  bool isrevolute();
  bool isprismatic();

  String display();
  String dyn();

  Eigen::Matrix<float, 4, 4> A(float q);

  float friction(float qd);

private:
  float theta, d, alpha, a;
  char jointtype;
  bool mdh, flip;
  float offset, m, B;
  float G;
  float Jm;
  Eigen::Matrix<float, 3, 1> r;
  Eigen::Matrix<float, 2, 1> qlim;
  Eigen::Matrix<float, 3, 3> I;
  //| Ixx  Ixy  Ixz |
  //| Ixy  Iyy  Iyz |
  //| Ixz  Iyz  Izz |
  Eigen::Matrix<float, 2, 1> Tc;
  String name;
};

#endif
