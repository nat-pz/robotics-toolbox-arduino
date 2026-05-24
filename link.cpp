#include "link.h"

Link::Link() {
}

Link::Link(float theta, float d, float alpha, float a) {
  set_theta(theta);
  set_d(d);
  set_a(a);
  set_alpha(alpha);
  set_jointtype('R');
  set_mdh(false);
  set_offset(0.0);
  set_flip(false);
  set_m(0.0);
  set_r(0.0, 0.0, 0.0);
  set_qlim(0.0, 0.0);
  set_I(0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0);
  set_B(0.0);
  set_Tc(0.0, 0.0);
  set_G(0.0);
  set_Jm(0.0);
}

float Link::get_theta() {
  return theta;
}

void Link::set_theta(float rad) {
  theta = rad;
}

float Link::get_d() {
  return d;
}

void Link::set_d(float valor) {
  d = valor;
}

float Link::get_alpha() {
  return alpha;
}

void Link::set_alpha(float rad) {
  alpha = rad;
}

float Link::get_a() {
  return a;
}

void Link::set_a(float valor) {
  a = valor;
}

char Link::get_jointtype() {
  return jointtype;
}

void Link::set_jointtype(char valor) {
  jointtype = valor;
}

bool Link::get_mdh() {
  return mdh;
}

void Link::set_mdh(bool valor) {
  if (valor == 1 || valor == 0) {
    mdh = valor;
  } else {
    mdh = mdh;
  }
}

float Link::get_offset() {
  return offset;
}

void Link::set_offset(float valor) {
  offset = valor;
}

bool Link::get_flip() {
  return flip;
}

void Link::set_flip(bool valor) {
  if (valor == 1 || valor == 0) {
    flip = valor;
  } else {
    flip = flip;
  }
}

float Link::get_m() {
  return m;
}

void Link::set_m(float valor) {
  m = valor;
}

Eigen::Matrix<float, 3, 1> Link::get_r() {
  return r;
}

void Link::set_r(float valor1, float valor2, float valor3) {
  r << valor1, valor2, valor3;
}

Eigen::Matrix<float, 2, 1> Link::get_qlim() {
  return qlim;
}

void Link::set_qlim(float min, float max) {
  qlim << min, max;
}

Eigen::Matrix<float, 3, 3> Link::get_I() {
  return I;
}

void Link::set_I(float a11, float a12, float a13,
                 float a21, float a22, float a23,
                 float a31, float a32, float a33) {
  I << a11, a12, a13,
    a21, a22, a23,
    a31, a32, a33;
}

float Link::get_B() {
  return B;
}

void Link::set_B(float valor) {
  B = valor;
}

Eigen::Matrix<float, 2, 1> Link::get_Tc() {
  return Tc;
}

void Link::set_Tc(float valor) {
  Tc << valor, -valor;
}

void Link::set_Tc(float valor1, float valor2) {
  Tc << valor1, valor2;
}

float Link::get_G() {
  return G;
}

void Link::set_G(float valor) {
  G = valor;
}

float Link::get_Jm() {
  return Jm;
}

void Link::set_Jm(float valor) {
  Jm = valor;
}

String Link::get_name() {
  return name;
}

void Link::set_name(String nombre) {
  name = nombre;
}

bool Link::isrevolute() {
  return jointtype == 'R';
}

bool Link::isprismatic() {
  return jointtype == 'P';
}

String Link::display() {
  String type = jointtype == 'R' ? "Revolution" : "Prismatic";
  return type + ": theta=" + String(theta) + ", d= " + String(d) + ", a=" + String(a) + ", alpha= " + String(alpha) + ", offset= " + String(offset) + "\n";
}

String Link::dyn() {
  String result = display();
  result += "m= " + String(get_m()) + "\n";
  result += "r= " + String(r(0)) + " " + String(r(1)) + " " + String(r(2)) + "\n";
  result += "I:\n" + String(I(0, 0)) + " " + String(I(0, 1)) + " " + String(I(0, 2)) + "\n";
  result += String(I(1, 0)) + " " + String(I(1, 1)) + " " + String(I(1, 2)) + "\n";
  result += String(I(2, 0)) + " " + String(I(2, 1)) + " " + String(I(2, 2)) + "\n";
  result += "G= " + String(get_G()) + "\n";
  result += "Jm= " + String(get_Jm()) + "\n";
  return result;
}

Eigen::Matrix<float, 4, 4> Link::A(float q) {
  Eigen::Matrix<float, 4, 4> T;
  if (flip) {
    q = -q + offset;
  } else {
    q = q + offset;
  }

  float st, ct, dd;
  if (jointtype == 'R') {  // Revolución
    st = sin(q);
    ct = cos(q);
    dd = d;
  } else {  // Prismática
    st = sin(theta);
    ct = cos(theta);
    dd = q;
  }

  if (mdh == 0) {  // Parámetros DH estándar
    T << ct, -st * cos(alpha), st * sin(alpha), a * ct,
      st, ct * cos(alpha), -ct * sin(alpha), a * st,
      0, sin(alpha), cos(alpha), dd,
      0, 0, 0, 1;
  } else {  // Parámetros DH modificados
    T << ct, -st, 0, a,
      st * cos(alpha), ct * cos(alpha), -sin(alpha), -sin(alpha) * dd,
      st * sin(alpha), ct * sin(alpha), cos(alpha), cos(alpha) * dd,
      0, 0, 0, 1;
  }
  return T;
}

float Link::friction(float qd) {
  float tau = get_B() * abs(get_G()) * qd;
  Eigen::Matrix<float, 1, 2> Tc = get_Tc();
  if (qd > 0) {
    tau = tau + Tc(0,0);
  } else if (qd < 0) {
    tau = tau + Tc(1,1);
  }
  tau = -abs(get_G()) * tau; 

  return tau;
}