#include "seriallink.h"
#include <Arduino.h>
#include <ArduinoEigen.h>

SerialLink::SerialLink() {
}

SerialLink::SerialLink(Link link) {
  add_link(link, 1);
  set_gravity(0.0, 0.0, 9.81);
  set_base(1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1);
  set_tool(1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1);
  set_grados(false);
  set_mdh(false);
  set_flip(false);
}

void SerialLink::add_link(Link link, int numero_de_enlace) {
  set_theta(link.get_theta(), numero_de_enlace);
  set_a(link.get_a(), numero_de_enlace);
  set_alpha(link.get_alpha(), numero_de_enlace);
  set_d(link.get_d(), numero_de_enlace);
  set_config(link.get_jointtype(), numero_de_enlace);
  set_offset(link.get_offset(), numero_de_enlace);
  set_G(link.get_G(), numero_de_enlace);
  set_qlim(link.get_qlim(), numero_de_enlace);
  set_m(link.get_m(), numero_de_enlace);
  set_r(link.get_r(), numero_de_enlace);
  Eigen::Matrix<float, 3, 3> inertia = link.get_I();
  set_I(inertia(0, 0), inertia(1, 1), inertia(2, 2), inertia(0, 1), inertia(0, 2), inertia(1, 2), numero_de_enlace);
  set_B(link.get_B(), numero_de_enlace);
  set_Tc(link.get_Tc(), numero_de_enlace);
  set_Jm(link.get_Jm(), numero_de_enlace);
}

Eigen::Matrix<float, 3, 1> SerialLink::get_gravity() {
  return gravity;
}

void SerialLink::set_gravity(float gx, float gy, float gz) {
  gravity(0) = gx;
  gravity(1) = gy;
  gravity(2) = gz;
}

void SerialLink::set_grados(bool valor) {
  grados = valor;
}

bool SerialLink::get_grados() {
  return grados;
}

// base
Eigen::Matrix4f SerialLink::get_base() {
  return base;
}

void SerialLink::set_base(float a11, float a12, float a13, float a14,
                          float a21, float a22, float a23, float a24,
                          float a31, float a32, float a33, float a34,
                          float a41, float a42, float a43, float a44) {
  base(0, 0) = a11;
  base(0, 1) = a12;
  base(0, 2) = a13;
  base(0, 3) = a14;
  base(1, 0) = a21;
  base(1, 1) = a22;
  base(1, 2) = a23;
  base(1, 3) = a24;
  base(2, 0) = a31;
  base(2, 1) = a32;
  base(2, 2) = a33;
  base(2, 3) = a34;
  base(3, 0) = a41;
  base(3, 1) = a42;
  base(3, 2) = a43;
  base(3, 3) = a44;
}

// tool
Eigen::Matrix4f SerialLink::get_tool() {
  return tool;
}

void SerialLink::set_tool(float a11, float a12, float a13, float a14,
                          float a21, float a22, float a23, float a24,
                          float a31, float a32, float a33, float a34,
                          float a41, float a42, float a43, float a44) {
  tool(0, 0) = a11;
  tool(0, 1) = a12;
  tool(0, 2) = a13;
  tool(0, 3) = a14;
  tool(1, 0) = a21;
  tool(1, 1) = a22;
  tool(1, 2) = a23;
  tool(1, 3) = a24;
  tool(2, 0) = a31;
  tool(2, 1) = a32;
  tool(2, 2) = a33;
  tool(2, 3) = a34;
  tool(3, 0) = a41;
  tool(3, 1) = a42;
  tool(3, 2) = a43;
  tool(3, 3) = a44;
}

bool SerialLink::get_flip() {
  return flip;
}

void SerialLink::set_flip(bool valor) {
  flip = valor;
}

// joint limits
Eigen::Matrix<float, NUMERO_DE_ESLABONES, 2> SerialLink::get_qlim() {
  return qlim;
}

Eigen::Matrix<float, 2, 1> SerialLink::get_qlim(int eslabon) {
  Eigen::Matrix<float, 2, 1> result;
  result(0) = qlim(eslabon - 1, 0);
  result(1) = qlim(eslabon - 1, 1);
  return result;
}

void SerialLink::set_qlim(Eigen::Matrix<float, 2, 1> limites, int numero_de_articulacion) {
  qlim(numero_de_articulacion - 1, 0) = limites(0);
  qlim(numero_de_articulacion - 1, 1) = limites(1);
}

// offset
float SerialLink::get_offset(int eslabon) {
  return offset[eslabon - 1];
}

void SerialLink::set_offset(float valor, int numero_de_articulacion) {
  offset[numero_de_articulacion - 1] = valor;
}

// name
String SerialLink::get_name() {
  return name;
}

void SerialLink::set_name(String nombre) {
  name = nombre;
}

// manufacturer
String SerialLink::get_manuf() {
  return manuf;
}

void SerialLink::set_manuf(String nombre) {
  manuf = nombre;
}

// comments
String SerialLink::get_comment() {
  return comment;
}

void SerialLink::set_comment(String comentario) {
  comment = comentario;
}

// modified DH
bool SerialLink::get_mdh() {
  return mdh;
}

void SerialLink::set_mdh(bool valor) {
  mdh = valor;
}

char SerialLink::get_config(int numero_de_articulacion) {
  return config[numero_de_articulacion - 1];
}

String SerialLink::get_config() {
  String result;
  for (int i = 0; i < NUMERO_DE_ESLABONES; ++i) {
    result += config[i];
  }
  return result;
}

void SerialLink::set_config(char valor, int numero_de_articulacion) {
  config[numero_de_articulacion - 1] = valor;
}

// theta
float SerialLink::get_theta(int eslabon) {
  return theta[eslabon - 1];
}

void SerialLink::set_theta(float valor, int eslabon) {
  theta[eslabon - 1] = valor;
}

// d
float SerialLink::get_d(int eslabon) {
  return d[eslabon - 1];
}

void SerialLink::set_d(float valor, int eslabon) {
  d[eslabon - 1] = valor;
}

// alpha
float SerialLink::get_alpha(int eslabon) {
  return alpha[eslabon - 1];
}

void SerialLink::set_alpha(float valor, int eslabon) {
  alpha[eslabon - 1] = valor;
}

// a
float SerialLink::get_a(int eslabon) {
  return a[eslabon - 1];
}

void SerialLink::set_a(float valor, int eslabon) {
  a[eslabon - 1] = valor;
}

// mass
float SerialLink::get_m(int eslabon) {
  return m[eslabon - 1];
}

void SerialLink::set_m(float valor, int eslabon) {
  m[eslabon - 1] = valor;
}

// centro de masa
Eigen::Matrix<float, 1, 3> SerialLink::get_r(int eslabon) {
  Eigen::Matrix<float, 1, 3> result;
  result(0) = r[eslabon - 1][0];
  result(1) = r[eslabon - 1][1];
  result(2) = r[eslabon - 1][2];
  return result;
}

void SerialLink::set_r(float valor1, float valor2, float valor3, int eslabon) {
  r[eslabon - 1][0] = valor1;
  r[eslabon - 1][1] = valor2;
  r[eslabon - 1][2] = valor3;
}

void SerialLink::set_r(Eigen::Matrix<float, 1, 3> valor, int eslabon) {
  r[eslabon - 1][0] = valor(0);
  r[eslabon - 1][1] = valor(1);
  r[eslabon - 1][2] = valor(2);
}

// inercia
Eigen::Matrix<float, 3, 3> SerialLink::get_I(int eslabon) {
  Eigen::Matrix<float, 3, 3> result;
  result(0, 0) = Ixx[eslabon - 1];
  result(1, 1) = Iyy[eslabon - 1];
  result(2, 2) = Izz[eslabon - 1];
  result(0, 1) = result(1, 0) = Ixy[eslabon - 1];
  result(0, 2) = result(2, 0) = Ixz[eslabon - 1];
  result(1, 2) = result(2, 1) = Iyz[eslabon - 1];
  return result;
}

void SerialLink::set_I(float Ixx, float Iyy, float Izz, float Ixy, float Ixz, float Iyz, int eslabon) {
  this->Ixx[eslabon - 1] = Ixx;
  this->Iyy[eslabon - 1] = Iyy;
  this->Izz[eslabon - 1] = Izz;
  this->Ixy[eslabon - 1] = Ixy;
  this->Ixz[eslabon - 1] = Ixz;
  this->Iyz[eslabon - 1] = Iyz;
}

// friccion
float SerialLink::get_B(int eslabon) {
  return B[eslabon - 1];
}

void SerialLink::set_B(float valor, int eslabon) {
  B[eslabon - 1] = valor;
}

// constantes torque motor
Eigen::Matrix<float, 1, 2> SerialLink::get_Tc(int eslabon) {
  Eigen::Matrix<float, 1, 2> result;
  result(0) = Tc[eslabon - 1][0];
  result(1) = Tc[eslabon - 1][1];
  return result;
}

void SerialLink::set_Tc(float Tc1, float Tc2, int eslabon) {
  Tc[eslabon - 1][0] = Tc1;
  Tc[eslabon - 1][1] = Tc2;
}

void SerialLink::set_Tc(Eigen::Matrix<float, 1, 2> valor, int eslabon) {
  Tc[eslabon - 1][0] = valor(0);
  Tc[eslabon - 1][1] = valor(1);
}

// gear ratio
float SerialLink::get_G(int eslabon) {
  return G[eslabon - 1];
}

void SerialLink::set_G(float valor, int eslabon) {
  G[eslabon - 1] = valor;
}

// inercia motor
float SerialLink::get_Jm(int eslabon) {
  return Jm[eslabon - 1];
}

void SerialLink::set_Jm(float valor, int eslabon) {
  Jm[eslabon - 1] = valor;
}

// transformada
Eigen::Matrix4f SerialLink::A(int eslabon, float q) {
  Eigen::Matrix4f T;
  float st = 0.0;
  float ct = 0.0;
  float dd = 0.0;

  if (this->get_flip() == true) {
    q = -q + (get_offset(eslabon));
  } else {
    q = q + (get_offset(eslabon));
  }

  if (get_config(eslabon) == 'R') {
    st = sin(q);
    ct = cos(q);
    dd = get_d(eslabon);
  } else if (get_config(eslabon) == 'P') {
    st = sin(get_theta(eslabon));
    ct = cos(get_theta(eslabon));
    dd = q;
  }

  if (get_mdh() == false) {
    // DH estandar
    T << ct, -st * cos(get_alpha(eslabon)), st * sin(get_alpha(eslabon)), get_a(eslabon) * ct,
      st, ct * cos(get_alpha(eslabon)), -ct * sin(get_alpha(eslabon)), get_a(eslabon) * st,
      0, sin(get_alpha(eslabon)), cos(get_alpha(eslabon)), dd,
      0, 0, 0, 1;
  } else {
    // DH modificada
    T << ct, -st, 0, get_a(eslabon),
      st * cos(get_alpha(eslabon)), ct * cos(get_alpha(eslabon)), -sin(get_alpha(eslabon)), -sin(get_alpha(eslabon)) * dd,
      st * sin(get_alpha(eslabon)), ct * sin(get_alpha(eslabon)), cos(get_alpha(eslabon)), cos(get_alpha(eslabon)) * dd,
      0, 0, 0, 1;
  }

  return T;
}


// cinematica directa
Eigen::Matrix4f SerialLink::fkine(float q[NUMERO_DE_ESLABONES]) {
  Eigen::Matrix4f t = this->get_base();
  //si se trabaja en grados
  if (this->get_grados() == true) {
    for (int i = 0; i < NUMERO_DE_ESLABONES; i++) {
      if (this->get_config(i + 1) == 'R') {
        q[i] = radianes_a_grados(q[i]);
      }
    }
  }
  //calcular
  for (int i = 0; i < NUMERO_DE_ESLABONES; i++) {
    t = t * this->A(i + 1, q[i]);
  }


  t = t * this->get_tool();

  return t;
}

// jacobiana
Eigen::Matrix<float, 6, NUMERO_DE_ESLABONES> SerialLink::jacobe(float q[NUMERO_DE_ESLABONES]) {
  Eigen::Matrix<float, 6, NUMERO_DE_ESLABONES> J = Eigen::Matrix<float, 6, NUMERO_DE_ESLABONES>::Zero();

  Eigen::Matrix4f U = this->get_tool();
  Eigen::Matrix4f UT;

  Eigen::Matrix<float, 3, 1> d;
  Eigen::Matrix<float, 3, 1> delta;

  for (int i = NUMERO_DE_ESLABONES; i > 0; i--) {
    if (this->get_mdh() == false) {
      U = this->A(i, q[i - 1]) * U;
      UT = U;
    }

    if (this->get_config(i) == 'R') {  // articulacion r
      d(0) = -UT(0, 0) * UT(1, 3) + UT(1, 0) * UT(0, 3);
      d(1) = -UT(0, 1) * UT(1, 3) + UT(1, 1) * UT(0, 3);
      d(2) = -UT(0, 2) * UT(1, 3) + UT(1, 2) * UT(0, 3);

      delta(0) = UT(2, 0);
      delta(1) = UT(2, 1);
      delta(2) = UT(2, 2);
    } else {  // articulacion p
      d(0) = UT(2, 0);
      d(1) = UT(2, 1);
      d(2) = UT(2, 2);

      delta(0) = 0;
      delta(1) = 0;
      delta(2) = 0;
    }

    for (int fila = 0; fila < 3; fila++) {
      J(fila, i - 1) = d(fila);
    }

    for (int fila = 3; fila < 6; fila++) {
      J(fila, i - 1) = delta(fila - 3);
    }

    if (this->get_mdh() == true) {
      U = this->A(i, q[i - 1]) * U;
      UT = U;
    }
  }

  return J;
}


// cinematica inversa
Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> SerialLink::ikine(Eigen::Matrix4f tr) {
  Eigen::Matrix4f TT = tr;
  Eigen::Matrix4f T = TT;

  //parametros para resolver
  int ilimit = 500;
  int rlimit = 100;
  float tol = 1e-6;
  float lambda = 0.1;
  float lambdamin = 0.0;
  Eigen::Matrix<float, 6, 1> mask;
  mask << 1, 1, 1, 1, 1, 1;

  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> q0;
  q0.setZero();

  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> q = q0;

  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> qnew;
  Eigen::Matrix<float, 6, 6> W = mask.asDiagonal();
  Eigen::Matrix<float, 6, 1> e, enew;
  Eigen::Matrix<float, 6, NUMERO_DE_ESLABONES> J;
  Eigen::Matrix<float, NUMERO_DE_ESLABONES, NUMERO_DE_ESLABONES> JtJ, identidad;
  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> dq;

  identidad.setIdentity();
  bool fallo = false;

  long iteraciones = 0L;
  int rechazado = 0;
  int total = 0;


  if (this->get_grados() == true) {
    this->set_grados(false);
    Serial.println("Operando en radianes...");
  }

  for (int i = 0; i < 4; i++) {

    iteraciones = 0;

    while (true) {
      iteraciones++;
      if (iteraciones > ilimit) {
        fallo = true;
        break;
      }

      e = tr2delta(fkine(q.data()), T);

      if (((W * e).norm()) < tol) {
        break;
      }

      J = jacobe(q.data());
      JtJ = J.transpose() * W * J;

      dq = (JtJ + (lambda + lambdamin) * identidad).inverse() * J.transpose() * W * e;
      qnew = q + dq;

      enew = tr2delta(fkine(qnew.data()), T);

      if (((W * enew).norm()) < ((W * e).norm())) {
        q = qnew;
        e = enew;
        lambda = lambda / 2;
        rechazado = 0;
      } else {
        lambda = lambda * 2;
        rechazado++;
        if (rechazado > rlimit) {
          Serial.print("Iteraciones: ");
          Serial.println(iteraciones);
          fallo = true;  //revisar esto
          break;
        }
        continue;
      }

      //wrap angles
      for (int i = 0; i < NUMERO_DE_ESLABONES; ++i) {
        if (this->get_config(i + 1) == 'R') {
          if (q(i) > PI) {
            q(i) -= 2 * PI;
          } else if (q(i) < -PI) {
            q(i) += 2 * PI;
          }
        }
      }
    }

    total = total + iteraciones;

    if (fallo) {
      Serial.println("La cinematica inversa no converge");
      q.setZero();
    }
  }
  // Serial.print("Iteraciones totales: ");
  // Serial.println(total);
  return q;
}


//-------------Dinamica-----------------------------------------------------------------------------------
//dinamica inversa
Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> SerialLink::rne(float q[NUMERO_DE_ESLABONES], float qd[NUMERO_DE_ESLABONES], float qdd[NUMERO_DE_ESLABONES]) {
  if (this->get_mdh()) {
    // dh modificada   poner mdh
    return this->rne_mdh(q, qd, qdd);
  } else {
    // dh estandar
    return this->rne_dh(q, qd, qdd);
  }
}

//rne para dh estandar
Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> SerialLink::rne_dh(float q[NUMERO_DE_ESLABONES], float qd[NUMERO_DE_ESLABONES], float qdd[NUMERO_DE_ESLABONES]) {
  Eigen::Matrix<float, 3, 1> z0(0, 0, 1);
  Eigen::Matrix<float, 3, 1> grav = this->get_gravity();

  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> tau = Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1>::Zero();

  Eigen::Matrix<float, 3, 1> w = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> wd = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> vd = grav;

  Eigen::Matrix<float, 3, 3> Rt;
  Eigen::Matrix3f R;
  Eigen::Matrix3f Rtransp;

  Eigen::Matrix<float, 6, 1> wbase;

  Eigen::Matrix4f Tj;
  float d = 0;
  float alpha, t;
  Eigen::Matrix<float, 3, 1> pstar;
  Eigen::Matrix<float, 3, 1> r;
  Eigen::Matrix<float, 1, 3> rtransp;
  Eigen::Matrix<float, 3, NUMERO_DE_ESLABONES> pstarm, Fm, Nm;
  Eigen::Matrix<float, 6, 1> fext = Eigen::Matrix<float, 6, 1>::Zero();

  Eigen::Matrix<float, 3, 3> Rm[NUMERO_DE_ESLABONES];

  Eigen::Matrix<float, 3, 1> nn;
  Eigen::Matrix<float, 3, 1> f;

  Eigen::Matrix3f Rb = t2r(this->get_base()).transpose();
  w = Rb * Eigen::Matrix<float, 3, 1>::Zero();
  wd = Rb * Eigen::Matrix<float, 3, 1>::Zero();
  vd = Rb * grav;


  for (int j = 0; j < NUMERO_DE_ESLABONES; j++) {
    Tj = this->A(j + 1, q[j]);

    switch (this->get_config(j + 1)) {
      case 'R':
        d = this->get_d(j + 1);
        break;
      case 'P':
        d = q[j];
        break;
    }
    alpha = this->get_alpha(j + 1);
    pstar << this->get_a(j + 1), d * sin(alpha), d * cos(alpha);

    pstarm.block<3, 1>(0, j) = pstar;

    Rm[j] = t2r(Tj);
  }

  for (int j = 0; j < NUMERO_DE_ESLABONES; j++) {
    Rt = Rm[j].transpose();
    pstar = pstarm.col(j);
    r = this->get_r(j + 1);

    switch (this->get_config(j + 1)) {
      case 'R':
        wd = Rt * (wd + z0 * qdd[j] + w.cross(z0 * qd[j]));
        w = Rt * (w + z0 * qd[j]);
        vd = wd.cross(pstar) + w.cross(w.cross(pstar)) + Rt * vd;
        break;
      case 'P':
        w = Rt * w;
        wd = Rt * wd;
        vd = Rt * (z0 * qdd[j] + vd) + wd.cross(pstar) + (2 * w.cross(Rt * z0 * qd[j])) + w.cross(w.cross(pstar));
        break;
    }

    Eigen::Matrix<float, 3, 1> vhat = wd.cross(r.transpose()) + w.cross(w.cross(r.transpose())) + vd;
    Eigen::Matrix<float, 3, 1> F = this->get_m(j + 1) * vhat;
    Eigen::Matrix<float, 3, 1> N = this->get_I(j + 1) * wd + w.cross(this->get_I(j + 1) * w);

    Fm.block<3, 1>(0, j) = F;
    Nm.block<3, 1>(0, j) = N;
  }

  f = fext.block<3, 1>(0, 0);
  nn = fext.block<3, 1>(3, 0);

  for (int j = NUMERO_DE_ESLABONES - 1; j >= 0; j--) {
    pstar = pstarm.col(j);

    if (j == (NUMERO_DE_ESLABONES - 1)) {
      R = Eigen::Matrix3f::Identity();
    } else {
      R = Rm[j + 1];
    }
    r = this->get_r(j + 1);

    rtransp = r.transpose();
    Rtransp = R.transpose();


    Eigen::Matrix<float, 3, 1> term1 = Rtransp * pstar;
    Eigen::Matrix<float, 3, 1> term2 = term1.cross(f);
    Eigen::Matrix<float, 3, 1> term3 = R * (nn + term2);
    Eigen::Matrix<float, 3, 1> term4 = pstar.cross(Fm.col(j));
    Eigen::Matrix<float, 3, 1> term5 = r.cross(Fm.col(j));
    nn = term3 + term4 + term5 + Nm.col(j);
    f = R * f + Fm.col(j);

    R = Rm[j];

    float G = (this->get_G(j + 1));
    float Jm = this->get_Jm(j + 1);
    switch (this->get_config(j + 1)) {
      case 'R':
        t = nn.transpose() * (R.transpose() * z0) + ((G * G) * Jm * qdd[j]) - (this->friction(j + 1, qd[j]));
        tau(j) = t;
        break;
      case 'P':
        t = (f.transpose() * (R.transpose() * z0)) + ((G * G) * Jm * qdd[j]) - this->friction(j + 1, qd[j]);
        tau(j) = t;
        break;
    }
  }
  // R = Rm[0];
  // nn = R * nn;
  // f = R * f;

  //wbase << f, nn;

  // for (int i = 0; i < 6; i++) {
  //   Serial.print("wbase[");
  //   Serial.print(i);
  //   Serial.print("]: ");
  //   Serial.println(wbase(i));
  // }
  return tau;
}

//rne para dh modificada
Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> SerialLink::rne_mdh(float q[NUMERO_DE_ESLABONES], float qd[NUMERO_DE_ESLABONES], float qdd[NUMERO_DE_ESLABONES]) {
  Eigen::Matrix<float, 3, 1> z0(0, 0, 1);
  Eigen::Matrix<float, 3, 1> grav = this->get_gravity();
  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> tau = Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1>::Zero();

  Eigen::Matrix<float, 3, 1> w_;
  Eigen::Matrix<float, 3, 1> wd_;
  Eigen::Matrix<float, 3, 1> vd_;

  Eigen::Matrix<float, 3, 1> w = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> wd = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> vd = grav;

  Eigen::Matrix<float, 3, 3> Rt;
  Eigen::Matrix3f R;

  Eigen::Matrix4f Tj;
  float d = 0;
  float alpha;
  Eigen::Matrix<float, 3, 1> pstar;
  Eigen::Matrix<float, 3, 1> r;
  Eigen::Matrix<float, 3, NUMERO_DE_ESLABONES> Pm, Fm, Nm;
  Eigen::Matrix<float, 6, 1> fext = Eigen::Matrix<float, 6, 1>::Zero();

  Eigen::Matrix<float, 3, 3> Rm[NUMERO_DE_ESLABONES];

  Eigen::Matrix<float, 3, 1> nn;
  Eigen::Matrix<float, 3, 1> f;

  Eigen::Matrix3f Rb = t2r(this->get_base()).transpose();
  w = Rb * Eigen::Matrix<float, 3, 1>::Zero();
  wd = Rb * Eigen::Matrix<float, 3, 1>::Zero();
  vd = Rb * grav;

  for (int j = 0; j < NUMERO_DE_ESLABONES; j++) {
    Tj = this->A(j + 1, q[j]);

    switch (this->get_config(j + 1)) {
      case 'R':
        d = this->get_d(j + 1);
        break;
      case 'P':
        d = q[j];
        break;
    }

    alpha = this->get_alpha(j + 1);

    Eigen::Matrix<float, 3, 1> pm;
    pm << this->get_a(j + 1), -d * sin(alpha), d * cos(alpha);

    if (j == 0) {
      pm = t2r(this->get_base()) * pm;
      Tj = this->get_base() * Tj;
    }

    Pm.block<3, 1>(0, j) = pm;
    Rm[j] = t2r(Tj);
  }

  for (int j = 0; j < NUMERO_DE_ESLABONES; j++) {
    R = Rm[j].transpose();
    Eigen::Matrix<float, 3, 1> P = Pm.col(j);
    Eigen::Matrix<float, 3, 1> Pc = this->get_r(j + 1);



    switch (this->get_config(j + 1)) {
      case 'R':
        w_ = R * w + z0 * qd[j];
        wd_ = R * wd + (R * w).cross(z0 * qd[j]) + z0 * qdd[j];
        vd_ = R * (wd.cross(P) + w.cross(w.cross(P)) + vd);
        break;
      case 'P':
        w_ = R * w;
        wd_ = R * wd;
        vd_ = R * (wd.cross(P) + w.cross(w.cross(P)) + vd) + 2 * (R * w).cross(z0 * qd[j]) + z0 * qdd[j];
        break;
    }
    w = w_;
    wd = wd_;
    vd = vd_;

    Eigen::Matrix<float, 3, 1> vdC = wd.cross(Pc) + w.cross(w.cross(Pc)) + vd;
    Eigen::Matrix<float, 3, 1> F = this->get_m(j + 1) * vdC;
    Eigen::Matrix<float, 3, 1> N = this->get_I(j + 1) * wd + w.cross(this->get_I(j + 1) * w);

    Fm.col(j) = F;
    Nm.col(j) = N;
  }

  f = fext.block<3, 1>(0, 0);
  nn = fext.block<3, 1>(3, 0);

  for (int j = NUMERO_DE_ESLABONES - 1; j >= 0; j--) {
    Eigen::Matrix<float, 3, 1> P;
    if (j == NUMERO_DE_ESLABONES - 1) {
      R = Eigen::Matrix3f::Identity();
      P.setZero();
    } else {
      R = Rm[j + 1];
      P = Pm.col(j + 1);
    }
    Eigen::Matrix<float, 3, 1> Pc = this->get_r(j + 1);

    Eigen::Matrix<float, 3, 1> f_ = R * f + Fm.col(j);
    Eigen::Matrix<float, 3, 1> nn_ = Nm.col(j) + R * nn + Pc.cross(Fm.col(j)) + P.cross(R * f);

    f = f_;
    nn = nn_;

    switch (this->get_config(j + 1)) {
      case 'R':
        tau(j) = (nn.transpose() * z0) + sq(this->get_G(j + 1)) * this->get_Jm(j + 1) * qdd[j] - this->friction(j + 1, qd[j]);
        break;
      case 'P':
        tau(j) = (f.transpose() * z0) + sq(this->get_G(j + 1)) * this->get_Jm(j + 1) * qdd[j] - this->friction(j + 1, qd[j]);
        break;
    }
  }
  return tau;
}

float SerialLink::friction(int eslabon, float qd) {
  float tau = this->get_B(eslabon) * abs(this->get_G(eslabon)) * qd;
  Eigen::Matrix<float, 1, 2> Tc = this->get_Tc(eslabon);

  if (qd > 0) {
    tau = tau + Tc(0, 0);
  } else if (qd < 0) {
    tau = tau + Tc(0, 1);
  }

  tau = -abs(this->get_G(eslabon)) * tau;

  return tau;
}