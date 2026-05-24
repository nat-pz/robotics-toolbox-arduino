#include "matespac.h"

float norm(float v) {
  return abs(v);
}

Eigen::Matrix<float, 6, 1> tr2delta(Eigen::Matrix4f T0, Eigen::Matrix4f T1) {
  Eigen::Matrix4f TD = T0.inverse() * T1;
  Eigen::Matrix<float, 6, 1> delta;

  // traslacion
  delta.block<3, 1>(0, 0) = TD.block<3, 1>(0, 3);

  // rotacion
  Eigen::Matrix3f R = TD.block<3, 3>(0, 0);
  Eigen::Matrix3f dR = R - Eigen::Matrix3f::Identity();
  delta(3) = dR(2, 1);
  delta(4) = dR(0, 2);
  delta(5) = dR(1, 0);

  return delta;
}


Eigen::Matrix<float, 6, 1> tr2delta(Eigen::Matrix4f argumento) {
  Eigen::Matrix4f A = argumento;
  Eigen::Matrix4f T1 = Eigen::Matrix4f::Zero();
  Eigen::Matrix4f T0 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f TD;
  Eigen::Matrix<float, 6, 1> delta;
  Eigen::Matrix3f identidad = Eigen::Matrix3f::Identity();

  T1 = A;
  Eigen::Matrix4f T0_inv = T0.inverse();
  TD = T0_inv * T1;

  Eigen::Vector3f delta_transl = transl(TD);
  Eigen::Matrix3f delta_rot_diff = t2r(TD) - identidad;
  Eigen::Vector3f delta_rot = vex(delta_rot_diff);
  for (int i = 0; i < 3; ++i) {
    delta(i) = delta_transl(i);
    delta(i + 3) = delta_rot(i);
  }
  return delta;
}

Eigen::Vector3f transl(Eigen::Matrix4f T) {
  Eigen::Vector3f vector_translation;
  vector_translation(0) = T(0, 3);
  vector_translation(1) = T(1, 3);
  vector_translation(2) = T(2, 3);
  return vector_translation;
}

Eigen::Vector3f vex(Eigen::Matrix3f S) {
  Eigen::Vector3f v;
  v(0) = 0.5 * (S(2, 1) - S(1, 2));
  v(1) = 0.5 * (S(0, 2) - S(2, 0));
  v(2) = 0.5 * (S(1, 0) - S(0, 1));
  return v;
}

float vex(Eigen::Matrix2f S) {
  float v;
  v = 0.5 * (S(1, 0) - S(0, 1));
  return v;
}

Eigen::Matrix3f t2r(Eigen::Matrix4f T) {
  Eigen::Matrix3f R;
  R = T.block<3, 3>(0, 0);
  return R;
}

Eigen::Matrix3f trnorm(Eigen::Matrix4f T) {
  Eigen::Matrix3f resultado;
  Eigen::Vector3f o, a, n;

  o << T(0, 1), T(1, 1), T(2, 1);
  a << T(0, 2), T(1, 2), T(2, 2);

  // calcular vector
  n = o.cross(a);

  // normalizar vector
  n.normalize();

  // crear matriz de rotación con el vector normalizado
  resultado.col(0) = n;
  resultado.col(1) = o;
  resultado.col(2) = a;

  return resultado;
}
