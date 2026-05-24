#ifndef SERIALLINK_H
#define SERIALLINK_H

#include "matespac.h"
#include "complementos.h"
#include "link.h"
#include <ArduinoEigen.h>

class SerialLink {
private:
  float theta[NUMERO_DE_ESLABONES];
  float d[NUMERO_DE_ESLABONES];
  float a[NUMERO_DE_ESLABONES];
  float alpha[NUMERO_DE_ESLABONES];
  char config[NUMERO_DE_ESLABONES];

  Eigen::Matrix<float, 3, 1> gravity;
  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 2> qlim;
  float offset[NUMERO_DE_ESLABONES];

  String name, manuf, comment;

  bool grados, flip, mdh;
  Eigen::Matrix4f base, tool;

  float G[NUMERO_DE_ESLABONES];
  float m[NUMERO_DE_ESLABONES];
  float r[NUMERO_DE_ESLABONES][3];
  float Ixx[NUMERO_DE_ESLABONES], Iyy[NUMERO_DE_ESLABONES], Izz[NUMERO_DE_ESLABONES];
  float Ixy[NUMERO_DE_ESLABONES], Ixz[NUMERO_DE_ESLABONES], Iyz[NUMERO_DE_ESLABONES];
  float B[NUMERO_DE_ESLABONES];
  float Tc[2][NUMERO_DE_ESLABONES];
  float Jm[NUMERO_DE_ESLABONES];

public:
  SerialLink();
  SerialLink(Link link);

  void add_link(Link link, int numero_de_enlace);

  float get_theta(int eslabon);
  void set_theta(float valor, int numero_de_enlace);

  float get_d(int eslabon);
  void set_d(float valor, int numero_de_articulacion);

  float get_a(int eslabon);
  void set_a(float valor, int numero_de_articulacion);

  float get_m(int eslabon);
  void set_m(float valor, int eslabon);

  Eigen::Matrix<float, 1, 3> get_r(int eslabon);
  void set_r(float valor1, float valor2, float valor3, int eslabon);
  void set_r(Eigen::Matrix<float, 1, 3> valor, int eslabon);

  Eigen::Matrix<float, 3, 3> get_I(int eslabon);
  void set_I(float Ixx, float Iyy, float Izz, float Ixy, float Ixz, float Iyz, int eslabon);
  //| Ixx  Ixy  Ixz |
  //| Ixy  Iyy  Iyz |
  //| Ixz  Iyz  Izz |

  float get_B(int eslabon);
  void set_B(float valor, int eslabon);

  Eigen::Matrix<float, 1, 2> get_Tc(int eslabon);
  void set_Tc(float Tc1, float Tc2, int eslabon);
  void set_Tc(Eigen::Matrix<float, 1, 2> valor, int eslabon);

  float get_G(int eslabon);
  void set_G(float valor, int eslabon);

  float get_Jm(int eslabon);
  void set_Jm(float valor, int eslabon);

  float get_alpha(int eslabon);
  void set_alpha(float valor, int numero_de_articulacion);

  Eigen::Matrix<float, 3, 1> get_gravity();
  void set_gravity(float gx, float gy, float gz);

  bool get_grados();
  void set_grados(bool valor);

  Eigen::Matrix4f get_base();
  void set_base(float a11, float a12, float a13, float a14,
                float a21, float a22, float a23, float a24,
                float a31, float a32, float a33, float a34,
                float a41, float a42, float a43, float a44);

  Eigen::Matrix4f get_tool();
  void set_tool(float a11, float a12, float a13, float a14,
                float a21, float a22, float a23, float a24,
                float a31, float a32, float a33, float a34,
                float a41, float a42, float a43, float a44);

  bool get_flip();
  void set_flip(bool valor);

  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 2> get_qlim();
  Eigen::Matrix<float, 2, 1> get_qlim(int numero_de_eslabon);
  void set_qlim(Eigen::Matrix<float, 2, 1> limites, int numero_de_articulacion);

  float get_offset(int eslabon);
  void set_offset(float valor, int numero_de_articulacion);

  String get_name();
  void set_name(String nombre);

  String get_manuf();
  void set_manuf(String nombre);

  String get_comment();
  void set_comment(String nombre);

  char get_config(int numero_de_articulacion);
  String get_config();
  void set_config(char valor, int numero_de_articulacion);
  String print_config();

  bool get_mdh();
  void set_mdh(bool valor);

  //------Cinemática------------------------------------------------------------------

  Eigen::Matrix4f A(int eslabon, float q);
  Eigen::Matrix4f identityMatrix();
  Eigen::Matrix4f fkine(float q[NUMERO_DE_ESLABONES]);
  Eigen::Matrix<float, 6, NUMERO_DE_ESLABONES> jacobe(float q[NUMERO_DE_ESLABONES]);
  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> ikine(Eigen::Matrix4f tr);

  //------Dinámica-------------------------------------------------------------------------------

  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> rne(float q[NUMERO_DE_ESLABONES], float qd[NUMERO_DE_ESLABONES], float qdd[NUMERO_DE_ESLABONES]);
  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> rne_dh(float q[NUMERO_DE_ESLABONES], float qd[NUMERO_DE_ESLABONES], float qdd[NUMERO_DE_ESLABONES]);
  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> rne_mdh(float q[NUMERO_DE_ESLABONES], float qd[NUMERO_DE_ESLABONES], float qdd[NUMERO_DE_ESLABONES]);
  float friction(int eslabon, float qd);
};

#endif  // SERIALLINK_H
