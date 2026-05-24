#include <Arduino.h>
#include "robot_lib.h"

//----Generación de trayectorias------------------------------------------------
Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> trayectoria;
unsigned long t1, t2;

int pasos = 10;  //El tiempo varía de 0 a 1 segundos en este número de pasos, cambiar al gusto

//Polinomio de 5o orden con aceleración inicial y final = 0
Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> tray(float qi[NUMERO_DE_ACTUADORES], float qf[NUMERO_DE_ACTUADORES], int pasos, float qdi, float qdf) {
  float A, B, C, E, F;
  float t = 0;
  Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> q(NUMERO_DE_ACTUADORES, pasos);

  for (int i = 0; i < NUMERO_DE_ACTUADORES; i++) {
    A = 6 * (qf[i] - qi[i]) - 3 * (qdf + qdi);
    B = -15 * (qf[i] - qi[i]) + 8 * qdi + 7 * qdf;
    C = 10 * (qf[i] - qi[i]) - 6 * qdi - 4 * qdf;
    // D = 0 porque no hay aceleración inicial ni final
    E = qdi;
    F = qi[i];

    for (int j = 0; j < pasos; j++) {
      t = float(j) / (pasos - 1);  // Normalización del tiempo
      q(i, j) = F + E * t + C * pow(t, 3) + B * pow(t, 4) + A * pow(t, 5);

      // imprimir resultados
      Serial.print("q[");
      Serial.print(i);
      Serial.print("][");
      Serial.print(j);
      Serial.print("] = ");
      Serial.println(q(i, j));
    }
  }

  return q;
}

//Polinomio de 5o orden con aceleración inicial y final definida por el usuario
Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> tray_acel(float qi[NUMERO_DE_ACTUADORES], float qf[NUMERO_DE_ACTUADORES], int pasos, float qdi, float qdf, float ai, float af) {
  float A, B, C, D, E, F;
  float t = 0;
  float T = pasos - 1;
  Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> q(NUMERO_DE_ACTUADORES, pasos);

  for (int i = 0; i < NUMERO_DE_ACTUADORES; i++) {

    A = (6 * (qf[i] - qi[i])) - (3 * (qdf + qdi)) + (0.5 * af);
    B = -(15 * (qf[i] - qi[i])) + (8 * qdi) + (7 * qdf) + (0.5 * ai) - af;
    C = (10 * (qf[i] - qi[i])) - (6 * qdi) - (4 * qdf) - ai + (0.5 * af);
    D = 0.5 * ai;
    E = qdi;
    F = qi[i];

    for (int j = 0; j < pasos; j++) {
      t = float(j) / T;  // normalización del tiempo
      q(i, j) = A * pow(t, 5) + B * pow(t, 4) + C * pow(t, 3) + D * pow(t, 2) + E * t + F;

      // imprimir resultados
      Serial.print("q[");
      Serial.print(i);
      Serial.print("][");
      Serial.print(j);
      Serial.print("] = ");
      Serial.println(q(i, j));
    }
  }

  return q;
}

// //----Definición de servos--------------------------------------------------------------------
//PWMServo servo1, servo2, servo3, servo4, servo5;

// //----Definición de encoders------------------------------------------------------------------
// // Si se usa placa Teensy no usar el pin 13: es el del LED de la placa
// Encoder enc1(5, 6);
// Encoder enc2(7, 8);
// Encoder enc3(9, 10);
// Encoder enc4(11, 12);
// Encoder enc5(14, 15);

//----Ajustar actuadores---------------------------------------------------------------------------------
void ajustar_actuadores(Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> trayectoria) {
  for (int n = 0; n < pasos; n++) {
    unsigned long t1 = millis();
    while (millis() < t1 + 20) {
      // Esperar 20 ms
    }

    // //----Servos (descomentar si se usan)-----------------
    // servo1.write(radianes_a_grados(trayectoria(0, n)));
    // Serial.println("Servo 1 ajustado");
    // servo2.write(radianes_a_grados(trayectoria(1, n)));
    // Serial.println("Servo 2 ajustado");
    // servo3.write(radianes_a_grados(trayectoria(2, n)));
    // Serial.println("Servo 3 ajustado");
    // servo4.write(radianes_a_grados(trayectoria(3, n)));
    // Serial.println("Servo 4 ajustado");
    // servo5.write(radianes_a_grados(trayectoria(4, n)));
    // Serial.println("Servo 5 ajustado");

    // //----Encoders (descomentar si se usan)-------------
    // enc1.write(radianes_a_grados(trayectoria(0, n)));
    // Serial.println("Encoder 1 ajustado");
    // enc2.write(radianes_a_grados(trayectoria(1, n)));
    // Serial.println("Encoder 2 ajustado");
    // enc3.write(radianes_a_grados(trayectoria(2, n)));
    // Serial.println("Encoder 3 ajustado");
    // enc4.write(radianes_a_grados(trayectoria(3, n)));
    // Serial.println("Encoder 4 ajustado");
    // enc5.write(radianes_a_grados(trayectoria(4, n)));
    // Serial.println("Encoder 5 ajustado");
  }
}


void setup() {
  Serial.begin(115200);   // Baudios para comunicación con el puerto serie

  //-------Servos-------------------------------------------------------------
  // Si se usa placa Teensy no usar el pin 13: es el del LED de la placa
  // servo1.attach(2);
  // servo2.attach(4);
  // servo3.attach(6);
  // servo4.attach(8);
  // servo5.attach(10);

  t1 = millis();          // Necesario para la generación de trayectorias
  
  while (!Serial) {}      // Asegurar que la comunicación serie está activada
  Serial.println("Setup iniciado")

  // //----Ejemplos de uso-------------------------------------------------------------
  // SerialLink robot = crearp560();      // Crear robot

  // float q0[NUMERO_DE_ESLABONES] = { 0.5, -0.5, 0.5, 0.5, -0.5, 0.5 };   // Crear vector
  // Eigen::Matrix4f T;                      // Crear matriz
  // T << 0.1897, -0.5831, 0.7900, -0.2203,  
  //   0.5831, -0.5804, -0.5684, -0.3303,
  //   0.7900, 0.5684, 0.2298, 0.2126,
  //   0, 0, 0, 1;                           // Dar valores a la matriz


  // Serial.println("Cinemática directa:");
  // printMatriz(robot.fkine(q0));           // Cinemática directa con argumento q0

  // Serial.println("Cinemática inversa:");
  // printMatriz(robot.ikine(T));            // Cinemática inversa con argumento T

  // float qd[NUMERO_DE_ESLABONES] = { 1, 1, 1, 1, 1, 1 };
  // float qdd[NUMERO_DE_ESLABONES] = { 0, 0, 0, 0, 0, 0 };
  // Serial.println("Dinámica Newton-Euler:");
  // printMatriz(robot.rne(q0, qd, qdd));


  // //---------Parámetros para generación de trayectorias-----------------------------------------------------------------------
  // //----Polinomio de 5o orden------------------------------------------------------------------------------------
  // float qi[NUMERO_DE_ACTUADORES] = { 0, 0, 0, 0, 0 };  // Posiciones iniciales
  // float qf[NUMERO_DE_ACTUADORES] = { 1, 1, 1, 1, 1 };  // Posiciones finales
  // float qdi = 0;                                       // Velocidad inicial
  // float qdf = 0;                                       // Velocidad final

  // Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> trayectoria = tray(qi, qf, pasos, qdi, qdf);
  // ajustar_servos(trayectoria);


  // //----Polinomio de 5o orden con aceleraciones definidas por el usuario------------------------------------------
  // float qi[NUMERO_DE_ACTUADORES] = { 0, 0, 0, 0, 0 };  // Posiciones iniciales
  // float qf[NUMERO_DE_ACTUADORES] = { 1, 1, 1, 1, 1 };  // Posiciones finales
  // float qdi = 0;                                       // Velocidad inicial
  // float qdf = 0;                                       // Velocidad final
  // float ai = 0;                                        // Aceleración inicial
  // float af = 0;                                        // Aceleración final

  // Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> trayectoria = tray_acel(qi, qf, pasos, qdi, qdf, ai, af);
  // ajustar_servos(trayectoria);

  Serial.println("Setup completado");
}

void loop() {
  // //----Lectura de encoders-----------------------------------------------------------------------------------------
  // // Definir cada encoder antes de void setup()
  // Serial.println(enc1.read());   // Cambiar enc1 por nombre dado al encoder


  // //----Potenciometro------------------------------------------------------------------------------------------------
  // // Poner dentro de analogRead el pin de salida del potenciometro que esta conectado a la placa
  // // vol = lectura del pin / 1023.0 * Vin -> sustituir Vin por el voltaje con el que se alimenta el potenciómetro
  // float vol = analogRead(A10)/ 1023.0 * 3.3;
  // bool porcentaje = true;     // true si se desea imprimir el porcentaje de la resistencia total del potenciómetro,
  //                             // false si se desea imprimir el voltaje de salida del potenciometro
  // if (porcentaje == true) {
  //   int valor = round((vol / 3.3) * 100);
  //   Serial.print(valor);
  //   Serial.println("%");
  // } else {
  //   Serial.print(vol, 2); // Imprimir voltaje con 2 decimales
  //   Serial.println("V");
  // }


  // //----Retardo entre cada impresión en milisegundos-----------------------------------------------------------------
  // delay(200);
}
