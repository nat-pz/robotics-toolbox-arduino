# robotics-toolbox-arduino
A library for multi-axis robotic arm control on Arduino and compatible microcontroller boards. Inspired by the Robotics Toolbox for MATLAB created by Peter Corke.

Developed as a Bachelor's thesis project at the Universitat Politècnica de València (UPV).

This project was last worked on in 2024. Feel free to fork and improve it!

---

## Features

- **Forward kinematics**: compute end-effector pose from joint angles using Denavit-Hartenberg (DH and MDH) parameters
- **Inverse kinematics**: compute joint angles from a desired end-effector transformation matrix
- **Jacobian**: compute the body Jacobian for a serial-link manipulator
- **Dynamics**: recursive Newton-Euler algorithm (RNE) for torque computation, including gravity, Coriolis and inertia effects
- **Trajectory generation**: 5th-order polynomial trajectories with configurable initial/final velocities
- **Actuator control**: servo and stepper motor support with encoder feedback
- **Sensor reading**: potentiometer input utilities

---

## Requirements

- Arduino IDE or compatible environment
- [Eigen library for Arduino](https://github.com/bolderflight/eigen)
- Compatible boards: Arduino, Teensy 4.1, and other AVR/ARM boards

---

## Installation

1. Download or clone this repository
2. Copy the library folder into your Arduino `libraries` directory
3. Install the Eigen library for Arduino
4. Include the library in your sketch:

```cpp
#include "robot_lib.h"
```

---

## Usage examples

### Create a PUMA 560 robot

```cpp
#include "robot_lib.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  SerialLink robot = crearp560();
  Serial.println("PUMA 560 robot created");
}

void loop() {}
```

### Forward kinematics

```cpp
#include "robot_lib.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  SerialLink robot = crearp560();
  float q[NUMERO_DE_ESLABONES] = { 0.5, -0.5, 0.5, 0.5, -0.5, 0.5 };

  Eigen::Matrix4f T = robot.fkine(q);
  printMatriz(T);
}

void loop() {}
```

### Inverse kinematics

```cpp
#include "robot_lib.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  SerialLink robot = crearp560();

  Eigen::Matrix4f T;
  T << 0.1897, -0.5831,  0.7900, -0.2203,
       0.5831, -0.5804, -0.5684, -0.3303,
       0.7900,  0.5684,  0.2298,  0.2126,
            0,       0,       0,       1;

  Eigen::Matrix<float, NUMERO_DE_ESLABONES, 1> q = robot.ikine(T);
  printMatriz(q);
}

void loop() {}
```

### Trajectory generation (5th-order polynomial)

```cpp
#include "robot_lib.h"

Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> trayectoria;
unsigned long t1;
int pasos = 5;

Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> tray(
  float qi[NUMERO_DE_ACTUADORES], float qf[NUMERO_DE_ACTUADORES],
  int pasos, float qdi, float qdf) {

  float A, B, C, E, F, t = 0;
  Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> q(NUMERO_DE_ACTUADORES, pasos);

  for (int i = 0; i < NUMERO_DE_ACTUADORES; i++) {
    A = 6 * (qf[i] - qi[i]) - 3 * (qdf + qdi);
    B = -15 * (qf[i] - qi[i]) + 8 * qdi + 7 * qdf;
    C = 10 * (qf[i] - qi[i]) - 6 * qdi - 4 * qdf;
    E = qdi;
    F = qi[i];
    for (int j = 0; j < pasos; j++) {
      t = float(j) / (pasos - 1);
      q(i, j) = F + E * t + C * pow(t, 3) + B * pow(t, 4) + A * pow(t, 5);
    }
  }
  return q;
}

void setup() {
  Serial.begin(115200);
  t1 = millis();
  while (!Serial) {}

  float qi[NUMERO_DE_ACTUADORES] = { 0, 0, 0, 0, 0 };
  float qf[NUMERO_DE_ACTUADORES] = { 1, 1, 1, 1, 1 };
  float qdi = 0, qdf = 0;

  Eigen::Matrix<float, NUMERO_DE_ACTUADORES, Eigen::Dynamic> trayectoria = tray(qi, qf, pasos, qdi, qdf);
  printMatriz(trayectoria);
}

void loop() {}
```

### Potentiometer reading

```cpp
#include "robot_lib.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
}

void loop() {
  leer_potenciometro(A10);
  delay(100);
}
```

There are more usage examples available in the [robotics-toolbox.ino](https://github.com/nat-pz/robotics-toolbox-arduino/blob/main/robotics-toolbox.ino) file.

---

## API Reference

### `Link` class

Represents a single joint/link with DH parameters:

| Method | Description |
|--------|-------------|
| `A(float q)` | Transformation matrix for this link |
| `friction(float qd)` | Joint friction torque |
| `isrevolute()` / `isprismatic()` | Joint type check |
| Getters/setters for `theta`, `d`, `alpha`, `a`, `m`, `qlim`, `offset`, `flip`, `G`, `Jm`, `B`, `Tc`, `I`, `r` | DH and dynamic parameters |

### `SerialLink` class

Represents a serial-link manipulator composed of `Link` objects:

| Method | Description |
|--------|-------------|
| `fkine(q[])` | Forward kinematics |
| `ikine(T)` | Inverse kinematics |
| `jacobe(q[])` | Body Jacobian |
| `rne(q[], qd[], qdd[])` | Recursive Newton-Euler (DH) |
| `rne_mdh(q[], qd[], qdd[])` | Recursive Newton-Euler (MDH) |
| `accel(q[], qd[], torque[])` | Joint accelerations |
| `gravload(q[])` | Gravity torque |
| `inertia(q[])` | Inertia matrix |
| `coriolis(q[], qd[])` | Coriolis matrix |

---

## Troubleshooting

**Inverse kinematics does not converge**
- Check that DH parameters and joint configurations are correct
- Verify that the target pose is within the robot's workspace and joint limits

**Eigen compilation errors**
- Make sure the Eigen library is correctly installed in your Arduino libraries folder

**Serial communication issues**
- Default baud rate is set for Teensy 4.1 (`115200`). For other boards, adjust `Serial.begin()` accordingly

## Documentation

Full project documentation (Spanish): [Riunet UPV](https://riunet.upv.es/handle/10251/214828)
