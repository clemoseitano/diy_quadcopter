# DIY Quadcopter Drone

This project documents my journey of building a self-stabilizing quadcopter drone using an Arduino. The initial focus is on motor control, with a basic script to pulse the four motors using SN754410NE motor driver chips.

---

## Current Progress

### Committed File
- **`quad_motor_driver.ino`**: A script that controls the quadcopter's motors in a simple pulsing pattern.

### Code Functionality
- **Motor Struct**: Defines motor control pins and speed.
- **Motor Control Logic**: Sets motor speed and direction (forward/reverse) using the `controlMotor` function.
- **Pulsing Sequence**:
  - Motors 1 & 4 pulse forward, and Motors 2 & 3 pulse in reverse.
  - Then the sequence reverses.
- **Serial Output**: Prints motor states for debugging.

---

## Hardware Setup
- **Microcontroller**: Arduino Nano.
- **Motor Driver Chips**: 2 x SN754410NE.
- **Motors**: 4 DC motors.

### Pin Configuration
| Motor  | Forward Pin | Reverse Pin | Enable Pin |
|--------|-------------|-------------|------------|
| Motor 1 | 7           | 4           | 6          |
| Motor 2 | 2           | 5           | 3          |
| Motor 3 | 11          | 13          | 9          |
| Motor 4 | 12          | 8           | 10         |

---

## Running the Code
1. Clone the repository:
   ```bash
   git clone https://github.com/clemoseitano/diy_quadcopter.git
   ```
2. Open `quad_motor_driver.ino` in the Arduino IDE.
3. Upload the code to your Arduino.
4. Open the Serial Monitor (baud rate: 9600) to observe motor activity.

---

## Next Steps
[ x ] Setup motor driver
[ x ] Integrate gyroscope and accelerometer for flight stabilization.
- Implement a PID control algorithm.
- Design quadcopter frame(s).
- Add wireless control via RF module.

---

## Component Datasheets
1. [Motor Driver Chip](https://www.ti.com/lit/ds/symlink/sn754410.pdf)
2. Inertial Measurement Unit
  - [Register Map and Description](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
  - [Product Specification](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)