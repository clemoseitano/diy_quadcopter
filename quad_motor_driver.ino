 /** 
* Arduino code for SN754410 H-bridge   
* motor driver control.   
* Clement Osei Tano January 2, 2025 
**/ 
struct Motor {
  int forwardPin;
  int reversePin;
  int enablePin;
  int currentSpeed;
};

Motor motor1 = {7, 4, 6, 0};  // Pins for motor 1
Motor motor2 = {2, 5, 3, 0};  // Pins for motor 2
Motor motor3 = {11, 13, 9, 0}; // Pins for motor 3
Motor motor4 = {12, 8, 10, 0}; // Pins for motor 4

void controlMotor(Motor motor, int speed, bool forward) {
  if (forward) {
    digitalWrite(motor.forwardPin, HIGH);
    digitalWrite(motor.reversePin, LOW);
  } else {
    digitalWrite(motor.forwardPin, LOW);
    digitalWrite(motor.reversePin, HIGH);
  }

  // Output speed as PWM value, if enablePin is available
  if (motor.enablePin != 0) {
    analogWrite(motor.enablePin, speed);
  }
}

void setup() {
  Serial.begin(9600);

  // Set digital I/O pins as outputs:
  pinMode(motor1.forwardPin, OUTPUT);
  pinMode(motor1.reversePin, OUTPUT);
  if (motor1.enablePin != 0) pinMode(motor1.enablePin, OUTPUT);

  pinMode(motor2.forwardPin, OUTPUT);
  pinMode(motor2.reversePin, OUTPUT);
  if (motor2.enablePin != 0) pinMode(motor2.enablePin, OUTPUT);

  pinMode(motor3.forwardPin, OUTPUT);
  pinMode(motor3.reversePin, OUTPUT);
  if (motor3.enablePin != 0) pinMode(motor3.enablePin, OUTPUT);

  pinMode(motor4.forwardPin, OUTPUT);
  pinMode(motor4.reversePin, OUTPUT);
  if (motor4.enablePin != 0) pinMode(motor4.enablePin, OUTPUT);
}

void loop() {
  // Motors 1 and 4 pulse forward, motors 2 and 3 pulse reverse
  Serial.println("Motors 1 & 4 forward, Motors 2 & 3 reverse...");
  for (int i = 0; i < 5; i++) {
    controlMotor(motor1, 255, true); // Motor 1 forward
    controlMotor(motor4, 255, true); // Motor 4 forward
    controlMotor(motor2, 255, false); // Motor 2 reverse
    controlMotor(motor3, 255, false); // Motor 3 reverse
    delay(500); // On for 500 ms

    controlMotor(motor1, 0, true); // Stop Motor 1
    controlMotor(motor4, 0, true); // Stop Motor 4
    controlMotor(motor2, 0, false); // Stop Motor 2
    controlMotor(motor3, 0, false); // Stop Motor 3
    delay(500); // Off for 500 ms
  }

  // Motors 1 and 4 pulse reverse, motors 2 and 3 pulse forward
  Serial.println("Motors 1 & 4 reverse, Motors 2 & 3 forward...");
  for (int i = 0; i < 5; i++) {
    controlMotor(motor1, 255, false); // Motor 1 reverse
    controlMotor(motor4, 255, false); // Motor 4 reverse
    controlMotor(motor2, 255, true); // Motor 2 forward
    controlMotor(motor3, 255, true); // Motor 3 forward
    delay(500); // On for 500 ms

    controlMotor(motor1, 0, false); // Stop Motor 1
    controlMotor(motor4, 0, false); // Stop Motor 4
    controlMotor(motor2, 0, true); // Stop Motor 2
    controlMotor(motor3, 0, true); // Stop Motor 3
    delay(500); // Off for 500 ms
  }

  // Add a delay before repeating the loop
  delay(2000);
}