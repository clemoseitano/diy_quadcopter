 /** 
* Arduino code for quad-motor control using SN754410 H-bridge   
* motor driver and MPU9250 compatible intertial measurement unit.
* I use both MPU6050(without magnetometer/compass) and MPU9250 in my set ups,
* though not in the same circuit. 
* Clement Osei Tano January 2, 2025 
**/ 
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <MPU9250_asukiaaa.h>

// Register addresses for the temperature data
#define TEMP_OUT_H 0x41
// #define TEMP_OUT_L 0x42; we won't use this since we can read 2 bytes
#define MPU9250_ADDRESS 0x68

// Define a motor struct so the pins are more manageable
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
float aX, aY, aZ, aSqrt, temperature, gyX,gyY,gyZ;

MPU9250_asukiaaa imuSensor;

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

  imuSensor.beginAccel();
  imuSensor.beginGyro();
  imuSensor.beginMag(); // Has no effect if sensor doesn't have the compass
  // TODO: Calibrate IMU; position the setup on a flat surface and get the values when it is at rest.
  // Use the calibrated values as your offset
}

void readIMU(){
  Serial.println("IMU Update");
    aX = imuSensor.accelX();
    aY = imuSensor.accelY();
    aZ = imuSensor.accelZ();
    gyX = imuSensor.gyroX();
    gyY = imuSensor.gyroY();
    gyZ = imuSensor.gyroZ();
    aSqrt = imuSensor.accelSqrt();

    // Display data on OLED
    Serial.println("Accel:");
    Serial.print(F("X: ")); 
    Serial.println(aX);
    Serial.print(F("Y: ")); 
    Serial.println(aY);
    Serial.print(F("Z: ")); 
    Serial.println(aZ);

    Serial.println("Gyro:");
    Serial.print(F("X: ")); 
    Serial.println(gyX);
    Serial.print(F("Y: ")); 
    Serial.println(gyY);
    Serial.print(F("Z: ")); 
    Serial.println(gyZ);
    
     // Read the raw temperature data
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(TEMP_OUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDRESS, 2, true); // Requesting 2 bytes
    int16_t tempRaw = (Wire.read() << 8) | Wire.read(); // Combine two bytes

    // Convert the raw data to temperature in degrees Celsius
    // float temperature = ((float)tempRaw) / 333.87 + 21.0; for MPU6250
    float temperature = ((float)tempRaw) / 340 + 35; // for MPU6050
    Serial.println("Temperature: ");
    Serial.print(temperature);
    Serial.println("C");
}

void loop() {
  // Read accelerometer data
  int result = imuSensor.accelUpdate();
  result = result+ imuSensor.gyroUpdate();
  Serial.print("Result: ");
  Serial.println(result);

  
  if (result == 0) {
    readIMU();
  }
  // Motors 1 and 4 pulse forward, motors 2 and 3 pulse reverse
  Serial.println("Motors 1 & 4 forward, Motors 2 & 3 reverse...");
  for (int i = 0; i < 5; i++) {
    controlMotor(motor1, 4, true); // Motor 1 forward
    controlMotor(motor4, 4, true); // Motor 4 forward
    controlMotor(motor2, 4, false); // Motor 2 reverse
    controlMotor(motor3, 4, false); // Motor 3 reverse
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
    controlMotor(motor1, 4, false); // Motor 1 reverse
    controlMotor(motor4, 4, false); // Motor 4 reverse
    controlMotor(motor2, 4, true); // Motor 2 forward
    controlMotor(motor3, 4, true); // Motor 3 forward
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
