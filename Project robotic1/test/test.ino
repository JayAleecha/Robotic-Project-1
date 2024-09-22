#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// TCA9548A I2C Multiplexer address
#define TCA9548A_ADDRESS 0x70

// PCA9685 PWM Driver address
#define PCA9685_ADDRESS 0x40

// Servo min and max pulse lengths (out of 4096)
#define SERVOMIN  125
#define SERVOMAX  575

// Create PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

void TCA9548A_SelectBus(uint8_t bus) {
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << bus); // Select the desired I2C bus
  Wire.endTransmission();
  Serial.print("Selected I2C bus: ");
  Serial.println(bus);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Select the correct bus on the TCA9548A for PCA9685
  TCA9548A_SelectBus(1);  // Bus 1 (adjust if connected to a different bus)

  // Initialize the PCA9685
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz

  Serial.println("PCA9685 initialized");
}

void loop() {
  // // Move servo 1 (connected to PWM 0) from min to max position
  // for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
  //   pwm.setPWM(0, 0, pulselen); // Servo 1 on channel 0
  //   delay(1);
  // }
  
  // // Move servo 2 (connected to PWM 1) from min to max position
  // for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
  //   pwm.setPWM(1, 0, pulselen); // Servo 2 on channel 1
  //   delay(1);
  // }

  // // Move servo 1 back from max to min position
  // for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
  //   pwm.setPWM(0, 0, pulselen); // Servo 1 on channel 0
  //   delay(1);
  // }

  // // Move servo 2 back from max to min position
  // for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
  //   pwm.setPWM(1, 0, pulselen); // Servo 2 on channel 1
  //   delay(1);
  }
  // pwm.setPWM(0, 0, SERVOMIN + ((SERVOMIN - SERVOMAX) / 2)); //>>
  // delay(1000);
    pwm.setPWM(0, 0, 350);
    delay(1000);
  // pwm.setPWM(2, 0, SERVOMIN + ((SERVOMAX - SERVOMIN) / 2)); //<<
  // delay(1000);
    pwm.setPWM(0, 0, 350);
    delay(1000);
  // pwm.setPWM(4, 0, SERVOMIN + ((SERVOMAX - SERVOMIN) / 2)); //<<
  // delay(1000);
  // pwm.setPWM(2, 0, SERVOMAX);
  // delay(500);

}
// #include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>

// // Create the PWM driver object
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// #define SERVOMIN  150 // Minimum pulse length count (adjust to your servo)
// #define SERVOMAX  600 // Maximum pulse length count (adjust to your servo)

// // Servo channels (0-15) on the PCA9685
// #define SERVO1_CHANNEL 0
// #define SERVO2_CHANNEL 1

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Initializing PCA9685...");

//   // Initialize the PWM driver
//   pwm.begin();
//   pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz

//   // Wait for the driver to initialize
//   delay(10);
// }

// void loop() {
//   // Example: Sweep servo 1 from 0 to 180 degrees
//   for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
//     pwm.setPWM(SERVO1_CHANNEL, 0, pulselen);
//     delay(1);  // Wait for the servo to reach the position
//   }
//   for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
//     pwm.setPWM(SERVO1_CHANNEL, 0, pulselen);
//     delay(1);  // Wait for the servo to reach the position
//   }

//   // Example: Move servo 2 to 90 degrees
//   uint16_t pulse90 = (SERVOMIN + SERVOMAX) / 2;
//   pwm.setPWM(SERVO2_CHANNEL, 0, pulse90);
//   delay(100);  // Wait for 1 second
// }
// #include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>

// // Create the PWM driver object
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// // Servo settings for 50Hz (20ms period)
// #define SERVOMIN  122  // Minimum pulse length count (1ms, corresponds to 0 degrees)
// #define SERVOMAX  615  // Maximum pulse length count (2ms, corresponds to 180 degrees)

// // Servo channels on the PCA9685
// #define SERVO1_CHANNEL 0
// #define SERVO2_CHANNEL 1
// // #define SERVO2_CHANNEL 1

// void setup() {
//   Serial.begin(9600);
//   Serial.println("Initializing PCA9685...");

//   // Initialize the PWM driver
//   pwm.begin();
//   pwm.setPWMFreq(50);

//   delay(10);
// }

// void loop() {
//   // Move both servos from 0 to 90 degrees
//   uint16_t pulse90 = SERVOMIN + ((SERVOMAX - SERVOMIN) * 90 / 180);

//   pwm.setPWM(SERVO1_CHANNEL, 0, pulse90);  // Move Servo 1 to 90 degrees
//   pwm.setPWM(SERVO2_CHANNEL, 0, pulse90);  // Move Servo 2 to 90 degrees
//   delay(1000);

//   // pwm.setPWM(SERVO1_CHANNEL, 0, SERVOMIN);  // Move Servo 1 to 0 degrees
//   // pwm.setPWM(SERVO2_CHANNEL, 0, SERVOMIN);  // Move Servo 2 to 0 degrees
//   // delay(1000);
// }
// #include <Wire.h>

// #define TCA9548A_ADDRESS 0x70  // I2C address for the multiplexer

// void TCA9548A_SelectBus(uint8_t bus) {
//   Wire.beginTransmission(TCA9548A_ADDRESS);
//   Wire.write(1 << bus);  // Select the desired I2C bus
//   Wire.endTransmission();
// }

// void scanBus(uint8_t bus) {
//   TCA9548A_SelectBus(bus);
//   Serial.print("Scanning I2C bus: ");
//   Serial.println(bus);

//   for (byte address = 1; address < 127; address++) {
//     Wire.beginTransmission(address);
//     byte error = Wire.endTransmission();

//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16) Serial.print("0");
//       Serial.println(address, HEX);
//     }
//   }
//   Serial.println();
// }

// void setup() {
//   Serial.begin(9600);
//   Wire.begin();

//   for (uint8_t bus = 0; bus < 8; bus++) {
//     scanBus(bus);
//     delay(1000);  // Short delay to avoid flooding
//   }
// }

// void loop() {
//   // Nothing in loop
// }
// #include <Wire.h>
// #include <AS5600.h>
// #include <Adafruit_PWMServoDriver.h>

// int angle_in = 0;

// #define TCA9548A_ADDRESS 0x70
// #define AS5600_ADDRESS 0x36

// AMS_5600 ams5600;  // Make sure this class name matches your library

// void TCA9548A_SelectBus(uint8_t bus) {
//   Wire.beginTransmission(TCA9548A_ADDRESS);
//   Wire.write(1 << bus);  // Select the desired I2C bus
//   Wire.endTransmission();
// }

// void setup() {
//   Serial.begin(9600);
//   Wire.begin();

//   // Initialize the first AS5600 on bus 2
//   TCA9548A_SelectBus(2);
//   TCA9548A_SelectBus(3);
// }

// void Angle(int i) {
//   TCA9548A_SelectBus(i);
//   int in;
//   in = map(ams5600.getRawAngle(),0,4095,0,360);
//   if (angle_in != in) {
//     angle_in = in;
//     Serial.print(angle_in);
//     Serial.print(" ");
//   }  
// }

// void loop() {
//   Angle(2);
//   Angle(3);
//   Serial.println();
// }