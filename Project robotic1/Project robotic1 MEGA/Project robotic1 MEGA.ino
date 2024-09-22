#include <math.h>  // Include this for sin() and cos()
#include <Wire.h>
#include <AS5600.h>
#include <Adafruit_PWMServoDriver.h>

#define RXD2 17
#define TXD2 16
#define PWMA 11
#define INA1 29
#define INA2 28
#define PWMB 10
#define INB1 33
#define INB2 32
#define PWMC 9
#define INC1 37
#define INC2 36
// TCA9548A I2C Multiplexer address
#define TCA9548A_ADDRESS 0x70
// PCA9685 PWM Driver address
#define PCA9685_ADDRESS 0x40
#define AS5600_ADDRESS 0x36

AMS_5600 ams5600_1;  // Make sure this class name matches your library
AMS_5600 ams5600_2;  // Make sure this class name matches your library

// Servo min and max pulse lengths (out of 4096)
#define SERVOMIN  125
#define SERVOMAX  575 

// int i = 0;
int angle_in = 0;
int state = 3;
// int stop = 0;
static bool packetStarted = false;
static int packetIndex = 0;
static uint8_t buffer[72];  // 4 integers * 4 bytes each = 16 bytes
int receivedData[9];        // Array to store the final integers
int V[9];
// int Va, Vb, Vc;
// int Va_mapped, Vb_mapped, Vc_mapped;
int W[3] = {0, 0, 0};
int PWM[3] = {PWMA, PWMB, PWMC};
int IN1[3] = {INA1, INB1, INC1};
int IN2[3] = {INA2, INB2, INC2};
int in1[3] = {0, 0, 0};
int in2[3] = {0, 0, 0};

double E = 0;
double XA = 65;
double YA = 51.24;
double XE = 128.5;
double YE = 51.24;
double a[6] = {73.74, 17.48, 54.47, 0, 90, 150,};
int angle[2];
int j1, j2; 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);  // Initialize the PCA9685

void TCA9548A_SelectBus(uint8_t bus) {
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << bus); // Select the desired I2C bus
  Wire.endTransmission();
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(500000);
  Wire.begin();
  // Select the correct bus on the TCA9548A for PCA9685
  TCA9548A_SelectBus(1);  // Bus 1 (adjust if connected to a different bus)
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz

  for(int i = 0; i < 3; i++) {
    pinMode(PWM[i], OUTPUT);
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
  }
  
}
void Angle() {
    TCA9548A_SelectBus(2);
    angle[0] = map(ams5600_1.getRawAngle(),0,4095,0,360);
    TCA9548A_SelectBus(3);
    angle[1] = map(ams5600_2.getRawAngle(),0,4095,0,360);
}
void receive() {
  while (Serial2.available()) {
    
    char c = Serial2.read();

    if(c == '<') {
      packetStarted = true;
      packetIndex = 0;
    } else if(c == '>' && packetStarted) {
      packetStarted = false;
      // Convert the buffer into integers
      for(int i = 0; i < 9; i++) {
        receivedData[i] = (int32_t)(
            ((uint32_t)buffer[i * 4 + 0] << 0) |
            ((uint32_t)buffer[i * 4 + 1] << 8) |
            ((uint32_t)buffer[i * 4 + 2] << 16) |
            ((uint32_t)buffer[i * 4 + 3] << 24) |
            ((uint32_t)buffer[i * 4 + 4] << 32) |
            ((uint32_t)buffer[i * 4 + 5] << 40) |
            ((uint32_t)buffer[i * 4 + 6] << 48) |
            ((uint32_t)buffer[i * 4 + 7] << 56) |
            ((uint32_t)buffer[i * 4 + 8] << 64)
        );
      }

      for(int i = 0; i < 9; i++) {
        V[i] = receivedData[i];
      }

      // Print the received integers for debugging
    //   for (int i = 0; i < 9; i++) {
    //     Serial.print("i");
    //     Serial.print(i);
    //     Serial.print(": ");
    //     Serial.print(receivedData[i]);
    //     Serial.print(" ");
    //   }
    // Serial.println();
    } else if(packetStarted && packetIndex < sizeof(buffer)) {
      buffer[packetIndex] = c;
      packetIndex++;
    }
  }
}
void check_state() {
  receive();
  if(V[0] == 1) {
    state = (state + 1) % 2;
    delay(1000);
  }
}
void robot_move() {
  receive();
  W[0] = (-V[1] + (-V[3] * 0.125 * 3)) * (255 / (127 * (sin(PI/6) + cos(PI/6) + (0.125 * 3))));
  W[1] = ((V[1] * sin(PI/6)) - (V[2] * cos(PI/6)) + (-V[3] * 0.125 * 3)) * (255 / (127 * (sin(PI/6) + cos(PI/6) + (0.125 * 3))));
  W[2] = ((V[1] * sin(PI/6)) + (V[2] * cos(PI/6)) + (-V[3] * 0.125) * 3) * (255 / (127 * (sin(PI/6) + cos(PI/6) + (0.125 * 3))));  

  for(int i = 0; i < 3; i++) {
    if(W[i] < 0) {
      digitalWrite(IN1[i], HIGH);
      digitalWrite(IN2[i], LOW);
    } else if(W[i] > 0) {
      digitalWrite(IN1[i], LOW);
      digitalWrite(IN2[i], HIGH);
    } else {
      digitalWrite(IN1[i], HIGH);
      digitalWrite(IN2[i], HIGH);
    }
    analogWrite(PWM[i], abs(W[i]));
  }

  // if(in1[1] == 1 && in2[1] == 0) {
  //   Serial.println("forward");
  // } else if(in1[1] == 0 && in2[1] == 1) {
  //   Serial.println("backward");
  // } else if(in1[1] == 1 && in2[1] == 1){
  //   Serial.println("stop");
  // }
  // Serial.print("V[0]: ");
  // Serial.print(W[0]);
  // Serial.print(", V[1]: ");
  // Serial.print(W[1]);
  // Serial.print(", V[2]: ");
  // Serial.print(W[2]);
  // Serial.println();  
}
void arm_robot_move() {
  receive();

  double L1 = 220;
  double L2 = 160;
  double L3 = 63.5;

  E = E + (V[7] * 1);

  if(E < 0.0) E = 0.0;
  if(E > 50) E = 50;

  XA = XA + ((V[6]/127) * 1); 
  YA = YA + ((V[2]/127) * 1);
  YE = YE + ((V[2]/127) * 1);

  if(XA < 65) XA = 65;
  if(XA > 260) XA = 260;
  if(YA < 51.24) {
    YA = 51.24;
    YE = YA; 
  }
  if(YA > 260) {
    YA = 260;
    YE = YA; 
  }

  XE = sqrt(pow(63.5,2) - pow(E,2)) + XA;

  a[0] = (atan(YA / XA) + acos(((pow(L2,2)) - (pow(L1,2)) - ((pow(XA,2)) + (pow(YA,2)))) / (-2 * L1 * sqrt((pow(XA,2) + pow(YA,2)))))) * (180 / PI);
  a[1] = (acos((pow(XA,2) + pow(YA,2) - pow(L1,2) - pow(L2,2)) / (-2 * L1 * L2))) * (180 / PI);
  a[3] = (atan(E / (XE - XA + 0.001))) * (180 / PI);
  a[2] = a[3] - a[0] - a[1] + 180;

  a[4] = a[4] + (V[8] * 0.5);  
  if (a[4] < 0) a[4] = 0;
  if (a[4] > 180) a[4] = 180;

  a[5] = a[5] + (V[5] * 0.5);
  if (a[5] < 0) a[5] = 0;
  if (a[5] > 150) a[5] = 150;

  j1 = a[0];
  j2 = a[1]; 
  
  int pulseLength3 = SERVOMIN + ((SERVOMAX - SERVOMIN) * abs(a[2] - 180 + 73.95)) / 180.0;
  int pulseLength4 = SERVOMIN + ((SERVOMAX - SERVOMIN) * a[4]) / 180.0;
  int pulseLength5 = SERVOMIN + ((SERVOMAX - SERVOMIN) * a[5]) / 180.0;
  pwm.setPWM(2, 0, pulseLength3);
  pwm.setPWM(3, 0, pulseLength4);
  pwm.setPWM(4, 0, pulseLength5);
  pwm.setPWM(0, 0, SERVOMIN + ((SERVOMAX - SERVOMIN) / 2));
  pwm.setPWM(1, 0, SERVOMIN + ((SERVOMAX - SERVOMIN) / 2));
  
  if (j1 < (angle[0] - 62)) {
    pwm.setPWM(0, 0, 385);
    // Serial.println("Moving servo 0");
  }
  if (j1 > (angle[0] - 62)) {
    pwm.setPWM(0, 0, 315);
    // Serial.println("Moving servo 0 to other position");
  }

  if (j2 < (angle[1] - 154)) {
    pwm.setPWM(1, 0, 385);
    // Serial.print(j2);
    // Serial.print(" ");
    // Serial.print(angle[1] - 154);
    // Serial.print(" ");
    // Serial.println("Moving servo 1");
  }
  if (j2 > (angle[1] - 116)) {
    pwm.setPWM(1, 0, 270);
    // Serial.println("Moving servo 1 to other position");
  }

    
  // if(j2+25 < (angle[1] - 116)) {
  //   pwm.setPWM(1, 0, SERVOMIN);
  // } else if(j2+25 > (angle[1] - 116)) {
  //   pwm.setPWM(1, 0, SERVOMAX);
  // } else if(j2+25 == (angle[1] - 116)) {
  //   pwm.setPWM(1, 0, SERVOMIN + ((SERVOMAX - SERVOMIN) / 2));
  // }
  // delay(1);
  // Serial.print(angle[0]);
  // Serial.print(" "); 
  // Serial.print(angle[1]);
  // Serial.print(" ");
  // Serial.println(j1);
  // Serial.print("t[0]: ");
  // Serial.print(a[0]);
  // Serial.print(", t[1]: ");
  // Serial.print(a[1]);
  // Serial.print(", t[2]: ");
  // Serial.print(abs(a[2] - 180 + 36.75));
  // Serial.print(", t[3]: ");
  // Serial.print(a[3]);
  // Serial.print(" ");
  
  // Serial.print((pow(XA,2) + pow(YA,2) - pow(L1,2) - pow(L2,2)) / (-2 * L1 * L2)); 
  // Serial.println();
  // Serial.print(j1);
  // Serial.print(" ");
  // Serial.print(angle[0] - 63);
  // Serial.println();
}
void loop() {
  Angle();
  receive();
  check_state();
  if(state == 3) {
    // Serial.println(state);
    TCA9548A_SelectBus(1);
    arm_robot_move();
    if((j1 == (angle[0] - 62)) and ((j2 <= (angle[1] - 154)) and (j2 >= (angle[1] - 116)))) {
      state = (state + 1) % 2;
    }
    Serial.print("angle[1] = ");
    Serial.print(angle[1] - 154);
    Serial.print(" j2 = ");
    Serial.println((j2));

  } else if(state == 0) {
    Serial.println(state);
    TCA9548A_SelectBus(1);
    pwm.setPWM(0, 0, 350);
    pwm.setPWM(1, 0, 350);

    // Serial.println(stop);
    robot_move();
  } else if(state == 1) {
    Serial.println(state);
    for(int i = 0; i < 3; i++){
      analogWrite(PWM[i], 0);
    }
    TCA9548A_SelectBus(1);
    arm_robot_move();
  }
      // pwm.setPWM(0, 0, 350);
      // delay(500);
      // pwm.setPWM(1, 0, 350);
      // delay(500);
  
  
    // Serial.print(" ");
    // Serial.print(XE);
    // Serial.print(" ");
    // Serial.print(YE + E);
    // Serial.print(" ");
    // Serial.print(XA);
    // Serial.print(" ");
    // Serial.print(YA);
    // Serial.println();  
  
  
  // Serial.println(state);
}