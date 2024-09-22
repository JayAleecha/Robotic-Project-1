// #include <ps5Controller.h>

// // #define RX2 16
// // #define TX2 17

// void setup() {

//   Serial.begin(500000);
//   ps5.begin("88:03:4C:3A:E8:5F");  //replace with MAC address of your controller
//   Serial.println("Ready.");

//   // Serial2.begin(9600);  //RX2 = 17, TX2 = 16
// }

// void loop() {

//   while (ps5.isConnected() == false) {  // commented out as ps5 controller seems to connect quicker when microcontroller is doing nothing

//     Serial.println("PS5 controller not found");
//     Serial2.println("PS5 controller not found");

//     delay(300);
//   }

//   while (ps5.isConnected() == true) {

//     if (ps5.Right()) Serial.println("Right Button");
//     Serial2.println("Right Button");

//     if (ps5.Down()) Serial.println("Down Button");
//     Serial2.println("Down Button");

//     if (ps5.Up()) Serial.println("Up Button");
//     Serial2.println("Up Button");

//     if (ps5.Left()) Serial.println("Left Button");
//     Serial2.println("Left Button");



//     if (ps5.Square()) Serial.println("Square Button");

//     if (ps5.Cross()) Serial.println("Cross Button");

//     if (ps5.Circle()) Serial.println("Circle Button");

//     if (ps5.Triangle()) Serial.println("Triangle Button");



//     if (ps5.UpRight()) Serial.println("Up Right");

//     if (ps5.DownRight()) Serial.println("Down Right");

//     if (ps5.UpLeft()) Serial.println("Up Left");

//     if (ps5.DownLeft()) Serial.println("Down Left");



//     if (ps5.L1()) Serial.println("L1 Button");

//     if (ps5.R1()) Serial.println("R1 Button");



//     if (ps5.Share()) Serial.println("Share Button");

//     if (ps5.Options()) Serial.println("Options Button");

//     if (ps5.L3()) Serial.println("L3 Button");

//     if (ps5.R3()) Serial.println("R3 Button");

//     if (ps5.PSButton()) Serial.println("PS Button");

//     if (ps5.Touchpad()) Serial.println("Touch Pad Button");

//     if (ps5.L2()) {

//       Serial.printf("L2 button at %d\n", ps5.L2Value());
//     }

//     if (ps5.R2()) {

//       Serial.printf("R2 button at %d\n", ps5.R2Value());
//     }

//     if (ps5.LStickX()) {

//       Serial.printf("Left Stick x at %d\n", ps5.LStickX());
//     }

//     if (ps5.LStickY()) {

//       Serial.printf("Left Stick y at %d\n", ps5.LStickY());
//     }

//     Serial.println();

//     // This delay is to make the output more human readable

//     // Remove it when you're not trying to see the output

//     //delay(300);
//   }
// }
#include <ps5Controller.h>

#define RXD2 16
#define TXD2 17

int XI, YI, W;
int SM;
int SHA, GP, XA, YA, GPR, YE;
int V[9];

void setup() {
  // Serial.begin(9600);
  Serial.begin(115200);
  ps5.begin("88:03:4C:3A:E8:5F");  //replace with MAC address of your controller
  Serial2.begin(500000, SERIAL_8N1, RXD2, TXD2);
}
void receive() {
  if(ps5.isConnected() == false) {
    Serial.println("PS5 controller not found");
    SM = 0;

    XI = 0;
    YI = 0;
    W = 0;

    SHA = 0;
    GP = 0;
    XA = 0;
    YA = 0;
  }
  if(ps5.isConnected() == true) {
    SM = ps5.R1();

    XI = ps5.LStickX();
    YI = ps5.LStickY();
    W = ps5.RStickX();

    SHA = ps5.R3();
    GP = ps5.Square() - ps5.Circle();
    XA = ps5.RStickY();
    YE = ps5.Up() - ps5.Down();
    GPR = ps5.Right() - ps5.Left();

    delay(100);
  }
}
void send() {
  receive();
  //Set mode
  V[0] = SM;
  //Robot move velocity
  V[1] = XI;
  V[2] = YI;
  V[3] = W;
  //Arm joint velocity
  V[4] = SHA;
  V[5] = GP;
  V[6] = XA;
  V[7] = YE;
  V[8] = GPR;

  Serial2.write('<');  // Start marker
  for(int i = 0; i < 9; i++) {
    Serial2.write((uint8_t*)&V[i], sizeof(int));  // Send each integer as 4 bytes
  }
  Serial2.write('>');  // End marker
}
void loop() {
  send();
}