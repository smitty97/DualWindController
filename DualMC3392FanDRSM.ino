// board Pololu A-Star 32U4
// port com6

// THIS VESION WORKS.  still has a 'blip' every few seconds....  
#include "DualMC33926MotorShield.h"   //Pololu 3A motor shield 
                                      // speed input vales 0-400
                                      
bool serialConnected = false;

int counter = 0;   // for sending stop command 
int maxcounter = 1000;  //  after this many counts

DualMC33926MotorShield md;           // fans are 24V 4" SEAFLO 3A Max

#define MOTORTHRESHOLD 50           // where the fans start spinning
#define PRINTCHANGES false
#define PRINTANYTHING false         //  printing causes slow loops after serial disconnect

// Pins used by mc33926
//    A0 - M1FB
//    A1 - M2FB
//    4 - D2
//    7  - m1DIR
//    8  - m2DIR
//    9  - m1PWM
//    10 - m2PWM
//    12 - SF

int serialpin = 2;  // SPDT center terminal to ground (14)
int manualpin = 3;  // serial-manual switch 
int knobpin = A2;    // pot knob pin for manual speed or max serial speed  (also GND, +5V)

int knobpos;
int switchpos;
int prev_switch = 0;
int prev_knob = 0;

int LSpeed;
int RSpeed;

byte bytesBuffer[6] = {0};

// Threshold values for 0% and 100% in DRSM (these are the stop and full-speed byte patterns)
const byte stopByte[] = {0x00, 0x00};  // Expected stop pattern at 0% (e.g., 00 00)
const byte fullSpeedByte[] = {0xFF, 0xFF}; // Expected full-speed pattern at 100% (e.g., FF FF)


// SETUP ***********************************************
void setup() {
  Serial.begin(115200);
  serialConnected = true;
  md.init();
  pinMode(serialpin, INPUT_PULLUP);
  pinMode(manualpin, INPUT_PULLUP);
}

void loop() {
  handleSerial();
  switchloop();
  printcurrent();
  // Add a small delay to avoid overwhelming the loop
  delay(5);
}

// Function to handle serial communication
void handleSerial() {
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
  }
}

// switch case
  // 0 is off
  // 1 is manual knob
  // -1 is serial
void switchloop() {
  switch (getSwitch()) {
    case 1 : {
        LSpeed = ReadKnob();
        RSpeed = LSpeed;
        blow(LSpeed, RSpeed);
        delay(50);
        break;
      }
    case -1 : {
        if (serialConnected) {
        ReadData();
        blow(LSpeed, RSpeed);
      } else {
          blow(0, 0);
        }
        break;
      }
    case 0 : {
        blow(0, 0);
        break;
      }
  }
}

// CHECK SWITCH ***********************************************
int getSwitch() {
  // 0 is off
  // 1 is manual knob
  // -1 is serial
  int value = 0;
  // get switch position
  int manualpos = digitalRead( manualpin );
  int serialpos = digitalRead( serialpin );
  delay (1);
  if (manualpos == LOW && serialpos == HIGH ) value = 1;
  if (manualpos == HIGH && serialpos == LOW  ) value = -1;

  //if (value != prev_switch && PRINTCHANGES && PRINTANYTHING) {  Serial.println(value);  }
  
  prev_switch = value;
  return value;
}

// BLOW ***********************************************
void blow( int LSpeedIn, int RSpeedIn) {
  //  don't spin until threshold
  LSpeed = (LSpeedIn <= MOTORTHRESHOLD) ? 0 : LSpeedIn;
  RSpeed = (RSpeedIn <= MOTORTHRESHOLD) ? 0 : RSpeedIn;

  md.setM1Speed(LSpeed);
  md.setM2Speed(RSpeed);
}

//  READ KNOB ********************************************
int ReadKnob() {
  int rawvalue = analogRead( knobpin );
  knobpos = MOTORTHRESHOLD + map(rawvalue, 0, 1023, 0, 400-MOTORTHRESHOLD);
  delay (1);
  
  if (knobpos != prev_knob && PRINTCHANGES && PRINTANYTHING) {
    Serial.print("Knob ");
    Serial.println(knobpos);
  }
  
  prev_knob = knobpos;
  return knobpos;
}

//  READ SERIAL  ********************************************
void ReadData() {
  int knobspeed = ReadKnob();
  // Check if at least 2 bytes are available to read from the serial port
  if (Serial.available() >= 2) {
    counter = 0;
    byte received[2];
    for (int i = 0; i < 2; i++) {
      received[i] = Serial.read();  // Read the incoming bytes
      }

    int LpwmValue = map(received[0], 0x00, 0xFF, 0, 255);  // Map to motor speed (0-255)
    int RpwmValue = map(received[1], 0x00, 0xFF, 0, 255);  // Map to motor speed (0-255)

      // Set the pins values
      LSpeed = MOTORTHRESHOLD + map(LpwmValue, 0, 255, 0, 400-MOTORTHRESHOLD);
      RSpeed = MOTORTHRESHOLD + map(RpwmValue, 0, 255, 0, 400-MOTORTHRESHOLD);

      // knob sets minimum
      if (LSpeed < knobspeed){
        LSpeed = knobspeed;
      };
      if (RSpeed < knobspeed){
        RSpeed = knobspeed;
      };

      // knob sets maximum
  //    LSpeed = map(LSpeed, MOTORTHRESHOLD, 400, MOTORTHRESHOLD, knobspeed);
  //    RSpeed = map(RSpeed, MOTORTHRESHOLD, 400, MOTORTHRESHOLD, knobspeed);

      
    }
  else {
    counter += 1;
    if (counter > maxcounter) {
      LSpeed = knobspeed;   // was 0
      RSpeed = knobspeed;    // blows fan with no serial
    
      }
    }
  }
  

  // FAULT ***********************************************
void stopIfFault() {
  if (md.getFault()) {
    md.setM1Speed(0);
    md.setM2Speed(0);
    while (1);
  }
}

// PRINT DEBUGGING***********************************************
void printcurrent() {
  if (Serial.available() > 0  && PRINTANYTHING){
  switch (getSwitch()) {
    case 0 : {
        Serial.println("OFF: ");
        break;
      }
    case 1 : {
        Serial.println("Manual: ");
        printdata();
        break;
      }
    case -1 : {
        if (serialConnected) {
        Serial.print("Serial: ");
        printdata();
        } else {
          Serial.println("Serial connection stopped.");
        }
        break;
      }
  }
}
}

void printdata() {
  if (Serial.available() > 0  && PRINTANYTHING) {
  Serial.print(serialConnected);
  Serial.print(knobpos);
  Serial.print("  M1: ");
  Serial.print(LSpeed);
  Serial.print("  mA:");
  Serial.print(md.getM1CurrentMilliamps());
  Serial.print("  M2: ");
  Serial.print(RSpeed);
  Serial.print("  mA:");
  Serial.println(md.getM2CurrentMilliamps());
}
}
