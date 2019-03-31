#include <AccelStepper.h> // library of the stepper motor
#include <Wire.h>         //I2C


#define motorPin_Dir  11 // Direction Pin
#define motorPin_Step 12 // Step Pin
#define MS1 8//Step devider bit1
#define MS2 9//Step devider bit2
#define MS3 10//Step devider bit3
#define TPP 7// Trigger Pin poisitve side (CCW)
#define TPN 6// Trigger Pin negative side (CW)
#define READY 5// Rotation READY Pin
#define motorPin_En 4 // Motor enable pin
#define LimitSig  2 // Optical limit swtich Signal line


//#define FullRot 1994 // number of stepps for full rotation
#define FullRot 3988 // number of stepps for full rotation
#define MaxSpeed 2500
#define DRIVER  1 // This is for NEMA 17 stepper motor

#define HoldMillis 250 // The amount of millis to hold the motors on
/****************** Objects ********************************/
AccelStepper LIDARMotor(DRIVER, motorPin_Step, motorPin_Dir); //For NEMA17 Motors (step, direction)
//LIDARLite    MyLIDAR;

/**************** Global variables *************************/
short int Buffer_index = 0;
char inData[52];       //Serial data buffer
char Command[52];      //The command received from serial port
unsigned int dist;
unsigned long StopMillis = 0; // The millis at which the motor stops
/**********************************************************/

void setup() {
  Serial.begin(4800);

  digitalWrite(LimitSig, LOW); // pulldown resistor
  pinMode(LimitSig, INPUT);

  digitalWrite(MS1, LOW); // pulldown resistor

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(motorPin_En, OUTPUT);

  pinMode(TPP, INPUT_PULLUP);
  pinMode(TPN, INPUT_PULLUP);
  pinMode(READY, OUTPUT);



  // The below settings will give 1/8 stepp
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, LOW);

  LIDARMotor.setMaxSpeed(MaxSpeed);
  LIDARMotor.setAcceleration(MaxSpeed * 700);

  Serial.println("Motor Driver V1.0");
  Serial.println("akram.hourani@rmit.edu.au");
  Serial.println("Firmware: 20170828");
  Serial.println("");
  Serial.println("Command format XSSSDDDDD#");
  Serial.println("X:      +,-         (CCW, CW)");
  Serial.println("SSSS:   0000-2000     (Speed)");
  Serial.println("DDDDD:  00000-99999 (Counter destination)");
  Serial.println("Motors check started...");
  RST_Rotine();

  digitalWrite(READY, HIGH); // This means the system is ready
}

void loop() {


  //****** This is to monitor the serial Port **********//
  if (Serial.available() > 0) {
    inData[Buffer_index] = Serial.read();
    Buffer_index++;

    //When a commdan is ready execute the following
    if (inData[Buffer_index - 1] == '#' && Buffer_index > 1)  {
      // Serial.print("Debug 1 ");
      memset(Command, 0, sizeof(Command)); // Clear recieved buffer
      strncpy(Command, inData, Buffer_index); // Save the command
      Buffer_index = 0; // reset the buffer pointer
      //Serial.println(Command);
      Serial.println(Command);

      Execute_Command();
    }

  }

  //****** This is to monitor the trigger pins **********//
  if (!digitalRead(TPP)) {
    while (!digitalRead(TPP)); // Wait untill the line goes high again
    digitalWrite(READY, LOW); // This mean that the motor driver is busy
    moveToFunc(MaxSpeed, FullRot);
    digitalWrite(READY, HIGH); // This mean the rotation is done

  }

  if (!digitalRead(TPN)) {
    while (!digitalRead(TPN)); // Wait untill the line goes high again
    digitalWrite(READY, LOW); // This mean that the motor driver is busy
    moveToFunc(MaxSpeed, -FullRot);
    digitalWrite(READY, HIGH); // This mean the rotation is done
  }

  // This will diable the motors to save energy
  if ( (millis() - StopMillis ) > HoldMillis) // Disable the motors after a period of HoldMillis
  {
    digitalWrite(motorPin_En, HIGH);
  }


}

/**** Motor check rotine ****/

void RST_Rotine() {
  bool SucessFlag = false; // Reset the sucess falg to 0
  

  moveToFunc(MaxSpeed / 2, -int(FullRot / 30));

  digitalWrite(motorPin_En, LOW);
  LIDARMotor.setCurrentPosition(0);
  LIDARMotor.moveTo(FullRot * 1.1); // Move 10% more than a full rotation
  LIDARMotor.setSpeed(MaxSpeed);
  //LIDARMotor.runToPosition();
  while (LIDARMotor.distanceToGo() != 0)
  {
    LIDARMotor.runSpeedToPosition(); // No acceleration
    if (digitalRead(LimitSig))
    {
      SucessFlag = true; // Set the sucess falg to 1
      break; // exite the loop when reaching the zero position

    }
  }

  if (SucessFlag)
  {
    Serial.println("Reset succeeded!");
    Serial.println("Ready to receive commands");
  }
  else
  {
    Serial.println("Reset failed! check the limit switch !!!!");
  }

}



/************************************************************************************/
// This is the main function of montion and reporting
void moveToFunc(int SpeedN, long SteppsN) {
  digitalWrite(motorPin_En, LOW);
  int positions;
  // Reset poistion
  LIDARMotor.setCurrentPosition(0);

  if (SpeedN < 0)   positions = -SteppsN ;
  else   positions = SteppsN ;

  LIDARMotor.moveTo(positions); // this relative move
  //LIDARMotor.setMaxSpeed(abs(SpeedN));
  LIDARMotor.setSpeed(abs(SpeedN));
  //LIDARMotor.runToPosition();
  while (LIDARMotor.distanceToGo() != 0)
  {
    LIDARMotor.runSpeedToPosition(); // No acceleration
  }

  StopMillis = millis();
}
/************************************************************************************/




/*******************************************************************************/
void Execute_Command() {

  bool error = 0;


  if (Command[0] == 'R') // Reset Command --> reset to zero position
  {
    RST_Rotine();
  }

  else // Motion Command
  {
    char SteppsNc[52];
    memset(SteppsNc, 0, sizeof(SteppsNc));

    char SpeedNc[48];
    memset(SpeedNc, 0, sizeof(SpeedNc));

    long SteppsN;
    long SpeedN;

    for (int i = 0 ; i < 4; i++) SpeedNc[i]  = Command[i + 1];  //copy  digits 1,2,3,4 left speed
    for (int i = 0 ; i < 5; i++) SteppsNc[i] = Command[i + 1 + 4]; //copy  digits 5,6,7,8,9 right speed


    if (Command[0] == '+')   SpeedN =   atoi(SpeedNc); // Moving Forward
    else if (Command[0] == '-')    SpeedN = - atoi(SpeedNc);// Moving Forward}
    else error = 1;

    SteppsN =   atol(SteppsNc); // Moving Forward

    if (error)
    {
      Serial.println("Invalid command !");
      // reset the buffer data and buffer index
      memset(inData, 0, sizeof(inData));
      Buffer_index = 0;
    }
    else {
      moveToFunc(SpeedN, SteppsN);

    }
  }

}

