
#include <AccelStepper.h> // library of the stepper motor
#include <MultiStepper.h>   // For coordinated movments

#define motorPin_L_En   48 // Enable Pin
#define motorPin_L_Dir  50 // Direction Pin
#define motorPin_L_Step 52 // Step Pin

#define motorPin_R_En   42 // Enable Pin
#define motorPin_R_Dir  44 //  Right Motor
#define motorPin_R_Step 46 // 
#define  DRIVER  1

#define DefaultSpeed 1000
#define HoldMillis 250

#define MS1R 22 // Motor step devider select pins (Right Motor)
#define MS2R 24 // Motor step devider select pins (Right Motor)
#define MS3R 26 // Motor step devider select pins (Right Motor)

#define MS1L 23 // Motor step devider select pins (Left Motor)
#define MS2L 25 // Motor step devider select pins (Left Motor)
#define MS3L 27 // Motor step devider select pins (Left Motor)

#define RST_LIDAR_Head 53 // Reset pin of the LIDAR head
#define RST_LIDAR_Motor 51 // Reset pin of the LIDAR stepper motor



/****************** Objects ********************************/
AccelStepper L_Motor(DRIVER, motorPin_L_Step, motorPin_L_Dir); //For NEMA17 Motors (step, direction)
AccelStepper R_Motor(DRIVER, motorPin_R_Step, motorPin_R_Dir);
MultiStepper MotorSet;
/***********************************************************/

/**************** Global variables *************************/
long positions[2];     // This will hold the target position for simultinuous running for two stepper motors
int StepsCounterL;     // Holding the actual stepps that the motor drove, this will be reported back to MATLAB
int StepsCounterR;
short int Buffer_index = 0;
char inData[52];       //Serial data buffer
char Command[52];      //The command received from serial port
unsigned long StopMillis = 0; // The millis at which the motor stops
/**********************************************************/


void setup() {  //MotorSet.addStepper(L_Motor);

  Serial.begin(115200);
  digitalWrite(RST_LIDAR_Head, HIGH);
  digitalWrite(RST_LIDAR_Motor, HIGH);
  pinMode(RST_LIDAR_Head, OUTPUT);
  pinMode(RST_LIDAR_Motor, OUTPUT);
  
  pinMode(motorPin_R_En, OUTPUT);
  pinMode(motorPin_L_En, OUTPUT);

  pinMode(MS1R, OUTPUT);
  pinMode(MS2R, OUTPUT);
  pinMode(MS3R, OUTPUT);

  pinMode(MS1L, OUTPUT);
  pinMode(MS2L, OUTPUT);
  pinMode(MS3L, OUTPUT);

  //set motor step devider
  digitalWrite(MS1R, LOW);
  digitalWrite(MS2R, HIGH);
  digitalWrite(MS3R, LOW);
 
  digitalWrite(MS1L, LOW);
  digitalWrite(MS2L, HIGH);
  digitalWrite(MS3L, LOW);


  L_Motor.setMaxSpeed(DefaultSpeed);
  L_Motor.setAcceleration(DefaultSpeed * 30);
  R_Motor.setMaxSpeed(DefaultSpeed);
  R_Motor.setAcceleration(DefaultSpeed * 30);

  MotorSet.addStepper(L_Motor);
  MotorSet.addStepper(R_Motor);

  //positions[0] = 1000;
  //positions[1] = 1000;
  //MotorSet.moveTo(positions);
  //MotorSet.runSpeedToPosition(); // Blocks until all are in position

  Serial.println("MapperBot V1.0");
  Serial.println("akram.hourani");
  Serial.println("@rmit.edu.au");
  Serial.println("20170813");
  Serial.println("");
  Serial.println("Command format XLLLLXRRRRDDDDD#");
  Serial.println("X:    +,-       (forward, backward)");
  Serial.println("L:    0000-4000   (left motor speed)");
  Serial.println("R:    0000-4000   (Right motor speed)");
  Serial.println("DDD:  0-99999 (Counter destination)");
  Serial.println("D# Disarm motors");
  Serial.println("R# Reset LIDAR Head");
  
  
  Serial.println("Motors check");


  // Motor check rotine
  MotorCheckRoutine();



  Serial.println("Ready !!");
}

void loop() {

  if (Serial.available() > 0) {
    inData[Buffer_index] = Serial.read();
    Buffer_index++;

    //When a commdan is ready execute the following
    if (inData[Buffer_index - 2] == '#' && Buffer_index > 1)  {
      // Serial.print("Debug 1 ");
      memset(Command, 0, sizeof(Command)); // Clear recieved buffer
      strncpy(Command, inData, Buffer_index); // Save the command
      Buffer_index = 0; // reset the buffer pointer
      //Serial.println(Command);
      Serial.print(Command);

      Execute_Command();
    }

  }


  if ( (millis()-StopMillis ) > HoldMillis) // Disable the motors after a period of HoldMillis
  {
    digitalWrite(motorPin_L_En, HIGH);
    digitalWrite(motorPin_R_En, HIGH);
  }

}

/************************************************************************************/
// This is the main function of montion and reporting
void moveToFunc(int SpeedLN, int SpeedRN, int SteppsLN) {
  digitalWrite(motorPin_L_En, LOW);
  digitalWrite(motorPin_R_En, LOW);

  // Reset poistion
  L_Motor.setCurrentPosition(0);
  R_Motor.setCurrentPosition(0);

  L_Motor.setMaxSpeed(abs(SpeedLN));
  R_Motor.setMaxSpeed(abs(SpeedRN));

  if (SpeedLN < 0)   positions[0] = -SteppsLN ;
  else   positions[0] = SteppsLN ;
  if (SpeedRN < 0)  positions[1] = -SteppsLN ;
  else   positions[1] = SteppsLN ;
  //MotorSet.moveTo(positions);
  L_Motor.moveTo(positions[0]);
  R_Motor.moveTo(positions[1]);

   MotorSet.moveTo(positions);
   MotorSet.runSpeedToPosition(); // Blocks until all are in position

  // This will run both motor simultinuously. The processor is blocked untill the command is executed.
 /* while (L_Motor.distanceToGo() != 0 || R_Motor.distanceToGo() != 0)
  {
    L_Motor.run();
    R_Motor.run();
    //L_Motor.runSpeedToPosition();
    //R_Motor.runSpeedToPosition();
  }*/


  StepsCounterL = L_Motor.currentPosition();
  StepsCounterR = -R_Motor.currentPosition();

  StopMillis = millis();


}
/************************************************************************************/


/************************************************************************************/
void Execute_Command() {

  bool error = 0;

  if (Command[0] == 'D') // disarm the motors
  {
    digitalWrite(motorPin_L_En, HIGH);
    digitalWrite(motorPin_R_En, HIGH);
  }

 else if (Command[0] == 'R') // Send a reset signal to the LIADR head
  {

  digitalWrite(RST_LIDAR_Head, LOW);
  digitalWrite(RST_LIDAR_Motor, LOW);
  delay(200);
  digitalWrite(RST_LIDAR_Head, HIGH);
  digitalWrite(RST_LIDAR_Motor, HIGH);
  }

  else // Motion Command
  {
    char SteppsLNc[52];
    memset(SteppsLNc, 0, sizeof(SteppsLNc));

    char SpeedLNc[48];
    memset(SpeedLNc, 0, sizeof(SpeedLNc));

    char SpeedRNc[48];
    memset(SpeedRNc, 0, sizeof(SpeedRNc));

    int SteppsLN;
    int SpeedLN;
    int SpeedRN;

    for (int i = 0 ; i < 4; i++) SpeedLNc[i] = Command[i + 1]; //copy  digits 1,2,3,4 left speed
    for (int i = 0 ; i < 4; i++) SpeedRNc[i] = Command[i + 6]; //copy  digits 6,7,8,9 right speed
    for (int i = 0 ; i < 5; i++) SteppsLNc[i] = Command[i + 10]; //copy digits 10,11,12,13,14 Destination counter


    if (Command[0] == '+')   SpeedLN =   atoi(SpeedLNc); // Moving Forward
    else if (Command[0] == '-')    SpeedLN = - atoi(SpeedLNc);// Moving Forward}
    else error = 1;


    if (Command[5] == '+')   SpeedRN =   -atoi(SpeedRNc);  // Moving Forward
    else if (Command[5] == '-')   SpeedRN =  atoi(SpeedRNc);  // Moving Forward


    else    error = 1;


    SteppsLN =   atoi(SteppsLNc); // Moving Forward

    if (error)
    {
      Serial.println("Invalid command !");
      Serial.flush(); //flush the serial buffer
      // reset the buffer data and buffer index
      memset(inData, 0, sizeof(inData));
      Buffer_index = 0;

    }
    else {

      moveToFunc(SpeedLN, SpeedRN, SteppsLN);
      Serial.println("Ok");

    }
  }

}


void MotorCheckRoutine(void)
{
  //Enable the motors
  digitalWrite(motorPin_L_En, LOW);
  digitalWrite(motorPin_R_En, LOW);

  positions[0] = 200;
  positions[1] = 200;

  L_Motor.moveTo(positions[0]);
  R_Motor.moveTo(positions[1]);

  while (L_Motor.distanceToGo() != 0 || R_Motor.distanceToGo() != 0)
  {
    L_Motor.run();
    R_Motor.run();
  }

  delay(200);
  positions[0] = -200;
  positions[1] = -200;

  L_Motor.moveTo(positions[0]);
  R_Motor.moveTo(positions[1]);
  while (L_Motor.distanceToGo() != 0 || R_Motor.distanceToGo() != 0)
  {
    L_Motor.run();
    R_Motor.run();
  }
delay(200);
  positions[0] = 0;
  positions[1] = 0;

  L_Motor.moveTo(positions[0]);
  R_Motor.moveTo(positions[1]);
  while (L_Motor.distanceToGo() != 0 || R_Motor.distanceToGo() != 0)
  {
    L_Motor.run();
    R_Motor.run();
  }

StopMillis = millis();
}

