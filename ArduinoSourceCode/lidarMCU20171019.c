#include <Wire.h>         //I2C
#include <LIDARLite.h>
#include <SoftwareSerial.h>
#include <EEPROM.h> //Needed to access the eeprom read write functions

/* For Nano */
#define TPP 7// Trigger Pin poisitve side (CCW)
#define TPN 6// Trigger Pin negative side (CW)
#define READY_Rotate 5// Rotation ready Pin
#define Calibration_N 4 // Number of rotations at each side to pefom the calibtation
#define AdrCalParam 0 // EEPROM address location for the rotation period
#define BufferSize 60 // this is the serial buffer size


/****************** Objects ********************************/
LIDARLite       MyLIDAR;
SoftwareSerial  SoftSerial(3, 2); // RX, TX
/**************** Global variables *************************/
short int Buffer_index = 0;
char inData[BufferSize];       //Serial data buffer
char Command[BufferSize];      //The command received from serial port
int LidarDist;    // This will store the measurments of the lidar in cm
float Theta;               // This will store the angle measurments in degrees
unsigned long TimeStart;        // This will hold the time when the rotation start
int Vector_Length ;    // This will store the numerb of the received samples
int RotationPeriod;
bool Continuous_Scan_Flag = false; // This is to indicate a continuous scan
bool Scan_Dir             = true;  // This is the scanning direction true for positive and flase for negative
unsigned long StopMillis = 0; // The millis at which the motor stops

/**********************************************************/
/**********************************************************/

void setup() {
  Serial.begin(500000);
  SoftSerial.begin(4800);

  digitalWrite(TPP, HIGH); // The pin is active low
  digitalWrite(TPN, HIGH); // The pin is active low
  pinMode(TPP, OUTPUT);
  pinMode(TPN, OUTPUT);
  digitalWrite(TPP, HIGH); // The pin is active low
  digitalWrite(TPN, HIGH); // The pin is active low
  pinMode(READY_Rotate, INPUT_PULLUP);


  Serial.println("LIDAR Scanner V1.0");
  Serial.println("Master MCU");
  Serial.println("akram.hourani@rmit.edu.au");
  Serial.println("Firmware: 20170828");
  Serial.println("");

  EEPROM.get(AdrCalParam, RotationPeriod); //retriving the rotation period from the EEPROM
  PrintHelpMenu();

  Serial.print("Activating LIDAR...");
  MyLIDAR.begin(0, true); // 1: Short range, high speed. Uses 0x1d maximum acquisition count.
  Serial.println("Active");

  Serial.print("Waiting motor driver... ");
  while (!digitalRead(READY_Rotate));
  delay(100);
  SoftSerial.print("+100025#");
  delay(100);
  SoftSerial.print("-1000050#");
  delay(100);
  SoftSerial.print("+100025#");
  Serial.println("Ok");


  Serial.println("Ready to receive commands !");

  /* LIDAR Unit Testing
    while(1)
    {
    Serial.println(MyLIDAR.distance());
    delay(500);
    }
  */

}

void loop() {


  //****** This is to monitor the serial Port **********//
  if (Serial.available() > 0) {
    inData[Buffer_index] = Serial.read();
    Buffer_index++;


    if (Buffer_index >= BufferSize) // This should protect the buffer from overflow
    {
      Serial.flush();
    }
    
    //When a commdan is ready execute the following
    if (inData[Buffer_index - 2] == '#' && Buffer_index > 1)  {
      // Serial.print("Debug 1 ");
      memset(Command, 0, sizeof(Command)); // Clear recieved buffer
      strncpy(Command, inData, Buffer_index); // Save the command
      Buffer_index = 0; // reset the buffer pointer
      //Serial.println(Command);

      SoftSerial.print(Command); // Copy everythig to the Motor driver
      Serial.println(Command);
      Serial.flush();
      Execute_Command();
      
    }
  }

  // this will excute contiuous scan
  if (Continuous_Scan_Flag)
    ScanDistance();



}



/*******************************************************************************/
void Execute_Command() {
  Serial.flush(); // clear the serial buffer
  bool error = 0;

  if (Command[0] == 'S') // Scanning commad: This cammand will make the platfrm to rotate 360 degrees and take lidar  measurements
  {
    ScanDistance();
  }

  if (Command[0] == 'C') // Continuous Scanning commad: This cammand will make the platfrm to rotate 360 degrees and take lidar  measurements
  {
    Continuous_Scan_Flag = true;
  }

  if (Command[0] == 'D') // Disable Continuous Scanning commad: This cammand will make the platfrm to rotate 360 degrees and take lidar  measurements
  {
    Continuous_Scan_Flag = false;
  }


  if (Command[0] == 'L')
  {
    CalibrateRotationTime(); // This will measure rotation time
  }

  if (Command[0] == '?')
  {
    Serial.println("");
    Serial.println("");
    Serial.println("Ok here is the help menu:");
    PrintHelpMenu();
  }


}



/* This function will excute the LIDAR scanning functionality */
void ScanDistance(void)
{

  /* I am unable to store the data in a vector. THe Nano memory ist too small
      so I am sending the data to the serial port as it arrives*/

  if (Scan_Dir) {
    digitalWrite(TPP, LOW); // The pint active low
    delay(1); // wait the line to stablize
    digitalWrite(TPP, HIGH);
  }
  else
  {
    digitalWrite(TPN, LOW); // The pint active low
    delay(1); // wait the line to stablize
    digitalWrite(TPN, HIGH);
  }
  TimeStart = millis();
  delay(2); //allow some time for the ready line to go low
  // Reset the memory location
    Serial.println("Begin");
  LidarDist   = MyLIDAR.distance(); // Take a LDIAR measurement

  if (Scan_Dir)
    Theta       =  (float) ((millis() - TimeStart) * 360 / RotationPeriod); //record the clock time at this measurment
  else
    Theta       =  -(float) ((millis() - TimeStart) * 360 / RotationPeriod); //record the clock time at this measurment
  //Serial.println("#");
  Serial.print(Theta,1);
  Serial.print(",");
  Serial.print(LidarDist);

  int Counter = 1;
  while (!digitalRead(READY_Rotate)) { // Do this while the READY_Rotate Line is still low
    LidarDist       = MyLIDAR.distance(); // Take a LDIAR measurement
    if (Scan_Dir)
      Theta  =   (float)((millis() - TimeStart) * 360.0 / RotationPeriod); //record the clock time at this measurment
    else
      Theta = - (float)((millis() - TimeStart) * 360.0 / RotationPeriod); //record the clock time at this measurment

    Serial.print(";");
    Serial.print(Theta,1);
    Serial.print(",");
    Serial.print(LidarDist);

    Counter++;
  }
  delay(5);
  Serial.print('\n');
  Scan_Dir = !Scan_Dir; // reverse scanning direction

}






void CalibrateRotationTime(void)
{
  unsigned long RotationPeriodP[4]; // This vector will store the period readings
  unsigned long RotationPeriodN[4];

  for (int ctr = 0; ctr < Calibration_N; ctr++) {
    digitalWrite(TPP, LOW); // The pin is active low
    delay(1); // wait the line to stablize
    digitalWrite(TPP, HIGH);
    TimeStart = millis();
    delay(5); //wait the line to stablize
    while (!digitalRead(READY_Rotate));
    RotationPeriodP[ctr] = millis() - TimeStart;
    Serial.print("Time CCW = ");
    Serial.println(RotationPeriodP[ctr]);
    delay(50);
  }

  for (int ctr = 0; ctr < Calibration_N; ctr++) {
    digitalWrite(TPN, LOW); // The pint active low
    delay(1); // wait the line to stablize
    digitalWrite(TPN, HIGH);
    TimeStart = millis();
    delay(5); //wait the line to stablize
    while (!digitalRead(READY_Rotate));
    RotationPeriodN[ctr] = millis() - TimeStart;
    Serial.print("Time CW = ");
    Serial.println(RotationPeriodP[ctr]);
    delay(50);
  }

  RotationPeriod = 0;
  for (int ctr = 0; ctr < Calibration_N; ctr++) {
    RotationPeriod =  (int) RotationPeriod + RotationPeriodN[ctr] + RotationPeriodP[ctr] ;
  }
  RotationPeriod = RotationPeriod / Calibration_N / 2;
  Serial.print("Average time = ");
  Serial.println(RotationPeriod);
  Serial.println("Saving to EEPROM");
  EEPROM.put(AdrCalParam, RotationPeriod);
}

void PrintHelpMenu(void)
{
  Serial.println("S# for 360 degrees scanning");
  Serial.println("C# for continuous scanning");
  Serial.println("D# to disable continuous scanning");
  Serial.print("Current rotation period [millis ticks]: ");
  Serial.println(RotationPeriod);
  Serial.println("L# to calibrate the rotation period");
  Serial.println("XSSSDDDDD# for manual rotation");
  Serial.println("X:      +,-         (CCW, CW)");
  Serial.println("SSSS:   0000-2000   (Speed)");
  Serial.println("DDDDD:  00000-99999 (Counter destination)");
  Serial.println("?# for help");
  Serial.println("");
}


