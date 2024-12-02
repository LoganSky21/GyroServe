//code for making your own Self Stabilizing Platform
//module MPU6050 has gyroscope too, but i'm currently reading only accln
//demonstration of finally finished circuit is here: https://youtu.be/mJHAUlLGScw
//you may publish this code anywhere you want, BUT give credits to me
//for help with MPU6050 part of the code refer to https://youtu.be/M9lZ5Qy5S2s of EEEnthusiast on youtube
//Servos have been controlled with "writeMicroseconds" for better resolution and stable motion
//_______________________________________________________________________________________________
//https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
#include<Wire.h>
#include<Servo.h>

//variables for measuring acceleration
long accelX, accelY, accelZ; 
//Variables for measuring gyro measurments   
long gyroX, gyroY, gyroZ;
//variables for storing g-forces 
float gForceX, gForceY, gForceZ; 
//rotational measurements
float rotX, rotY, rotZ;

float pitch, roll, prevP, prevR;
float errorP, errorR;

int writeVal1, writeVal2, i=0;
int *pitchCorrect, *rollCorrect;

Servo myServo1;
Servo myServo2;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  myServo1.attach(11);      //myServo1, 11 => roll
  myServo2.attach(10);      //myServo2, 10 => pitch
}

//function for setting up MPU6050 i2c communication
void setupMPU(){
  Wire.beginTransmission(0x68); // This is the same as (0b1101000)
  Wire.write(0x6B); // accessing register 6B - power management
  Wire.write(0); // same as (0x00000000)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B); // accessing register 1B - Gyro. config
  Wire.write(0x00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0x1C); // accel. config 
  Wire.write(0b00000000);
  Wire.endTransmission();
}

//function for reading acceleration data
void recordAccelRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B); // satrting the register for the accel readings - stores the most reacent accelerometer measurements (sample rate is defined in register 25)
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); // request 6 registers (3B - 40)
  while(Wire.available()<6);
  accelX = Wire.read()<<8|Wire.read(); // first two bytes
  accelY = Wire.read()<<8|Wire.read(); // middle two bytes
  accelZ = Wire.read()<<8|Wire.read(); //last two bytes
  processAccelData();
}

//function for processing acceleration data
//pitch & roll found here along with acceleration in g's
void processAccelData(){
  gForceX = accelX/16384.0; //16384 is sensitivity from data sheet. The value received from the register must be used to calculated a value that is in "g's"
  gForceY = (accelY/16384.0); // (-4) subtract by 4 when mpu is flat
  gForceZ = accelZ/16384.0;
  pitch = (atan(gForceX/(sqrt(pow(gForceY, 2)+pow(gForceZ, 2)))))*(2000.0/PI)+1500;
  roll = (atan(-gForceY/gForceZ))*(2000.0/PI)+1500;
  //formulas used to calculate pitch and roll using the gForce
}

// Function used to record measuremnets from gyro
void recordGyroRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43); //starting register to read gyro measurements
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //registers 43 - 48
  while(Wire.available()<6);
  gyroX = Wire.read()<<8|Wire.read(); // first two bytes
  gyroY = Wire.read()<<8|Wire.read(); // middle two bytes
  gyroZ = Wire.read()<<8|Wire.read(); //last two bytes
  processGyroData();
}

void processGyroData(){
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotX = gyroX / 131.0;
}

void loop(){
  recordAccelRegisters();
  //delay(10);
  printData();
  pitchCor(pitch);
  rollCor(roll);
  delay(10);
}

//function for printing the acceleration data
void printData(){
  Serial.print("accel X =");
  Serial.print(gForceX);
  Serial.print("g ");
  Serial.print(" accel Y =");
  Serial.print(gForceY);
  Serial.print("g ");
  Serial.print(" accel Z =");
  Serial.print(gForceZ);
  Serial.print("g ");
  Serial.print("pitch = ");
  Serial.print(pitch);
  Serial.print(" roll = ");
  Serial.print(roll);
  Serial.println("");
  //delay(100);
}

//function for correcting for pitch deviations
  void pitchCor(float pitch){
    
    if(pitch>1505.0 || pitch<1500){
      myServo2.writeMicroseconds(3000-pitch);
      delay(10);
    }
}

//function for correcting for roll deviations
  void rollCor( float roll){
  if(roll>1502.0 || roll<1500){
      myServo1.writeMicroseconds(1500+(1500-roll));
      delay(10);
  }
}