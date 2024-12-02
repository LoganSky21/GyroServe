#include<Wire.h>
#include<Servo.h>

long accelX, accelY, accelZ;  
long gyroX, gyroY, gyroZ;
float gForceX, gForceY, gForceZ; 
float rotX, rotY, rotZ;
float pitch, roll, prevP, prevR, rollA, pitchA, rollComp, pitchComp;
float errorP, errorR;
int writeVal1, writeVal2, i=0;
int *pitchCorrect, *rollCorrect;
unsigned long tStart, tStop;
unsigned long tLoop = 0;
int cnt = 0;

Servo myServo1;
Servo myServo2;

void setup(){
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  setupMPU();
  delay(1000);
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
  Serial.print("Setup complete");
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
  gForceY = accelY/16384.0;
  gForceZ = accelZ/16384.0;
  pitch = (atan(gForceX/(sqrt(pow(gForceY, 2)+pow(gForceZ, 2)))))*(2000.0/PI)+1500;
  roll = (atan(-gForceY/gForceZ))*(2000.0/PI)+1500;
  //formulas used to calculate pitch and roll using the gForce
  //pitchA = atan(accelX/accelZ)/2/PI*360;
  //rollA = atan(accelX/accelZ)/2/PI*360;
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
  rotX = gyroX / 131.0; //using gyro and rot will determine the sensitivity
  rotY = gyroY / 131.0;
  rotX = gyroX / 131.0;
}

void compFilter(){
  tStart = millis();
  rollComp= roll*.05 + .95*(rollComp+rotY*tLoop);
  pitchComp= pitch*.05 + .95*(pitchComp+rotX*tLoop);
  cnt = cnt + 1;
  if (cnt==10) {
  cnt=0;
  Serial.print("pitch = ");
  Serial.print(pitch);
  Serial.print(" roll = ");
  Serial.print(roll);
  Serial.print("pitch2 = ");
  Serial.print(pitchComp);
  Serial.print(" roll2 = ");
  Serial.print(rollComp);
  Serial.println("");
  }
  tStop = millis();
  tLoop=(tStop-tStart)*.001;
  
  myServo1.writeMicroseconds(rollComp);
  myServo2.writeMicroseconds(pitchComp); //
  //delay(10);
}

//function for correcting for pitch deviations
void pitchCor(float pitch){
    
    if(pitch>1505.0 || pitch<1500){
      myServo2.writeMicroseconds(3000-pitch); //
      //delay(10);
    }
}

//function for correcting for roll deviations
void rollCor( float roll){
  if(roll>1502.0 || roll<1500){
      myServo1.writeMicroseconds(1500+(1500-roll));
      //delay(10);
  }
}

void loop(){
  recordAccelRegisters();
  recordGyroRegisters();
  compFilter();
 // pitchCor(pitchComp);
 // rollCor(rollComp);
  //delay(10);
}