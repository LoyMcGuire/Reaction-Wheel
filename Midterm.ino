/*
 * References: MPU 6050: https://github.com/VRomanov89/EEEnthusiast/blob/master/MPU-6050%20Implementation/MPU6050_Implementation/MPU6050_Implementation.ino  2/5/17
 *                       http://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf   2/5/17
 *             Motor:    https://www.youtube.com/watch?v=Kp0aSLpxgEE   3/15/17
*/
#include <Wire.h>   // For I2C communication
#include<Servo.h>   //Using servo library to control ESC

//Place the button input on the digital 0 pin
#define BUTTON_PIN 0

//Variables for finding the orientation
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

//Variables for calibrating
int timer = 0;
float X0, Y0, Z0, calY;
boolean button_pressed;

//Varibales for setting the motor speed
Servo myservo;
int motorSpeed = 48;

void setup() {
  Serial.begin(9600);   // Initializes to serial port at 9600 bits/sec
  Wire.begin();   // Needed to initialize I2C communications
  pinMode(BUTTON_PIN, INPUT);   //Set BUTTON_PIN as an input
  digitalWrite(BUTTON_PIN, HIGH);   // connect internal pull-up
  setupMPU();   //Initalize the MPU 6050
  myservo.attach(8);    //Set the digital 8 pin as the signal for the motor
  delay(1);
  myservo.write(10);
}

void loop() {
  button_pressed = handle_button();
  recordAccelRegisters();
  recordGyroRegisters();
  calibrateUpright();
  motor();
  //Printing info to the Serial port for debugging
  Serial.print(button_pressed);
  Serial.print("   ");
  Serial.print(timer);
  Serial.print("   ");
  Serial.print(gForceX);
  Serial.print("   ");
  Serial.print(gForceY);
  Serial.print("   ");
  Serial.print(gForceZ);
  Serial.print("   ");
  Serial.print(calY);
  Serial.print("   ");
  Serial.println(motorSpeed);
  delay(10);
}

void setupMPU() {
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
                                     // One MPU = AC0 low, Multiple MPU = AC0 high
  Wire.write(0x6B); //Accessing the register 6B - Power Management
  Wire.write(0b00000000); //Setting SLEEP register to 0
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

//Convert the readings into acceleration unit (G force)
void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

//Convert the readings into angular velocity (Degrees/second)
void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

//Reads when the button is pressed
boolean handle_button() {
  int button_pressed = !digitalRead(BUTTON_PIN); // pin low -> pressed
  return button_pressed;
}

//Calibrate to your resting upright position
void calibrateUpright() {
  //Initialize calibration
  if(button_pressed) {
    timer = 0;
    X0 = 0;
    Y0 = 0;
    Z0 = 0;
    return;
  }
  //Sum the next 5 gForce readings
  if(timer < 5) {
    X0 += gForceX;
    Y0 += gForceY;
    Z0 += gForceZ;
    timer++;
    return;
  }
  //Divide by 5 to find the average resting gForce
  else if(timer == 5) {
    X0 = X0/5;
    Y0 = Y0/5;
    Z0 = Z0/5;
    timer++;
    return;
  } else {
    return;
  }
}

//Set the motor speed
void motor() {
  //Find the calibrated Y gForce
  calY = gForceY - Y0;
  //MotorSpeed is dependent on both the angle it is at and it's angular velocity
  //0.35 is the G force I wanted the motor to reach max speed at
  //250 comes from the full range of values that the gyroZ could be
  motorSpeed = 48 + 4*calY*(1/0.35) - 8*gyroZ*(1/250);
  //Set a limit for safety
  if(motorSpeed > 57) {
    motorSpeed = 60;
  }
  myservo.write(motorSpeed);
}

