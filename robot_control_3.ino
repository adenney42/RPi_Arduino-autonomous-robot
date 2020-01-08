

/*
  SCRIPT FOR ARDUINO LEONARDO
  USING:
      HC-SR04 ULTRASONIC SENSOR
      POLOLU qik 2S9V1 DC MOTOR CONTROLLER
      POLOLU MINI MAESTRO 24CH SERVO CONTROLLER
      ITG/MPU 6 AXIS GYROSCOPE ACCELEROMETER

  Maestro Control Center settings:
  Serial mode: UART, fixed baud rate
  Baud rate: 9600
  CRC disabled

  Required connections between Arduino and ITG/MPU (MPU-6050 I2C):

  +5V - Vcc
  GND - Gnd
  SDA - SDA
  SCL - SCL
  Digital Pin 13 - INT

  Required connections between Arduino and ultrasonic:

  +5V - Vcc
  Digital Pin 12 - Trig
  Digital Pin 13 - Echo
  GND - Gnd

  Required connections between Arduino and Polou devices:

      Arduino   qik 2s9v1
  -------------------------
  Digital Pin 8 - TX
  Digital Pin 9 - RX
  Digital Pin 10 - RESET
  Digital Pin 11 - ERROR

      Arduino   mini maestro
  -------------------------
  Digital Pin 2 - TX
  Digital Pin 3 - RX
  Digital Pin 4 - RESET
  Digital Pin 5 - ERROR
*/

#include <SoftwareSerial.h>
#include <PololuQik.h>
#include <PololuMaestro.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

PololuQik2s9v1 qik(8, 9, 10);
SoftwareSerial maestroSerial(2, 3);
MiniMaestro maestro(maestroSerial);
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_WORLDACCEL

const int MAX = 80;           //  set max DC motor speed
const int STEP = 500;         //  set step duration in ms
const byte numChars = 32;     //  maximum length of received serial data
char receivedChars[numChars]; //  an array to store the received data
// defines pins numbers for ultrasonic range finder
#define trigPin 12
#define echoPin 13
// defines variables for ultrasonic range finder
unsigned long duration;
int distance;
int loopCounter = 0;
// defines variables for gyroscope
#define MPU6050_INT_PIN 7
#define MPU6050_INT digitalPinToInterrupt(MPU6050_INT_PIN)

int sensorPin = A1;   // select the analog input pin for the photoresistor

boolean newData = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  maestroSerial.begin(9600);
  maestro.setSpeed(0, 10);
  maestro.setAcceleration(0, 5);
  qik.init();
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(57600);   //  might need to be 9600 for qik board
  delayMicroseconds(1000); 

  readVoltage();    // check initial battery voltage
  
  // initialize all servos and set to rest positions
  maestro.setTarget(0, 6000);
  maestro.setTarget(1, 6000);
  maestro.setTarget(6, 4000);
  maestro.setTarget(12, 8000);
  maestro.setTarget(7, 8000);
  maestro.setTarget(13, 4000);
  maestro.setTarget(8, 4500);
  maestro.setTarget(14, 75 + 00);

  maestro.setSpeed(0, 30);
  maestro.setAcceleration(0, 10);

  //     GYRO CODE
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  while (!Serial); // wait for Leonardo enumeration
  // initialize device
  mpu.initialize();

  // verify connection
  Serial.println(mpu.testConnection() ? F("MPUconnected") : F("MPUfailed"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    pinMode(MPU6050_INT, INPUT);
    attachInterrupt(MPU6050_INT, dmpDataReady, RISING);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  Serial.println("Leo ready");
  readVoltage();
}
  void readVoltage() {
    float volts;
    volts = analogRead(0);
    volts = (volts / 4.092) / 10;
    Serial.print("v");
    Serial.println(volts);
  }

  void utRange() { //                                                  ultrasonic rangefinder code
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    
    // Prints the duration on the Serial Monitor
    //Serial.print("d");
    //Serial.println(duration);
    
    // Calculating the distance
    //distance = duration * 0.034 / 2;   //inches?
    distance = (duration/2) / 29.1;   //centimeters?
    
    // Prints the distance on the Serial Monitor
    Serial.print("d");
    Serial.println(distance);
  }

  void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    // if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();

      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        ndx = 0;
        newData = true;
      }
    }
  }

  void showNewData() {
    if (newData == true) {
      newData = false;
      if (receivedChars[0] == 'm') moveMotors();
      if (receivedChars[0] = 's') moveServos();
    }
  }

  void moveServos() {
    int servoStep = 0;  int servoCh = 99;
    int mult = 1000;
    for (int n = 3; n < 7; n++)     //  conver last four char values to step count for servo
    {
      servoStep += (receivedChars[n] - '0') * mult;
      mult /= 10;   // mult is used to get ones, tens, hundreds and thousands
    }
    if (receivedChars[1] == 'h' && receivedChars[2] == 'p') servoCh = 0;    // head pan, servo 9, ch 0
    if (receivedChars[1] == 'h' && receivedChars[2] == 't') servoCh = 1;    // head tilt, servo 10, ch 1
    if (receivedChars[1] == 'r' && receivedChars[2] == 'f') servoCh = 8;    // right forearm, servo 3, ch 8
    if (receivedChars[1] == 'l' && receivedChars[2] == 'f') servoCh = 14;    // left forearm, servo 8, ch 14
    if (receivedChars[1] == 'r' && receivedChars[2] == 'u') servoCh = 7;    // right upper arm, servo 4, ch 7
    if (receivedChars[1] == 'l' && receivedChars[2] == 'u') servoCh = 13;    // left upper arm, servo 7, ch 13
    if (receivedChars[1] == 'r' && receivedChars[2] == 's') servoCh = 6;    // right shoulder, servo 5, ch 6
    if (receivedChars[1] == 'l' && receivedChars[2] == 's') servoCh = 12;    // left shoulder, servo 6, ch 12

    maestro.setTarget(servoCh, servoStep);
    Serial.println("step");
  }

  void moveMotors() {
    switch (receivedChars[1])
    {
      case 's':     //  stop motors
        qik.setM0Speed(0); qik.setM1Speed(0);
        break;
      case 'f':     //  forward
        qik.setM0Speed(MAX); qik.setM1Speed(MAX);
        delay(STEP);
        qik.setM0Speed(0); qik.setM1Speed(0);
        break;
      case 'b':     //  reverse
        qik.setM0Speed(-MAX); qik.setM1Speed(-MAX);
        delay(STEP);
        qik.setM0Speed(0); qik.setM1Speed(0);
        break;
      case 'r':     //  turn right
        qik.setM0Speed(-MAX); qik.setM1Speed(MAX);
        delay(STEP);
        qik.setM0Speed(0); qik.setM1Speed(0);
        break;
      case 'l':     //  turn left
        qik.setM0Speed(MAX); qik.setM1Speed(-MAX);
        delay(STEP);
        qik.setM0Speed(0); qik.setM1Speed(0);
        break;
    }
    Serial.println("step");
  }

  void loop() {
    Serial.flush();
    delayMicroseconds(10);
    loopCounter += 1;   // count loop iterations for function triggers (voltage read every 100000 iterations)
    
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    if (loopCounter % 2000 == 0) {
      Serial.print("p");
      Serial.println(analogRead(sensorPin));      // read photoresistor (light) value
      //delayMicroseconds(10);
      getGyro();
      delayMicroseconds(10);
      utRange();
      delayMicroseconds(50);
    }
    
    recvWithEndMarker();
    showNewData();
    
    if (loopCounter >= 30000) {
      loopCounter = 0;
      readVoltage();
      delayMicroseconds(10);
    }
  }
  
  void getGyro() {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
  
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      //Serial.println(F("FIFO overflow!"));
  
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      
      
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
  
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print(":");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print(":");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.println(":");
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("acc");
            Serial.print(aaWorld.x);
            Serial.print(":");
            Serial.print(aaWorld.y);
            Serial.print(":");
            Serial.print(aaWorld.z);
            Serial.println(":");
        #endif
    }
  }
