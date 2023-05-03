/* 
RobotRoverFinalProject_FastSerial2023.ino

This Arduino code handles fast serial communication between MATLAB and Arduino
by parsing I/O data as character strings and passing them over serial at a high
baudrate.

Data is received over seria from MATLAB in the form: '<int, int, int>' e.g., <120,60,46>

Data is sent over serial to MATLAB in the form: '<int, int, int, int, int, int, int, int>'
e.g., <0, 12, 239, 242, 87, 121, 356, 1000>

kmbanisi 2023, RevA

*/


#include <stdlib.h>
#include <string.h>
#include <Servo.h>
#include <Arduino_LSM9DS1.h>


/* initialize I/O pins */
const int velocityServoPin = 3;
const int steerServoPin = 5;
const int panServoPin = 6;
const int numSensors = 8;
int analogPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

/* initialize data variables */
const int numChars = 16; 
const int numActuators = 3; 
int sensorData[numSensors] = { 0 };   // int array to store analog sensor readings
char recvChars[numChars];             // char array to store cmd list sent over serial from MATLAB
int cmdArray[numActuators] = { 0.0 }; // int array to store parsed cmd list to be written to actuators

/* initialize actuator servo objects */
Servo panServo, steerServo, velocityServo;

/* initialize helper variables to control parsing and others */
boolean newData = false;             
boolean recvCmd = false;

/* initialize position and orientation variables */
float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;

/* initialize IMU variables */
sensors_event_t gyroEvent;
sensors_event_t accelEvent;
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;
float heading, pitch, roll;

/*--------------------------------------------------------------------------
 * SETUP FUNCTION only runs once
 --------------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  Serial.println("<Arduino is ready>"); // indicate we are ready

  /* set servo pins to servo objects */
  panServo.attach(panServoPin);
  steerServo.attach(steerServoPin);
  velocityServo.attach(velocityServoPin);

  /* command servos to 'home' or zero position */
  panServo.write(90);
  steerServo.write(90);
  velocityServo.write(94);

  /* initialize the IMU */
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

/*--------------------------------------------------------------------------
 * LOOP FUNCTION runs over and over
 --------------------------------------------------------------------------*/
void loop() {
  recvData();           // reads data (cmd list) from MATLAB sent over serial
  writeToActuators();   // writes cmd list to actuators (servos)
  readSensors();        // reads sensor data from analog sensors
  readIMU();            // reads data from the onboard IMU
  updateOrientation();   // updates robot orientation
  sendData();           // sends sensor data over serial to MATLAB

  delay(100);
}


/*--------------------------------------------------------------------------
 * RECVDATA function
 * This function reads character string data sent over serial from MATLAB.
 * The data is parsed into an int array (cmdArray) by calling the parseString
 * function.
 --------------------------------------------------------------------------*/
void recvData() {
  static boolean recvInProgress = false;
  static byte ndx = 1;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  /* receive data over Serial */
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    recvChars[0] = '<';
    if (recvInProgress == true) {   // wait until you see a startMarker
      if (rc != endMarker) {        // store chars until you see the endMarker
        recvChars[ndx] = rc;
        ndx++;
      }
      else {                      // once endMarker is seen, reset process and terminate data string
        recvChars[ndx] = '>';     // terminate the string
        recvChars[ndx+1] = '\0'; 
        recvInProgress = false;
        ndx = 1;
        newData = true;           // indicate that data packet is complete
        recvCmd = true;           // indicate that a new complete cmd list was received from MATLAB
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }

  /* parse the string into int array */
  parseString();   
}


/*--------------------------------------------------------------------------
 * SENDDATA function
 * This function reads analog sensor data from the sensors, parsing them into a
 * char array with start and end markers and then sends the array over serial to 
 * MATLAB
 --------------------------------------------------------------------------*/
void sendData(){
  if (newData == true){           // check to ensure a new cmd list has been received from MATLAB
        
    char sendChars[32] = "<";     // creates a char array to store sensor data

    /* iterates over all sensors and stores current reading into sendChars */
    for(int i = 0; i<numSensors; i++ ) {
        char tmp[8];
        sprintf(tmp, "%d", sensorData[i]);
        strcat(sendChars, tmp);
      if (i < numSensors-1)
        strcat(sendChars, ",");
    }

      /* add robot's position and orientation to the sendChars array */
    char tmpX[8];
    sprintf(tmpX, ",%d", robotX);
    strcat(sendChars, tmpX);
    char tmpY[8];
    sprintf(tmpY, ",%d", robotY);
    strcat(sendChars, tmpY);
    char tmpTheta[8];
    sprintf(tmpTheta, ",%d", robotTheta);
    strcat(sendChars, tmpTheta);

    /* adds a terminator character to the end of the char array */
    char endChar[2] = ">";
    strcat(sendChars, endChar);

    /* sends char array representing sensor readings over serial to MATLAB */
    Serial.println(sendChars);
    newData = false;
  }
}


/*--------------------------------------------------------------------------
 * READSENSORS function
 * This is a simple function that calls the analogRead function for all defined
 * analog sensor pins and stores data in the sensorData variable
 --------------------------------------------------------------------------*/
void readSensors(){
  for(int i = 0; i<numSensors; i++ )
    sensorData[i] = analogRead(analogPins[i]);
}


/*--------------------------------------------------------------------------
 * WRITETOACTUATORS function
 * This is a simple function that writes servo commands to the servo objects.
 * We included a condition to ensure that only the current command is sent over
 * to the actuator. This ensures that if no command is being sent, the robot
 * will come to a halt.
 --------------------------------------------------------------------------*/
void writeToActuators(){
  if (recvCmd == true) {
    panServo.write(cmdArray[0]);
    steerServo.write(cmdArray[1]);
    velocityServo.write(cmdArray[2]);
  }
  else {    // if no new cmd is received, send zero position command to servos
    /* // This is commented out so that the rover doesnt reset between data being sent
    panServo.write(90);
    steerServo.write(90);
    velocityServo.write(90);
    */
  }
  recvCmd = false;
}


/*--------------------------------------------------------------------------
 * PARSESTRING function
 * This function converts a character array into an int array using the comma
 * delimiter
 --------------------------------------------------------------------------*/
void parseString(){
  int i, j = 0;
  char buffer[50];        // create a buffer char array with large capacity
  int bufferIndex = 0;
  
  int num_chars = sizeof(recvChars)/sizeof(recvChars[0]);

  /* iterate over all characters in the recvChars array and parse using the
    comma delimiter*/
  for (i = 0; i < num_chars; i++) {
      if (recvChars[i] == '<') {
          continue;
      }
      if (recvChars[i] == ',' || recvChars[i] == '>') {
          buffer[bufferIndex] = '\0';
          cmdArray[j] = atoi(buffer); // convert char array to int
          j++;
          bufferIndex = 0;
      } else {
          buffer[bufferIndex] = recvChars[i];
          bufferIndex++;
      }
  }
}

/*--------------------------------------------------------------------------
 * READIMU function
 * This function reads the gyro and accelerometer data from the onboard IMU.
 --------------------------------------------------------------------------*/
void readIMU() {
  IMU.readGyro(&gyroEvent);
  gyroX = gyroEvent.gyro.x;
  gyroY = gyroEvent.gyro.y;
  gyroZ = gyroEvent.gyro.z;

  IMU.readAccelerometer(&accelEvent);
  accelX = accelEvent.acceleration.x;
  accelY = accelEvent.acceleration.y;
  accelZ = accelEvent.acceleration.z;

  /* calculate heading, pitch, and roll using the accelerometer data */
  roll = atan2(-accelX, accelZ) * 180.0 / PI;
  pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;

  /* calculate heading using the gyro data */
  float deltaT = 0.1;  // time between readings in seconds
  heading += gyroZ * deltaT;
  if (heading > 360.0) {
    heading -= 360.0;
  }
  else if (heading < 0.0) {
    heading += 360.0;
  }
}


/*--------------------------------------------------------------------------
 * UPDATEORIENTATION function
 * This function updates the robot's orientation based on the IMU data.
 --------------------------------------------------------------------------*/
void updateOrientation() {
  float dX = 0.0;
  float dY = 0.0;
  float dTheta = 0.0;

  /* calculate change in position and orientation */
  dTheta = gyroZ * 0.1;  // time between readings in seconds
  robotTheta += dTheta;

  /* calculate the X and Y components of the robot's displacement */
  dX = sin(robotTheta * PI / 180.0) * accelY * 0.1;  // time between readings in seconds
  dY = cos(robotTheta * PI / 180.0) * accelY * 0.1;  // time between readings in seconds

  /* update the robot's position */
  robotX += dX;
  robotY += dY;
}

