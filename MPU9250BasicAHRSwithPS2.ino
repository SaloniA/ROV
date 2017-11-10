/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND

 Analog stick values
 RX: 110-145
 RY: 110-145
 LX: 115-140
 LY: 115-140
 */

#include <AutoPID.h>

#include <PID_v1.h>

#include <EEPROM.h>
int EEsize = 4096; 

#include "quaternionFilters.h"
#include "MPU9250.h"

#include <Servo.h>
#include <PS2X_lib.h>

#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

MPU9250 myIMU;
float FilteredYaw = 0;
float FilteredPitch = 0;
float FilteredRoll = 0;

float exponentialFilter(float w, float previous, float newValue) {
  return w*newValue + (1-w)*previous;
}

//////////////////////////////////PRESSURE SENSOR////////////////////
// Begin class with selected address
// available addresses (selected by jumper on board) 
// default is ADDRESS_HIGH

//  ADDRESS_HIGH = 0x76
//  ADDRESS_LOW  = 0x77

MS5803 sensor(ADDRESS_HIGH);

//Create variables to store results
float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;

// Create Variable to store altitude in (m) for calculations;
double base_altitude = 1655.0; // Altitude of SparkFun's HQ in Boulder, CO. in (m)
//////////////////////////////////PRESSURE SENSOR//////////////////////

//////////////////////////////////PS2////////////////////////////////
//Create motor classes

Servo motorUp;
Servo motorDown;
Servo motorLeft;
Servo motorRight;

//Create PS2 controller class
PS2X ps2x;
//PWM pins

int motorUpPin = 2; 
int motorDownPin = 3;
int motorLeftPin = 4;
int motorRightPin = 5;

//Error code for the various problems that may occur with the PS2 remote
int error = 0;
//Initialization values for PS2 analog sticks
int leftStickY = 0;
int rightStickY = 0;
//Placeholder value used to write to motors
int value = 0;
//Used to identify type of controller
byte type = 0;
//Used for activating vibration on controller
byte vibrate = 0;
float deadValue = 1460, maxValue = 2399, minValue = 770, maxSpeed50 = 1600, minSpeed50 = 1300, minSpeed20 = 1355, maxSpeed20 = 1511;

String incomingByte;
char curLetter;
int inputSpeed = 255, maxSpeed = 1715, minSpeed = 1205, testSpeed = 1590, maxSpeed75 = 1652, minSpeed75 = 1269;

int speedL = deadValue, speedR = deadValue;
int counter = 0, top = 0, bottom = 0;
double fcounter = 0;

///////////////////////////////PS2/////////////////////////////////


void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
//  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
//  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
//    Serial.print("x-axis self test: acceleration trim within : ");
//    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
//    Serial.print("y-axis self test: acceleration trim within : ");
//    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
//    Serial.print("z-axis self test: acceleration trim within : ");
//    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
//    Serial.print("x-axis self test: gyration trim within : ");
//    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
//    Serial.print("y-axis self test: gyration trim within : ");
//    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
//    Serial.print("z-axis self test: gyration trim within : ");
//    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
//    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
//    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
//    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
//    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
//      Serial.print("X-Axis sensitivity adjustment value ");
//      Serial.println(myIMU.magCalibration[0], 2);
//      Serial.print("Y-Axis sensitivity adjustment value ");
//      Serial.println(myIMU.magCalibration[1], 2);
//      Serial.print("Z-Axis sensitivity adjustment value ");
//      Serial.println(myIMU.magCalibration[2], 2);
    }

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    float count = 0;
  }

  ////////////////PS2////////////////////////////////////////

    //Begin serial communications
  //Setting up motors with their minimum and maximum us values
//  motorPort.attach(motorPortPin, minValue, maxValue);
//  motorStarboard.attach(motorStarboardPin, minValue, maxValue);
//  motorVertical.attach(motorVerticalPin, minValue, maxValue);

  motorLeft.attach(motorLeftPin, minValue, maxValue);
  motorRight.attach(motorRightPin, minValue, maxValue);
  motorUp.attach(motorUpPin, minValue, maxValue);
  motorDown.attach(motorDownPin, minValue, maxValue);
  
  //Initializing motors at dead value (1460us)
  motorUp.writeMicroseconds(deadValue);
  motorDown.writeMicroseconds(deadValue);
  motorLeft.writeMicroseconds(deadValue);
  motorRight.writeMicroseconds(deadValue);
  delay (2000);

  //Setup pins and settings for PS2 controller:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(10, 11, 12, 13, false, false);

  //If you experience issues with the port motor value always being set to 0 despite the stick position, activate this code
  /*
    leftStickY = ps2x.Analog(PSS_LY);
    if (leftStickY == 0){
    error = 1;
    Serial.println("Controller thinks analog stick is off centre, restart program");
    }
  */

  if (error == 0) {
    Serial.println("Found Controller, configured successful");
  }


  else if (error == 1)
    Serial.println("No controller found, check wiring");

  else if (error == 2)
    Serial.println("Controller found but not accepting commands");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it.");

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      //There isn't any control code in place for a GuitarHero controller but feel free to make some ;)
      Serial.println("GuitarHero Controller Found");
      break;
  }

  ////////////////////////////PS2////////////////////////////

  ////////////////////////////PRESSURE SENSOR///////////////
  //Retrieve calibration constants for conversion math.
    sensor.reset();
    sensor.begin();
    
    pressure_baseline = sensor.getPressure(ADC_4096);
  ////////////////////////////PRESSURE SENSOR////////////////

  delay(60000);
}

void loop()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      myIMU.count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw   -= 8.5;
      myIMU.roll  *= RAD_TO_DEG;

      FilteredYaw = exponentialFilter(0.2, myIMU.yaw, FilteredYaw);
 ///     Serial.print(FilteredYaw, 2);
 ///     Serial.print(" ");

      FilteredPitch = exponentialFilter(0.2, myIMU.pitch, FilteredPitch);
 ///     Serial.print(FilteredPitch, 2);
 ///     Serial.print(" ");

      FilteredRoll = exponentialFilter(0.2, myIMU.roll, FilteredRoll);
 ///     Serial.println(FilteredRoll, 2);
   
      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)

///////////////////////////////////PRESSURE SENSOR/////////////////////////
// To measure to higher degrees of precision use the following sensor settings:
  // ADC_256 
  // ADC_512 
  // ADC_1024
  // ADC_2048
  // ADC_4096
    
  // Read temperature from the sensor in deg C. This operation takes about 
 // temperature_c = sensor.getTemperature(CELSIUS, ADC_512);
  
  // Read temperature from the sensor in deg F. Converting
  // to Fahrenheit is not internal to the sensor.
  // Additional math is done to convert a Celsius reading.
//  temperature_f = sensor.getTemperature(FAHRENHEIT, ADC_512);
  
  // Read pressure from the sensor in mbar.
  pressure_abs = sensor.getPressure(ADC_4096);
  
  // Let's do something interesting with our data.
  
  // Convert abs pressure with the help of altitude into relative pressure
  // This is used in Weather stations.
  pressure_relative = sealevel(pressure_abs, base_altitude);
  
  // Taking our baseline pressure at the beginning we can find an approximate
  // change in altitude based on the differences in pressure.   
  altitude_delta = altitude(pressure_abs , pressure_baseline);
  
  // Report values via UART
  
//  Serial.print("Pressure abs (mbar)= ");
  Serial.print(pressure_abs);
  Serial.print(" ");
   
//  Serial.print("Pressure relative (mbar)= ");
  Serial.print(pressure_relative); 
  Serial.print(" ");
  
//  Serial.print("Altitude change (m) = ");
  Serial.println(altitude_delta); 

    delay(3000);
    Serial.println("done waiting");
    EEPROM.put(counter, altitude_delta);
     counter+=sizeof(double);
     fcounter = altitude_delta;
     EEPROM.put(counter, fcounter); 
     Serial.print(altitude_delta);
     Serial.print(" ");
     Serial.println(fcounter);
     motorDown.writeMicroseconds(minSpeed50);
     delay(1000);
     motorDown.writeMicroseconds(deadValue);
     //delay(3000);
 

///////////////////////////////////PRESSURE SENSOR/////////////////////////

  ///////////////////////////////////////PS2////////////////////////////////
  
  if (error == 1) //Skip loop if no controller found
    return;

  if (type == 0 || type == 1) { //DualShock or unknown controller

    ps2x.read_gamepad(false, vibrate);//Read controller and setup vibration

    motorLeft.writeMicroseconds(deadValue);
    motorRight.writeMicroseconds(deadValue);
//
//    Serial.print(ps2x.Analog(PSS_RX));
//    Serial.print(" ");
//    
//    Serial.print(ps2x.Analog(PSS_RY));
//    Serial.print(" ");
//    
//    Serial.print(ps2x.Analog(PSS_LX));
//    Serial.print(" "); 
//    
//    Serial.println(ps2x.Analog(PSS_LY));
    int analogRY = 128 - ps2x.Analog(PSS_RY); 
    int analogRX = ps2x.Analog(PSS_RX) - 128; 
    int analogLY = 128 - ps2x.Analog(PSS_LY); 
    int analogLX = ps2x.Analog(PSS_LX) - 128; //NOT USED!!!!
    int rR = sqrt(pow(analogRY, 2) + pow(analogRX, 2))-18;
    int rL = sqrt(pow(analogLY, 2) + pow(analogLX, 2)) - 18;   
    
    //Forwards
    if (ps2x.Button(PSB_PAD_UP)) {
      motorLeft.writeMicroseconds(minSpeed50);
      motorRight.writeMicroseconds(maxSpeed50);
    } 

    if (ps2x.ButtonReleased(PSB_PAD_UP)) {
      motorLeft.writeMicroseconds(deadValue);
      motorRight.writeMicroseconds(deadValue);
    }

    
    //Forwards MAX
    if (ps2x.Button(PSB_L2)) {
      motorLeft.writeMicroseconds(minSpeed);
      motorRight.writeMicroseconds(maxSpeed);
    } 

    if (ps2x.ButtonReleased(PSB_L2)) {
      motorLeft.writeMicroseconds(deadValue);
      motorRight.writeMicroseconds(deadValue);
    }

    // UP 
    if (analogLY > 18) {
        int speed = (((maxSpeed75 - deadValue)/(128-18))*analogRY) + deadValue;
      motorDown.writeMicroseconds(speed);
      motorUp.writeMicroseconds(speed);
    }

    //DOWN
    if (analogLY < -18) {
        int speed = (((deadValue - minSpeed75)/(128-18))*analogRY) + deadValue;
      motorDown.writeMicroseconds(speed);
      motorUp.writeMicroseconds(speed);
    }

    if (rR > 0) {
      if (analogRY > 0 ) {
        if (analogRX > -18 && analogRX < 18) {
          speedL = (((deadValue - minSpeed75)/(128-18))*(analogRY)) + deadValue;
          speedR = -1*(((deadValue - minSpeed75)/(128-18))*(analogRY)) + deadValue;
        }
        else if (analogRX > 18) {

          if (rR - 2*analogRX < 0) {
            speedL = deadValue;
            speedR = -1*(((deadValue - minSpeed75)/(128-18))*(rR)) + deadValue;
          }
          else {
            speedL = (((deadValue - minSpeed75)/(128-18))*(rR - 2*analogRX)) + deadValue;
            speedR = -1*(((deadValue - minSpeed75)/(128-18))*(rR)) + deadValue;
          }
        }
        else {
          if (rR + 2*analogRX < 0) {
            speedL = (((deadValue - minSpeed75)/(128-18))*(rR)) + deadValue;
            speedR = deadValue;
          } 
          else {
            speedL = (((deadValue - minSpeed75)/(128-18))*(rR)) + deadValue;
            speedR = -1*(((deadValue - minSpeed75)/(128-18))*(rR + 2*analogRX)) + deadValue;
          }
        }
      } 
      else {
        if (analogRX > -18 && analogRX < 18) {
          speedL = (((deadValue - minSpeed75)/(128-18))*(analogRY)) + deadValue;
          speedR = -1*(((deadValue - minSpeed75)/(128-18))*(analogRY)) + deadValue;
        }
        else if (analogRX > 18) {
          if (-1*rR + 2*analogRX > 0) {
            speedL = (((deadValue - minSpeed75)/(128-18))*(-1 * rR)) + deadValue;
            speedR = deadValue;           
          }

          else {
            speedL = (((deadValue - minSpeed75)/(128-18))*(-1 * rR)) + deadValue;
            speedR = -1*(((deadValue - minSpeed75)/(128-18))*(-1*rR + 2*analogRX)) + deadValue;
          }
        }
        else {

          if (-1 * rR - 2*analogRX > 0) {
            speedL = deadValue;
            speedR = -1*(((deadValue - minSpeed75)/(128-18))*(-1 * rR)) + deadValue;
          }

          else {
            speedL = (((deadValue - minSpeed75)/(128-18))*(-1 * rR - 2*analogRX)) + deadValue;
            speedR = -1*(((deadValue - minSpeed75)/(128-18))*(-1 * rR)) + deadValue;
          }
        }
       }
    }
    else {
      speedL = deadValue;
      speedR = deadValue;
    }

    motorLeft.writeMicroseconds(speedL);
    motorRight.writeMicroseconds(speedR);
    
//    Serial.print(rR);
//    Serial.print(" ");
//
//        Serial.print(analogRX);
//    Serial.print(" ");
//
//        Serial.print(analogRY);
//    Serial.print(" ");
//    
//    Serial.print(speedL-deadValue);
//    Serial.print(" ");
//    
//    Serial.println(speedR-deadValue);
    

    //Backwards 
    if (ps2x.Button(PSB_PAD_DOWN)) {
      motorLeft.writeMicroseconds(maxSpeed50);
      motorRight.writeMicroseconds(minSpeed50);
    } 

    if (ps2x.ButtonReleased(PSB_PAD_DOWN)) {
      motorLeft.writeMicroseconds(deadValue);
      motorRight.writeMicroseconds(deadValue);
    }
     //Up
    if (ps2x.Button(PSB_L1)) {
      motorDown.writeMicroseconds(minSpeed50);
      motorUp.writeMicroseconds(minSpeed50);
      // Are the up and down propellers going in the same direction?
    } 

    if (ps2x.ButtonReleased(PSB_L1)) {
      motorDown.writeMicroseconds(deadValue);
      motorUp.writeMicroseconds(deadValue);
    }

    //Down
    if (ps2x.Button(PSB_R1)) {
      motorDown.writeMicroseconds(maxSpeed50);
      motorUp.writeMicroseconds(maxSpeed50);
      // Are the up and down propellers going in the same direction? 
    } 

    if (ps2x.ButtonReleased(PSB_R1)) {
      motorDown.writeMicroseconds(deadValue);
      motorUp.writeMicroseconds(deadValue);
    }

    //Left
    if (ps2x.Button(PSB_PAD_LEFT)) {
      motorRight.writeMicroseconds(maxSpeed50);
    } 

    if (ps2x.ButtonReleased(PSB_PAD_LEFT)) {
      motorRight.writeMicroseconds(deadValue);
    }

    //Right
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      motorLeft.writeMicroseconds(minSpeed50);
    } 

    if (ps2x.ButtonReleased(PSB_PAD_RIGHT)) {
      motorLeft.writeMicroseconds(deadValue);
    }

    //IMU Control
    if (ps2x.Button(PSB_R2)) {
      // set north to 30-33
      if (FilteredYaw < 20){
        motorRight.writeMicroseconds(maxSpeed50);
//        delay(50);
//        motorRight.writeMicroseconds(deadValue);
      }
      else if (FilteredYaw > 30) { //This was left
        motorRight.writeMicroseconds(maxSpeed50);
//        delay(50);
//        motorRight.writeMicroseconds(deadValue);
      }

      else {
        motorRight.writeMicroseconds(deadValue);
        motorLeft.writeMicroseconds(deadValue);
//        delay(50);
      }
    }

    if (ps2x.ButtonReleased(PSB_R2)) {
      motorRight.writeMicroseconds(deadValue);
      motorLeft.writeMicroseconds(deadValue);
    }

    // Pressure sensor

    if(ps2x.Button(PSB_TRIANGLE)) {
      delay(3000);
      EEPROM.write(counter, altitude_delta);
    }

    if (ps2x.ButtonReleased(PSB_TRIANGLE)) {
      counter++;
    }

    if (ps2x.Button(PSB_CIRCLE)) {
      for (int j = 0; j < EEPROM.length(); j++){
        Serial.println(EEPROM.read(j));
        if (j == 0) {
          top = EEPROM.read(j);
        }
        if (j==1) {
          bottom = EEPROM.read (j);
        }
        
      }
    }

    if (ps2x.Button(PSB_SQUARE)) {
        if (altitude_delta > (top - 1000)) { //DOWN
            motorDown.writeMicroseconds(maxSpeed50);
            motorUp.writeMicroseconds(maxSpeed50);
        }

        else if (altitude_delta < (bottom + 1000)) { //UP
            motorDown.writeMicroseconds(minSpeed50);
            motorUp.writeMicroseconds(minSpeed50);
        }
    }
    if (ps2x.ButtonReleased(PSB_SQUARE)) {
            motorDown.writeMicroseconds(deadValue);
            motorUp.writeMicroseconds(deadValue); 
    }


//    if (altitude_delta > (top - 1000)) { //DOWN
//        motorDown.writeMicroseconds(maxSpeed50);
//        motorUp.writeMicroseconds(maxSpeed50);
//    }
//
//    else if (altitude_delta < (bottom + 1000)) { //UP
//        motorDown.writeMicroseconds(minSpeed50);
//        motorUp.writeMicroseconds(minSpeed50);
//    }

  }

 ///////////////////////////////////////PS2////////////////////////////////
  // delay(1000); ???? pressure sensor, imu
}




// HELPERS

double sealevel(double P, double A)
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
  return(P/pow(1-(A/44330.0),5.255));
}


double altitude(double P, double P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
  return(44330.0*(1-pow(P/P0,1/5.255)));
}
