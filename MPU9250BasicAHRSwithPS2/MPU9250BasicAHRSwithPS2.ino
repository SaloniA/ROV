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
#define MAG_CALIBRATION false //Set to true to calibrate the mag sensor

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
byte USaddr = 0;
boolean error1 = 0;  //Create a bit to check for catch errors as needed.
int dist;

///////////////////////////////PS2/////////////////////////////////

//ULTRASONIC
const int pwPin1 = 6;
long USsensor, mm, inches;
//ULTRASONIC

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
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
    
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);

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
    if (MAG_CALIBRATION) {
      magcalMPU9250(myIMU.magBias, myIMU.magScale);
      Serial.println("AK8963 mag biases (mG)"); Serial.println(myIMU.magBias[0]); Serial.println(myIMU.magBias[1]); Serial.println(myIMU.magBias[2]); 
      Serial.println("AK8963 mag scale (mG)"); Serial.println(myIMU.magScale[0]); Serial.println(myIMU.magScale[1]); Serial.println(myIMU.magScale[2]); 
      delay(2000); // add delay to see results before serial spew of data
    }
    else {
      //Values from previous calibration
      myIMU.magBias[0] = -61.73;
      myIMU.magBias[1] = 1260.07;
      myIMU.magBias[2] = -1726.72;

      myIMU.magScale[0] = 1.25;
      myIMU.magScale[1] = 1.32;
      myIMU.magScale[2] = 0.69;
    }
    if (SerialDebug)
    {
      Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }

  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    float count = 0;
  }

  ////////////////PS2////////////////////////////////////////

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
  
    leftStickY = ps2x.Analog(PSS_LY);
    if (leftStickY == 0){
    error = 1;
    Serial.println("Controller thinks analog stick is off centre, restart program");
    }
  

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
  
  // ULTRASONIC
  pinMode(pwPin1, INPUT);
  //ULTRASONIC
}

void loop()
{
  read_sensor();
  print_range();
  
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magBias[2];
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
  
  pressure_relative = sealevel(pressure_abs, base_altitude);
  altitude_delta = altitude(pressure_abs , pressure_baseline);
  
//  // Report values via UART
//  
////  Serial.print("Pressure abs (mbar)= ");
//  Serial.print(pressure_abs);
//  Serial.print(" ");
//   
////  Serial.print("Pressure relative (mbar)= ");
//  Serial.print(pressure_relative); 
//  Serial.print(" ");
//  
////  Serial.print("Altitude change (m) = ");
//  Serial.println(altitude_delta); 
//
//    delay(3000);
//    Serial.println("done waiting");
//    EEPROM.put(counter, altitude_delta);
//     counter+=sizeof(double);
//     fcounter = altitude_delta;
//     EEPROM.put(counter, fcounter); 
//     Serial.print(altitude_delta);
//     Serial.print(" ");
//     Serial.println(fcounter);
//     motorDown.writeMicroseconds(minSpeed50);
//     delay(1000);
//     motorDown.writeMicroseconds(deadValue);
//     //delay(3000);
 

///////////////////////////////////PRESSURE SENSOR/////////////////////////

///////////////////////////////US  
//  //Take a range reading 
//  if (!error1){                  //If you had an error starting the sensor there is little point in reading it as you will get old data.
//    delay(100);
//    dist = read_sensor(USaddr);   //reading the sensor will return an integer value -- if this value is 0 there was an error
//    Serial.print("D:");Serial.println(dist);
//  }
///////////////////////////////US

  ///////////////////////////////////////PS2////////////////////////////////
  
  if (error == 1) //Skip loop if no controller found
    return;

  if (type == 0 || type == 1) { //DualShock or unknown controller

    ps2x.read_gamepad(false, vibrate);//Read controller and setup vibration

    int analogRY = 128 - ps2x.Analog(PSS_RY); 
    int analogRX = ps2x.Analog(PSS_RX) - 128; 
    int analogLY = 128 - ps2x.Analog(PSS_LY); 
    int analogLX = ps2x.Analog(PSS_LX) - 128; //NOT USED!!!!
    int rR = sqrt(pow(analogRY, 2) + pow(analogRX, 2))-18;
    int rL = sqrt(pow(analogLY, 2) + pow(analogLX, 2)) - 18;   
    
    Serial.print(analogLY);
    Serial.print(" ");
    Serial.print(analogLX);
    Serial.print(" ");
    Serial.print(analogRY);
    Serial.print(" ");
    Serial.println(analogRX);

    // UP 
    if (analogLY > 18) {
      int speed = (((maxSpeed75 - deadValue)/(128-18))*analogRY) + deadValue;
      moveY(speed, speed);
    }

    //DOWN
    else if (analogLY < -18) {
      int speed = (((deadValue - minSpeed75)/(128-18))*analogRY) + deadValue;
      moveY(speed, speed);
    }

    else {
      moveY(deadValue, deadValue);
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

    moveX(speedL,speedR);
    
    
    
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
    

    //IMU Control
    if (ps2x.Button(PSB_R2)) {
      // set north to 30-33
      if (FilteredYaw < 20){
        motorRight.writeMicroseconds(maxSpeed50);
      }
      else if (FilteredYaw > 30) { //This was left
        motorRight.writeMicroseconds(maxSpeed50);
      }

      else {
        motorRight.writeMicroseconds(minSpeed50);
        motorLeft.writeMicroseconds(minSpeed50);
        delay(1000);
        motorRight.writeMicroseconds(deadValue);
        motorLeft.writeMicroseconds(deadValue);
      }
    }

    if (ps2x.ButtonReleased(PSB_R2)) {
      motorRight.writeMicroseconds(deadValue);
      motorLeft.writeMicroseconds(deadValue);
    }

    // Pressure sensor

    if(ps2x.Button(PSB_TRIANGLE)) {
      EEPROM.put(counter, pressure_abs);
    }

    if (ps2x.ButtonReleased(PSB_TRIANGLE)) {
      counter++;
    }

    if(ps2x.Button(PSB_SQUARE)) {
      
      EEPROM.put(counter, inches);
    }
    
    if (ps2x.ButtonReleased(PSB_SQUARE)) {
            counter++; 
    }

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

//CALIBRATION
void magcalMPU9250(float * dest1, float * dest2) 
 {
   uint16_t ii = 0, sample_count = 0;
   int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
   int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
  
   Serial.println("Mag Calibration: Wave device in a figure eight until done!");
   delay(4000);
  
  // shoot for ~fifteen seconds of mag data
  /*if(myIMU.Mmode == 0x02)*/ sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
  //if(myIMU.Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
  for(ii = 0; ii < sample_count; ii++) {
    myIMU.readMagData(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    /*if(myIMU.Mmode == 0x02)*/ delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    //if(myIMU.Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
  }
  
  
  // Get hard iron correction
   mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
   mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
   mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
  
   dest1[0] = (float) mag_bias[0]*myIMU.mRes*myIMU.magCalibration[0];  // save mag biases in G for main program
   dest1[1] = (float) mag_bias[1]*myIMU.mRes*myIMU.magCalibration[1];   
   dest1[2] = (float) mag_bias[2]*myIMU.mRes*myIMU.magCalibration[2];  
     
  // Get soft iron correction estimate
   mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
   mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
   mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
  
   float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
   avg_rad /= 3.0;
  
   dest2[0] = avg_rad/((float)mag_scale[0]);
   dest2[1] = avg_rad/((float)mag_scale[1]);
   dest2[2] = avg_rad/((float)mag_scale[2]);
  
   Serial.println("Mag Calibration done!");
 }

void read_sensor (){
  USsensor = pulseIn(pwPin1, HIGH);
  mm = USsensor/5.8;
  inches = mm/25.4;
}
 
void print_range(){
  //Serial.print("S1 = ");
  //Serial.print(mm);
  //Serial.print(" ");
  //Serial.println(inches);
}
 
void moveY(int speedU, int speedD){
     motorUp.writeMicroseconds(speedU);
     motorDown.writeMicroseconds(speedD);
}
 
void moveX(int speedL, int speedR){
     motorLeft.writeMicroseconds(speedL);
     motorRight.writeMicroseconds(speedR);  
}
