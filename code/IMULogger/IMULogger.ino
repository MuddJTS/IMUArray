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
*/

#include "quaternionFilters.h"
#include "MPU9250.h"


#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

#define I2Cclock 400000
#define I2Cport Wire
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

void setup()
{
  Wire.begin();
  Serial.begin(115200);

  while (!Serial) {};

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c != 0x71) {

    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
  //Serial.println(F("MPU9250 is online..."));

  // Start by performing self test and reporting values
  myIMU.MPU9250SelfTest(myIMU.selfTest);
  /*Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0], 1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1], 1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2], 1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3], 1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4], 1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5], 1); Serial.println("% of factory value");
  */
  // Calibrate gyro and accelerometers, load biases in bias registers
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  // Initialize device for active mode read of acclerometer, gyroscope, and
  // temperature
  //Serial.println("MPU9250 initialized for active data mode....");

  // Read the WHO_AM_I register of the magnetometer, this is a good test of
  // communication
  byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  if (d != 0x48) {
    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

  // Get magnetometer calibration from AK8963 ROM
  myIMU.initAK8963(myIMU.factoryMagCalibration);
  // Initialize device for active mode read of magnetometer
  //Serial.println("AK8963 initialized for active data mode....");

  if (false)
  {
    //  Serial.println("Calibration values: ");
    Serial.print("X-Axis factory sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[0], 2);
    Serial.print("Y-Axis factory sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[1], 2);
    Serial.print("Z-Axis factory sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[2], 2);
  }

  // Get sensor resolutions, only need to do this once
  myIMU.getAres();
  myIMU.getGres();
  myIMU.getMres();

  /*
    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    //    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
    //    delay(2000); // Add delay to see results before serial spew of data
  */
  if (false)
  {
    Serial.println("Magnetometer:");
    Serial.print("X-Axis sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[2], 2);
  }

}

void loop()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
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
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);


  // Serial print and/or display at 0.5 s rate independent of data rates
  myIMU.delt_t = millis() - myIMU.count;

  unsigned long count = 0;
  
  // update LCD once per half-second independent of read rate
  if (true)
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
    myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ()
                                * *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1)
                        * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3)
                        * *(getQ() + 3));
    myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ()
                                * *(getQ() + 2)));
    myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2)
                                * *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1)
                        * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3)
                        * *(getQ() + 3));
    myIMU.pitch *= RAD_TO_DEG;
    myIMU.yaw   *= RAD_TO_DEG;

    // Declination of East Dorm is 11.98 on Mar 2019
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    myIMU.yaw  -= 11.83;
    myIMU.roll *= RAD_TO_DEG;


    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;

    struct IMUData {
      unsigned long timestamp;
      int ax, ay, az;
      int gx, gy, gz;
      int mx, my, mz;
      float yaw, pitch, roll;
    };
    IMUData point;

    point.timestamp = millis();
    point.ax = myIMU.ax * 1000;
    point.ay = myIMU.ay * 1000;
    point.az = myIMU.az * 1000;

    point.gx = myIMU.gx;
    point.gy = myIMU.gy;
    point.gz = myIMU.gz;

    point.mx = myIMU.mx;
    point.my = myIMU.my;
    point.mz = myIMU.mz;

    point.yaw = myIMU.yaw;
    point.pitch = myIMU.pitch;
    point.roll = myIMU.roll;

    //Serial.write(*getQ());
    //Serial.write(*(getQ() + 1));
    //Serial.write(*(getQ() + 2));
    //Serial.write(*(getQ() + 3));
    if (Serial.available()>0) {
      Serial.write((byte*)&point, sizeof(point));
      count++;
      if (count >= 1000) {
        char c = Serial.read();
        count = 0;
      }
    }

  }
}
