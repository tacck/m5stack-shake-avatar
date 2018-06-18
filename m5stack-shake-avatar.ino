#include <M5Stack.h>

#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"
MPU9250 IMU;

#include "Avatar.h"
using namespace m5avatar;

class MyFace : public Face
{
  public:
    MyFace()
      : Face(new Mouth(50, 60, 60, 20),
             new Eye(20, false),
             new Eye(20, true),
             new Eyeblow(32, 0, false),
             new Eyeblow(32, 0, true))
    {}
};

Avatar *avatar;

Face* faces[2];
const int facesSize = sizeof(faces) / sizeof(Face*);
int faceIdx = 0;

const Expression expressions[] = {
  Expression::Neutral
};
const int expressionsSize = sizeof(expressions) / sizeof(Expression);
int idx = 0;

ColorPalette* cps[2];
const int cpsSize = sizeof(cps) / sizeof(ColorPalette*);
int cpsIdx = 0;

void setup()
{
  M5.begin();
  Wire.begin();

  M5.Lcd.setBrightness(30);
  M5.Lcd.clear();

  // MPU9250
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  // if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    IMU.MPU9250SelfTest(IMU.SelfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

    IMU.initMPU9250();

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    // Get magnetometer calibration from AK8963 ROM
    IMU.initAK8963(IMU.magCalibration);

  } // if (c == 0x71)

  updateIMU();

  // m5stack-avatar
  avatar = new Avatar();

  faces[0] = avatar->getFace();
  faces[1] = new MyFace();

  cps[0] = new ColorPalette();
  cps[1] = new ColorPalette();
  cps[1]->set(COLOR_PRIMARY, TFT_BLACK);
  cps[1]->set(COLOR_BACKGROUND, TFT_WHITE);

  avatar->init();
  avatar->setColorPalette(*cps[0]);
}

void loop()
{
  delay(20);
  updateIMU();

  M5.update();

  if (isShake())
  {
    avatar->setColorPalette(*cps[1]);
    avatar->setFace(faces[1]);
  }
  else
  {
    avatar->setColorPalette(*cps[0]);
    avatar->setFace(faces[0]);
  }
}

void updateIMU()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
    IMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;

    IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
    IMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    IMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    IMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    IMU.magbias[2] = +125.;

    IMU.mx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] -
             IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] -
             IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] -
             IMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  IMU.updateTime();

  MahonyQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx * DEG_TO_RAD,
                         IMU.gy * DEG_TO_RAD, IMU.gz * DEG_TO_RAD, IMU.my,
                         IMU.mx, IMU.mz, IMU.deltat);

  IMU.delt_t = millis() - IMU.count;

  if (IMU.delt_t > 100)
  {
    Serial.print("ax = "); Serial.print((int)1000 * IMU.ax);
    Serial.print(" ay = "); Serial.print((int)1000 * IMU.ay);
    Serial.print(" az = "); Serial.print((int)1000 * IMU.az);
    Serial.println(" mg");

    Serial.print("gx = "); Serial.print( IMU.gx, 2);
    Serial.print(" gy = "); Serial.print( IMU.gy, 2);
    Serial.print(" gz = "); Serial.print( IMU.gz, 2);
    Serial.println(" deg/s");

    Serial.print("mx = "); Serial.print( (int)IMU.mx );
    Serial.print(" my = "); Serial.print( (int)IMU.my );
    Serial.print(" mz = "); Serial.print( (int)IMU.mz );
    Serial.println(" mG");

    Serial.print("q0 = "); Serial.print(*getQ());
    Serial.print(" qx = "); Serial.print(*(getQ() + 1));
    Serial.print(" qy = "); Serial.print(*(getQ() + 2));
    Serial.print(" qz = "); Serial.println(*(getQ() + 3));

    IMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                              *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                      - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
    IMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                              *(getQ() + 2)));
    IMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                              *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                      - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
    IMU.pitch *= RAD_TO_DEG;
    IMU.yaw   *= RAD_TO_DEG;
    IMU.yaw   -= 8.5;
    IMU.roll  *= RAD_TO_DEG;

    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(IMU.yaw, 2);
    Serial.print(", ");
    Serial.print(IMU.pitch, 2);
    Serial.print(", ");
    Serial.println(IMU.roll, 2);

    Serial.print("rate = ");
    Serial.print((float)IMU.sumCount / IMU.sum, 2);
    Serial.println(" Hz");
    Serial.println("");

    IMU.count = millis();
    IMU.sumCount = 0;
    IMU.sum = 0;
  }
}

bool isShake()
{
  return (abs(IMU.gx) + abs(IMU.gy) + abs(IMU.gz)) > 60;
}

