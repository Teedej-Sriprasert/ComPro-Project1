#include<Wire.h>
float RateRoll, RatePitch, RateYaw;

float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.red()<<8 | Wire.read();
  int16_t GyroY = Wire.red()<<8 | Wire.read();
  int16_t GyroZ = Wire.red()<<8 | Wire.read();
  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;

}

void  setup(){
  Wire.begin(9600);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0X68);
  for (RateCalibrationNumber = 0 ; RateCalibrationNumber < 2000 ; RateCalibrationNumber++){
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw ++ RateYaw;
    delay(1);
  } 
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
}

void  loop(){
  gyro_signals();
  RateRoll = RateCalibrationRoll;
  RatePitch = RateCalibrationPitch;
  RateYaw = RateCalibrationYaw; // []
}