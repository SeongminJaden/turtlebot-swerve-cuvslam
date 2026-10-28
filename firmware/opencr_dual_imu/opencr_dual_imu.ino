// Uses the official ROBOTIS OpenCR cIMU driver (MPU9250 / ICM20648 selector).
// IMU-only USB firmware: no actuator initialization or torque commands.
#include <IMU.h>
#include <stdint.h>
#include <string.h>
cIMU imu;
struct __attribute__((packed)) Packet {
  char magic[4]; uint32_t sequence, sample_us, reply_us;
  float gyro[3], accel[3], quaternion_wxyz[4];
  uint32_t flags, crc;
};
static_assert(sizeof(Packet)==64,"Host protocol size mismatch");
Packet packet;
uint32_t sequence=0, sample_us=0;
uint32_t checksum(const uint8_t *bytes,size_t n) {
  uint32_t c=0xFFFFFFFFu;
  for(size_t i=0;i<n;i++){c^=bytes[i];for(int b=0;b<8;b++)c=(c>>1)^(0xEDB88320u & (0u-(c&1u)));}
  return c^0xFFFFFFFFu;
}
void setup() {
  Serial.begin(115200);
  imu.begin(200); // Keep both boards stationary during official gyro calibration.
  memcpy(packet.magic,"IMU1",4);
}
void loop() {
  if(imu.update()>0) {
    sample_us=micros(); // Read-completion timestamp; not a hardware data-ready stamp.
    packet.sequence=++sequence;packet.sample_us=sample_us;
    packet.flags=(imu.bConnected && imu.SEN.gyro_cali_get_done())?1u:0u;
    for(int k=0;k<3;k++) {
      // Actual device setup is +/-2 g, +/-2000 deg/s (both official drivers).
      // Do not use cIMU.aRes: its 8 g constant disagrees with the sensor setup.
      packet.accel[k]=imu.SEN.accADC[k]*(9.80665f/16384.0f);
      packet.gyro[k]=imu.SEN.gyroADC[k]*(2000.0f/32768.0f)*(3.14159265358979323846f/180.0f);
      if(abs(imu.SEN.accRAW[k])>32700 || abs(imu.SEN.gyroRAW[k])>32700)packet.flags|=2u;
    }
    for(int k=0;k<4;k++)packet.quaternion_wxyz[k]=imu.quat[k];
  }
  while(Serial.available()) {
    if(Serial.read()=='R') {
      packet.reply_us=micros();packet.crc=checksum((const uint8_t*)&packet,sizeof(packet)-4);
      Serial.write((const uint8_t*)&packet,sizeof(packet));
    }
  }
}
