import struct,zlib
import pytest
from turtlebot_swerve.opencr_protocol import decode,measurement_stamp,FORMAT

def packet(sample=1000,reply=2000,flags=1):
 raw=struct.pack('<4sIII10fI',b'IMU1',7,sample,reply,0.,0.,1.,0.,0.,9.80665,1.,0.,0.,0.,flags)
 return raw+struct.pack('<I',zlib.crc32(raw))

def test_si_and_quaternion_order():
 raw=packet();assert len(raw)==FORMAT.size==64
 p=decode(raw);assert p['gyro']==(0.,0.,1.);assert p['accel'][2]==pytest.approx(9.80665)
 assert p['quaternion_wxyz']==(1.,0.,0.,0.)
 assert measurement_stamp(p,10.,10.004)==pytest.approx((10.001,.002))

def test_rollover_age():
 assert decode(packet(0xffffff00,0x100))['sample_age_s']==pytest.approx(.000512)

def test_crc_saturation_and_uninitialized_rejected():
 raw=bytearray(packet());raw[20]^=1
 with pytest.raises(ValueError,match='CRC'):decode(raw)
 for flags in (0,3):
  with pytest.raises(ValueError):decode(packet(flags=flags))
 with pytest.raises(ValueError):decode(packet(sample=1000,reply=31000))

def test_delayed_usb_is_not_precise_timestamp():
 with pytest.raises(ValueError):measurement_stamp(decode(packet()),1.,1.1)
