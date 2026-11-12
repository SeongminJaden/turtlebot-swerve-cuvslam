"""Versioned IMU-only OpenCR USB protocol; MCU units are already SI."""
import struct,zlib,math
FORMAT=struct.Struct('<4sIII10fII')

def decode(data):
    if len(data)!=FORMAT.size:raise ValueError('Invalid IMU packet length')
    fields=FORMAT.unpack(data)
    if fields[0]!=b'IMU1':raise ValueError('Wrong firmware/protocol magic')
    if zlib.crc32(data[:-4])!=fields[-1]:raise ValueError('IMU CRC mismatch')
    if not fields[-2]&1:raise ValueError('Sensor disconnected or gyro calibration incomplete')
    if fields[-2]&2:raise ValueError('IMU ADC saturation')
    if not all(math.isfinite(v) for v in fields[4:14]):raise ValueError('Nonfinite IMU data')
    age_us=(fields[3]-fields[2])&0xffffffff
    if age_us>20000:raise ValueError('Stale MCU sample')
    return dict(sequence=fields[1],sample_us=fields[2],reply_us=fields[3],gyro=fields[4:7],accel=fields[7:10],quaternion_wxyz=fields[10:14],sample_age_s=age_us*1e-6)


def measurement_stamp(packet,request_s,receive_s,max_rtt=.02):
    """USB round-trip midpoint estimate, not hardware time synchronization.
    Timing uncertainty is bounded by RTT/2 + sensor read/sampling delay.
    """
    rtt=receive_s-request_s
    if not 0<=rtt<=max_rtt:raise ValueError('USB timing uncertainty too large')
    return (request_s+receive_s)/2-packet['sample_age_s'],rtt/2
