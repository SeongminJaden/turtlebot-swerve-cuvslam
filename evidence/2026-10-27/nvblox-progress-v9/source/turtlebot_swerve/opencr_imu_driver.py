"""One read-only IMU USB endpoint per OpenCR, explicit ports and ROS topics."""
import math,time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from rclpy.qos import qos_profile_sensor_data
from .opencr_protocol import decode,measurement_stamp,FORMAT

class OpenCrImuDriver(Node):
    def __init__(self, **node_options):
        super().__init__('opencr_imu_driver', **node_options)
        def p(n,v):return self.declare_parameter(n,v).value
        port=p('port','')
        if not port:raise ValueError('Set stable /dev/serial/by-id path for this OpenCR')
        self.frame=p('frame_id','opencr_imu');self.max_rtt=p('max_rtt_s',.02);self.last_sequence=None;self.last_stamp=-1
        self.pub=self.create_publisher(Imu,p('topic','imu/data'),qos_profile_sensor_data)
        self.diag=self.create_publisher(DiagnosticArray,'diagnostics',10)
        self.usb=serial.Serial(port,115200,timeout=.025,write_timeout=.025,exclusive=True)
        self.usb.reset_input_buffer();self.next_retry=0.;self.timer=self.create_timer(1./p('poll_hz',200.),self.poll)
    def diagnostics(self,level,message,rtt=None):
        a=DiagnosticArray();a.header.stamp=self.get_clock().now().to_msg();s=DiagnosticStatus();s.name=self.get_fully_qualified_name();s.hardware_id=self.usb.port;s.level=bytes([level]) if isinstance(s.level,bytes) else level;s.message=message
        if rtt is not None:s.values=[KeyValue(key='timestamp_uncertainty_s',value=str(rtt)),KeyValue(key='timestamp_method',value='USB request/reply midpoint; not hardware sync')]
        a.status=[s];self.diag.publish(a)
    def poll(self):
        try:
            if not self.usb.is_open:
                if time.monotonic()<self.next_retry:return
                self.usb.open();self.usb.reset_input_buffer();self.last_sequence=None;self.last_stamp=-1
            start=self.get_clock().now().nanoseconds*1e-9;self.usb.write(b'R');raw=self.usb.read(FORMAT.size);end=self.get_clock().now().nanoseconds*1e-9
            packet=decode(raw);stamp,uncertainty=measurement_stamp(packet,start,end,self.max_rtt)
            if packet['sequence']==self.last_sequence:raise ValueError('No new sensor sample')
            if stamp<=self.last_stamp:raise ValueError('Nonmonotonic estimated measurement time')
            self.last_sequence=packet['sequence'];self.last_stamp=stamp
            msg=Imu();ns=round(stamp*1e9);msg.header.stamp.sec=ns//1_000_000_000;msg.header.stamp.nanosec=ns%1_000_000_000;msg.header.frame_id=self.frame
            msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=packet['gyro'];msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=packet['accel']
            w,x,y,z=packet['quaternion_wxyz'];norm=math.sqrt(w*w+x*x+y*y+z*z)
            if norm>.5:
                msg.orientation.x=x/norm;msg.orientation.y=y/norm;msg.orientation.z=z/norm;msg.orientation.w=w/norm
            else:msg.orientation.w=1.;msg.orientation_covariance[0]=-1.
            # Zero covariance = unknown under sensor_msgs, not perfect accuracy.
            self.pub.publish(msg);self.diagnostics(0,'Official cIMU SI data received',uncertainty)
        except ValueError as e:
            self.diagnostics(2,str(e))
            try:self.usb.reset_input_buffer()
            except serial.SerialException:self.usb.close();self.next_retry=time.monotonic()+1.
        except serial.SerialException as e:
            self.diagnostics(2,str(e));self.usb.close();self.next_retry=time.monotonic()+1.
    def destroy_node(self):
        self.usb.close();return super().destroy_node()

def main(args=None):
    rclpy.init(args=args);n=None
    try:n=OpenCrImuDriver();rclpy.spin(n)
    finally:
        if n:n.destroy_node()
        rclpy.shutdown()
