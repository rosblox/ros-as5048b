# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import smbus
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class AS5048B():
    #Derived from sosandroid's AMS_AS5048B Arduino library,
    #https://github.com.cnpmjs.org/sosandroid/AMS_AS5048B, and
    #https://forums.raspberrypi.com/viewtopic.php?t=265284

    RE_ZEROMSB_REG = 0x16 #Zero, most significant byte
    RE_ZEROLSB_REG = 0x17 #Zero, least significant byte
    RE_ANGLEMSB_REG = 0xFE #Angle, most significant byte
    RE_ANGLELSB_REG = 0xFF #Angle, least significant byte

    RESOLUTION = 16384.0 #14 bits

    def __init__(self, i2c_device: int, i2c_address: int):
        self.bus=smbus.SMBus(i2c_device)
        self.i2c_address = i2c_address
        self.set_position_to_zero()

    def set_position_to_zero(self):
        self.bus.write_byte_data(self.i2c_address, self.RE_ZEROMSB_REG, 0x00)
        self.bus.write_byte_data(self.i2c_address, self.RE_ZEROLSB_REG, 0x00)
        angle_most_significant_bits = self.bus.read_byte_data(self.i2c_address, self.RE_ANGLEMSB_REG)
        angle_least_significant_bits = self.bus.read_byte_data(self.i2c_address, self.RE_ANGLELSB_REG)
        self.bus.write_byte_data(self.i2c_address, self.RE_ZEROMSB_REG, angle_most_significant_bits)
        self.bus.write_byte_data(self.i2c_address, self.RE_ZEROLSB_REG, angle_least_significant_bits)

    def get_position_raw(self) -> float:
        angle_most_significant_bits = self.bus.read_byte_data(self.i2c_address, self.RE_ANGLEMSB_REG)
        angle_least_significant_bits = self.bus.read_byte_data(self.i2c_address,self.RE_ANGLELSB_REG)
        angle_position_raw = (angle_most_significant_bits<<6)+(angle_least_significant_bits & 0x3f)
        return float(angle_position_raw)

    def get_position_rad(self) -> float:
        return (self.get_position_raw() / self.RESOLUTION) * 2.0 * math.pi

    def get_position_deg(self) -> float:
        return (self.get_position_raw() / self.RESOLUTION) * 360.0




class RosAs5048bPublisher(Node):

    def __init__(self):
        super().__init__('ros_as5048b_publisher')

        i2c_device =  self.declare_parameter('i2c_device', 1).get_parameter_value().integer_value
        i2c_address = self.declare_parameter('i2c_address', 0x40).get_parameter_value().integer_value

        self.as5048b = AS5048B(i2c_device, i2c_address)

        self.encoder_publisher = self.create_publisher(Float32, f"/encoder/at_{hex(self.as5048b.i2c_address)}/deg", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = self.as5048b.get_position_deg()
        self.encoder_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    ros_as5048b_publisher = RosAs5048bPublisher()

    rclpy.spin(ros_as5048b_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros_as5048b_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
