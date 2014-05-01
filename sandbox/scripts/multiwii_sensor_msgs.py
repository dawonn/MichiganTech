#!/usr/bin/env python
# 
# Publish ROS standard messages for Vectornav sensors
#
# Copyright (c) 2013 Dereck Wonnacott <dereck@gmail.com>
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# 

from copy             import deepcopy

import rospy
from sensor_msgs.msg  import Imu
from std_msgs.msg     import Int16MultiArray

from numpy  import *

def sub_imuCB(msg_in): 
  global pub_imu
  global msg_imu
  
  FS_SEL  = 131.0   / 2.0
  AFS_SEL = 16384.0 / 2.0 / 9.8 
  # This is as accurate of a gravity vector can be without accounting for latitude
  
  
  accel = array(msg_in.data[0:3]) / AFS_SEL
  gyro  = array(msg_in.data[3:6]) / FS_SEL
  
  msg_imu.header.stamp = rospy.Time.now()
  msg_imu.angular_velocity.x    = gyro[0]
  msg_imu.angular_velocity.y    = gyro[1]
  msg_imu.angular_velocity.z    = gyro[2]
  msg_imu.linear_acceleration.x = accel[0]
  msg_imu.linear_acceleration.y = accel[1]
  msg_imu.linear_acceleration.z = accel[2]
  pub_imu.publish(msg_imu)               
  



if __name__ == '__main__':
  rospy.init_node('multiwii_sensor_msgs')
  
  global msg_imu
  msg_imu = Imu()
  msg_imu.header.frame_id = "imu"
  
  global pub_imu
  pub_imu  = rospy.Publisher("/imu", Imu)
  
  rospy.Subscriber("/multiwii", Int16MultiArray,  sub_imuCB)
  rospy.spin()
  
  
  
  
  
  
  
  
  
