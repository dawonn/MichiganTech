#!/usr/bin/env python
#
# Translate Imu Orentation into a Pose @ Origin of Hokuyo
#  

import rospy
import tf2_ros
from   geometry_msgs.msg  import TransformStamped


def gpsCB(msg):
  global pub_tf       
  
  msg_tf.header.seq = msg.header.seq
  msg_tf.header.stamp = msg.header.stamp

  msg_tf.transform.translation.x = msg.latitude  - 47.1194486
  msg_tf.transform.translation.y = msg.longitude + 88.5482309
  msg_tf.transform.translation.z = msg.altitude - 195.0
  
  pub_tf.sendTransform(msg_tf)
                     

if __name__ == '__main__':
  global pub_tf   
  global msg_tf   
  
  
  rospy.init_node('imutest')
  
  msg_tf = TransformStamped()
  msg_tf.header.frame_id = "tilt_mount_base"
  msg_tf.child_frame_id  = "tilt_mount"
  
  pub_tf = tf2_ros.TransformBroadcaster()
  
  rospy.spin()
  
  
  
  
  
  
  
  
  
