#!/usr/bin/env python
#
# Hokuyo lidar orientation from IMU
#  

import rospy
import tf2_ros
import tf
from tf.transformations import euler_from_quaternion

from   geometry_msgs.msg   import TransformStamped
from   geometry_msgs.msg   import Vector3
from   vectornav.msg       import ins
from   vectornav.msg       import sensors
from   std_msgs.msg         import UInt16


def insCB(msg_in):
  global pub_tf
  global msg_tf   

  msg_tf.header.stamp = rospy.Time.now()

  
  quaternion = tf.transformations.quaternion_from_euler( msg_in.RPY.x/180.0 * 3.1415,
                                                        -msg_in.RPY.y/180.0 * 3.1415,
                                                        -msg_in.RPY.z/180.0 * 3.1415)
  
  msg_tf.transform.rotation.x = quaternion[0]
  msg_tf.transform.rotation.y = quaternion[1]
  msg_tf.transform.rotation.z = quaternion[2]
  msg_tf.transform.rotation.w = quaternion[3]
  
  pub_tf.sendTransform(msg_tf)


def sonarCB(msg_in):  
  global msg_tf


  # Find actual height based on rotation
  Q = [msg_tf.transform.rotation.x, msg_tf.transform.rotation.y, msg_tf.transform.rotation.z, msg_tf.transform.rotation.w]
  #Q = [0, 0, 0, 1]
  V = [0, 0, msg_in.data / 1000.0, 0]
  
  v = tf.transformations.quaternion_multiply(V, Q) 
  print v



  # Update estimated pose
  msg_tf.transform.translation.x = 0
  msg_tf.transform.translation.y = 0
  msg_tf.transform.translation.z = msg_in.data / 1000.0;
  
  
  



if __name__ == '__main__':
  global pub_tf   
  global msg_tf
  
  rospy.init_node('vn_orient')
  
  msg_tf = TransformStamped()
  msg_tf.header.frame_id = "world"
  msg_tf.child_frame_id  = "laser"
  pub_tf = tf2_ros.TransformBroadcaster()
  
  
  
  rospy.Subscriber("vectornav/ins", ins, insCB)
  rospy.Subscriber("sonar", UInt16, sonarCB)
  rospy.spin()
  
  
  
  
  
  
  
  
  
