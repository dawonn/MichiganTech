#!/usr/bin/env python
#
# Hokuyo lidar orientation from IMU
#  

import rospy
import tf2_ros
import tf
from   geometry_msgs.msg   import TransformStamped
from   vectornav.msg       import ins


def subCB(msg_in):
  global pub_tf
  global msg_tf   

  msg_tf.header.stamp = rospy.Time.now()

  msg_tf.transform.translation.x = 0
  msg_tf.transform.translation.y = 0
  msg_tf.transform.translation.z = 0
  
  quaternion = tf.transformations.quaternion_from_euler(msg_in.RPY.x/180.0 * 3.1415,
                                                        -msg_in.RPY.y/180.0 * 3.1415,
                                                        -msg_in.RPY.z/180.0 * 3.1415)
  msg_tf.transform.rotation.w = quaternion[3]
  msg_tf.transform.rotation.x = quaternion[0]
  msg_tf.transform.rotation.y = quaternion[1]
  msg_tf.transform.rotation.z = quaternion[2]
  
  pub_tf.sendTransform(msg_tf)
                     


if __name__ == '__main__':
  global pub_tf   
  global msg_tf   
  
  rospy.init_node('vn_orient')
  
  msg_tf = TransformStamped()
  msg_tf.header.frame_id = "world"
  msg_tf.child_frame_id  = "laser"
  pub_tf = tf2_ros.TransformBroadcaster()
  
  
  rospy.Subscriber("vectornav/ins", ins, subCB)
  rospy.spin()
  
  
  
  
  
  
  
  
  
