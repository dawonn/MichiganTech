#!/usr/bin/env python
#
# Sweep the laser scanner with a servo and broadcast a TF frame
#  

import rospy
from std_msgs.msg import Float32
import tf
import tf2_ros
from   geometry_msgs.msg  import TransformStamped


def sweepCB(event):
  global pub_tf      
  global angle      
  global fwd       
  
  # Sent angle request to servo  
  if fwd:
    angle = angle + 0.25
  else:
    angle = angle - 0.25
  
  if angle >= 135:
    fwd = False
  if angle <= 45:
    fwd = True
    
  pub = rospy.Publisher('servo', Float32)
  pub.publish(Float32(angle))
  
  
  # Publish TF frame for tilt mount base
  q = tf.transformations.quaternion_from_euler(0, 3.1415967 * (angle-90)/180.0, 0)
  
  msg_tf.header.stamp = rospy.get_rostime() + rospy.Duration.from_sec(0.1)
  msg_tf.transform.rotation.x = q[0]
  msg_tf.transform.rotation.y = q[1]
  msg_tf.transform.rotation.z = q[2]
  msg_tf.transform.rotation.w = q[3]
  
  pub_tf.sendTransform(msg_tf)
                     

if __name__ == '__main__':  
  # Init ROS node
  rospy.init_node('tilt_sweep')
  
  
  # TF publisher
  global pub_tf   
  global msg_tf   
  msg_tf = TransformStamped()
  msg_tf.header.frame_id = "tilt_mount_base"
  msg_tf.child_frame_id  = "tilt_mount"
  pub_tf = tf2_ros.TransformBroadcaster()
  
  
  # Servo angle control
  global angle   
  global direction
  angle = 90
  fwd = True
  rospy.Timer(rospy.Duration(0.02), sweepCB)
  
  rospy.spin()
  
  
  
  
  
  
  
  
  
