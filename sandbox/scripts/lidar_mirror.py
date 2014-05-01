#!/usr/bin/env python
#
# find the height of the scanner from laser hits aimed at the ground
#  

import rospy
import math
from   sensor_msgs.msg  import LaserScan
import tf
import numpy


def scan_cb(msg_in):
  
  
  #New measurment
  ranges = numpy.array(msg_in.ranges)
  ranges = ranges[~numpy.isnan(ranges)]
  m =  max(ranges)  #(sum( ranges ) /  len( ranges ))
  
 
  # Find average  
  if(not numpy.isnan(m)) and (abs(m - scan_cb.height) < 3.0):
    K = 0.05
    scan_cb.height = (1.0 - K) * scan_cb.height + K * m
  #print scan_cb.height
  
  br = tf.TransformBroadcaster()
  br.sendTransform((0, 0, scan_cb.height),
                     (0, 0, 0, 1),
                     rospy.Time.now() + rospy.Duration.from_sec(0.5),
                     "laser_base",
                     "footprint")
# Static var init                   
scan_cb.height = 0                     


if __name__ == '__main__':  
 

  # Init ROS node
  rospy.init_node('lidar_mirror', anonymous=True)
  
  
  
  # Setup the subscriber
  rospy.Subscriber("scan_down", LaserScan, scan_cb)
  rospy.spin()
  
  
  
  
  
  
  
  
  
