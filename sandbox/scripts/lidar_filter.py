#!/usr/bin/env python
#
#  Republish a subset of a laserscan. 
#  

import rospy
import math
from   sensor_msgs.msg  import LaserScan

from copy import deepcopy

import dynamic_reconfigure.client

def scan_cb(msg_in):
  global pub
  global angle_min
  global angle_max
  global frame
  
  msg_out  = deepcopy(msg_in)
  
  if(frame is not None):
    msg_out.header.frame_id = frame
  
  # override existing angle limits
  if(angle_min is not None):
    msg_out.angle_min = (angle_min * math.pi / 180.0);
    
  if(angle_max is not None):
    msg_out.angle_max = (angle_max * math.pi / 180.0);
  
  
  # Find index subsets for the new points
  start_pt = 0
  if(msg_out.angle_min > msg_in.angle_min):
    start_pt = (msg_out.angle_min - msg_in.angle_min) / msg_in.angle_increment
    

  end_pt = len(msg_in.ranges)
  if(msg_out.angle_max < msg_in.angle_max):
    end_pt = start_pt + (msg_out.angle_max - msg_out.angle_min) / msg_in.angle_increment
  
  
  # Drop dem points
  msg_out.ranges      = msg_in.ranges[int(start_pt):int(end_pt)]
  msg_out.intensities = msg_in.intensities[int(start_pt):int(end_pt)]
  
  # publish filtered message
  pub.publish(msg_out)
                     


if __name__ == '__main__':  
  global pub
  global angle_min
  global angle_max
  global frame

  # Init ROS node
  rospy.init_node('lidar_filter', anonymous=True)
  
  # Read parameters (Degrees, string)
  angle_min = rospy.get_param('~angle_min', None)
  angle_max = rospy.get_param('~angle_max', None)
  frame     = rospy.get_param('~frame'    , None)
  
  # Setup the publisher
  pub = rospy.Publisher('scan_out', LaserScan)
  
  # Setup the subscriber
  rospy.Subscriber("first", LaserScan, scan_cb)
  rospy.spin()
  
  
  
# <node pkg="gmapping" type="slam_gmapping" name="gmapping" output="screen">	
#	  <rosparam file="$(find sandbox)/config/gmapping.yaml" command="load" />
#	  
#	  <remap from="scan"          to="scan_out"/>
#	</node>
  
  
  
  
  
