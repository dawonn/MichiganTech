#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>
#include <iostream>


// You can remap these in a launch file or change them here and recompile
char laserTopic[]   = "scan_out";
char laserFrameID[] = "laser"; 
char cloudTopic[]   = "my_cloud";

class LaserScanToPointCloud 
{
  public:
    ros::NodeHandle node;
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;
    ros::Publisher scan_pub;
    
    
    LaserScanToPointCloud( ros::NodeHandle n): 
      node(n), 
      laser_sub(node, laserTopic, 10),
      laser_notifier(laser_sub, listener, laserFrameID, 10) 
      {
        laser_notifier.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier.setTolerance(ros::Duration(.01));
        scan_pub = node.advertise<sensor_msgs::PointCloud2>(cloudTopic, 1);
      }

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in) 
    {
      sensor_msgs::PointCloud2 cloud;
      try 
      {
        projector.transformLaserScanToPointCloud(laserFrameID, *scan_in, cloud, listener);
      }
      catch(tf::TransformException& e) 
      {
        std::cout << e.what();
        return;
      }
      scan_pub.publish(cloud);
    }
};

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  ros::spin();
  return 0;
}
