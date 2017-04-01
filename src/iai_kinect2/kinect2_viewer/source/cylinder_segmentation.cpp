#include <ros/ros.h>
#include <pcl/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

class CylinderSeg
{
public:
  CylinderSeg()
  {
//    pcl_sub = nh.advertise("/read_pcl_out", 1, &CylinderSeg::cylinderCB, this); 能用点心吗？
    pcl_sub = nh.subscribe("/read_pcl_out", 1, &CylinderSeg::cylinderCB, this);

  }

public:
  void cylinderCB(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_input;
    pcl::fromROSMsg(input,cloud_input);


  }
private:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;


  ros::Timer viewertimer;
  };
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cylinder_segmentation");

  CylinderSeg cylindeseg;

  ros::spin();

  ROS_INFO("Hello world!");
}
