/*
 * 从磁盘读入一个pcd文件，转换为ROS类型消息，发布出去
*/
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "pcl_read");

  ROS_INFO("Hello world!");

  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output",1);

  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  if(pcl::io::loadPCDFile("/home/jack/ros/view_ws/src/iai_kinect2/kinect2_viewer/data/test_pcd.pcd",cloud)==-1)
    {
      ROS_INFO("read file failed !");
    }

    pcl::toROSMsg(cloud,output);
    output.header.frame_id = "odom";

    ros::Rate loop_rate(1); //1秒钟发布一次

    while(ros::ok()){
      pcl_pub.publish(output);
      ros::spinOnce();      //还可以做别的事情哦
      loop_rate.sleep();
    }

}
