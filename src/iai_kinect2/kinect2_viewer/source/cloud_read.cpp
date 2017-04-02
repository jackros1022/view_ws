#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "cloud_read");
//    ROS_INFO("Hello world!");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/read_pcl_out",1);

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    if(pcl::io::loadPCDFile("/home/jack/ros/view_ws/src/iai_kinect2/kinect2_viewer/data/table_scene_mug_stereo_textured.pcd",cloud)==-1)
      {
        ROS_INFO("read file failed !");
      }

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";


    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();  //1Hz
    }

    return 0;

}
