/*
 * 1. roslaunch kinect2_bridge kinect2_bridge.launch max_depth:=1才有/kinect2/sd/points的topic
 *
 *
 *
 * */

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp> //opencv头文件有何作用？

class cloudHandler
{
public:
  cloudHandler()   :viewer("Cloud Viewer ")   //构造函数
  {
    pcl_sub = nh.subscribe("/kinect2/sd/points", 1, &cloudHandler::cloudCB, this);
    viewer_timer = nh.createTimer(ros::Duration(0.05), &cloudHandler::timerCB, this);     //定时器
  }
  void cloudCB(const sensor_msgs::PointCloud2 &input)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;   //普通点云类型
    pcl::fromROSMsg(input, cloud);  // 回调函数接收的ros类型的点云（实际上，ros提供master，消息订阅发布），需要转换为普通点云

//  //-----------filtering-----------
//    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
//    statFilter.setInputCloud(cloud.makeShared());
//    statFilter.setMeanK(10);
//    statFilter.setStddevMulThresh(0.2);
//    statFilter.filter(cloud_filtered);

//  //-----------downsampling-----------
//    pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
//    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
//    voxelSampler.setInputCloud(cloud.makeShared());
//    voxelSampler.setLeafSize(0.02f, 0.02f, 0.02f);
//    voxelSampler.filter(cloud_downsampled);

    viewer.showCloud(cloud.makeShared());   //转化为指针,若是发布出去还需要转化ros类型点云
                                                                          // 交给回调函数处理，显示
  }

  void timerCB(const ros::TimerEvent& )
  {

    if(viewer.wasStopped())
    {
      ros::shutdown();                                       //窗口若关闭，ros停止发布订阅
                                                                        //否则node没有关闭，孤儿进程
    }
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Timer viewer_timer;
  pcl::visualization::CloudViewer viewer;

};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "pcl_cloud_viewer");
  ROS_INFO("Hello world!");

  cloudHandler handler;

  ros::spin();
  return 0;
}
