/*
 *write by jack 2017-04-12
 *Aim：
 *1.订阅最原始消息进行初步处理，然后发布出去消息
 * 1.1 pass though filter
 * 1.2 Statistical Outlier Removal
 * 1.3 voxel downsampling
 *
 *2.关键问题
 * 2.1 是否organized cloud，涉及到求法线的具体方法
 *
 *3.输入与输出参数
 * 3.1 输入
 * cloud_filter_in
 * 3.2 输出
 * cloud_filter_out
 *
*/

//#define /kinect2/sd/points cloud_filter_in
//#define /read_pcl_out cloud_filter_in
//#define cloud_XX cloud_filter_out

#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

class cloudHandler
{
public:
  cloudHandler() //构造函数
  {
//    pcl_sub = nh.subscribe("/read_pcl_out", 10, &cloudHandler::cloudCB,this);
    pcl_sub = nh.subscribe("/kinect2/sd/points", 10, &cloudHandler::cloudCB,this);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_filter_out",1);

  }
public:
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud )
  {
    ROS_INFO("The size of cloud: [%ld]", cloud->data.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *cloud_in);

    ROS_INFO("filter cloud begin:");

    //pass though filter
    pcl::PassThrough<pcl::PointXYZ> pt ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pt(new pcl::PointCloud<pcl::PointXYZ>);
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(0,1.5);
//    pt.setKeepOrganized(true);
//    pt.setUserFilterValue(0);

    pt.filter(*cloud_pt);
    ROS_INFO("Get PassThrough cloud size:[%ld]",cloud_pt->points.size());

    //Statistical Outlier Removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor (new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud(cloud_pt);
    sor.setMeanK(20);
    sor.setStddevMulThresh(0.2);
//    sor.setKeepOrganized(true);
//    sor.setUserFilterValue(0);

    sor.filter(*cloud_sor);
    ROS_INFO("Get StatisticalOutlierRemoval cloud size:[%ld]",cloud_sor->points.size());

    //voxel downsampling
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vg(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud_sor);
    vg.setLeafSize(0.01f,0.01f,0.01f);

    vg.filter(*cloud_vg);
    ROS_INFO("Get VoxelGrid cloud size:[%ld]",cloud_vg->points.size());


//    //NormalEstimation
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

//    ne.setSearchMethod (tree);
//    ne.setInputCloud (cloud_filtered);
//    ne.setKSearch (50);
//    ne.compute (*cloud_normals);

    // publish the cloud
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*cloud_vg, cloud_out);
    pcl_pub.publish(cloud_out);
    ROS_INFO("Publish cloud ,topic: /cloud_filter_out");

  }

private:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pcl_pub;

};

int main(int argc, char **argv)
{
  ROS_INFO("Hello main!");
  ros::init(argc, argv, "filter_cloud_6dof");

  cloudHandler handler;

  ros::spin();
  return 0;
}
