#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

//#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>  //法线
#include <pcl/features/integral_image_normal.h> //积分图法线

class NormalHandler
{
public:
  NormalHandler():viewer("Normals")    //构造函数
  {
    pcl_sub = nh.subscribe("cloud_out_topic", 1, &NormalHandler::NormalCB, this);

    view_timer = nh.createTimer(ros::Duration(0.1), &NormalHandler::timeCB,this);
    //设置viewer默认参数
    viewer.initCameraParameters();
    viewer.setShowFPS(true);
    viewer.setCameraPosition(0, 0, 0, 0, -1, 0);
  }

  void NormalCB(const sensor_msgs::PointCloud2ConstPtr& cloud_input)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_input, cloud);
    pcl::PointCloud<pcl::Normal> normal;

       //normalEstimaton  ==>too slow
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimaton;
//    normalEstimaton.setInputCloud(cloud.makeShared());
//    normalEstimaton.setRadiusSearch(0.03);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
//    normalEstimaton.setSearchMethod(kdtree);

//    normalEstimaton.compute(normal);

      //IntegralImage  ==> efficient
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimations;
    normalEstimations.setInputCloud(cloud.makeShared());
    normalEstimations.setNormalEstimationMethod(normalEstimations.AVERAGE_3D_GRADIENT);
    normalEstimations.setMaxDepthChangeFactor(0.02f);
    normalEstimations.setNormalSmoothingSize(10.0f);

    normalEstimations.compute(normal);

    viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud.makeShared(), normal.makeShared(), 20, 0.03, "normals");


  }
  void timeCB(const ros::TimerEvent&)
  {
    // viewer.spinOnce(1,true);
    viewer.spinOnce();
    
    while (viewer.wasStopped())
    {
      ROS_INFO("Kill the node !");
      ros::shutdown();

    }
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pcl_pub;

  pcl::visualization::PCLVisualizer viewer;
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Normals") );
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> pclviewer (new pcl::visualization::PCLVisualizer);
  ros::Timer view_timer;

};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "normalEstimation");

  NormalHandler normalhandler;

  ros::spin();

  ROS_INFO("Hello world!");
}
