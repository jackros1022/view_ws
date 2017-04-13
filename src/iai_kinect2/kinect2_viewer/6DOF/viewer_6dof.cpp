#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

class CloudHandler
{
public:
  CloudHandler()
      : viewer("timerCB Viewer")
  {
    pcl_sub = nh.subscribe("/pcl_segment_out", 1, &CloudHandler::cloudprocessCB, this);
//    pcl_sub = nh.subscribe("/cloud_filter_out", 1, &CloudHandler::cloudprocessCB, this);
//    pcl_sub = nh.subscribe("/kinect2/sd/points", 1, &CloudHandler::cloudprocessCB, this);
    //指定时间，周期行的执行回调函数。一直执行
    viewer_timer = nh.createTimer(ros::Duration(0.1), &CloudHandler::timerCB, this);


    //设置viewer默认参数
    viewer.initCameraParameters();
    viewer.setShowFPS(true);
    viewer.setCameraPosition(0, 0, 0, 0, -1, 0);
  }

  void cloudprocessCB(const sensor_msgs::PointCloud2ConstPtr &cloud_input)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    pcl::fromROSMsg(*cloud_input, cloud1);


    viewer.removePointCloud("p_cloud");
    viewer.addPointCloud<pcl::PointXYZ>(cloud1.makeShared(), "p_cloud");

//    viewer.spinOnce();
    ROS_INFO(" cloudprocessCB once again!");
  }

  void timerCB(const ros::TimerEvent &) //周期性回来检查viewer是否关闭，然后关闭node，实际上效果不好
  {

    viewer.spinOnce();

    ROS_INFO("_________________ timerCB once again!");

    if (viewer.wasStopped())
    {
      ROS_INFO("Kill the node !");
      ros::shutdown();
    }
  }

private:
  ros::NodeHandle nh; //节点进程的句柄（也就是说这个进程的别的代号）
  ros::Subscriber pcl_sub;
  ros::Publisher pcl_pub;
  pcl::visualization::PCLVisualizer viewer;
  //  ros::Rate rate_loop(0.1);  //定义出错
  sensor_msgs::PointCloud2 cloud_out; //发布出去的点云

  ros::Timer viewer_timer;

};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "viewer_6dof");

  //  ros::Rate rate_loop(10);

  ROS_INFO("Init node!");

  CloudHandler cloudhandler;

  ros::spin(); //否者一闪而过

  return 0;
}
