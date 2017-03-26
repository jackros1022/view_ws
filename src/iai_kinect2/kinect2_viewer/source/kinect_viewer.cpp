#include <ros/ros.h>

//三剑客
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

class  CloudHandler
{
public:
    CloudHandler()
      :viewer("Cloud Viewer")
    {
      pcl_sub = nh.subscribe("/kinect2/sd/points", 1, &CloudHandler::cloudprocessCB, this);
      viewer_timer = nh.createTimer(ros::Duration(0.1), &CloudHandler::timerCB, this);  //设置定时器，每隔0.1秒周期性的处理回调函数，而回调函数的功能是检查viewer是否关闭

      pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_out_topic", 1);//缓冲区大小

      //设置viewer默认参数
      viewer.initCameraParameters();
      viewer.setShowFPS(true);
      viewer.setCameraPosition(0, 0, 0, 0, -1, 0);
      
    }

    void cloudprocessCB(const sensor_msgs::PointCloud2ConstPtr& cloud_input )
    {
      pcl::PointCloud<pcl::PointXYZ> cloud1;
      pcl::fromROSMsg(*cloud_input,cloud1);
     

      //都放在回调函数中处理，如果需要时间很久，会出现什么错误？
      //【1】Pass through Filter
      pcl::PointCloud<pcl::PointXYZ> cloud_pass;
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud1.makeShared());
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0,1);

      pass.filter(cloud_pass);

      //【2】Statistical Outlier Removal
      pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;

      statFilter.setInputCloud(cloud_pass.makeShared());
      statFilter.setMeanK(20);
      statFilter.setStddevMulThresh(0.2);
      statFilter.filter(cloud_filtered);

      //【3】voxel downsampling
       pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
       pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;

       voxelSampler.setInputCloud(cloud_filtered.makeShared());
       voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
       voxelSampler.filter(cloud_downsampled);

     
       
       pcl::toROSMsg(cloud_downsampled, cloud_out );
         //这里不需要设置tf吗？
         // 要设置，接收/kinect2/sd/points的时候已经设置tf

//       while(ros::ok())
//      {
//        pcl_pub.publish(cloud_out);
//        ros::spinOnce();
//        rate_loop.sleep();
//      }

        viewer.removePointCloud("p_cloud");
        viewer.addPointCloud<pcl::PointXYZ>(cloud_downsampled.makeShared(), "p_cloud");
        

    }

    void timerCB(const ros::TimerEvent&)//周期性回来检查viewer是否关闭，然后关闭node，实际上效果不好
    {
        viewer.spinOnce();  //这一句干嘛？

        ROS_INFO("timerCB once again!");
        if(viewer.wasStopped())     
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
  sensor_msgs::PointCloud2 cloud_out;   //发布出去的点云

  ros::Timer viewer_timer;

  int v1 , v2;      //两个窗口

};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "kinect_viewer");

//  ros::Rate rate_loop(10);

  ROS_INFO("Init node!");

  CloudHandler cloudhandler;

  ros::spin();  //否者一闪而过


  return 0;
}
