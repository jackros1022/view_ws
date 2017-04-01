#include <ros/ros.h>

//三剑客
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/integral_image_normal.h> //积分图法线
#include <pcl/features/normal_3d.h>

class CloudHandler
{
public:
  CloudHandler()
      : viewer("Normal Viewer")
  {
    pcl_sub = nh.subscribe("/kinect2/sd/points", 1, &CloudHandler::cloudprocessCB, this);
    viewer_timer = nh.createTimer(ros::Duration(0.1), &CloudHandler::timerCB, this); //设置定时器，每隔0.1秒周期性的处理回调函数，而回调函数的功能是检查viewer是否关闭

    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_out_topic", 1); //缓冲区大小

    //设置viewer默认参数
    viewer.initCameraParameters();
    viewer.setShowFPS(true);
    viewer.setCameraPosition(0, 0, 0, 0, -1, 0);
  }

  void cloudprocessCB(const sensor_msgs::PointCloud2ConstPtr &cloud_input)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    pcl::fromROSMsg(*cloud_input, cloud1);

    //【1】Pass through Filter
    pcl::PointCloud<pcl::PointXYZ> cloud_pass;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud1.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1);
    pass.setKeepOrganized(true);
    pass.setUserFilterValue(0);

    pass.filter(cloud_pass);

    //   //使用条件滤波
    // //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZ>);
    //   pcl::PointCloud<pcl::PointXYZ> cloud_pass;
    //   pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition (new pcl::ConditionAnd<pcl::PointXYZ> );
    //   condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
    //   condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 2.0)));

    //   // Filter object.
    //   pcl::ConditionalRemoval<pcl::PointXYZ> filter;
    //   filter.setCondition(condition);
    //   filter.setInputCloud(cloud1.makeShared());
    //   filter.setKeepOrganized(true);
    //   filter.setUserFilterValue(0.0);

    //   filter.filter(cloud_pass);

    //【2】Statistical Outlier Removal
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;

    statFilter.setInputCloud(cloud_pass.makeShared());
    statFilter.setMeanK(20);
    statFilter.setStddevMulThresh(0.2);
    statFilter.setKeepOrganized(true);
    statFilter.setUserFilterValue(0.0);

    statFilter.filter(cloud_filtered);

    //【3】voxel downsampling
    pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;

    voxelSampler.setInputCloud(cloud_filtered.makeShared());
    voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);

    voxelSampler.filter(cloud_downsampled);

    //

    // 【4】 IntegralImage==》Normal
    pcl::PointCloud<pcl::Normal> normal;

    //不是organized的，无法使用IntergralImage
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimations;
    normalEstimations.setInputCloud(cloud_downsampled.makeShared());
    normalEstimations.setNormalEstimationMethod(normalEstimations.AVERAGE_3D_GRADIENT);
    normalEstimations.setMaxDepthChangeFactor(0.02f);
    normalEstimations.setNormalSmoothingSize(10.0f);

    normalEstimations.compute(normal);

    //  【4】 求取法线
    //       pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimaton;
    //       normalEstimaton.setInputCloud(cloud_downsampled.makeShared());
    //       normalEstimaton.setRadiusSearch(0.3);
    //       pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    //       normalEstimaton.setSearchMethod(kdtree);

    //       normalEstimaton.compute(normal);

    pcl::toROSMsg(cloud_filtered, cloud_out);

    viewer.removePointCloud("p_cloud");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_downsampled.makeShared(), "p_cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_downsampled.makeShared(), normal.makeShared(), 20, 0.03, "normals");

    //      viewer.updatePointCloud(cloud_downsampled.makeShared(),"p_cloud");
  }

  void timerCB(const ros::TimerEvent &)
  {

    viewer.spinOnce();
    ROS_INFO("timerCB once again!");
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

  int v1, v2; //两个窗口
};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "kinect_view_normal");
  ROS_INFO("Init node!");

  CloudHandler cloudhandler;

  ros::spin(); //否者一闪而过

  return 0;
}
