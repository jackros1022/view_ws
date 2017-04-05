/*
 *参考代码 conditional_euclidean_clustering.cpp
 * 怎么显示出来？
 *
 *
*/
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

//#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>

class SegCloud
{
public:
  SegCloud()
  {
    //    pcl_sub = nh.subscribe("/kinect2/sd/points",1 ,&SegCloud::segmentCB, this);
    pcl_sub = nh.subscribe("/read_pcl_out", 1, &SegCloud::segmentCB, this);
    //    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("segment_out", 1);
  }

public:
  void segmentCB(const sensor_msgs::PointCloud2 &input)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>); //指针为什么不能定义到类中，而只能放到回调函数中？
    pcl::fromROSMsg(input, *cloud_input);
    ROS_INFO("I get PointCloud ,  want to do something");

    //voxelgrid
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud_input);
    vg.setLeafSize(0.01, 0.01, 0.01);
    vg.filter(*cloud_filtered);
    ROS_INFO("PointCloud after filtering has:[ %ld ] data points.", cloud_filtered->points.size());

    // create SACSegmentaton
    pcl::SACSegmentation<pcl::PointXYZ> SACSeg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>); //
    pcl::PCDWriter writer;

    SACSeg.setOptimizeCoefficients(true);
    SACSeg.setModelType(pcl::SACMODEL_PLANE);
    SACSeg.setMethodType(pcl::SAC_RANSAC);
    SACSeg.setMaxIterations(100); //自动迭代
    SACSeg.setDistanceThreshold(0.01);

    int i = 0, nr_points = (int)cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.2 * nr_points) //0.2这个参数很难调节啊？
    {

      SACSeg.setInputCloud(cloud_filtered);
      SACSeg.segment(*inliers, *coefficients);

      if (inliers->indices.size() == 0)
      {
        ROS_WARN("Oh! no. I can not estimate a planar model for the given dateset");
        //break;
        exit(-1);
      }
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);

      extract.filter(*cloud_plane);
      ROS_INFO("PointCloud representing the planar component: [%ld] data points", cloud_plane->points.size());

      extract.setNegative(true);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
      extract.filter(*cloud_f);

      *cloud_filtered = *cloud_f;

      ROS_INFO("haha , estimate a planar model spends: [%d] times", i++);
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    std::vector<pcl::PointIndices> cluster_indices;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);

    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      ROS_INFO("The vector :cluster_indices Segmentation");
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      //        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*

      ROS_INFO("The number of cluster :[%d] ", j++);
    }
    ros::shutdown();
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pcl_pub;
};
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "segmentation_cloud");
  SegCloud segcloud;

  ROS_INFO("Hello world!");

  ros::spin(); //频率多少？　没有所谓是频率，只是让程序在实例化类中持续执行。
  return 0;
}
