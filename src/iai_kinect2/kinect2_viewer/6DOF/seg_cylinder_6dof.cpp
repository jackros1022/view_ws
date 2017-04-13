#include <ros/ros.h>

#include <pcl/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/sac_segmentation.h>

class CylinderSeg
{
public:
  CylinderSeg()
  {
    pcl_sub = nh.subscribe("/cloud_filter_out", 1, &CylinderSeg::cylinderCB, this);

    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_segment_out",10);

  }

public:
  void cylinderCB(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    ROS_INFO("I go to cylinderCB().");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud,*cloud_input);
    ROS_INFO("I get cloud, I will work now ! ! ! ");

    //Estimate point Normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal (new pcl::PointCloud<pcl::Normal>);
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_input);
    ne.setKSearch(50);
    ne.compute(*cloud_normal);
    ROS_INFO("NormalEstimation Size:[%ld]",cloud_normal->size());

    //create the segmentation
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr cofficients_plane(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_input);
    seg.setInputNormals(cloud_normal);
    seg.segment(*inliers_plane, *cofficients_plane);
    std::cerr << "Plane coefficients: " << *cofficients_plane << std::endl;
    ROS_INFO( "Plane coefficients");

    pcl::ExtractIndices<pcl::PointXYZ> extract; //点云提取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud_input);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    ROS_INFO("ExtractIndices cloud_plane Size:[%ld]",cloud_plane->size());
    pcl::PCDWriter writer;
    writer.write("cloud_plane.pcd",*cloud_plane);
    ROS_INFO("I save file:cloud_plane.pcd ");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_plane(new pcl::PointCloud<pcl::PointXYZ>);   //保存除平面以外的点云
    pcl::PointCloud<pcl::Normal>::Ptr cloud_no_plane_normals (new pcl::PointCloud<pcl::Normal>);  //保存除平面以外的点云的法线
    pcl::ExtractIndices<pcl::Normal> extract_normals;   //法线提取
    extract.setNegative(true);
    extract.filter(*cloud_no_plane);
    ROS_INFO("I get pointcloud: cloud_no_plane ");

    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normal);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_no_plane_normals);
    ROS_INFO("I get Normals: cloud_no_plane_normals ");

    //Now ,we will get the cylinder...
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr cofficients_cylinder(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(1000);//迭代10000次，那么高！
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0,0.1);
    seg.setInputCloud(cloud_no_plane);
    seg.setInputNormals(cloud_no_plane_normals);
    ROS_INFO("the cylinder, setInputCloud and setInputCloud ");

    seg.segment(*inliers_cylinder,*cofficients_cylinder);
    ROS_INFO("I get cofficients_cylinder ");

    // extract cylinders
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud_no_plane);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);
    ROS_INFO("I get the cylinder.");

    // publish the segmentation of cloud_cylinder
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*cloud_cylinder, cloud_out);
    pcl_pub.publish(cloud_out);
    ROS_INFO("haha ! I finally publish the cylinder.");

  }
private:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pcl_pub;


  };


int main(int argc, char **argv)
{
  ros::init(argc, argv, "seg_cylinder_6dof");
    ROS_INFO("Hello main! I am in main of CylinderSeg ");
  ros::NodeHandle nh;

  CylinderSeg cylinderseg;
  ros::spin();

  return 0;


}
