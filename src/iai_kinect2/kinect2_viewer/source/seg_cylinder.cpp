#include <ros/ros.h>

#include <pcl/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/sac_segmentation.h>

class CylinderSeg
{
public:
  CylinderSeg()
  {
//    pcl_sub = nh.advertise("/read_pcl_out", 1, &CylinderSeg::cylinderCB, this); 能用点心吗？
    pcl_sub = nh.subscribe("/read_pcl_out", 1, &CylinderSeg::cylinderCB, this);

  }

public:
  void cylinderCB(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    ROS_INFO("I go to cylinderCB().");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input,*cloud_input);
    ROS_INFO("I get cloud, I will work now ! ! ! ");

    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(cloud_input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0 , 1.5);
    pass.filter(*cloud_filtered);
    ROS_INFO("PassThrough Size:[%ld]",cloud_filtered->points.size());

    //Estimate point Normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal (new pcl::PointCloud<pcl::Normal>);
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
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
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normal);
    seg.segment(*inliers_plane, *cofficients_plane);
    std::cerr << "Plane coefficients: " << *cofficients_plane << std::endl;
    ROS_INFO( "Plane coefficients: [%ld]",*cofficients_plane);

    pcl::ExtractIndices<pcl::PointXYZ> extract; //点云提取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud_filtered);
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
    ROS_INFO("I get cofficients_cylinder:[%ld] ", *cofficients_cylinder);

    // extract cylinders
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud_no_plane);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);
    ROS_INFO("haha ! I finally get the cylinder.");

    ros::shutdown();




  }
private:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;

  ros::Timer viewertimer;


  };
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cylinder_segmentation");
  ROS_INFO("Hello world!");

  CylinderSeg cylindeseg;

  ros::spin();

  return 0;
}
