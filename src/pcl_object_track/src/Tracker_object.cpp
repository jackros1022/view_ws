#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

//openCV includes
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>


class Object_track
{
	cv::Mat cv_image;
	pcl::PCLPointCloud2 pcl_pointcloud;
	ros::NodeHandle nh;
    ros::Publisher pub_1;
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
    

 public:

 	Object_track();

 private:

    void pointcloud_callback(const pcl::PCLPointCloud2ConstPtr& pcl_ptr);
    void image_callback(const sensor_msgs::ImageConstPtr& img_ptr);

};

Object_track::Object_track()    //构造函数
{
    pub_1=nh.advertise<sensor_msgs::PointCloud2>("pointcloud/object", 1);

    sub_1=nh.subscribe<pcl::PCLPointCloud2>("/kinect2/sd/points", 1, &Object_track::pointcloud_callback,this);
    sub_2=nh.subscribe<sensor_msgs::Image>("image_mask", 1, &Object_track::image_callback,this);
    
}

void Object_track::image_callback(const sensor_msgs::ImageConstPtr& img_ptr)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img_ptr);
	cv_image=cv_ptr->image;
    
}

void Object_track::pointcloud_callback(const pcl::PCLPointCloud2ConstPtr& pcl_ptr)
{
    
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;      // Convert PCL to XYZ data type
    pcl::fromPCLPointCloud2(*pcl_ptr, cloud_xyz);  //变身
    pcl::PointCloud<pcl::PointXYZ> cloud_new;
    cloud_new.width=cloud_xyz.width;
    cloud_new.height=cloud_xyz.height;
    cloud_new.points.resize(cloud_new.width*cloud_new.height);

    int count=0;
    //拷贝点云
    for(int j=0;j<cv_image.rows;j++) 
   {
     for (int i=0;i<cv_image.cols;i++)
     {

      if(cv_image.at<uchar>(j,i)==255 ) 
       {
        cloud_new.points[count].x=cloud_xyz.points[count].x;
        cloud_new.points[count].y=cloud_xyz.points[count].y;
        cloud_new.points[count].z=cloud_xyz.points[count].z;
        ROS_INFO("Data assigned: %G", cloud_new.points[count].z );
       }  
       count++;        
     }
    }
    pcl::toPCLPointCloud2(cloud_new, pcl_pointcloud);
    pcl_pointcloud.header.frame_id=pcl_ptr->header.frame_id;
    pcl_pointcloud.header.stamp=pcl_ptr->header.stamp;

    pub_1.publish(pcl_pointcloud);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Object_track");
  
  Object_track tracker;
  
  ros::spin();
}
