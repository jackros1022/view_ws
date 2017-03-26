#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

class cloudHandler
{
public:
    cloudHandler()  //构造函数，默认执行
    {
        pcl_sub = nh.subscribe("pcl_output", 10, &cloudHandler::cloudCB, this);     //订阅的消息队列为10
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);        //消息队列缓存为1
    }

    void cloudCB(const sensor_msgs::PointCloud2& input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
        statFilter.setInputCloud(cloud.makeShared()); //数据转换为指针格式
        statFilter.setMeanK(10);        //K设置10
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(cloud_filtered);

        pcl::toROSMsg(cloud_filtered, output);
        pcl_pub.publish(output);    //topic中发布数据 发布速率是多少？==》订阅的
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;   
};

main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_filter"); //初始化节点

    cloudHandler handler;

    ros::spin();    //一直循环  This method is mostly useful when your node does all of its work in subscription callbacks.

    return 0;
}

