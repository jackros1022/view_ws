# 顺序
- filter
- downsampling
- matching
- partitioning
- segmentation
- 

# pcl_filter.cpp
## pcl_pub.publish(output);  2017-03-25 
1. 发布速率是多少？
1. 为什么没有采用ros::Rate？

- `发布的速率和订阅的速率一样的，也可以说小于等于`


# pcl_matching.cpp
## void cloudCB(const sensor_msgs::PointCloud2 &input) {} 2017-03-25 20:11:58
1. 形参是常引用，不是指针类型
2. `回调函数的参数自己定义的`