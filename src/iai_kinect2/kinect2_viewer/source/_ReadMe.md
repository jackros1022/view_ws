<!-- TOC -->

- [Tips](#tips)
- [kinect_viewer.cpp](#kinect_viewercpp)
    - [1.node不能关闭](#1node不能关闭)
    - [2.ros::Rate rate_loop(0.1);  //定义出错](#2rosrate-rate_loop01--定义出错)
    - [3.viewer.spinOnce();  //这一句干嘛？](#3viewerspinonce--这一句干嘛)
    - [4.变量使用指针类型是不是应该更快一点呢？](#4变量使用指针类型是不是应该更快一点呢)
- [normalEstimation.cpp](#normalestimationcpp)
    - [1.求法线，只能显示第一帧](#1求法线只能显示第一帧)
    - [2.使用isOrgnaized()函数判断点云](#2使用isorgnaized函数判断点云)
- [segmentation_cloud.cpp](#segmentation_cloudcpp)
    - [1.指针为什么不能定义到类中，而只能放到回调函数中？](#1指针为什么不能定义到类中而只能放到回调函数中)
    - [2.指针ModelCoefficients类型的区别？](#2指针modelcoefficients类型的区别)
    - [3.随机采样分割模型以及分割方法](#3随机采样分割模型以及分割方法)
    - [4.ros::spin();  //频率多少？](#4rosspin--频率多少)

<!-- /TOC -->
# Tips
-　处理点云多个步骤，分开写多个node
- 

# kinect_viewer.cpp
## 1.node不能关闭  
1. 点云显示后，直接关闭窗口node被shutdown（）关闭
2. 按下Ctrl+C后，node不能被关闭
## 2.ros::Rate rate_loop(0.1);  //定义出错 
1. 和spinOnce函数一起使用，指定消息发布频率
2. 定义全局变量，能编译无法运行
3. 定义成类的成员，无法编译，报错 `error: expected identifier before numeric constant`
## 3.viewer.spinOnce();  //这一句干嘛？ 
## 4.变量使用指针类型是不是应该更快一点呢？ 

# normalEstimation.cpp
## 1.求法线，只能显示第一帧 
1. 订阅 kinect_viewer 显示第一帧
2. 直接订阅/kinect2/sd/points也只是显示第一帧，及时处理慢也应该有一点点的变化啊
## 2.使用isOrgnaized()函数判断点云 

# segmentation_cloud.cpp
## 1.指针为什么不能定义到类中，而只能放到回调函数中？ 
- pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_input (new pcl::PointCloud< pcl::PointXYZ > );
## 2.指针ModelCoefficients类型的区别？
- pcl::ModelCoefficientsPtr
- pcl::ModelCoefficients::Ptr 这两个指针的区别
## 3.随机采样分割模型以及分割方法
- SACSeg.setModelType(pcl::SACMODEL_PLANE);
- SACSeg.setMethodType(pcl::SAC_RANSAC);
## 4.ros::spin();  //频率多少？　
- 没有所谓是频率，只是让程序在实例化类中持续执行。
- subscriber()函数每接收一个消息，就去调用一次回调函数。
- /todo 设置Kinect2发送频率和订阅函数的消息队列的数据

## 5.3日

