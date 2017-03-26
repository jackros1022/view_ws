<!-- TOC -->

- [kinect_viewer.cpp](#kinect_viewercpp)
    - [node不能关闭  2017-03-24](#node不能关闭--2017-03-24)
    - [ros::Rate rate_loop(0.1);  //定义出错 2017-03-25](#rosrate-rate_loop01--定义出错-2017-03-25)
    - [viewer.spinOnce();  //这一句干嘛？ 2017-03-25 19:16:14](#viewerspinonce--这一句干嘛-2017-03-25-191614)
    - [变量使用指针类型是不是应该更快一点呢？ 2017-03-26 23:16:19](#变量使用指针类型是不是应该更快一点呢-2017-03-26-231619)
- [normalEstimation.cpp](#normalestimationcpp)
    - [2017-03-25 求法线，只能显示第一帧](#2017-03-25-求法线只能显示第一帧)

<!-- /TOC -->


# kinect_viewer.cpp
##  node不能关闭  2017-03-24
1. 点云显示后，直接关闭窗口node被shutdown（）关闭
2. 按下Ctrl+C后，node不能被关闭


##  ros::Rate rate_loop(0.1);  //定义出错 2017-03-25
1. 和spinOnce函数一起使用，指定消息发布频率
2. 定义全局变量，能编译无法运行
3. 定义成类的成员，无法编译，报错 `error: expected identifier before numeric constant`


##  viewer.spinOnce();  //这一句干嘛？ 2017-03-25 19:16:14

## 变量使用指针类型是不是应该更快一点呢？ 2017-03-26 23:16:19

# normalEstimation.cpp
## 2017-03-25 求法线，只能显示第一帧
1. 订阅 kinect_viewer 显示第一帧
2. 直接订阅/kinect2/sd/points也只是显示第一帧，及时处理慢也应该有一点点的变化啊

