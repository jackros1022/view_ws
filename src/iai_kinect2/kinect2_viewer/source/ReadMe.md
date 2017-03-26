[TOC]

# kinect_viewer.cpp
## 2017-03-24 node不能关闭
1. 点云显示后，直接关闭窗口node被shutdown（）关闭
2. 按下Ctrl+C后，node不能被关闭


## 2017-03-25 ros::Rate rate_loop(0.1);  //定义出错
1. 和spinOnce函数一起使用，指定消息发布频率
2. 定义全局变量，能编译无法运行
3. 定义成类的成员，无法编译，报错 `error: expected identifier before numeric constant`


## 2017-03-25 19:16:14 viewer.spinOnce();  //这一句干嘛？

# normalEstimation.cpp
## 2017-03-25 求法线，只能显示第一帧
1. 订阅 kinect_viewer 显示第一帧
2. 直接订阅/kinect2/sd/points也只是显示第一帧，及时处理慢也应该有一点点的变化啊