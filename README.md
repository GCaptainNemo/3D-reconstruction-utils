# 三维重建工具箱

## 一、介绍

依赖库: `PCL` +` Eigen`

本仓库保存一些平时写的关于三维重建的工具

## 二、功能

* 利用`PCL`的可视化模块，绘制相机轨迹。(src/draw_trajectory.cpp)
* 多帧深度图转点云 (src/range2pc.cpp)
* 视觉里程计(src/visual_odometry.cpp)

## 三、效果

### 3.1 相机轨迹绘制

<p align="center"><img  src="./resources/res.png" width="40%" height="35%"></p>

<h6 align="center">绘制效果</h6>

### 3.2 多帧RGBD转点云(用视觉里程计估计外参)

<p align="center"><img  src="./resources/range2pc.png" width="40%" height="35%"></p>

<h6 align="center">结果</h6>

## 四、数据来源

[https://vision.in.tum.de/data/datasets/rgbd-dataset/download ](https://vision.in.tum.de/data/datasets/rgbd-dataset/download ) 



