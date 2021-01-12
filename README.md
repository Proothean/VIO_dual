# VIO-Dual：Stero VIO based on VINS_mono


## 1.Requirements
```
Ubuntu 16.04/Ubuntu 18.04
ROS Kinetic/melodic
Ceres Solver
OpenCV 3.3.1
Eigen 3.3.3
```

## 2.Deployment
```
cd ~/catkin_ws/src
git clone https://github.com/iwander-all/VIO_dual.git
cd ../
catkin_make
```

## 3. Download dataset
[EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)。

Open a terminal：
```
roscore
```
Open another two terminals:
```
source ~/catkin_dual/devel/setup.bash
```

and type:
```
roslaunch vins_estimator euroc_no_posegraph.launch  
roslaunch vins_estimator vins_rviz.launch
```

Play dataset on another terminal with：
```
rosbag play YOUR_PATH_TO_DATASET/YOUR_DATASET.bag 
```

## 4.Reference
T.Qin, P.L.Li, S.J.Shen, VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator.

