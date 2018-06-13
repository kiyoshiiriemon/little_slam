# little_slam
[LittleSLAM](https://github.com/furo-org/LittleSLAM.git)をROSに移植したものです。

# インストール
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/kiyoshiiriemon/little_slam
cd little_slam/src/little_slam
git clone https://github.com/furo-org/p2o
cd ~/catkin_ws/
catkin_make
``` 

# 実行方法
```
rosrun little_slam little_slam_node
```
入力トピックは
- nav_msgs::Odometry ("odom")
- sensor_msgs::LaserScan ("scan")

出力は
- sensor_msgs::PointCloud ("pcmap")

です。

