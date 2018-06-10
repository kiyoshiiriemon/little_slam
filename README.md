# little_slam
ROS porting of [LittleSLAM](https://github.com/furo-org/LittleSLAM.git)

# Install
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/kiyoshiiriemon/little_slam
cd little_slam/src/little_slam
git clone https://github.com/furo-org/p2o
cd ~/catkin_ws/
catkin_make
``` 

# Run
```
rosrun little_slam little_slam_node
```
Little_slam requires Odometry ("odom") and LaserScan ("scan").

