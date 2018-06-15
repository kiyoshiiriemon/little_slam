#include <boost/circular_buffer.hpp>
#include "ros/ros.h"
#include "tf/tf.h"
#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "little_slam/framework/SlamFrontEnd.h"
#include "little_slam/cui/FrameworkCustomizer.h"

#include <sstream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

static Pose2D pose;
SlamFrontEnd *sf;
ros::Publisher pcmap_pub;
nav_msgs::Odometry *p_odom = nullptr;
boost::circular_buffer<

void odom_cb(const nav_msgs::Odometry &odom)
{
    //ROS_INFO("odom received");
    if (!p_odom) {
        p_odom = new nav_msgs::Odometry();
    }
    *p_odom = odom;
}

void scan_cb(const sensor_msgs::LaserScan &scan)
{
    //ROS_INFO("scan received");
    Scan2D scan2d;

    if (!p_odom) return;
    tf::Pose p;
    tf::poseMsgToTF(p_odom->pose.pose, p);
    scan2d.pose.tx = p_odom->pose.pose.position.x;
    scan2d.pose.ty = p_odom->pose.pose.position.y;
    scan2d.pose.th = tf::getYaw(p.getRotation()) / 3.1416 * 180.0;
    scan2d.pose.calRmat();

    for(size_t i=0; i< scan.ranges.size(); ++i) {
        LPoint2D lp;
        double th = scan.angle_min + scan.angle_increment * i;
        double r = scan.ranges[i];
        if (scan.range_min < r && r < scan.range_max) {
            lp.x = r * cos(th);
            lp.y = r * sin(th);
            scan2d.lps.push_back(lp);
        }
    }
    std::cout << "odom :" << pose.tx << " " << pose.ty << std::endl;
    sf->process(scan2d);
    PointCloudMap *map = sf->getPointCloudMap();
    double minx = 0, miny = 0, maxx = 0, maxy = 0;

    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = p_odom->header.frame_id;
    msg->height = msg->width = 1;
    for (auto lp: map->globalMap) {
        msg->points.push_back(pcl::PointXYZ(lp.x, lp.y, 0));
        if (minx > lp.x) {
            minx = lp.x;
        }
        if (miny > lp.y) {
            miny = lp.y;
        }
        if (maxx < lp.x) {
            maxx = lp.x;
        }
        if (maxy < lp.y) {
            maxy = lp.y;
        }
    }
    msg->width = msg->points.size();
    pcmap_pub.publish(msg);
}

int main(int argc, char **argv)
{

    sf = new SlamFrontEnd();
    FrameworkCustomizer fc;
    fc.setSlamFrontEnd(sf);
    fc.makeFramework();
    //fc.customizeA();
    fc.customizeI();

    ros::init(argc, argv, "little_slam");
    ros::NodeHandle n;

    ros::Subscriber laser_sub = n.subscribe("scan", 10, scan_cb);
    ros::Subscriber odom_sub = n.subscribe("odom", 10, odom_cb);
    pcmap_pub = n.advertise<sensor_msgs::PointCloud2>("pcmap", 10);

    ros::spin();

    return 0;
}


