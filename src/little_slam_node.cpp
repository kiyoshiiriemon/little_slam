#include <boost/circular_buffer.hpp>

#include "ros/ros.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"

#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "little_slam/framework/SlamFrontEnd.h"
#include "little_slam/cui/FrameworkCustomizer.h"

#include <sstream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//boost::circular_buffer<sensor_msgs::LaserScan> scan_buf(1000);
boost::circular_buffer<Scan2D> scan_buf(1000);
tf::TransformListener *listener;

static bool make_scan2d(Scan2D &out_scan, const sensor_msgs::LaserScan &scan)
{
    tf::StampedTransform tr;
    try{
        //listener->waitForTransform("/odom", "/base_link", scan.header.stamp, ros::Duration(1.0));
        listener->lookupTransform("/odom", "/base_link", scan.header.stamp, tr);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      return false;
    }
    out_scan.pose.tx = tr.getOrigin().x();
    out_scan.pose.ty = tr.getOrigin().y();
    out_scan.pose.th = RAD2DEG(tf::getYaw(tr.getRotation()));
    out_scan.pose.calRmat();
    //std::cout << "scan pose: " << out_scan.pose.tx << " " << out_scan.pose.ty << " " << out_scan.pose.th << std::endl;

    out_scan.lps.clear();
    for(size_t i=0; i< scan.ranges.size(); ++i) {
        LPoint2D lp;
        double th = scan.angle_min + scan.angle_increment * i;
        double r = scan.ranges[i];
        if (scan.range_min < r && r < scan.range_max) {
            lp.x = r * cos(th);
            lp.y = r * sin(th);
            out_scan.lps.push_back(lp);
        }
    }

    return true;
}

static void scan_cb(const sensor_msgs::LaserScan &scan)
{
    //ROS_INFO("scan received");
    Scan2D scan2d;
    if (make_scan2d(scan2d, scan)) {
        scan_buf.push_back(scan2d);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "little_slam");
    ros::NodeHandle n;

    SlamFrontEnd *sf = new SlamFrontEnd();
    FrameworkCustomizer fc;
    fc.setSlamFrontEnd(sf);
    fc.makeFramework();

    std::string cparam;
    if (ros::param::has("~customize")) {
        ros::param::get("~customize", cparam);
        std::cerr << "Using customize " << cparam << std::endl;
    } else {
        cparam = "I";
    }
    if (cparam.compare("A") == 0) {
        fc.customizeA();
    } else if (cparam.compare("B") == 0) {
        fc.customizeB();
    } else if (cparam.compare("C") == 0) {
        fc.customizeC();
    } else if (cparam.compare("D") == 0) {
        fc.customizeD();
    } else if (cparam.compare("E") == 0) {
        fc.customizeE();
    } else if (cparam.compare("F") == 0) {
        fc.customizeF();
    } else if (cparam.compare("G") == 0) {
        fc.customizeG();
    } else if (cparam.compare("H") == 0) {
        fc.customizeH();
    } else if (cparam.compare("I") == 0) {
        fc.customizeI();
    } else {
        std::cerr << "Invalid customize: " << cparam << std::endl;
        exit(-1);
    }

    ros::Subscriber laser_sub = n.subscribe("scan", 100, scan_cb);
    ros::Publisher pcmap_pub = n.advertise<sensor_msgs::PointCloud2>("pcmap", 10);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path", 10);

    listener = new tf::TransformListener();

    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if (scan_buf.size() == 0) continue;

        Scan2D scan2d = scan_buf.front();
        scan_buf.pop_front();

        if (true) {
            sf->process(scan2d);
            PointCloudMap *map = sf->getPointCloudMap();

            PointCloud::Ptr msg(new PointCloud);
            msg->header.frame_id = "map";
            msg->height = msg->width = 1;
            for (auto lp: map->globalMap) {
                msg->points.push_back(pcl::PointXYZ(lp.x, lp.y, 0));
            }
            msg->width = msg->points.size();
            pcmap_pub.publish(msg);

            nav_msgs::Path path;
            path.header.frame_id = "map";
            for(auto p : map->poses) {
                geometry_msgs::PoseStamped pose_s;
                pose_s.pose.position.x = p.tx;
                pose_s.pose.position.y = p.ty;
                pose_s.pose.position.z = 0;
                pose_s.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, DEG2RAD(p.th));
                path.poses.push_back(pose_s);
            }
            path_pub.publish(path);
        }
    }

    return 0;
}

