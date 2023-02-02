#ifndef PICO_SIMULATOR_ROBOT_H_
#define PICO_SIMULATOR_ROBOT_H_

#include "virtualbase.h"
#include "jsonconfig.h"
#include "world.h"

// ROS
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <iostream>
#include <string>

class Robot
{

public:
    Robot(const std::string &name, Id id, bool disable_speedcap = false, bool uncertain_odom = false);

    ~Robot();

    void pubTransform(geo::Pose3D pose);

    geo::Pose3D laser_pose;
    std::string robot_name;
    Id robot_id;
    Virtualbase base;

    geometry_msgs::Twist::ConstPtr base_ref_;
    bool request_open_door_;

    // Publisher
    ros::Publisher pub_bumperF;
    ros::Publisher pub_bumperR;
    ros::Publisher pub_laser;
    ros::Publisher pub_odom;
    tf2_ros::TransformBroadcaster pub_tf2;
    tf2_ros::StaticTransformBroadcaster pub_tf2static;

private:
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber sub_base_ref;
    ros::Subscriber sub_open_door;
    ros::Subscriber sub_speak;
    void baseReferenceCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void openDoorCallback(const std_msgs::Empty::ConstPtr& msg);
    void speakCallback(const std_msgs::String::ConstPtr& msg);
};

typedef std::shared_ptr<Robot> RobotPtr;


#endif
