#include "robot.h"

// ----------------------------------------------------------------------------------------------------

void Robot::baseReferenceCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    base_ref_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Robot::openDoorCallback(const std_msgs::Empty::ConstPtr& msg)
{
    request_open_door_ = true;
}

void Robot::speakCallback(const std_msgs::String::ConstPtr& msg)
{
    std::cout << robot_name << " says: " << "\033[1;31m" << msg->data << "\033[0m\n"  << std::endl;
}

// ----------------------------------------------------------------------------------------------------

Robot::Robot(std::string name, Id id)
{
    robot_name = name;
    robot_id = id;

    // Set laser pose (in robot frame)
    laser_pose = geo::Pose3D::identity();
    laser_pose.t.z = 0.3;

    nh = ros::NodeHandle();
    // Publishers
    pub_laser = nh.advertise<sensor_msgs::LaserScan>("/" + robot_name + "/base_scan", 1);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/" + robot_name + "/base/measurements", 1);

    // Subscribers
    sub_base_ref = nh.subscribe<geometry_msgs::Twist>("/" + robot_name + "/base/references", 1, &Robot::baseReferenceCallback, this);
    sub_open_door = nh.subscribe<std_msgs::Empty>("/" + robot_name + "/open_door", 1, &Robot::openDoorCallback, this);
    sub_speak = nh.subscribe<std_msgs::String>("/" + robot_name + "/speak", 1, &Robot::speakCallback, this);
}

// ----------------------------------------------------------------------------------------------------

Robot::~Robot()
{
}
