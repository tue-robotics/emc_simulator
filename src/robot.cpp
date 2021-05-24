#include "robot.h"

/*
// ----------------------------------------------------------------------------------------------------

void Robot::baseReferenceCallback(geometry_msgs::Twist::ConstPtr& msg)
{
    base_ref_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Robot::openDoorCallback(std_msgs::Empty::ConstPtr& msg)
{
    request_open_door_ = true;
}

void Robot::speakCallback(std_msgs::String::ConstPtr& msg)
{
    std::cout << "Pico says: " << "\033[1;31m" << msg->data << "\033[0m\n"  << std::endl;
}

// ----------------------------------------------------------------------------------------------------
*/

Robot::Robot(std::string name, Id id)
{
    //nh = nodehandle;
    robot_name = name;
    robot_id = id;

    /*
    // configure lrf
    lrf.setAngleLimits(-2, 2);
    lrf.setNumBeams(1000);
    lrf.setRangeLimits(0.01, 10);
    laser_pose = geo::Pose3D::identity();
    laser_pose.t.z = 0.3;

    */

    /*
    // Publishers
    ros::NodeHandle nh;
    pub_laser = nh->advertise<sensor_msgs::LaserScan>("/pico/laser", 1);
    pub_odom = nh->advertise<nav_msgs::Odometry>("/pico/odom", 1);

    // Subscribers
    sub_base_ref = nh->subscribe<geometry_msgs::Twist>("/pico/cmd_vel", 1, baseReferenceCallback);
    sub_open_door = nh->subscribe<std_msgs::Empty>("/pico/open_door", 1, openDoorCallback);
    sub_speak = nh->subscribe<std_msgs::String>("/pico/speak",1,speakCallback);
    */
}

// ----------------------------------------------------------------------------------------------------

Robot::~Robot()
{
}
