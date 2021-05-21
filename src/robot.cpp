#include "robot.h"

#include "random"

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
    // configure base
    base.setWheelUncertaintyFactors(1.0,1.0,1.0);
    if(config.uncertain_odom.value()){
        std::random_device rd;
        std::mt19937 gen(rd());
        //std::normal_distribution<double> dis(0.0,0.003);
        std::uniform_real_distribution<double> dis(-0.002,0.002);
        base.setWheelUncertaintyFactors(1.0 +dis(gen),1.0+dis(gen),1.0+dis(gen));
    }
    base.setDisableSpeedCap(config.disable_speedcap.value());
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
