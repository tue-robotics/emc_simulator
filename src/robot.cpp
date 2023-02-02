#include "robot.h"
// ----------------------------------------------------------------------------------------------------

void Robot::baseReferenceCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    base_ref_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void Robot::openDoorCallback(const std_msgs::Empty::ConstPtr& /*msg*/)
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
    request_open_door_ = false;

    // Set laser pose (in robot frame)
    laser_pose = geo::Pose3D::identity();
    laser_pose.t.z = 0.3;

    nh = ros::NodeHandle();
    // Publishers
    pub_bumperF = nh.advertise<std_msgs::Bool>("/" + robot_name + "/base_f_bumper_sensor", 1);
    pub_bumperR = nh.advertise<std_msgs::Bool>("/" + robot_name + "/base_b_bumper_sensor", 1);
    pub_laser = nh.advertise<sensor_msgs::LaserScan>("/transformed_scan", 1);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 1);

    // Subscribers
    sub_base_ref = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &Robot::baseReferenceCallback, this);
    sub_open_door = nh.subscribe<std_msgs::Empty>("/" + robot_name + "/open_door", 1, &Robot::openDoorCallback, this);
    sub_speak = nh.subscribe<std_msgs::String>("/" + robot_name + "/text_to_speech/input", 1, &Robot::speakCallback, this);

    // Publish LRF-base_link transform only once
    geometry_msgs::TransformStamped tfbase2lrf;

    tfbase2lrf.header.stamp = ros::Time::now();
    tfbase2lrf.header.frame_id = robot_name + "/base_link";
    tfbase2lrf.child_frame_id = "laserframe";
    tfbase2lrf.transform.translation.x = 0;
    tfbase2lrf.transform.translation.y = 0;
    tfbase2lrf.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0,0,0);
    tfbase2lrf.transform.rotation.x = quat.x();
    tfbase2lrf.transform.rotation.y = quat.y();
    tfbase2lrf.transform.rotation.z = quat.z();
    tfbase2lrf.transform.rotation.w = quat.w();
    pub_tf2static.sendTransform(tfbase2lrf);
}   

// ----------------------------------------------------------------------------------------------------

Robot::~Robot()
{
}

void Robot::pubTransform(geo::Pose3D pose)
{
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = robot_name + "/base_link";
  transformStamped.transform.translation.x = pose.t.x;
  transformStamped.transform.translation.y = pose.t.y;
  transformStamped.transform.translation.z = pose.t.z;
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.getYaw());
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  pub_tf2.sendTransform(transformStamped);
}