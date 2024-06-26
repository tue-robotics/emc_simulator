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
    ROS_WARN_STREAM(robot_name << " says: " << "\033[1;31m" << msg->data << "\033[0m\n");
}

// ----------------------------------------------------------------------------------------------------

Robot::Robot(const std::string &name, Id id, bool disable_speedcap, bool uncertain_odom) : base(disable_speedcap, uncertain_odom)
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
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);

    // Subscribers
    sub_base_ref = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &Robot::baseReferenceCallback, this);
    sub_open_door = nh.subscribe<std_msgs::Empty>("/" + robot_name + "/open_door", 1, &Robot::openDoorCallback, this);
    sub_speak = nh.subscribe<std_msgs::String>("/" + robot_name + "/text_to_speech/input", 1, &Robot::speakCallback, this);
}   

// ----------------------------------------------------------------------------------------------------

Robot::~Robot()
{
}

void Robot::pubTransform(const geo::Pose3D &pose)
{
    // Calculate tf transform
    tf2::Transform tf_robot;

    tf_robot.setOrigin(tf2::Vector3(pose.t.x, pose.t.y, pose.t.z + 0.044));
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.getYaw());
    tf_robot.setRotation(q);

    // Publish tf transform
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "robot_pose";
    transformStamped.transform.translation.x =   tf_robot.getOrigin().x();
    transformStamped.transform.translation.y =   tf_robot.getOrigin().y();
    transformStamped.transform.translation.z =   tf_robot.getOrigin().z();

    transformStamped.transform.rotation.x = tf_robot.getRotation().x();
    transformStamped.transform.rotation.y = tf_robot.getRotation().y();
    transformStamped.transform.rotation.z = tf_robot.getRotation().z();
    transformStamped.transform.rotation.w = tf_robot.getRotation().w();
    pub_tf2.sendTransform(transformStamped);

    // Publish posestamped
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = "map";
    poseStamped.pose.position.x =   tf_robot.getOrigin().x();
    poseStamped.pose.position.y =   tf_robot.getOrigin().y();
    poseStamped.pose.position.z =   tf_robot.getOrigin().z();

    poseStamped.pose.orientation.x = tf_robot.getRotation().x();
    poseStamped.pose.orientation.y = tf_robot.getRotation().y();
    poseStamped.pose.orientation.z = tf_robot.getRotation().z();
    poseStamped.pose.orientation.w = tf_robot.getRotation().w();
    pub_pose.publish(poseStamped);
}

void Robot::internalTransform()
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "robot_pose";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.rotation.w = 1;
    pub_tf2static.sendTransform(transformStamped);
}
