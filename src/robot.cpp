#include "robot.h"

Robot::Robot(const std::string &name, Id id, bool disable_speedcap, bool uncertain_odom): Node("robot"), base(disable_speedcap, uncertain_odom), pub_tf2(this), pub_tf2static(this)
{
    robot_name = name;
    robot_id = id;
    request_open_door_ = false;

    // Set laser pose (in robot frame)
    laser_pose = geo::Pose3D::identity();
    laser_pose.t.z = 0.3;

    // Publishers
    pub_bumperF = this->create_publisher<std_msgs::msg::Bool>("/" + robot_name + "/base_f_bumper_sensor", 10);
    pub_bumperR = this->create_publisher<std_msgs::msg::Bool>("/" + robot_name + "/base_b_bumper_sensor", 10);
    pub_laser = this->create_publisher<sensor_msgs::msg::LaserScan>("/transformed_scan", 10);
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    pub_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
    pub_marker = this->create_publisher<visualization_msgs::msg::Marker>("/robot_marker", 10);

    // Subscribers
    sub_base_ref = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&Robot::baseReferenceCallback, this, std::placeholders::_1));
    sub_open_door = this->create_subscription<std_msgs::msg::Empty>(
        "/" + robot_name + "/open_door", 10, std::bind(&Robot::openDoorCallback, this, std::placeholders::_1));
    sub_speak = this->create_subscription<std_msgs::msg::String>(
        "/" + robot_name + "/text_to_speech/input", 10, std::bind(&Robot::speakCallback, this, std::placeholders::_1));
}

Robot::~Robot()
{
}

void Robot::baseReferenceCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msg)
{
    base_ref_ = msg;
}

void Robot::openDoorCallback(const std::shared_ptr<const std_msgs::msg::Empty>& /*msg*/)
{
    request_open_door_ = true;
}

void Robot::speakCallback(const std::shared_ptr<const std_msgs::msg::String>& msg)
{
    RCLCPP_WARN(this->get_logger(), "%s says: %s", robot_name.c_str(), msg->data.c_str());
}

void Robot::pubMarker(const geo::Pose3D &pose)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "robot_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = pose.t.x;
    marker.pose.position.y = pose.t.y;
    marker.pose.position.z = pose.t.z + 0.044;

    tf2::Quaternion q;
    q.setRPY(0, 0, pose.getYaw());
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();


    marker.scale.x = 0.21; 
    marker.scale.y = 0.25;
    marker.scale.z = 0.2;
    marker.color.r = 0.2; 
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    marker.color.a = 1.0; 

    pub_marker->publish(marker);
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
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "robot_pose";
    transformStamped.transform.translation.x = tf_robot.getOrigin().x();
    transformStamped.transform.translation.y = tf_robot.getOrigin().y();
    transformStamped.transform.translation.z = tf_robot.getOrigin().z();

    transformStamped.transform.rotation.x = tf_robot.getRotation().x();
    transformStamped.transform.rotation.y = tf_robot.getRotation().y();
    transformStamped.transform.rotation.z = tf_robot.getRotation().z();
    transformStamped.transform.rotation.w = tf_robot.getRotation().w();
    pub_tf2.sendTransform(transformStamped);

    // Publish posestamped
    geometry_msgs::msg::PoseStamped poseStamped;
    poseStamped.header.stamp = this->now();
    poseStamped.header.frame_id = "map";
    poseStamped.pose.position.x = tf_robot.getOrigin().x();
    poseStamped.pose.position.y = tf_robot.getOrigin().y();
    poseStamped.pose.position.z = tf_robot.getOrigin().z();

    poseStamped.pose.orientation.x = tf_robot.getRotation().x();
    poseStamped.pose.orientation.y = tf_robot.getRotation().y();
    poseStamped.pose.orientation.z = tf_robot.getRotation().z();
    poseStamped.pose.orientation.w = tf_robot.getRotation().w();
    pub_pose->publish(poseStamped);
}

void Robot::internalTransform()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "robot_pose";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.rotation.w = 1;
    pub_tf2static.sendTransform(transformStamped);
}