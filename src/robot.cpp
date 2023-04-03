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

void Robot::mapCallback(const nav_msgs::MapMetaData::ConstPtr& msg)
{
    mapconfig.mapResolution = msg->resolution;
    mapconfig.mapOffsetX =  ((msg->height)*msg->resolution)/2;
    mapconfig.mapOffsetY = -((msg->width)*msg->resolution)/2;

    tf2::Quaternion q(msg->origin.orientation.x, 
                      msg->origin.orientation.y, 
                      msg->origin.orientation.z, 
                      msg->origin.orientation.w);

    tf2::Matrix3x3 T(q);

    double roll, pitch, yaw;
    T.getRPY(roll, pitch, yaw);

    mapconfig.mapOrientation = yaw + M_PI/2;
    mapconfig.mapInitialised = true;
    sub_mapdata.shutdown();
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
    //parameters
    std::string laser_param, odom_param, bumper_f_param, bumper_b_param, base_ref_param, open_door_param, speak_param, play_param;
    if (!nh.getParam("laser_", laser_param)) {ROS_ERROR_STREAM("Parameter " << "laser_" << " not set");};
    if (!nh.getParam("odom_", odom_param)) {ROS_ERROR_STREAM("Parameter " << "odom_" << " not set");};
    if (!nh.getParam("bumper_f_", bumper_f_param)) {ROS_ERROR_STREAM("Parameter " << "bumper_f_" << " not set");};
    if (!nh.getParam("bumper_b_", bumper_b_param)) {ROS_ERROR_STREAM("Parameter " << "bumper_b_" << " not set");};
    //if (!nh.getParam("base_ref_", base_ref_param)) {ROS_ERROR_STREAM("Parameter " << "base_ref_" << " not set");};
    if (!nh.getParam("open_door_", open_door_param)) {ROS_ERROR_STREAM("Parameter " << "open_door_" << " not set");};
    if (!nh.getParam("speak_", speak_param)) {ROS_ERROR_STREAM("Parameter " << "speak_" << " not set");};
    //if (!nh.getParam("play_", play_param)) {ROS_ERROR_STREAM("Parameter " << "play_" << " not set");};

    // Publishers
    pub_bumperF = nh.advertise<std_msgs::Bool>(bumper_f_param, 1);
    pub_bumperR = nh.advertise<std_msgs::Bool>(bumper_b_param, 1);
    pub_laser = nh.advertise<sensor_msgs::LaserScan>(laser_param, 1);
    pub_odom = nh.advertise<nav_msgs::Odometry>(odom_param, 1);
    pub_joints_ground_truth = nh.advertise<sensor_msgs::JointState>("/viz_ground_truth/joint_states", 1);
    pub_joints_internal = nh.advertise<sensor_msgs::JointState>("/viz_internal/joint_states", 1);

    // Subscribers
    sub_base_ref = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &Robot::baseReferenceCallback, this);
    sub_open_door = nh.subscribe<std_msgs::Empty>(open_door_param, 1, &Robot::openDoorCallback, this);
    sub_speak = nh.subscribe<std_msgs::String>(speak_param, 1, &Robot::speakCallback, this);
    sub_mapdata = nh.subscribe<nav_msgs::MapMetaData>("/map_metadata",1, &Robot::mapCallback, this);

}   

// ----------------------------------------------------------------------------------------------------

Robot::~Robot()
{
}

void Robot::pubTransform(const geo::Pose3D &pose, const MapConfig &mapconfig)
{
    // Publish jointstate
    sensor_msgs::JointState jointState;
    jointState.header.stamp = ros::Time::now();
    jointState.name = {"front_left_wheel_hinge", 
                       "front_right_wheel_hinge", 
                       "rear_left_wheel_hinge", 
                       "rear_right_wheel_hinge"};
    jointState.position = {0, 0, 0, 0};
    pub_joints_ground_truth.publish(jointState);
    pub_joints_internal.publish(jointState);

    // Publish tf transform
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "ground_truth/base_link";
    transformStamped.transform.translation.x =   pose.t.x + cos(mapconfig.mapOrientation) * mapconfig.mapOffsetX 
                                                          + sin(mapconfig.mapOrientation) * mapconfig.mapOffsetY;

    transformStamped.transform.translation.y =   pose.t.y - sin(mapconfig.mapOrientation) * mapconfig.mapOffsetX
                                                          + cos(mapconfig.mapOrientation) * mapconfig.mapOffsetY;

    transformStamped.transform.translation.z =   pose.t.z + 0.044;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.getYaw() + mapconfig.mapOrientation);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    pub_tf2.sendTransform(transformStamped);
}

void Robot::internalTransform()
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "ground_truth/base_link";
    transformStamped.child_frame_id = "internal/base_link";
    transformStamped.transform.rotation.w = 1;
    pub_tf2static.sendTransform(transformStamped);
}
