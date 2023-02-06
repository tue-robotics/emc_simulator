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
    // Publishers
    pub_bumperF = nh.advertise<std_msgs::Bool>("/" + robot_name + "/base_f_bumper_sensor", 1);
    pub_bumperR = nh.advertise<std_msgs::Bool>("/" + robot_name + "/base_b_bumper_sensor", 1);
    pub_laser = nh.advertise<sensor_msgs::LaserScan>("/transformed_scan", 1);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    pub_joints = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    // Subscribers
    sub_base_ref = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &Robot::baseReferenceCallback, this);
    sub_open_door = nh.subscribe<std_msgs::Empty>("/" + robot_name + "/open_door", 1, &Robot::openDoorCallback, this);
    sub_speak = nh.subscribe<std_msgs::String>("/" + robot_name + "/text_to_speech/input", 1, &Robot::speakCallback, this);
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
    pub_joints.publish(jointState);

    // Publish tf transform
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "/base_link";
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
