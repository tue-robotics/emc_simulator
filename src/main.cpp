#include "world.h"
#include "visualization.h"
#include "heightmap.h"
#include "lrf.h"
#include "door.h"

#include <unistd.h>

#include <tue/profiling/timer.h>

#include <geolib/ros/msg_conversions.h>

#include <geolib/Box.h>

// ROS
#include <ros/init.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <ros/package.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <iostream>

#include <geolib/CompositeShape.h>
#include "virtualbase.h"
#include "moving_object.h"


geometry_msgs::Twist::ConstPtr base_ref_;
bool request_open_door_;

// ----------------------------------------------------------------------------------------------------

void baseReferenceCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    base_ref_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void openDoorCallback(const std_msgs::Empty::ConstPtr& msg)
{
    request_open_door_ = true;
}

void speakCallback(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "Pico says: " << "\033[1;31m" << msg->data << "\033[0m\n"  << std::endl;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    //check bash arguments
    bool noslip = false;
    for(int i = 1; i < argc; i++){
        std::string str(argv[i]);
        if(str.compare("--noslip")== 0){
            std::cout << "wheelslip disabled" << std::endl;
            noslip=true;
        }
    }

    ros::init(argc, argv, "pico_simulator");

    std::string heightmap_filename;
    heightmap_filename = ros::package::getPath("emc_simulator") + "/data/heightmap.pgm";
    if (argc > 1) {
        std::string str(argv[1]);
        if (str.compare("--noslip") != 0) {
            heightmap_filename = argv[1];
        }
    }

    World world;
    LRF lrf;
    lrf.setAngleLimits(-2, 2);
    lrf.setNumBeams(1000);
    lrf.setRangeLimits(0.01, 10);

    double cycle_freq = 30;
    double cycle_time = 1 / cycle_freq;

    bool visualize = true;

    // Load heightmap
    std::vector<Door> doors;
    geo::ShapePtr heightmap = createHeightMapShape(heightmap_filename, doors);
    if (!heightmap)
    {
        std::cout << "[PICO SIMULATOR] Heightmap could not be loaded" << std::endl;
        return 1;
    }

    world.addObject(geo::Pose3D::identity(), heightmap);


    // Ad moving objects
    MovingObject cart1;
    geo::Pose3D p; p.setOrigin(geo::Vector3(1.5,1.0,0)); p.setRPY(0,0.0,0.3);
    cart1.init_pose = p;
    std::vector<MovingObject> movingobjects;
    movingobjects.push_back(cart1);

    for(std::vector<MovingObject>::iterator it = movingobjects.begin(); it != movingobjects.end(); ++it){
        cart1.id = world.addObject(it->init_pose,makeWorldSimObject(*it),geo::Vector3(0,1,1));
        world.setVelocity(cart1.id,geo::Vector3(0.0,0.0,0.0),0.0);
    }

    // Add robot
    geo::Pose3D robot_pose = geo::Pose3D::identity();
    Id robot_id = world.addObject(robot_pose);
    Virtualbase picobase(1.03,1.0,1.0);
    if(noslip){
        picobase.setWheelUncertaintyFactors(1.0,1.0,1.0);
    }

    // Add door
    for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
    {
        Door& door = *it;
        door.id = world.addObject(door.init_pose, door.shape, geo::Vector3(0, 1, 0));
    }

    // Publishers
    ros::NodeHandle nh;
    ros::Publisher pub_laser = nh.advertise<sensor_msgs::LaserScan>("/pico/laser", 1);
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/pico/odom", 1);

    // Subscribers
    ros::Subscriber sub_base_ref = nh.subscribe<geometry_msgs::Twist>("/pico/cmd_vel", 1, baseReferenceCallback);
    ros::Subscriber sub_open_door = nh.subscribe<std_msgs::Empty>("/pico/open_door", 1, openDoorCallback);
    ros::Subscriber sub_speak = nh.subscribe<std_msgs::String>("/pico/speak",1,speakCallback);

    // Set laser pose (in robot frame)
    geo::Pose3D laser_pose = geo::Pose3D::identity();
    laser_pose.t.z = 0.3;

    ros::Rate r(cycle_freq);
    while(ros::ok())
    {
        base_ref_.reset();
        request_open_door_ = false;
        ros::spinOnce();

        if (base_ref_) // If there is a twist message in the queue
        {
            // Set robot velocity
            picobase.applyTwistAndUpdate(*base_ref_,cycle_time);
            geometry_msgs::Twist actual_twist = picobase.getActualTwist();
            world.setVelocity(robot_id, geo::Vector3(actual_twist.linear.x, actual_twist.linear.y, 0), actual_twist.angular.z);
        }
        else{ // apply previous one again
            picobase.update(cycle_time);
            geometry_msgs::Twist actual_twist = picobase.getActualTwist();
            world.setVelocity(robot_id, geo::Vector3(actual_twist.linear.x, actual_twist.linear.y, 0), actual_twist.angular.z);
        }


        //check if object should start moving
        geo::Vector3 dist_obj_pico = world.object(robot_id).pose.getOrigin() -  world.object(cart1.id).pose.getOrigin();

        if(dist_obj_pico.length() < 1.5){
            world.setVelocity(cart1.id,geo::Vector3(0.0,0.3,0.0),0.0);
        }

        //check collisions with robot
        bool collision = false;
        geo::Vector3 rp1 = world.object(robot_id).pose*geo::Vector3(0.05,0.15,0.0);
        geo::Vector3 rp2 = world.object(robot_id).pose*geo::Vector3(0.05,-0.15,0.0);
        geo::Vector3 rp3 = world.object(robot_id).pose*geo::Vector3(-0.05,0.15,0.0);
        geo::Vector3 rp4 = world.object(robot_id).pose*geo::Vector3(-0.05,-0.15,0.0);
        if( heightmap->intersect(rp1,0.01) || heightmap->intersect(rp2,0.01) || heightmap->intersect(rp3,0.01) || heightmap->intersect(rp4,0.01)){
            collision = true;
        }

        if (request_open_door_)
        {
            for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
            {
                Door& door = *it;
                if (!door.closed)
                    continue;

                // Test if robot is in front of door. If not, don't open it
                geo::Pose3D rel_robot_pose = door.init_pose.inverse() * world.object(robot_id).pose;

                if (std::abs(rel_robot_pose.t.y) > 1 || std::abs(rel_robot_pose.t.x) > door.size / 2)
                    continue;

                world.setVelocity(door.id, door.open_vel, 0);
                door.closed = false;
            }
        }

        // Stop doors that have moved far enough
        for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
        {
            Door& door = *it;
            if (!door.closed && (door.init_pose.t - world.object(door.id).pose.t).length2() > door.open_distance * door.open_distance)
                world.setVelocity(door.id, geo::Vector3(0, 0, 0), 0);
        }

        ros::Time time = ros::Time::now();

        world.update(time.toSec());

        // Create laser data
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.frame_id = "/pico/laser";
        scan_msg.header.stamp = time;
        lrf.generateLaserData(world, world.object(robot_id).pose * laser_pose, scan_msg);
        pub_laser.publish(scan_msg);

        // Create odom data
        nav_msgs::Odometry odom_msg = picobase.getOdom();
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = "odomframe";
        //geo::convert(world.object(robot_id).pose, odom_msg.pose.pose);


        pub_odom.publish(odom_msg);

        // Visualize
        if (visualize)
            visualization::visualize(world, robot_id, collision);

        r.sleep();
    }

    return 0;
}
