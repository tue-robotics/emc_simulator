#include "world.h"
#include "visualization.h"
#include "heightmap.h"
#include "lrf.h"
#include "door.h"
#include "robot.h"

#include <unistd.h>
#include <tue/profiling/timer.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/Box.h>
#include "jsonconfig.h"

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
#include <string>

#include <geolib/CompositeShape.h>
#include "virtualbase.h"
#include "moving_object.h"
#include "random"

#include <vector>


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

// -----------------------------------------------------------------------------------------------------

int main(int argc, char **argv){

    ros::init(argc, argv, "pico_simulator");

    std::string heightmap_filename;
    heightmap_filename = ros::package::getPath("emc_simulator") + "/data/heightmap.pgm";

    std::string config_filename;
    config_filename = ros::package::getPath("emc_simulator") + "/data/defaultconfig.json";

    for(int i = 1; i < argc; i++){
        std::string config_supplied("--config");
        std::string map_supplied("--map");
        if(config_supplied.compare(argv[i])==0){
            std::cout << "User config file supplied!" << std::endl;
            config_filename = std::string(argv[i+1]);
        }
        if(map_supplied.compare(argv[i])==0){
            std::cout << "User map file supplied!" << std::endl;
            heightmap_filename = std::string(argv[i+1]);
        }
    }

    Config config(config_filename);
    config.print();

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

    // Get centerbox for height map (the visualization is constraining canvas within this box)
    visualization::Bbox bbox; bbox.xmin = -1; bbox.xmax = 1; bbox.ymin=-1; bbox.ymax = 1;

    std::vector<geo::Vector3> meshpoints = heightmap->getMesh().getPoints();

    bbox.xmin = std::max_element(meshpoints.begin(), meshpoints.end(),[](geo::Vec3T<double> a, geo::Vec3T<double> b){return a.getX() > b.getX() ;} )->getX();
    bbox.ymin = std::max_element(meshpoints.begin(), meshpoints.end(),[](geo::Vec3T<double> a, geo::Vec3T<double> b){return a.getY() > b.getY() ;} )->getY();
    bbox.xmax = std::max_element(meshpoints.begin(), meshpoints.end(),[](geo::Vec3T<double> a, geo::Vec3T<double> b){return a.getX() < b.getX() ;} )->getX();
    bbox.ymax = std::max_element(meshpoints.begin(), meshpoints.end(),[](geo::Vec3T<double> a, geo::Vec3T<double> b){return a.getY() < b.getY() ;} )->getY();

    //std::cout << "bbox: " << bbox.xmin << "   " << bbox.xmax << "  "  << bbox.ymin << "  " << bbox.ymax << std::endl;




    // Ad moving objects
    for(std::vector<MovingObject>::iterator it = config.moving_objects.value().begin(); it != config.moving_objects.value().end(); ++it) {
        it->id = world.addObject(it->init_pose,makeWorldSimObject(*it),geo::Vector3(0,1,1));
        world.setVelocity(it->id,geo::Vector3(0.0,0.0,0.0),0.0);
    }

    // Add robot
    geo::Pose3D robot_pose = geo::Pose3D::identity();
    Id robot_id = world.addObject(robot_pose);
    Robot robot("pico", robot_id);
    std::random_device rd;
    std::mt19937 gen(rd());
    //std::normal_distribution<double> dis(0.0,0.003);
    std::uniform_real_distribution<double> dis(-0.002,0.002);
    Virtualbase picobase(1.0 +dis(gen),1.0+dis(gen),1.0+dis(gen), config.disable_speedcap.value());
    if(!config.uncertain_odom.value()){
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
    double time_ = 0;
    double dt;
    while(ros::ok())
    {
        base_ref_.reset();
        request_open_door_ = false;
        ros::spinOnce();
        ros::Time time = ros::Time::now();

        if(time_==0){
            dt = 0;
            time_ = time.toSec();
        }
        else{
            dt = time.toSec() - time_;
            time_ = time.toSec();
        }

        if (base_ref_) // If there is a twist message in the queue
        {
            // Set robot velocity
            geometry_msgs::Twist cmd = *base_ref_;
            picobase.applyTwistAndUpdate(cmd, dt);
        }
        else{ // apply previous one again
            picobase.update(dt);
        }
        geometry_msgs::Twist actual_twist = picobase.getActualTwist();
        world.setVelocity(robot_id, geo::Vector3(actual_twist.linear.x, actual_twist.linear.y, 0), actual_twist.angular.z);


        //check if object should start moving
        for(std::vector<MovingObject>::iterator it = config.moving_objects.value().begin(); it != config.moving_objects.value().end(); ++it){

            // check if it should start
            geo::Vector3 dist_obj_pico = world.object(robot_id).pose.getOrigin() -  world.object(it->id).pose.getOrigin();
            double safety_radius = sqrt( std::pow(it->width/2,2) + std::pow(it->length/2,2)) + 0.3;
            if(dist_obj_pico.length() < it->trigger_radius && it->is_moving == false && it->finished_moving == false){
                it->is_moving = true;
                geo::Vector3 unit_vel = (it->final_pose.getOrigin() - it->init_pose.getOrigin());
                unit_vel =world.object(it->id).pose.R.transpose()*unit_vel / unit_vel.length();
                world.setVelocity(it->id, unit_vel*it->velocity,0.0);
            }

            // check if it should stop
            geo::Vector3 dist_obj_dest = world.object(it->id).pose.getOrigin() -  it->final_pose.getOrigin();
            if(dist_obj_dest.length() < 0.1 && it->is_moving == true && it->finished_moving == false && it->repeat == false){
                it->is_moving = false;
                it->finished_moving = true;
                world.setVelocity(it->id, geo::Vector3(0.0,0.0,0.0),0.0);
            }

            // reverse and repeat ... :o)
            if(dist_obj_dest.length() < 0.1 && it->is_moving == true && it->finished_moving == false && it->repeat == true){
                it->is_moving = true;
                it->finished_moving = false;
                geo::Pose3D placeholder = it->init_pose;
                it->init_pose = it->final_pose;
                it->final_pose = placeholder;
                geo::Vector3 unit_vel = (it->final_pose.getOrigin() - it->init_pose.getOrigin());
                unit_vel =world.object(it->id).pose.R.transpose()*unit_vel / unit_vel.length();
                world.setVelocity(it->id, unit_vel*it->velocity,0.0);
            }

            // Check if it should pause
            if(dist_obj_pico.length() < safety_radius && it->is_moving == true && it->is_paused == false){
                it->is_paused = true;
                world.setVelocity(it->id, geo::Vector3(0.0,0.0,0.0),0.0);
            }

            // Check if it should continue after being paused
            if(dist_obj_pico.length() > safety_radius && it->is_paused == true){
                it->is_paused = false;
                geo::Vector3 unit_vel = (it->final_pose.getOrigin() - it->init_pose.getOrigin());
                unit_vel =world.object(it->id).pose.R.transpose()*unit_vel / unit_vel.length();
                world.setVelocity(it->id, unit_vel*it->velocity,0.0);
            }



        }

        //check collisions with robot
        bool collision = false;
        geo::Vector3 rp1 = world.object(robot_id).pose*geo::Vector3(0.08,0.18,0.0);
        geo::Vector3 rp2 = world.object(robot_id).pose*geo::Vector3(0.08,-0.18,0.0);
        geo::Vector3 rp3 = world.object(robot_id).pose*geo::Vector3(-0.08,0.18,0.0);
        geo::Vector3 rp4 = world.object(robot_id).pose*geo::Vector3(-0.08,-0.18,0.0);
        if( heightmap->intersect(rp1,0.001) || heightmap->intersect(rp2,0.001) || heightmap->intersect(rp3,0.001) || heightmap->intersect(rp4,0.001)){
            collision = true;
        }

        for(std::vector<MovingObject>::iterator it = config.moving_objects.value().begin(); it != config.moving_objects.value().end(); ++it){
            rp1 = world.object(it->id).pose.inverse()* world.object(robot_id).pose*geo::Vector3(0.08,0.18,0.0);
            rp2 = world.object(it->id).pose.inverse()* world.object(robot_id).pose*geo::Vector3(0.08,-0.18,0.0);
            rp3 = world.object(it->id).pose.inverse()* world.object(robot_id).pose*geo::Vector3(-0.08,0.18,0.0);
            rp4 = world.object(it->id).pose.inverse()* world.object(robot_id).pose*geo::Vector3(-0.08,-0.18,0.0);

            if(  world.object(it->id).shape->intersect(rp1,0.001) ||
                 world.object(it->id).shape->intersect(rp2,0.001) ||
                 world.object(it->id).shape->intersect(rp3,0.001) ||
                 world.object(it->id).shape->intersect(rp4,0.001)){
                collision = true;
            }
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



        //
        if(config.uncertain_odom.value() && time.sec%6 == 0 ){
            picobase.setWheelUncertaintyFactors(1.0 +dis(gen),1.0+dis(gen),1.0+dis(gen));
        }

        world.update(time.toSec());

        // Create laser data
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.frame_id = "/pico/laser";
        scan_msg.header.stamp = time;
        lrf.generateLaserData(world, world.object(robot_id).pose * laser_pose, scan_msg);
        pub_laser.publish(scan_msg);

        // Create odom data
        nav_msgs::Odometry odom_msg = picobase.getOdom();
        if(!config.uncertain_odom.value()){
            geo::convert(world.object(robot_id).pose, odom_msg.pose.pose);
        }
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = "odomframe";

        pub_odom.publish(odom_msg);

        // Visualize
        if (visualize)
            visualization::visualize(world, robot_id, collision, config.show_full_map.value(),bbox);

        r.sleep();
    }

    return 0;
}
