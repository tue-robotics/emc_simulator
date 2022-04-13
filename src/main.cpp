#include "world.h"
#include "visualization.h"
#include "heightmap.h"
#include "lrf.h"
#include "bumper.h"
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
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>

#include <geolib/CompositeShape.h>
#include "virtualbase.h"
#include "moving_object.h"
#include "random"

#include <vector>


int main(int argc, char **argv){

    ros::init(argc, argv, "hero_simulator");

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

    // The bumper class implementation is a specialization of the lrf class
    Bumper bumper;
    double robot_radius = 0.36; //[m] TODO check with ruben
    bumper.setRobotRadius(robot_radius);
    double bumperSize = 0.05; // [m]
    bumper.setBumperRadius(bumperSize);

    double cycle_freq = 30;
    double cycle_time = 1 / cycle_freq;

    bool visualize = true;

    // Load heightmap
    std::vector<Door> doors;
    geo::ShapePtr heightmap = createHeightMapShape(heightmap_filename, doors);
    if (!heightmap)
    {
        std::cout << "[HERO SIMULATOR] Heightmap could not be loaded" << std::endl;
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

    // Add robots
    double robot_width = 0.36;
    double robot_length = 0.16;
    geo::CompositeShapePtr robot_shape = makeWorldSimObject(robot_width, robot_length);
    geo::Vector3 robot_color(0, 0, 1);
    
    std::vector<Robot*> robots;

    Id hero_id = world.addObject(geo::Pose3D::identity(), robot_shape, robot_color);
    Robot hero("hero", hero_id);
    hero.base.setDisableSpeedCap(config.disable_speedcap.value());
    hero.base.setUncertainOdom(config.uncertain_odom.value());
    robots.push_back(&hero);

    if (config.enable_taco.value()){
        Id taco_id = world.addObject(geo::Pose3D::identity(), robot_shape, robot_color);
        robots.push_back(new Robot("taco", taco_id));
        robots.back()->base.setDisableSpeedCap(config.disable_speedcap.value());
        robots.back()->base.setUncertainOdom(config.uncertain_odom.value());
    }

    // Add door
    for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
    {
        Door& door = *it;
        door.id = world.addObject(door.init_pose, door.shape, geo::Vector3(0, 1, 0));
    }

    std::cout << "start cycle" << std::endl;
    ros::Rate r(cycle_freq);
    double time_ = 0;
    double dt;
    while(ros::ok())
    {
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
        bool collision = false;

        for (std::vector<Robot*>::iterator it = robots.begin(); it != robots.end(); ++it)
        {
            Robot& robot = **it;
            geo::Pose3D robot_pose = world.object(robot.robot_id).pose;
            if (robot.base_ref_) // If there is a twist message in the queue
            {
                // Set robot velocity
                geometry_msgs::Twist cmd = *robot.base_ref_;
                robot.base.applyTwistAndUpdate(cmd, dt);
                robot.base_ref_.reset();
            }
            else{ // apply previous one again
                robot.base.update(dt);
            }
            geometry_msgs::Twist actual_twist = robot.base.getActualTwist();
            world.setVelocity(robot.robot_id, geo::Vector3(actual_twist.linear.x, actual_twist.linear.y, 0), actual_twist.angular.z);

            //check if object should start moving
            for(std::vector<MovingObject>::iterator ito = config.moving_objects.value().begin(); ito != config.moving_objects.value().end(); ++ito){

                MovingObject& obj = *ito;
                // check if it should start
                geo::Vector3 dist_obj_robot = world.object(robot.robot_id).pose.getOrigin() -  world.object(obj.id).pose.getOrigin();
                double safety_radius = sqrt( std::pow(obj.width/2,2) + std::pow(obj.length/2,2)) + 0.3;
                if(dist_obj_robot.length() < obj.trigger_radius && obj.is_moving == false && obj.finished_moving == false){
                    obj.is_moving = true;
                    geo::Vector3 unit_vel = (obj.final_pose.getOrigin() - obj.init_pose.getOrigin());
                    unit_vel =world.object(obj.id).pose.R.transpose()*unit_vel / unit_vel.length();
                    world.setVelocity(obj.id, unit_vel*obj.velocity,0.0);
                }

                // check if it should stop
                geo::Vector3 dist_obj_dest = world.object(obj.id).pose.getOrigin() -  obj.final_pose.getOrigin();
                if(dist_obj_dest.length() < 0.1 && obj.is_moving == true && obj.finished_moving == false && obj.repeat == false){
                    obj.is_moving = false;
                    obj.finished_moving = true;
                    world.setVelocity(obj.id, geo::Vector3(0.0,0.0,0.0),0.0);
                }

                // reverse and repeat ... :o)
                if(dist_obj_dest.length() < 0.1 && obj.is_moving == true && obj.finished_moving == false && obj.repeat == true){
                    obj.is_moving = true;
                    obj.finished_moving = false;
                    geo::Pose3D placeholder = obj.init_pose;
                    obj.init_pose = obj.final_pose;
                    obj.final_pose = placeholder;
                    geo::Vector3 unit_vel = (obj.final_pose.getOrigin() - obj.init_pose.getOrigin());
                    unit_vel =world.object(obj.id).pose.R.transpose()*unit_vel / unit_vel.length();
                    world.setVelocity(obj.id, unit_vel*obj.velocity,0.0);
                }

                // Check if it should pause
                if(dist_obj_robot.length() < safety_radius && obj.is_moving == true && obj.is_paused == false){
                    obj.is_paused = true;
                    world.setVelocity(obj.id, geo::Vector3(0.0,0.0,0.0),0.0);
                }

                // Check if it should continue after being paused
                if(dist_obj_robot.length() > safety_radius && obj.is_paused == true){
                    obj.is_paused = false;
                    geo::Vector3 unit_vel = (obj.final_pose.getOrigin() - obj.init_pose.getOrigin());
                    unit_vel =world.object(obj.id).pose.R.transpose()*unit_vel / unit_vel.length();
                    world.setVelocity(obj.id, unit_vel*obj.velocity,0.0);
                }
            }

            //check collisions with robot
            geo::Vector3 rp1 = robot_pose*geo::Vector3(robot_width/2, robot_length/2, 0.0);
            geo::Vector3 rp2 = robot_pose*geo::Vector3(robot_width/2, -robot_length/2, 0.0);
            geo::Vector3 rp3 = robot_pose*geo::Vector3(-robot_width/2, robot_length/2, 0.0);
            geo::Vector3 rp4 = robot_pose*geo::Vector3(-robot_width/2, -robot_length/2, 0.0);
            if( heightmap->intersect(rp1,0.001) || heightmap->intersect(rp2,0.001) || heightmap->intersect(rp3,0.001) || heightmap->intersect(rp4,0.001)){
                collision = true;
            }

            for(std::vector<MovingObject>::iterator itobj = config.moving_objects.value().begin(); itobj != config.moving_objects.value().end(); ++itobj){
                geo::Vector3 op1 = world.object(itobj->id).pose.inverse()* rp1;
                geo::Vector3 op2 = world.object(itobj->id).pose.inverse()* rp2;
                geo::Vector3 op3 = world.object(itobj->id).pose.inverse()* rp3;
                geo::Vector3 op4 = world.object(itobj->id).pose.inverse()* rp4;

                if(  world.object(itobj->id).shape->intersect(op1,0.001) ||
                     world.object(itobj->id).shape->intersect(op2,0.001) ||
                     world.object(itobj->id).shape->intersect(op3,0.001) ||
                     world.object(itobj->id).shape->intersect(op4,0.001)){
                    collision = true;
                }
            }

            for (std::vector<Robot*>::iterator it2 = it+1; it2 != robots.end(); ++it2)
            {
                geo::Vector3 op1 = world.object((*it2)->robot_id).pose.inverse()* rp1;
                geo::Vector3 op2 = world.object((*it2)->robot_id).pose.inverse()* rp2;
                geo::Vector3 op3 = world.object((*it2)->robot_id).pose.inverse()* rp3;
                geo::Vector3 op4 = world.object((*it2)->robot_id).pose.inverse()* rp4;

                if(  world.object((*it2)->robot_id).shape->intersect(op1,0.001) ||
                     world.object((*it2)->robot_id).shape->intersect(op2,0.001) ||
                     world.object((*it2)->robot_id).shape->intersect(op3,0.001) ||
                     world.object((*it2)->robot_id).shape->intersect(op4,0.001)){
                    collision = true;
                }
            }

            // handle door requests
            if (robot.request_open_door_)
            {
                for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
                {
                    Door& door = *it;
                    if (!door.closed)
                        continue;

                    // Test if robot is in front of door. If not, don't open it
                    geo::Pose3D rel_robot_pose = door.init_pose.inverse() * robot_pose;

                    if (std::abs(rel_robot_pose.t.y) > 1 || std::abs(rel_robot_pose.t.x) > door.size / 2)
                        continue;

                    world.setVelocity(door.id, door.open_vel, 0);
                    door.closed = false;
                }
                robot.request_open_door_ = false;
            }

            if(config.uncertain_odom.value() && time.sec%6 == 0 ){
                robot.base.updateWheelUncertaintyFactors();
            }
        } // end iterate robots

        // Stop doors that have moved far enough
        for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
        {
            Door& door = *it;
            if (!door.closed && (door.init_pose.t - world.object(door.id).pose.t).length2() > door.open_distance * door.open_distance)
                world.setVelocity(door.id, geo::Vector3(0, 0, 0), 0);
        }

        world.update(time.toSec());

        // create output
        for (std::vector<Robot*>::iterator it = robots.begin(); it != robots.end(); ++it)
        {
            Robot& robot = **it;
            // Create laser data
            sensor_msgs::LaserScan scan_msg;
            scan_msg.header.frame_id = "laserframe";
            scan_msg.header.stamp = time;
            lrf.generateLaserData(world, robot, scan_msg);
            robot.pub_laser.publish(scan_msg);

            // Create bumper data 
            std_msgs::Bool bump_msg_f;
            std_msgs::Bool bump_msg_r;
            bumper.generateBumperData(world,robot,bump_msg_f,bump_msg_r);
            robot.pub_bumperF.publish(bump_msg_f);
            robot.pub_bumperR.publish(bump_msg_r);

            // Create odom data
            nav_msgs::Odometry odom_msg = robot.base.getOdom();
            if(!config.uncertain_odom.value()){
                geo::convert(world.object(robot.robot_id).pose, odom_msg.pose.pose);
            }
            odom_msg.header.stamp = time;
            odom_msg.header.frame_id = "odomframe";

            robot.pub_odom.publish(odom_msg);
        }

        // Visualize
        if (visualize)
            visualization::visualize(world, robots, collision, config.show_full_map.value(),bbox);

        if (collision)
            std::cout << "\033[1;;7;33m" << "COLLISION!" << "\033[0m\n"  << std::endl;

        r.sleep();
    }

    return 0;
}
