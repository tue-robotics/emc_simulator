#include "world.h"
#include "visualization.h"
#include "heightmap.h"
#include "lrf.h"

#include <unistd.h>

#include <tue/profiling/timer.h>

// ROS
#include <ros/init.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <ros/package.h>

#include <geometry_msgs/Twist.h>

geometry_msgs::Twist::ConstPtr base_ref_;

// ----------------------------------------------------------------------------------------------------

void baseReferenceCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    base_ref_ = msg;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pico_simulator");

    std::string heightmap_filename;
    if (argc > 1)
        heightmap_filename = argv[1];
    else
        heightmap_filename = ros::package::getPath("emc_simulator") + "/data/heightmap.pgm";

    World world;
    LRF lrf;
    lrf.setAngleLimits(-2, 2);
    lrf.setNumBeams(1000);
    lrf.setRangeLimits(0.01, 10);

    double cycle_freq = 30;
    double cycle_time = 1 / cycle_freq;

    bool visualize = true;

    // Load heightmap
    geo::ShapePtr heightmap = createHeightMapShape(heightmap_filename);
    if (!heightmap)
    {
        std::cout << "[PICO SIMULATOR] Heightmap could not be loaded" << std::endl;
        return 1;
    }

    world.addObject(geo::Pose3D::identity(), heightmap);

    // Add robot
    geo::Pose3D robot_pose = geo::Pose3D::identity();
    Id robot_id = world.addObject(robot_pose);

    // Publishers
    ros::NodeHandle nh;
    ros::Publisher pub_laser = nh.advertise<sensor_msgs::LaserScan>("/pico/laser", 1);

    // Subscribers
    ros::Subscriber sub_base_ref = nh.subscribe<geometry_msgs::Twist>("/pico/base/reference", 1, baseReferenceCallback);

    // Set laser pose (in robot frame)
    geo::Pose3D laser_pose = geo::Pose3D::identity();
    laser_pose.t.z = 0.3;

    ros::Rate r(cycle_freq);
    while(ros::ok())
    {
        base_ref_.reset();
        ros::spinOnce();

        if (base_ref_)
        {
            // Set robot velocity
            world.setVelocity(robot_id, geo::Vector3(base_ref_->linear.x, base_ref_->linear.y, 0), base_ref_->angular.z);
        }

        world.update(ros::Time::now().toSec());

        // Create laser data
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.frame_id = "/pico/laser";
        lrf.generateLaserData(world, world.object(robot_id).pose * laser_pose, scan_msg);
        pub_laser.publish(scan_msg);

        // Visualize
        if (visualize)
            visualization::visualize(world, robot_id);

        r.sleep();
    }

    return 0;
}
