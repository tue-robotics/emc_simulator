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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pico_simulator");

    if (argc <= 1)
    {
        std::cout << "[PICO SIMULATOR] Please provide heightmap" << std::endl;
        return 1;
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
    geo::ShapePtr heightmap = createHeightMapShape(argv[1]);
    if (!heightmap)
    {
        std::cout << "[PICO SIMULATOR] Heightmap could not be loaded" << std::endl;
        return 1;
    }

    world.addObject(geo::Pose3D::identity(), heightmap);

    // Add robot
    geo::Pose3D robot_pose = geo::Pose3D::identity();
    Id robot_id = world.addObject(robot_pose);
    world.setVelocity(robot_id, geo::Vector3(0.1, 0, 0.0), 0.2);

    // Publishers
    ros::NodeHandle nh;
    ros::Publisher pub_laser = nh.advertise<sensor_msgs::LaserScan>("/pico/laser", 1);

    // Set laser pose (in robot frame)
    geo::Pose3D laser_pose = geo::Pose3D::identity();
    laser_pose.t.z = 0.3;

    ros::Rate r(cycle_freq);
    while(ros::ok())
    {
        world.update(ros::Time::now().toSec());

        // Create laser data
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.frame_id = "/pico/laser";
        lrf.generateLaserData(world, world.object(robot_id).pose * laser_pose, scan_msg);
        pub_laser.publish(scan_msg);

        // Visualize
        if (visualize)
            visualization::visualize(world);

        r.sleep();
    }

    return 0;
}
