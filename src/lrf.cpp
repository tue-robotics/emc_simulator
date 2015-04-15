#include "lrf.h"
#include "world.h"

#include <geolib/Shape.h>

// ----------------------------------------------------------------------------------------------------

float randomUniform(float min, float max)
{
    return ((float)rand() / RAND_MAX) * (max - min) + min;
}

// ----------------------------------------------------------------------------------------------------

LRF::LRF()
{
}

// ----------------------------------------------------------------------------------------------------

LRF::~LRF()
{
}

// ----------------------------------------------------------------------------------------------------

void LRF::setAngleLimits(double a_min, double a_max)
{
    lrf_.setAngleLimits(a_min, a_max);

}

// ----------------------------------------------------------------------------------------------------

void LRF::setNumBeams(unsigned int n)
{
    lrf_.setNumBeams(n);
}

// ----------------------------------------------------------------------------------------------------

void LRF::setRangeLimits(double r_min, double r_max)
{
    lrf_.setRangeLimits(r_min, r_max);
}

// ----------------------------------------------------------------------------------------------------

void LRF::generateLaserData(const World& world, const geo::Pose3D& laser_pose, sensor_msgs::LaserScan& scan_msg) const
{
    geo::Pose3D laser_pose_inv = laser_pose.inverse();

    std::vector<double> ranges(lrf_.getNumBeams(), 0);
    for(std::vector<Object>::const_iterator it = world.objects().begin(); it != world.objects().end(); ++it)
    {
        const Object& obj = *it;
        if (!obj.shape)
            continue;

        // Set render options
        geo::LaserRangeFinder::RenderOptions opt;
        opt.setMesh(obj.shape->getMesh(), laser_pose_inv * obj.pose);

        geo::LaserRangeFinder::RenderResult res(ranges);
        lrf_.render(opt, res);
    }

    // Create message
    scan_msg.angle_increment = lrf_.getAngleIncrement();
    scan_msg.angle_min = lrf_.getAngleMin();
    scan_msg.angle_max = lrf_.getAngleMax();
    scan_msg.range_min = lrf_.getRangeMin();
    scan_msg.range_max = lrf_.getRangeMax();

    // Make sure ranges in scan message is correct size
    scan_msg.ranges.resize(lrf_.getNumBeams());

    // Copy ranges to scan message and add noise
    for(unsigned int i = 0; i < ranges.size(); ++i)
    {
        double r = ranges[i];
        scan_msg.ranges[i] = r + randomUniform(-0.01, 0.01);
    }

    // Stamp with current ROS time
    scan_msg.header.stamp = ros::Time(world.time());
}


