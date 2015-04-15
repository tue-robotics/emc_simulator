#ifndef PICO_SIMULATOR_LRF_H_
#define PICO_SIMULATOR_LRF_H_

#include <geolib/sensors/LaserRangeFinder.h>
#include <sensor_msgs/LaserScan.h>

class World;

class LRF
{

public:

    LRF();

    ~LRF();

    void generateLaserData(const World& world, const geo::Pose3D& laser_pose, sensor_msgs::LaserScan& scan_msg) const;

    void setAngleLimits(double a_min, double a_max);

    void setNumBeams(unsigned int n);

    void setRangeLimits(double r_min, double r_max);

private:

    geo::LaserRangeFinder lrf_;

    sensor_msgs::LaserScan msg_prototype_;

};

#endif
