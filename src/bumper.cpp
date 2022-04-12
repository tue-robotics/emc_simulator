#include "bumper.h"

Bumper::Bumper()
{
}

Bumper::~Bumper()
{

}

void Bumper::generateBumperData(const World& world, const Robot& robot, std_msgs::Bool& scan_msg_f, std_msgs::Bool& scan_msg_r) const
{
    sensor_msgs::LaserScan lrf_msg;
    LRF::generateLaserData(world, robot, lrf_msg);

    scan_msg_f.data = false;
    scan_msg_r.data = false;
}

double Bumper::_radiusTheta(const double theta) const
{
    return _robotRadius;
}

double Bumper::_bummperRadiusTheta(const double theta) const
{
    return _robotRadius + _bumperRadius;
}

bool Bumper::_isRear(const double theta) const
{
        return true;
}

bool Bumper::_isFront(const double theta) const
{
        return true;
}


