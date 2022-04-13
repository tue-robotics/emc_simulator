#include "bumper.h"

Bumper::Bumper(): _lrf()
{
    _lrf.setAngleLimits(-M_PI,M_PI);
    _lrf.setNumBeams(50);
    _lrf.setRangeLimits(0.01,1);
}

void Bumper::setRobotRadius(double radius)
{
    _robotRadius = radius;
}

void Bumper::setBumperRadius(double size)
{
    _bumperRadius = size;
}

void Bumper::generateBumperData(const World& world, const Robot& robot, std_msgs::Bool& scan_msg_f, std_msgs::Bool& scan_msg_r) const
{
    // Generate laser data to perform bumper check
    sensor_msgs::LaserScan lrf_msg;
    _lrf.generateLaserData(world, robot, lrf_msg);

    bool hitF = false;
    bool hitR = false;

    int N = lrf_msg.ranges.size();
    for(int i = 0; i<lrf_msg.ranges.size(); i++)
    {
        double theta_i = lrf_msg.angle_min + i*lrf_msg.angle_increment;

        // If range larger than bumper radius do nothing
        if(lrf_msg.ranges[i] > _bumperRadiusTheta(theta_i))
        {
            continue;
        }
        if(lrf_msg.ranges[i]< lrf_msg.range_min)
        {
            continue;
        }
        // If range smaller than bumper radius, and is behind the robot
        if (_isRear(theta_i))
        {
            hitR = true;
            continue;
        }

        // If range smaller than bumper radius, and is in front of the robot
        if (_isFront(theta_i))
        {
            hitF = true;
            continue;
        }
    }
    // Write variables to message 
    scan_msg_f.data = hitF;
    scan_msg_r.data = hitR;
}

double Bumper::_radiusTheta(const double theta) const
{
    return _robotRadius;
}

double Bumper::_bumperRadiusTheta(const double theta) const
{
    return _radiusTheta(theta) + _bumperRadius;
}

bool Bumper::_isRear(const double theta) const
{
    bool check1 = theta > 0.5*M_PI;
    bool check2 = theta < -0.5*M_PI;
    return check1 || check2;
    
}

bool Bumper::_isFront(const double theta) const
{
    bool check1 = theta >= -0.5*M_PI;
    bool check2 = theta <= 0.5*M_PI;
    return check1 && check2;
}


