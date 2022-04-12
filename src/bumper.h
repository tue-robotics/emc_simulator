#include "lrf.h"
#include <std_msgs/Bool.h>


class Bumper : public LRF
{

    public:
    Bumper();
    ~Bumper();

    void setRobotRadius(double width, double length);
    void setRobotRadius(double radius);

    void setBumperRadius(double bumperSize);
    // Generate the bumper data msgs 
    void generateBumperData(const World& world, const Robot& robot, std_msgs::Bool& scan_msg_f, std_msgs::Bool& scan_msg_r) const;

    private:
    // Radius of the robot
    double _robotRadiusWidth;
    double _robotRadiusLength;
    // EXTRA radius of the bumper
    double _bumperRadius;
    // Radius of the robot as a function of theta
    double _radiusTheta(const double theta) const;
    // Raidus of the robot plus the bumper as a function of theta
    double _bumperRadiusTheta(const double theta) const;
    // Check whether theta is in front of the robot
    bool _isFront(const double theta) const;
    // Check whether theta is behind the robot
    bool _isRear(const double theta) const;

};