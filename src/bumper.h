#include "lrf.h"
#include <std_msgs/Bool.h>


class Bumper : public LRF
{

    public:
    Bumper();
    ~Bumper();

    void generateBumperData(const World& world, const Robot& robot, std_msgs::Bool& scan_msg_f, std_msgs::Bool& scan_msg_r) const;
    private:

    double _robotRadius;
    double _bumperRadius; 

    double _radiusTheta(const double theta) const;
    double _bummperRadiusTheta(const double theta) const;

    bool _isFront(const double theta) const;
    bool _isRear(const double theta) const;

};