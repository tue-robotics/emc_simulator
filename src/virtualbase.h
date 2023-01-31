//
// Created by bob on 25-3-20.
//

#ifndef EMC_SYSTEM_VIRTUALBASE_H
#define EMC_SYSTEM_VIRTUALBASE_H

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>

#include "random"
#include <cmath>

/**
 * Class that contains functionality for "virtual base" to mimic uncertainty
 */

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class Virtualbase
{

public:
    Virtualbase(bool disable_speedcap = false, bool uncertain_odom = false);
    /**
     * Set flag for disabling speedcap
     */
    void setDisableSpeedCap(bool disable_speedcap)
    {
        disable_speedcap_ = disable_speedcap;
    }

    /**
     * Set flag for inacuracies in the odometry
     */
    void setUncertainOdom(bool uncertain_odom)
    {
        uncertain_odom_ = uncertain_odom;
        updateWheelUncertaintyFactors();
    }

    /**
     * Apply the input that is to be sent to the robot
     */
    void applyTwistAndUpdate(const geometry_msgs::Twist& cmd, double dt);

    /**
     * Apply the input that was last sent again if no new input is available
     */
    void update(double dt){
        odometry_state = update_odometry(odometry_state,reference_twist,dt);
    }

    /**
    * Resample the inaccuracies in the odometry from the distribution.
    */
    void updateWheelUncertaintyFactors()
    {
        if (uncertain_odom_)
        {
            a1 = 1.0 + dis(gen);
            a2 = 1.0 + dis(gen);
            a3 = 1.0 + dis(gen);
        }
        else
        {
            a1 = 1.0; a2 = 1.0; a3 =1.0;
        }
    }

    geometry_msgs::Twist getActualTwist() const{
        return actual_twist;
    }

    nav_msgs::Odometry getOdom() const{
        return odometry_state;
    }

private:
    /**
     * update odometry by an instantaneous twist (stateless function)
     */
    nav_msgs::Odometry update_odometry(const nav_msgs::Odometry odom, const geometry_msgs::Twist twist, double dt) const;

    geometry_msgs::Twist reference_twist;
    geometry_msgs::Twist actual_twist;
    nav_msgs::Odometry odometry_state;
    double a1, a2, a3;
    bool disable_speedcap_;
    bool uncertain_odom_;

    // random effects
    std::mt19937 gen;
    std::uniform_real_distribution<double> dis;
};

#endif //EMC_SYSTEM_VIRTUALBASE_H
