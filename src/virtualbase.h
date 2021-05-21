//
// Created by bob on 25-3-20.
//

#ifndef EMC_SYSTEM_VIRTUALBASE_H
#define EMC_SYSTEM_VIRTUALBASE_H

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

/**
 * Class that contains functionality for "virtual base" to mimic uncertainty
 */

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class Virtualbase
{

public:
    Virtualbase()
    {
        odometry_state.pose.pose.position.x = 0.0;
        odometry_state.pose.pose.position.y = 0.0;
        odometry_state.pose.pose.position.z = 0.0;
        odometry_state.pose.pose.orientation.x = 0.0;
        odometry_state.pose.pose.orientation.y = 0.0;
        odometry_state.pose.pose.orientation.z = 0.0;
        odometry_state.pose.pose.orientation.w = 1.0;
    }

    Virtualbase(double a1_, double a2_, double a3_, bool disable_speedcap) : a1(a1_), a2(a2_), a3(a3_), disable_speedcap_(disable_speedcap)
    {
        Virtualbase();
    }

    /**
     * Set wheel radius mismatch factors
     */
    void setWheelUncertaintyFactors(double a1_, double a2_, double a3_){
        a1 = a1_;
        a2 = a2_;
        a3 = a3_;
    }

    /**
     * Apply the input that is to be sent to the robot
     */
    void applyTwistAndUpdate(const geometry_msgs::Twist& cmd, double dt){
        // apply speedcap
        geometry_msgs::Twist twist;
        if(disable_speedcap_ == false){
            twist.linear.x  = sgn<double>(cmd.linear.x)  * std::min(std::abs(cmd.linear.x),0.5);
            twist.linear.y  = sgn<double>(cmd.linear.y)  * std::min(std::abs(cmd.linear.y),0.5);
            twist.angular.z = sgn<double>(cmd.angular.z) * std::min(std::abs(cmd.angular.z),1.2);
        }
        else{
            twist = cmd;
        }

        // save twist to keep odometry updated
        reference_twist = twist;

        // apply twist to odometry state (what it thinks it drove)
        odometry_state = update_odometry(odometry_state,twist,dt);

        // Calculate wheel speeds (topview clockwise start 9oclock) that would have resulted in the twist
        const double lw = 0.1; // distance to wheel
        double v1 = twist.linear.x - lw*twist.angular.z;
        double v2 = -0.5*twist.linear.x - 0.5*sqrt(3.0)*twist.linear.y - lw*twist.angular.z;
        double v3 = -0.5*twist.linear.x + 0.5*sqrt(3.0)*twist.linear.y - lw*twist.angular.z;

        v1 = a1*v1; v2=a2*v2; v3=a3*v3;

        // Calculate actual twist from wrong wheelspeeds
        actual_twist.linear.x = (2.0/3.0)* v1 -(1/3.0)*v2 -(1/3.0)*v3;
        actual_twist.linear.y = -sqrt(3.0)/3 * v2 + sqrt(3.0)/3.0 * v3;
        actual_twist.angular.z = -v1 / (3.0*lw) -v2 / (3.0*lw) -v3 / (3.0*lw);
    }

    /**
     * Apply the input that was last sent again if no new input is available
     */
    void update(double dt){
        odometry_state = update_odometry(odometry_state,reference_twist,dt);
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
    nav_msgs::Odometry update_odometry(const nav_msgs::Odometry odom, const geometry_msgs::Twist twist, double dt) const{
        tf::Transform delta;
        delta.setOrigin( tf::Vector3(twist.linear.x*dt, twist.linear.y*dt, 0));
        delta.setRotation(tf::createQuaternionFromYaw(twist.angular.z*dt));

        tf::Transform Todom;
        Todom.setRotation(tf::Quaternion(odom.pose.pose.orientation.x,
                                         odom.pose.pose.orientation.y,
                                         odom.pose.pose.orientation.z,
                                         odom.pose.pose.orientation.w));
        Todom.setOrigin(tf::Vector3(odom.pose.pose.position.x,
                                    odom.pose.pose.position.y,
                                    0));

        tf::Transform new_odom = Todom*delta;

        nav_msgs::Odometry new_odom_msg;
        new_odom_msg.pose.pose.position.x = new_odom.getOrigin()[0];
        new_odom_msg.pose.pose.position.y = new_odom.getOrigin()[1];
        new_odom_msg.pose.pose.orientation.x = new_odom.getRotation().getX();
        new_odom_msg.pose.pose.orientation.y = new_odom.getRotation().getY();
        new_odom_msg.pose.pose.orientation.z = new_odom.getRotation().getZ();
        new_odom_msg.pose.pose.orientation.w = new_odom.getRotation().getW();

        return new_odom_msg;
    }

    geometry_msgs::Twist reference_twist;
    geometry_msgs::Twist actual_twist;
    nav_msgs::Odometry odometry_state;
    double a1, a2, a3;
    bool disable_speedcap_;
};

#endif //EMC_SYSTEM_VIRTUALBASE_H
