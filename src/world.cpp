#include "world.h"

// ----------------------------------------------------------------------------------------------------

World::World() : time_(0)
{
    nh = ros::NodeHandle();
    pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/map",1);

    nav_msgs::OccupancyGrid msg;
    msg.header.seq = 0;
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time();

    //msg.data = 
    //pub_map.publish(msg);
}

// ----------------------------------------------------------------------------------------------------

World::~World()
{
}

// ----------------------------------------------------------------------------------------------------

void World::update(double time)
{
    if (time_ == 0)
    {
        time_ = time;
        return;
    }

    // Calculate dt;
    double dt = time - time_;

    for(std::vector<Object>::iterator it = objects_.begin(); it != objects_.end(); ++it)
    {
        Object& obj = *it;

        geo::Transform delta;
        delta.t = geo::Vector3(dt * obj.vel_trans.x, dt * obj.vel_trans.y, 0);
        delta.R.setRPY(0, 0, dt * obj.vel_angular);

        obj.pose = obj.pose * delta;
    }


    // Set new time
    time_ = time;
}
