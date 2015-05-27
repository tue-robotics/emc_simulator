#ifndef PICO_SIMULATOR_DOOR_H_
#define PICO_SIMULATOR_DOOR_H_

#include "world.h"

struct Door
{
    Id id;
    double size;
    geo::Pose3D init_pose;
    double open_distance;
    bool closed;
    geo::ShapePtr shape;
    geo::Vector3 open_vel;
};

#endif
