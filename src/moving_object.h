//
// Created by bob on 25-3-20.
//

#ifndef PROJECT_MOVING_OBJECT_H
#define PROJECT_MOVING_OBJECT_H

#include "world.h"

#include <geolib/CompositeShape.h>

struct MovingObject
{
    Id id;
    double length, width;
    double trigger_radius;
    double safety_radius;
    bool is_moving;
    bool finished_moving;
    bool is_paused;
    bool repeat;
    geo::Pose3D init_pose;
    geo::Pose3D final_pose;
    double velocity;
    geo::ShapePtr shape;
};

geo::CompositeShapePtr makeWorldSimObject(double width, double height);
geo::CompositeShapePtr makeWorldSimObject(MovingObject object);
geo::CompositeShapePtr makeApproxRoundWorldSimObject(double radius, int nCorners);


#endif //PROJECT_MOVING_OBJECT_H
