#ifndef PICO_SIMULATOR_WORLD_H_
#define PICO_SIMULATOR_WORLD_H_

#include <geolib/datatypes.h>

typedef int Id;

struct Object
{
    Object() : vel_trans(0, 0, 0), vel_angular(0) {}

    geo::ShapeConstPtr shape;
    geo::Pose3D pose;
    geo::Vector3 vel_trans;
    double vel_angular;
};

class World
{

public:

    World();

    ~World();

    void update(double dt);

    Id addObject(const geo::Pose3D& pose, const geo::ShapeConstPtr& shape = geo::ShapeConstPtr())
    {
        objects_.push_back(Object());
        Object& obj = objects_.back();
        obj.pose = pose;
        obj.shape = shape;
        return objects_.size() - 1;
    }

    const std::vector<Object>& objects() const { return objects_; }

    void setVelocity(Id id, const geo::Vector3& trans, double ang)
    {
        Object& obj = objects_[id];
        obj.vel_trans = trans;
        obj.vel_angular = ang;
    }

    const Object& object(Id id) const { return objects_[id]; }

    double time() const { return time_; }

private:

    double time_;

    std::vector<Object> objects_;

};

#endif
