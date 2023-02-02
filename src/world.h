#ifndef PICO_SIMULATOR_WORLD_H_
#define PICO_SIMULATOR_WORLD_H_

#include <geolib/datatypes.h>

#include <vector>

typedef int Id;
typedef enum {undefinedtype, walltype, movingObjecttype, doortype, robottype} ObjectType;


struct Object
{
    Object() : shape(nullptr), pose(), vel_trans(0, 0, 0), vel_angular(0), color(1, 1, 1), type(undefinedtype) {}

    geo::ShapeConstPtr shape;
    geo::Pose3D pose;
    geo::Vector3 vel_trans;
    double vel_angular;
    geo::Vector3 color;
    ObjectType type;
};

class World
{

public:

    World();

    ~World();

    void update(double dt);

    Id addObject(const geo::Pose3D& pose, const geo::ShapeConstPtr& shape = geo::ShapeConstPtr(), const geo::Vector3& color = geo::Vector3(1, 1, 1), ObjectType type = undefinedtype)
    {
        objects_.push_back(Object());
        Object& obj = objects_.back();
        obj.pose = pose;
        obj.shape = shape;
        obj.color = color;
        obj.type  = type;
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
