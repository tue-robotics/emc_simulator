#ifndef PICO_SIMULATOR_WORLD_H_
#define PICO_SIMULATOR_WORLD_H_

#include <geolib/datatypes.h>

typedef int Id;

struct Object
{
    geo::ShapeConstPtr shape;
    geo::Pose3D pose;
};

class World
{

public:

    World();

    ~World();

    void update(double dt);

    Id addObject(const geo::Pose3D& pose, const geo::ShapeConstPtr& shape)
    {
        objects_.push_back(Object());
        Object& obj = objects_.back();
        obj.pose = pose;
        obj.shape = shape;
        return objects_.size() - 1;
    }

    const std::vector<Object>& objects() const { return objects_; }

    double time() const { return time_; }

private:

    double time_;

    geo::Pose3D robot_pose_;

    std::vector<Object> objects_;

};

#endif
