//
// Created by bob on 25-3-20.
//

#ifndef PROJECT_MOVING_OBJECT_H
#define PROJECT_MOVING_OBJECT_H

#include <geolib/CompositeShape.h>

struct MovingObject
{
    Id id;
    double length, width;
    double trigger_radius;
    double safety_radius;
    bool is_moving;
    bool finished_moving;
    bool repeat;
    geo::Pose3D init_pose;
    geo::Pose3D final_pose;
    double velocity;
    geo::ShapePtr shape;
};


boost::shared_ptr<geo::CompositeShape> makeWorldSimObject(MovingObject object){

    boost::shared_ptr<geo::CompositeShape> shape_ptr(new geo::CompositeShape);
    geo::Shape* sub_shape = new geo::Shape();
    geo::Mesh* mesh = new geo::Mesh();
    mesh->addPoint(geo::Vector3(-object.length/2, -object.width/2, -1));
    mesh->addPoint(geo::Vector3(-object.length/2, -object.width/2, 1));
    mesh->addPoint(geo::Vector3(-object.length/2, object.width/2, 1));
    mesh->addPoint(geo::Vector3(-object.length/2, object.width/2, -1));

    mesh->addPoint(geo::Vector3(-object.length/2, object.width/2, -1));
    mesh->addPoint(geo::Vector3(-object.length/2, object.width/2, 1));
    mesh->addPoint(geo::Vector3(object.length/2, object.width/2, 1));
    mesh->addPoint(geo::Vector3(object.length/2, object.width/2, -1));

    mesh->addPoint(geo::Vector3(object.length/2, object.width/2, -1));
    mesh->addPoint(geo::Vector3(object.length/2, object.width/2, 1));
    mesh->addPoint(geo::Vector3(object.length/2, -object.width/2, 1));
    mesh->addPoint(geo::Vector3(object.length/2, -object.width/2, -1));

    mesh->addPoint(geo::Vector3(object.length/2, -object.width/2, -1));
    mesh->addPoint(geo::Vector3(object.length/2, -object.width/2, 1));
    mesh->addPoint(geo::Vector3(-object.length/2, -object.width/2, 1));
    mesh->addPoint(geo::Vector3(-object.length/2, -object.width/2, -1));

    mesh->addTriangle(0,1,2);
    mesh->addTriangle(1,2,3);
    mesh->addTriangle(4,5,6);
    mesh->addTriangle(5,6,7);
    mesh->addTriangle(8,9,10);
    mesh->addTriangle(9,10,11);
    mesh->addTriangle(12,13,14);
    mesh->addTriangle(13,14,15);
    sub_shape->setMesh(*mesh);
    shape_ptr->addShape(*sub_shape, geo::Pose3D::identity());


    return shape_ptr;
}



#endif //PROJECT_MOVING_OBJECT_H
