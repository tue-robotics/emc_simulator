#include "moving_object.h"

geo::CompositeShapePtr makeWorldSimObject(double width, double length){

    geo::CompositeShapePtr shape_ptr(new geo::CompositeShape);
    geo::Shape* sub_shape = new geo::Shape();
    geo::Mesh* mesh = new geo::Mesh();
    mesh->addPoint(geo::Vector3(-length/2, -width/2, -1));
    mesh->addPoint(geo::Vector3(-length/2, -width/2, 1));
    mesh->addPoint(geo::Vector3(-length/2, width/2, 1));
    mesh->addPoint(geo::Vector3(-length/2, width/2, -1));

    mesh->addPoint(geo::Vector3(-length/2, width/2, -1));
    mesh->addPoint(geo::Vector3(-length/2, width/2, 1));
    mesh->addPoint(geo::Vector3(length/2, width/2, 1));
    mesh->addPoint(geo::Vector3(length/2, width/2, -1));

    mesh->addPoint(geo::Vector3(length/2, width/2, -1));
    mesh->addPoint(geo::Vector3(length/2, width/2, 1));
    mesh->addPoint(geo::Vector3(length/2, -width/2, 1));
    mesh->addPoint(geo::Vector3(length/2, -width/2, -1));

    mesh->addPoint(geo::Vector3(length/2, -width/2, -1));
    mesh->addPoint(geo::Vector3(length/2, -width/2, 1));
    mesh->addPoint(geo::Vector3(-length/2, -width/2, 1));
    mesh->addPoint(geo::Vector3(-length/2, -width/2, -1));

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

geo::CompositeShapePtr makeWorldSimObject(MovingObject object){
    return makeWorldSimObject(object.width, object.length);
}
