#include "moving_object.h"

geo::ShapePtr makeWorldSimObject(double width, double length){

    geo::ShapePtr shapePtr = std::make_shared<geo::Shape>();
    geo::MeshPtr mesh = std::make_shared<geo::Mesh>();

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
    mesh->addTriangle(0,2,3);
    mesh->addTriangle(4,5,6);
    mesh->addTriangle(4,6,7);
    mesh->addTriangle(8,9,10);
    mesh->addTriangle(8,10,11);
    mesh->addTriangle(12,13,14);
    mesh->addTriangle(12,14,15);
    sub_shape->setMesh(*mesh);

    return shapePtr;
}

geo::ShapePtr makeWorldSimObject(MovingObject object){
    return makeWorldSimObject(object.width, object.length);
}

