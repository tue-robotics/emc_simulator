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
    mesh->addTriangle(0,2,3);
    mesh->addTriangle(4,5,6);
    mesh->addTriangle(4,6,7);
    mesh->addTriangle(8,9,10);
    mesh->addTriangle(8,10,11);
    mesh->addTriangle(12,13,14);
    mesh->addTriangle(12,14,15);
    sub_shape->setMesh(*mesh);
    shape_ptr->addShape(*sub_shape, geo::Pose3D::identity());

    return shape_ptr;
}

geo::CompositeShapePtr makeWorldSimObject(MovingObject object){
    return makeWorldSimObject(object.width, object.length);
}

#include "moving_object.h"

geo::CompositeShapePtr makeApproxRoundWorldSimObject(double radius, int nCorners){

    geo::CompositeShapePtr shape_ptr(new geo::CompositeShape);
    geo::Shape* sub_shape = new geo::Shape();
    geo::Mesh* mesh = new geo::Mesh();

    double x1, x2, y1, y2, theta1, theta2;
    for (int i=1; i<=nCorners; i++) {
        theta1 = 2*M_PI*(i-1)/nCorners;
        theta2 = 2*M_PI*(double) i / nCorners;
        x1 = radius*cos(theta1);
        x2 = radius*cos(theta2);
        y1 = radius*sin(theta1);
        y2 = radius*sin(theta2);

        mesh->addPoint(geo::Vector3(x1, y1, -1));
        mesh->addPoint(geo::Vector3(x1, y1, 1));
        mesh->addPoint(geo::Vector3(x2, y2, 1));
        mesh->addPoint(geo::Vector3(x2, y2, -1));
    }

    for (int i=1; i<=nCorners; i++) {
        mesh->addTriangle(4*(i-1), 4*(i-1)+1, 4*(i-1)+2);
        mesh->addTriangle(4*(i-1), 4*(i-1)+2, 4*(i-1)+3);
    }
    sub_shape->setMesh(*mesh);
    shape_ptr->addShape(*sub_shape, geo::Pose3D::identity());

    return shape_ptr;
}

