#include "visualization.h"
#include "world.h"

#include <geolib/Shape.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

double resolution = 0.01;
cv::Point2d canvas_center;

namespace visualization
{

cv::Point2d worldToCanvas(const geo::Vector3& p)
{
    return cv::Point2d(-p.y / resolution, -p.x / resolution) + canvas_center;
}

// ----------------------------------------------------------------------------------------------------

void visualize(const World& world, Id robot_id, bool collision = false, bool show_full_map = false)
{
    const Object& robot = world.object(robot_id);

    int dim = 500;
    if(show_full_map){
        dim = 1200;
    }

    cv::Mat canvas(dim, dim, CV_8UC3, cv::Scalar(100, 100, 100));
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols / 2);

    // Draw robot
    cv::Scalar robot_color(0, 0, 255);

    std::vector<geo::Vector3> robot_points;
    robot_points.push_back(geo::Vector3( 0.1,  -0.2, 0));
    robot_points.push_back(geo::Vector3( 0.1,  -0.1, 0));
    robot_points.push_back(geo::Vector3( 0.05, -0.1, 0));
    robot_points.push_back(geo::Vector3( 0.05,  0.1, 0));
    robot_points.push_back(geo::Vector3( 0.1,   0.1, 0));
    robot_points.push_back(geo::Vector3( 0.1,   0.2, 0));
    robot_points.push_back(geo::Vector3(-0.1,   0.2, 0));
    robot_points.push_back(geo::Vector3(-0.1,  -0.2, 0));

    if(show_full_map == true){
        for(unsigned int i = 0; i < robot_points.size(); ++i) {
            robot_points[i] = world.object(robot_id).pose * robot_points[i];
        }
    }


    for(unsigned int i = 0; i < robot_points.size(); ++i)
    {
        unsigned int j = (i + 1) % robot_points.size();
        cv::Point2d p1 = worldToCanvas(robot_points[i]);
        cv::Point2d p2 = worldToCanvas(robot_points[j]);
        cv::line(canvas, p1, p2, robot_color, 2);
    }

    for(std::vector<Object>::const_iterator it = world.objects().begin(); it != world.objects().end(); ++it)
    {
        const Object& obj = *it;
        if (!obj.shape)
            continue;

        const std::vector<geo::Vector3>& vertices = obj.shape->getMesh().getPoints();
        const std::vector<geo::TriangleI>& triangles = obj.shape->getMesh().getTriangleIs();

        cv::Scalar line_color(obj.color.x * 255, obj.color.y * 255, obj.color.z * 255);

        geo::Transform t;
        if(show_full_map== false){
           t =  robot.pose.inverse() * obj.pose;
        } else{
            t = obj.pose;
        }


        for(std::vector<geo::TriangleI>::const_iterator it2 = triangles.begin(); it2 != triangles.end(); ++it2)
        {
            const geo::TriangleI& triangle = *it2;

            cv::Point2d p1_2d = worldToCanvas(t * vertices[triangle.i1_]);
            cv::Point2d p2_2d = worldToCanvas(t * vertices[triangle.i2_]);
            cv::Point2d p3_2d = worldToCanvas(t * vertices[triangle.i3_]);

            cv::line(canvas, p1_2d, p2_2d, line_color, 2);
            cv::line(canvas, p2_2d, p3_2d, line_color, 2);
            cv::line(canvas, p1_2d, p3_2d, line_color, 2);
        }
    }

    if(collision){
        cv::putText(canvas,"COLLISION!",cv::Point(20,20),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,0,255));
    }

    cv::imshow("simulator", canvas);
    cv::waitKey(3);
}

}
