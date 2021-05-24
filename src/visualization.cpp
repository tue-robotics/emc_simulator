#include "visualization.h"
#include "world.h"

#include <geolib/Shape.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <vector>

double resolution = 0.01;
cv::Point2d canvas_center;

namespace visualization
{


cv::Point2d worldToCanvas(const geo::Vector3& p)
{
    return cv::Point2d(-p.y / resolution, -p.x / resolution) + canvas_center;
}

// ----------------------------------------------------------------------------------------------------

void visualize(const World& world, const std::vector<Robot*>& robots, bool collision = false, bool show_full_map = false, Bbox centerframe = {-1e5, -1e5, 1e5, 1e5})
{
    int dim = 500;
    if(show_full_map){
        dim = 1000;
    }

    cv::Mat canvas(dim, dim, CV_8UC3, cv::Scalar(100, 100, 100));
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols / 2);

    // Determine camera pose
    geo::Pose3D frame_center_pose = geo::Pose3D::identity();
    if (!show_full_map){
        frame_center_pose = world.object(robots[0]->robot_id).pose;
    }

    Bbox midpointframe; //maximum range for the midpoint of the view.
    midpointframe.xmin = centerframe.xmin + dim*resolution/2 -0.5;
    midpointframe.xmax = centerframe.xmax - dim*resolution/2 +0.5;
    midpointframe.ymin = centerframe.ymin + dim*resolution/2 -0.5;
    midpointframe.ymax = centerframe.ymax - dim*resolution/2 +0.5;

    // check translation of world within bbox for sliding camera
    double xview, yview;
    xview = frame_center_pose.getOrigin().getX();
    if(frame_center_pose.getOrigin().getX() > midpointframe.xmax )
        xview = midpointframe.xmax;
    if(frame_center_pose.getOrigin().getX() < midpointframe.xmin )
        xview = midpointframe.xmin;
    yview = frame_center_pose.getOrigin().getY();
    if(frame_center_pose.getOrigin().getY() > midpointframe.ymax )
        yview = midpointframe.ymax;
    if(frame_center_pose.getOrigin().getY() < midpointframe.ymin)
        yview = midpointframe.ymin;

    // If there is not enough mapsize for a midpointbox, fix views to center
    if(midpointframe.xmax < midpointframe.xmin){
        xview = midpointframe.xmax + midpointframe.xmin /2;
    }
    if(midpointframe.ymax < midpointframe.ymin){
        yview = midpointframe.ymax + midpointframe.ymin /2;
    }
    frame_center_pose.t.x = xview;
    frame_center_pose.t.y = yview;

    // Draw robots
    for (std::vector<Robot*>::const_iterator it = robots.begin(); it != robots.end(); ++it)
    {
        const Object& robot = world.object((*it)->robot_id);
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

        for(unsigned int i = 0; i < robot_points.size(); ++i) {
                robot_points[i] = (frame_center_pose.inverse() * robot.pose * robot_points[i]);
            }

        for(unsigned int i = 0; i < robot_points.size(); ++i)
        {
            unsigned int j = (i + 1) % robot_points.size();
            cv::Point2d p1 = worldToCanvas(robot_points[i]);
            cv::Point2d p2 = worldToCanvas(robot_points[j]);
            cv::line(canvas, p1, p2, robot_color, 2);
        }
    }

    for(std::vector<Object>::const_iterator it = world.objects().begin(); it != world.objects().end(); ++it)
    {
        const Object& obj = *it;
        if (!obj.shape)
            continue;

        const std::vector<geo::Vector3>& vertices = obj.shape->getMesh().getPoints();
        const std::vector<geo::TriangleI>& triangles = obj.shape->getMesh().getTriangleIs();

        cv::Scalar line_color(obj.color.x * 255, obj.color.y * 255, obj.color.z * 255);

        geo::Transform t = frame_center_pose.inverse() * obj.pose;

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
