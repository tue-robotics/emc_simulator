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

void visualize(const World& world, const std::vector<RobotPtr>& robots, bool collision = false, bool show_full_map = false, Bbox centerframe = {-1e5, -1e5, 1e5, 1e5}, double robotRadius = 0.17)
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


    auto robot = world.object(robots[0]->robot_id);
    // check translation of world within bbox for sliding camera
    double xview, yview;
    xview = robot.pose.getOrigin().getX();
    if(robot.pose.getOrigin().getX() > midpointframe.xmax )
        xview = midpointframe.xmax;
    if(robot.pose.getOrigin().getX() < midpointframe.xmin )
        xview = midpointframe.xmin;
    yview = robot.pose.getOrigin().getY();
    if(robot.pose.getOrigin().getY() > midpointframe.ymax )
        yview = midpointframe.ymax;
    if(robot.pose.getOrigin().getY() < midpointframe.ymin)
        yview = midpointframe.ymin;

    // If there is not enough mapsize for a midpointbox, fix views to center
    if(midpointframe.xmax < midpointframe.xmin){
        xview = midpointframe.xmax + midpointframe.xmin /2;
    }
    if(midpointframe.ymax < midpointframe.ymin){
        yview = midpointframe.ymax + midpointframe.ymin /2;
    }
//    frame_center_pose.t.x = xview;
//    frame_center_pose.t.y = yview;

    // Draw robots
    for (std::vector<RobotPtr>::const_iterator it = robots.begin(); it != robots.end(); ++it)
    {
        const Object& robot = world.object((*it)->robot_id);
        cv::Scalar robot_color(0, 0, 255);

        // scaling of the head and LRF (which should be drawn inside the circumference of the robot)
        const double scaling = robotRadius/0.21;

        geo::Vector3 robot_center_point = geo::Vector3( 0.0, 0.0, 0);
        geo::Vector3 lrf_point = scaling*geo::Vector3( 0.15, 0.0, 0);
        std::vector<geo::Vector3> robot_head_points;
        std::vector<geo::Vector3> eye_points;
        std::vector<geo::Vector3> eye_pupil_points;

        robot_head_points.push_back(scaling*geo::Vector3( 0.08, -0.16, 0));
        robot_head_points.push_back(scaling*geo::Vector3( 0.08,  0.16, 0));
        robot_head_points.push_back(scaling*geo::Vector3(-0.08,  0.16, 0));
        robot_head_points.push_back(scaling*geo::Vector3(-0.08, -0.16, 0));
        eye_points.push_back(scaling*geo::Vector3( 0.0,  0.07, 0));
        eye_points.push_back(scaling*geo::Vector3( 0.0, -0.07, 0));
        eye_pupil_points.push_back(scaling*geo::Vector3( 0.025,  0.07, 0));
        eye_pupil_points.push_back(scaling*geo::Vector3( 0.025, -0.07, 0));

        if(show_full_map == true) {
            robot_center_point = (robot.pose * robot_center_point) + geo::Vector3(-xview, -yview, 0);
            lrf_point = (robot.pose * lrf_point) + geo::Vector3(-xview, -yview, 0);
            for (unsigned int i = 0; i < robot_head_points.size(); ++i) {
                robot_head_points[i] = (robot.pose * robot_head_points[i]) + geo::Vector3(-xview, -yview, 0);
            }
            for (unsigned int i = 0; i < eye_points.size(); ++i) {
                eye_points[i] = (robot.pose * eye_points[i]) + geo::Vector3(-xview, -yview, 0);
            }
            for (unsigned int i = 0; i < eye_pupil_points.size(); ++i) {
                eye_pupil_points[i] = (robot.pose * eye_pupil_points[i]) + geo::Vector3(-xview, -yview, 0);
            }
        }
        else
        {
            robot_center_point = (frame_center_pose.inverse() * robot.pose * robot_center_point);
            lrf_point = (frame_center_pose.inverse() * robot.pose * lrf_point);
            for (unsigned int i = 0; i < robot_head_points.size(); ++i) {
                robot_head_points[i] = (frame_center_pose.inverse() * robot.pose * robot_head_points[i]);
            }
            for (unsigned int i = 0; i < eye_points.size(); ++i) {
                eye_points[i] = (frame_center_pose.inverse() * robot.pose * eye_points[i]);
            }
            for (unsigned int i = 0; i < eye_pupil_points.size(); ++i) {
                eye_pupil_points[i] = (frame_center_pose.inverse() * robot.pose * eye_pupil_points[i]);
            }
        }

        cv::Point2d pLRF = worldToCanvas(lrf_point);
        cv::circle(canvas, pLRF, scaling*2, robot_color, 2);
        cv::Point2d pRobotCenter = worldToCanvas(robot_center_point);
        cv::circle(canvas, pRobotCenter, robotRadius/resolution, robot_color, 2);

        for(unsigned int i = 0; i < robot_head_points.size(); ++i)
        {
            unsigned int j = (i + 1) % robot_head_points.size();
            cv::Point2d p1 = worldToCanvas(robot_head_points[i]);
            cv::Point2d p2 = worldToCanvas(robot_head_points[j]);
            cv::line(canvas, p1, p2, robot_color, 2);
        }
        for(unsigned int i = 0; i < eye_points.size(); ++i)
        {
            cv::Point2d pEye = worldToCanvas(eye_points[i]);
            cv::circle(canvas, pEye, scaling*4, robot_color, 2);
        }
        for(unsigned int i = 0; i < eye_pupil_points.size(); ++i)
        {
            cv::Point2d pEyePupil = worldToCanvas(eye_pupil_points[i]);
            cv::circle(canvas, pEyePupil, scaling*1.5, robot_color, 2);
        }
    }

    for(std::vector<Object>::const_iterator it = world.objects().begin(); it != world.objects().end(); ++it)
    {
        const Object& obj = *it;
        if (!obj.shape)
            continue;
        if (obj.type == robottype)
            continue;

        const std::vector<geo::Vector3>& vertices = obj.shape->getMesh().getPoints();
        const std::vector<geo::TriangleI>& triangles = obj.shape->getMesh().getTriangleIs();

        cv::Scalar line_color(obj.color.x * 255, obj.color.y * 255, obj.color.z * 255);

        auto robot = world.object(robots[0]->robot_id);
        geo::Transform t;
        if(show_full_map== false){
            t = robot.pose.inverse() * obj.pose;
        } else{
            geo::Transform viewbox(-xview, -yview,0,0,0,0);
            t = viewbox*obj.pose;
        }
        //geo::Transform t = frame_center_pose.inverse() * obj.pose;

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
