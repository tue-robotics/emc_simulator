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

void visualize(const World& world, Id robot_id, bool collision = false, bool show_full_map = false, Bbox centerframe = {-1e5, -1e5, 1e5, 1e5})
{
    const Object& robot = world.object(robot_id);

    int dim = 500;
    if(show_full_map){
        dim = 1000;
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

    Bbox midpointframe;
    midpointframe.xmin = centerframe.xmin + dim*resolution/2 -0.5;
    midpointframe.xmax = centerframe.xmax - dim*resolution/2 +0.5;
    midpointframe.ymin = centerframe.ymin + dim*resolution/2 -0.5;
    midpointframe.ymax = centerframe.ymax - dim*resolution/2 +0.5;


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


    // Draw cabinets
    std::vector<std::vector<geo::Vector3>> cabinets({
                                                         {{(geo::Vector3(6.1 , -2.8 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(6.1 , -3.2 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(6.5 , -3.2 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(6.5 , -2.8 , 0.0))}
                                                         },
                                                         {{(geo::Vector3(2.3 , -2.8 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(1.9 , -2.8 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(1.9 , -2.4 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(2.3 , -2.4 , 0.0))}
                                                         },
                                                         {{(geo::Vector3(0.2 , -0.8 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(0.6 , -0.8 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(0.6 , -0.4 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(0.2 , -0.4 , 0.0))}
                                                         },
                                                         {{(geo::Vector3(0.6 , 0.6 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(0.2 , 0.6 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(0.2 , 0.2 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(0.6 , 0.2 , 0.0))}
                                                         },
                                                         {{(geo::Vector3(2.3 , 2.6 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(1.9 , 2.6 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(1.9 , 2.2 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(2.3 , 2.2 , 0.0))}
                                                         },
                                                         {{(geo::Vector3(0.2 , 5.8 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(0.6 , 5.8 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(0.6 , 6.2 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(0.2 , 6.2 , 0.0))}
                                                         },
                                                         {{(geo::Vector3(6.5 , 3.0 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(6.1 , 3.0 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(6.1 , 2.6 , 0.0))},
                                                                 {(
                                                                         geo::Vector3(6.5 , 2.6 , 0.0))}
                                                         }

                                                 });







    //cv::polylines(canvas,




    if(show_full_map == true){
        for(unsigned int i = 0; i < robot_points.size(); ++i) {
            robot_points[i] = (robot.pose * robot_points[i]) + geo::Vector3(-xview,-yview,0);
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
           t = robot.pose.inverse() * obj.pose;
        } else{
            geo::Transform viewbox(-xview, -yview,0,0,0,0);
            t = viewbox*obj.pose;
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

    std::pair<double,double> offset[7] = {{0.4, 0},{0,0.4},{0,-0.4},{0,-0.4},{0,0.4},{0,-0.4},{-0.4,0}};

    int index = 0;
    for(auto cabinet : cabinets){
        std::vector<cv::Point> cvpoints;
        for(auto p : cabinet){
            cvpoints.push_back(worldToCanvas( geo::Vector3(p.getY()-xview+2.7 +0.4, -p.getX()-yview+4.925 +0.425, 0.0)   ));
        }

        geo::Vector3 center((cabinet[0].getX() + cabinet[2].getX())/2,(cabinet[0].getY() + cabinet[2].getY())/2,0.0  );

        cv::putText(canvas,std::to_string(index),worldToCanvas(geo::Vector3(center.getY()-xview+2.7-0.1+0.4+offset[index].first, -center.getX()-yview+4.925+0.1+0.425+offset[index].second, 0.0)),cv::FONT_HERSHEY_TRIPLEX,1,cv::Scalar(30,255,30));
        cv::polylines(canvas,cvpoints,true,cv::Scalar(255,255,255),2);

        cv::line(canvas, worldToCanvas(geo::Vector3(cabinet[0].getY()-xview+2.7+0.4, -cabinet[0].getX()-yview+4.925+0.425, 0.0)),
                 worldToCanvas(geo::Vector3(cabinet[1].getY()-xview+2.7+0.4, -cabinet[1].getX()-yview+4.925+0.425, 0.0)),cv::Scalar(10,255,10), 3);
        index++;
    }

    cv::imshow("simulator", canvas);
    cv::waitKey(3);
}

}
