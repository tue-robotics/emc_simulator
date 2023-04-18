#include "heightmap.h"
#include "door.h"

#include <ros/console.h>
#include <ros/rate.h>

#include "polypartition/polypartition.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geolib/CompositeShape.h>
#include <geolib/datatypes.h>

#include <queue>


MapLoader::MapLoader()
{
    ros::SubscribeOptions map_sub_options = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(&MapLoader::mapCallback, this, _1), ros::VoidPtr(), &cb_map);
    sub_map = nh.subscribe(map_sub_options);
}

void MapLoader::getMap(nav_msgs::OccupancyGrid& mapRef)
{
    if (!initialized)
    {
        load();
    }
    mapRef = map;
}

void MapLoader::getMapMetadata(nav_msgs::MapMetaData& metadataRef)
{
    if (!initialized)
    {
        load();
    }
    metadataRef = map.info;
}

void MapLoader::getMapImage(cv::Mat& imageRef)
{
    if (!initialized)
    {
        load();
    }
    imageRef = mapImage;
}

void MapLoader::load()
{
    ros::Rate r(10);
    ROS_INFO_STREAM("Waiting for map");
    while(!initialized && ros::ok())
    {
        cb_map.callAvailable();
        r.sleep();
    }
}

void MapLoader::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map = *msg;
    //convert to image
    mapImage = cv::Mat(map.info.height, map.info.width, CV_8UC1);
    for (uint row = 0; row < map.info.height; ++row) {
        for (uint col = 0; col < map.info.width; ++col) {
            uint index = col + (map.info.height-row) * map.info.width;
            if (map.data[index] == 0) {
                mapImage.at<uchar>(row, col) = 255;  // free space
            } else if (map.data[index] == 100) {
                mapImage.at<uchar>(row, col) = 0;    // occupied space
            } else {
                mapImage.at<uchar>(row, col) = 127;  // door
            }
        }
    }
    initialized = true;
    sub_map.shutdown();
}

// ----------------------------------------------------------------------------------------------------

void findContours(const cv::Mat& image, const geo::Vec2i& p, int d_start, std::vector<geo::Vec2i>& points,
                  std::vector<geo::Vec2i>& line_starts, cv::Mat& contour_map, bool add_first)
{
    static int dx[4] = {1,  0, -1,  0 };
    static int dy[4] = {0,  1,  0, -1 };

    unsigned char v = image.at<unsigned char>(p.y, p.x);

    int d_current = d_start; // Current direction
    int x2 = p.x;
    int y2 = p.y;

    int line_piece_min = 1e9; // minimum line piece length of current line
    int line_piece_max = 0; // maximum line piece length of current line

    int d_main = d_current; // The main direction in which we're heading. If we follow a line
                            // that gradually changes to the side (1-cell side steps), this direction
                            // denotes the principle axis of the line

    if (add_first)
        points.push_back(p - geo::Vec2i(1, 1));

    int n_uninterrupted = 1;
    geo::Vec2i p_corner = p;

    while (true)
    {
        bool found = false;
        int d = (d_current + 3) % 4; // check going left first

        for(int i = -1; i < 3; ++i)
        {
            if (image.at<unsigned char>(y2 + dy[d], x2 + dx[d]) == v)
            {
                found = true;
                break;
            }

            d = (d + 1) % 4;
        }

        if (!found)
            return;

        geo::Vec2i p_current(x2, y2);

        if ((d + 2) % 4 == d_current)
        {
            // 180 degree turn

            if (x2 == p.x && y2 == p.y) // Edge case: if we returned to the start point and
                                        // this is a 180 degree angle, return without adding it
                return;


            geo::Vec2i q = p_current;
            if (d == 0 || d_current == 0) // right
                --q.y;
            if (d == 3 || d_current == 3) // up
                --q.x;

            points.push_back(q);
            d_main = d;
            line_piece_min = 1e9;
            line_piece_max = 0;

        }
        else if (d_current != d_main)
        {
            // Not moving in main direction (side step)

            if (d != d_main)
            {
                // We are not moving back in the main direction
                // Add the corner to the list and make this our main direction

                points.push_back(p_corner);
                d_main = d_current;
                line_piece_min = 1e9;
                line_piece_max = 0;
            }
        }
        else
        {
            // Moving in main direction (no side step)

            if (d_current != d)
            {
                // Turning 90 degrees

                // Check if the length of the last line piece is OK w.r.t. the other pieces in this line. If it differs to much,
                // (i.e., the contour has taken a different angle), add the last corner to the list. This way, we introduce a
                // bend in the contour
                if (line_piece_max > 0 && (n_uninterrupted < line_piece_max - 2 || n_uninterrupted > line_piece_min + 2))
                {
                    // Line is broken, add the corner as bend
                    points.push_back(p_corner);

                    line_piece_min = 1e9;
                    line_piece_max = 0;
                }

                // Update the line piece lenth boundaries with the current found piece
                line_piece_min = std::min(line_piece_min, n_uninterrupted);
                line_piece_max = std::max(line_piece_max, n_uninterrupted);
            }
        }

        if (d_current != d)
        {
            geo::Vec2i q = p_current;
            if (d == 0 || d_current == 0) // right
                --q.y;
            if (d == 3 || d_current == 3) // up
                --q.x;

            p_corner = q;
            n_uninterrupted = 0;
        }

        if ((d_current == 3 && d != 2) || (d == 3 && d != 0)) // up
            line_starts.push_back(p_current);

        contour_map.at<unsigned char>(p_current.y, p_current.x) = 1;

        ++n_uninterrupted;

        if (points.size() > 1 && x2 == p.x && y2 == p.y)
            return;

        x2 = x2 + dx[d];
        y2 = y2 + dy[d];

        d_current = d;
    }
}

// ----------------------------------------------------------------------------------------------------


geo::ShapePtr createHeightMapShape(const std::string& filename, std::vector<Door>& doors)
{
    cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);   // Read the file
    return createHeightMapShape(image, 0.025, doors);
}

geo::ShapePtr createHeightMapShape(const cv::Mat& image_tmp, double resolution, std::vector<Door>& doors)
{
    double origin_z = 0;
    double blockheight = 1.0;

    cv::Mat image;
    // Pad Input image with a white border of 1 pixel around, to prevent indexing issues
    cv::copyMakeBorder(image_tmp, image, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(255));

    if (!image.data)
    {
        ROS_ERROR_STREAM("[PICO SIMULATOR] Loading heightmap failed.");
        return geo::ShapePtr();
    }

    double origin_x = (-image.cols / 2.0 + 1) * resolution;
    double origin_y = (-image.rows / 2.0 + 1) * resolution;

    // Find doors
    for(int y = 0; y < image.rows; ++y)
    {
        for(int x = 0; x < image.cols; ++x)
        {
            unsigned char c = image.at<unsigned char>(y, x);

            if (c < 50 || c > 205)
                continue;

            std::queue<cv::Point2i> Q;
            cv::Point2i p_start(x, y);
            Q.push(p_start);

            cv::Point2i p_min = p_start;
            cv::Point2i p_max = p_start;

            int n = 0;
            while(!Q.empty())
            {
                const cv::Point2i& p = Q.front();

                if (image.at<unsigned char>(p) > 50 && image.at<unsigned char>(p) < 205)
                {
                    // Remove this part of the door
                    image.at<unsigned char>(p) = 255;

                    if (image.at<unsigned char>(p.y, p.x + 1) > 50 && image.at<unsigned char>(p.y, p.x + 1) < 205) Q.push(cv::Point2i(p.x + 1, p.y));
                    if (image.at<unsigned char>(p.y, p.x - 1) > 50 && image.at<unsigned char>(p.y, p.x - 1) < 205) Q.push(cv::Point2i(p.x - 1, p.y));
                    if (image.at<unsigned char>(p.y + 1, p.x) > 50 && image.at<unsigned char>(p.y + 1, p.x) < 205) Q.push(cv::Point2i(p.x, p.y + 1));
                    if (image.at<unsigned char>(p.y - 1, p.x) > 50 && image.at<unsigned char>(p.y - 1, p.x) < 205) Q.push(cv::Point2i(p.x, p.y - 1));

                    p_min.x = std::min(p_min.x, p.x);
                    p_min.y = std::min(p_min.y, p.y);

                    p_max.x = std::max(p_max.x, p.x);
                    p_max.y = std::max(p_max.y, p.y);

                    ++n;
                }
                Q.pop();
            }
            // Account for pixel thickness
            p_max.x += 1;
            p_max.y += 1;

            // Find length in both directions
            int dx = p_max.x - p_min.x;
            int dy = p_max.y - p_min.y;

            // Calculate thickness
            int thickness_pixels = n / (std::max(dx, dy));

            // Calculate coordinates of one side of the door
            if (dy > dx)
                p_max.x = std::max(p_min.x, p_max.x - thickness_pixels);
            else
                p_max.y = std::max(p_min.y, p_max.y - thickness_pixels);

            // Convert to world coordinates
            geo::Vector3 p_world_min((image.rows - p_min.y) * resolution + origin_y, (image.cols - p_min.x - 1) * resolution + origin_x, 0);
            geo::Vector3 p_world_max((image.rows - p_max.y) * resolution + origin_y, (image.cols - p_max.x - 1) * resolution + origin_x, 0);

            double thickness = resolution * thickness_pixels;

            // Move back to centre
            if (dy > dx)
            {
                p_world_max.y -= thickness / 2.0;
                p_world_min.y -= thickness / 2.0;
            }
            else
            {
                p_world_max.x -= thickness / 2.0;
                p_world_min.x -= thickness / 2.0;
            }

            doors.push_back(Door());
            Door& door = doors.back();

            geo::Vector3 v = p_world_max - p_world_min;

            if (p_start.x > (p_max.x + p_min.x) / 2)
                v.y = -v.y;

            door.size = v.length();
            v = v / door.size;

            geo::Matrix3 m(v.x, -v.y, 0,
                           v.y,  v.x, 0,
                           0,    0,   1);

            door.init_pose = geo::Pose3D(m, (p_world_max + p_world_min) / 2);
            door.open_vel = geo::Vector3(0.3, 0, 0);
            if (c < 128)
                door.open_vel = -door.open_vel;

            door.open_distance = door.size;
            door.shape.reset(new geo::Box(geo::Vector3(-door.size / 2, -thickness / 2, 0), geo::Vector3(door.size / 2, thickness / 2, 1)));
            door.closed = true;
        }
    }

    cv::Mat vertex_index_map(image.rows, image.cols, CV_32SC1, -1);
    cv::Mat contour_map(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

    geo::CompositeShapePtr shape = std::make_shared<geo::CompositeShape>();

    for(int y = 0; y < image.rows; ++y)
    {
        for(int x = 0; x < image.cols; ++x)
        {
            unsigned char v = image.at<unsigned char>(y, x);

            if (v < 255)
            {
                std::vector<geo::Vec2i> points, line_starts;
                findContours(image, geo::Vec2i(x, y), 0, points, line_starts, contour_map, true);

                unsigned int num_points = points.size();

                if (num_points > 2)
                {
                    geo::Mesh mesh;

                    double min_z = origin_z;
                    double max_z = origin_z + (double)(255 - v) / 255 * blockheight;

                    std::list<TPPLPoly> testpolys;

                    TPPLPoly poly;
                    poly.Init(num_points);

                    for(unsigned int i = 0; i < num_points; ++i)
                    {
                        poly[i].x = points[i].x;
                        poly[i].y = points[i].y;

                        // Convert to world coordinates
                        double wy = -(points[i].x * resolution + origin_x);
                        double wx = (image.rows - points[i].y - 1) * resolution + origin_y;

                        vertex_index_map.at<int>(points[i].y, points[i].x) = mesh.addPoint(geo::Vector3(wx, wy, min_z));
                        mesh.addPoint(geo::Vector3(wx, wy, max_z));
                    }

                    testpolys.push_back(poly);

                    // Calculate side triangles
                    for(unsigned int i = 0; i < num_points; ++i)
                    {
                        int j = (i + 1) % num_points;
                        mesh.addTriangle(i * 2, i * 2 + 1, j * 2);
                        mesh.addTriangle(i * 2 + 1, j * 2 + 1, j * 2);
                    }

                    for(unsigned int i = 0; i < line_starts.size(); ++i)
                    {
                        int x2 = line_starts[i].x;
                        int y2 = line_starts[i].y;

                        while(image.at<unsigned char>(y2, x2) == v)
                            ++x2;

                        if (contour_map.at<unsigned char>(y2, x2 - 1) == 0)
                        {
                            // found a hole, so find the contours of this hole
                            std::vector<geo::Vec2i> hole_points;
                            findContours(image, geo::Vec2i(x2 - 1, y2 + 1), 1, hole_points, line_starts, contour_map, false);

                            if (hole_points.size() > 2)
                            {
                                TPPLPoly poly_hole;
                                poly_hole.Init(hole_points.size());
                                poly_hole.SetHole(true);

                                for(unsigned int j = 0; j < hole_points.size(); ++j)
                                {
                                    poly_hole[j].x = hole_points[j].x;
                                    poly_hole[j].y = hole_points[j].y;

                                    // Convert to world coordinates
                                    double wy = -(hole_points[j].x * resolution + origin_x);
                                    double wx = (image.rows - hole_points[j].y - 1) * resolution + origin_y;

                                    vertex_index_map.at<int>(hole_points[j].y, hole_points[j].x) = mesh.addPoint(geo::Vector3(wx, wy, min_z));
                                    mesh.addPoint(geo::Vector3(wx, wy, max_z));
                                }
                                testpolys.push_back(poly_hole);

                                // Calculate side triangles
                                for(unsigned int j = 0; j < hole_points.size(); ++j)
                                {
                                    const geo::Vec2i& hp1 = hole_points[j];
                                    const geo::Vec2i& hp2 = hole_points[(j + 1) % hole_points.size()];

                                    int i1 = vertex_index_map.at<int>(hp1.y, hp1.x);
                                    int i2 = vertex_index_map.at<int>(hp2.y, hp2.x);

                                    mesh.addTriangle(i1, i1 + 1, i2);
                                    mesh.addTriangle(i2, i1 + 1, i2 + 1);
                                }
                            }
                        }
                    }

                    TPPLPartition pp;
                    std::list<TPPLPoly> result;

                    if (!pp.Triangulate_EC(&testpolys, &result))
                    {
                        ROS_ERROR_STREAM("[PICO SIMULATOR] Could not triangulate polygon.");
                        return shape;
                    }

                    for(std::list<TPPLPoly>::iterator it = result.begin(); it != result.end(); ++it)
                    {
                        TPPLPoly& cp = *it;

                        int i1 = vertex_index_map.at<int>(cp[0].y, cp[0].x) + 1;
                        int i2 = vertex_index_map.at<int>(cp[1].y, cp[1].x) + 1;
                        int i3 = vertex_index_map.at<int>(cp[2].y, cp[2].x) + 1;

                        mesh.addTriangle(i1, i3, i2);
                    }

                    geo::Shape sub_shape;
                    sub_shape.setMesh(mesh);

                    shape->addShape(sub_shape, geo::Pose3D::identity());

                    cv::floodFill(image, cv::Point(x, y), 255);
                }
            }
        }
    }

    return shape;
}
