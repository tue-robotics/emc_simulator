#ifndef PICO_SIMULATOR_HEIGHTMAP_H_
#define PICO_SIMULATOR_HEIGHTMAP_H_

#include <geolib/datatypes.h>

#include <vector>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/imgproc.hpp>

struct Door;

class MapLoader
{
    public:
    MapLoader();

    void getMap(nav_msgs::OccupancyGrid& mapRef);

    void getMapMetadata(nav_msgs::MapMetaData& metadataRef);

    void getMapImage(cv::Mat& imageRef);
    
    bool isInitialized(){ return initialized; };

    private:
    ros::NodeHandle nh;

    void load();

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    ros::Subscriber sub_map;
    ros::CallbackQueue cb_map;
    bool initialized = false;
    nav_msgs::OccupancyGrid map;
    cv::Mat mapImage;
};

geo::ShapePtr createHeightMapShape(const std::string& filename, std::vector<Door>& doors);

geo::ShapePtr createHeightMapShape(const cv::Mat& image_tmp, double resolution, std::vector<Door>& doors);

#endif
