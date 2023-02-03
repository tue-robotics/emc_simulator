#ifndef PICO_SIMULATOR_VISUALIZATION_H_
#define PICO_SIMULATOR_VISUALIZATION_H_

#include <visualization_msgs/MarkerArray.h>

#include "world.h"
#include "robot.h"

namespace visualization
{
    struct Bbox {
        double xmin, ymin, xmax, ymax;
    };

void visualize(const World& world, const std::vector<RobotPtr>& robots, bool collision, bool show_full_map,Bbox bbox, double robotRadius);

visualization_msgs::MarkerArray create_rviz_objectmsg(const World &world, const MapConfig &mapconfig);
}
#endif
