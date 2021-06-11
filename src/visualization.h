#ifndef PICO_SIMULATOR_VISUALIZATION_H_
#define PICO_SIMULATOR_VISUALIZATION_H_

#include "world.h"
#include "robot.h"

namespace visualization
{
    struct Bbox {
        double xmin, ymin, xmax, ymax;
    };

void visualize(const World& world, const std::vector<Robot*>& robots, bool collision, bool show_full_map,Bbox bbox);

}

#endif
