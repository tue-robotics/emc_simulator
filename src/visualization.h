#ifndef PICO_SIMULATOR_VISUALIZATION_H_
#define PICO_SIMULATOR_VISUALIZATION_H_

#include "world.h"

namespace visualization
{
    struct Bbox {
        double xmin, ymin, xmax, ymax;
    };

void visualize(const World& world, Id robot_id, bool collision, bool show_full_map,Bbox bbox);

}

#endif
