#ifndef PICO_SIMULATOR_HEIGHTMAP_H_
#define PICO_SIMULATOR_HEIGHTMAP_H_

#include <geolib/datatypes.h>

geo::ShapePtr createHeightMapShape(const std::string& filename);

#endif
