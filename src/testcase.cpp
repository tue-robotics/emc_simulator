#include "heightmap.h"
#include "door.h"

#include <geolib/CompositeShape.h>
#include <geolib/HeightMap.h>
#include <geolib/Exporter.h>

#include <ros/init.h>
#include <ros/package.h>

#include <string>



int main(int argc, char **argv)
{
    std::string heightmap_filename = "data/heightmap";
    if (argc > 1)
        heightmap_filename = argv[1];
    std::string heightmap_ext = ".pgm";
    std::string heightmap_export = ".stl"; 
    std::string heightmap_in = ros::package::getPath("emc_simulator") + "/" + heightmap_filename + heightmap_ext;

    geo::Vector3 rpose(0, 0, 0);
    if (argc > 4)
            rpose = geo::Vector3(std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4]));

    std::vector<Door> doors;
    geo::ShapePtr heightmap = createHeightMapShape(heightmap_in, doors);

    bool collision = false;
    double dist = 0;

    double dist_0 = 0.15;
    double dist_step= 0.004;
    int N = 100;

    for (int i = 0; i < N; i++)
    {
        dist = dist_0 + i * dist_step;
        collision = heightmap->intersect(rpose,dist);
 
        if (collision)
        {
            break;
        }
    }   

    if (collision)
    {
        std::cout<< "Dist: " << dist << " Collision" <<std::endl;
    }
    else
    {
        std::cout<< "Dist: " << dist << " NO Collision" <<std::endl;
    }

    geo::Exporter exp;
    std::string exportFilename = ros::package::getPath("emc_simulator") + "/" + heightmap_filename + heightmap_export;
    exp.writeMeshFile(exportFilename, *heightmap);
    }
    
