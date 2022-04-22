#include "heightmap.h"
#include "door.h"

#include <geolib/CompositeShape.h>

#include <ros/init.h>
#include <ros/package.h>

#include <string>



int main(int argc, char **argv)
{
    std::string heightmap_filename;
    heightmap_filename = ros::package::getPath("emc_simulator") + "/data/heightmap_correct.pgm";

    geo::Vector3 rpose;
    std::cout<<rpose<<std::endl;

    std::vector<Door> doors;
    geo::ShapePtr heightmap = createHeightMapShape(heightmap_filename, doors);

    for (int i = 0; i < 100; i++)
    {
        double dist = 0.15 + i * 0.004;
        bool test = heightmap->intersect(rpose,dist);
 
        if (test)
        {
            std::cout<< "Dist " << dist <<" Collision" <<std::endl;
        }
        else
        {
            std::cout<< "Dist " << dist <<" NO Collision"<<std::endl;
        }
    }   

    }
    