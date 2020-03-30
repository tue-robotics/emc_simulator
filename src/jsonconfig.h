//
// Created by bob on 30-3-20.
//

#ifndef PROJECT_JSONCONFIG_H
#define PROJECT_JSONCONFIG_H

#include "moving_object.h"
#include "../3rdparty/json.hpp"
#include <fstream>

class Config{

public:
    // Read config options from supplied json file
    Config(std::string filename){

        // Read the json object
        std::ifstream i(filename);
        nlohmann::json doc = nlohmann::json::parse(i);

        // read mapfile
        assert(doc.find("mapfile") != doc.end());
        std::string s = doc.at("mapfile");
        mapfile = s;

        // Read odometry settings
        assert(doc.find("uncertain_odom") != doc.end());
        bool b = doc.at("uncertain_odom");
        uncertain_odom = b;


        // read moving objects
        std::vector<MovingObject> ms;
        for (const auto& object : doc.at("objects") )
        {
            assert(object.is_object());
            assert(object.find("length") != object.end());  //
            assert(object.find("width") != object.end());  //
            assert(object.find("trigger_radius") != object.end());  //
            assert(object.find("init_pose") != object.end());  //
            assert(object.find("final_pose") != object.end());  //
            assert(object.find("velocity") != object.end());  //

            MovingObject m;

            m.length = object.at("length");
            m.width = object.at("length");
            m.trigger_radius = object.at("trigger_radius");
            m.velocity = object.at("velocity");
            m.init_pose.setOrigin(geo::Vector3(object.at("init_pose")[0], object.at("init_pose")[1],0));
            m.init_pose.setRPY(0,0,object.at("init_pose")[2]);
            m.final_pose.setOrigin(geo::Vector3(object.at("final_pose")[0], object.at("final_pose")[1],0));
            m.final_pose.setRPY(0,0,object.at("final_pose")[2]);

            ms.push_back(m);

        }
        moving_objects = ms;
    }



public:
    boost::optional<bool> uncertain_odom;
    boost::optional<std::string> mapfile;
    boost::optional<std::vector<MovingObject>> moving_objects;
};



#endif //PROJECT_JSONCONFIG_H
