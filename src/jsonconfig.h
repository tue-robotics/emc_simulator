//
// Created by bob on 30-3-20.
//

#ifndef PROJECT_JSONCONFIG_H
#define PROJECT_JSONCONFIG_H

#include "moving_object.h"
#include "../3rdparty/json.hpp"
#include <fstream>
#include <boost/optional.hpp>

#include <vector>

class Config{

public:
    // Read config options from supplied json file
    Config(std::string filename){

        // Read the json object
        std::ifstream i(filename);
        nlohmann::json doc;
        try {
            doc = nlohmann::json::parse(i);
        }
        catch (const std::exception &e) {
            throw std::invalid_argument("[Error] json file not found or not a json file");
        }

//        // read mapfile
//        assert(doc.find("map_file") != doc.end());
//        std::string s = doc.at("map_file");
//        mapfile = s;

        // Read odometry settings
        assert(doc.find("uncertain_odom") != doc.end());
        bool b = doc.at("uncertain_odom");
        uncertain_odom = b;

        // Read odometry settings
        assert(doc.find("show_full_map") != doc.end());
        bool b2 = doc.at("show_full_map");
        show_full_map = b2;


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
            assert(object.find("repeat") != object.end());  //

            MovingObject m;

            m.length = object.at("length");
            m.width = object.at("width");
            m.trigger_radius = object.at("trigger_radius");
            m.velocity = object.at("velocity");
            m.init_pose.setOrigin(geo::Vector3(object.at("init_pose")[0], object.at("init_pose")[1],0));
            m.init_pose.setRPY(0,0,object.at("init_pose")[2]);
            m.final_pose.setOrigin(geo::Vector3(object.at("final_pose")[0], object.at("final_pose")[1],0));
            m.final_pose.setRPY(0,0,object.at("final_pose")[2]);
            m.repeat = object.at("repeat");

            ms.push_back(m);

        }
        moving_objects = ms;

        //Check if speedcap disabled is in json
        if(doc.find("disable_speedcap") != doc.end()){
            disable_speedcap = doc.at("disable_speedcap");
        }
        else{
            disable_speedcap = false;
        }

        //Check if taco enabled is in json
        if(doc.find("enable_taco") != doc.end()){
            enable_taco = doc.at("enable_taco");
        }
        else{
            enable_taco = false;
        }

    }

    void print(){
        std::cout << "Simulator configuration settings:" << std::endl;
        std::cout << "Uncertain odom: " << uncertain_odom.value() << std::endl;
        std::cout << "Show full map: " << show_full_map.value() << std::endl;
        std::cout << "disable_speedcap: " << disable_speedcap.value() << std::endl;
        std::cout << "enable taco: " << enable_taco.value() << std::endl;
        std::cout << "imported " << moving_objects.value().size() << " moving objects" << std::endl;
    }



public:
    boost::optional<bool> show_full_map;
    boost::optional<bool> uncertain_odom;
    boost::optional<std::vector<MovingObject>> moving_objects;
    boost::optional<bool> disable_speedcap;
    boost::optional<bool> enable_taco;
};



#endif //PROJECT_JSONCONFIG_H
