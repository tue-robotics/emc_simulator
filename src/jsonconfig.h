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
        if(doc.find("uncertain_odom") != doc.end()){
            uncertain_odom = doc.at("uncertain_odom");
        }
        else{
            uncertain_odom = false;
        }

        // Read full map settings
        if(doc.find("show_full_map") != doc.end()){
            show_full_map = doc.at("show_full_map");
        }
        else{
            show_full_map = false;
        }

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

            m.is_moving = false;
            m.is_paused = false;
            m.finished_moving = false;

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

        //Check if pyro is used in json
        if(doc.find("use_pyro") != doc.end()){
            use_pyro = doc.at("use_pyro");
        }
        else{
            use_pyro = true;
        }
        
        if(doc.find("spawn") != doc.end()){
            std::vector<double> spawn_location = doc.at("spawn");
            assert(spawn_location.size()==3);
            spawn = geo::Pose3D(spawn_location[0],spawn_location[1],0,0,0,spawn_location[2]);
        }
        else{
            spawn = geo::Pose3D::identity();
        }
        

    }

    void print(){// todo add custom spawn location
        std::cout << "Simulator configuration settings:" << std::endl;
        std::cout << "Uncertain odom: " << uncertain_odom.value() << std::endl;
        std::cout << "Show full map: " << show_full_map.value() << std::endl;
        std::cout << "disable_speedcap: " << disable_speedcap.value() << std::endl;
        std::cout << "enable taco: " << enable_taco.value() << std::endl;
        std::cout << "use pyro: " << use_pyro.value() << std::endl;
        std::cout << "imported " << moving_objects.value().size() << " moving objects" << std::endl;
    }



public:
    boost::optional<bool> show_full_map;
    boost::optional<bool> uncertain_odom;
    boost::optional<std::vector<MovingObject>> moving_objects;
    boost::optional<bool> disable_speedcap;
    boost::optional<bool> enable_taco;
    boost::optional<bool> use_pyro;
    boost::optional<geo::Pose3D> spawn;
};



#endif //PROJECT_JSONCONFIG_H
