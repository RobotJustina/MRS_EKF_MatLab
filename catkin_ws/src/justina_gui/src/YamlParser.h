#pragma once
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

class YamlParser
{
public:
    YamlParser();
    ~YamlParser();

    YAML::Node nodeLaPredefined;
    YAML::Node nodeRaPredefined;
    YAML::Node nodeHdPredefined;

    void loadLaPredefinedPositions(std::string file);
    void loadRaPredefinedPositions(std::string file);
    void loadHdPredefinedPositions(std::string file);
};
