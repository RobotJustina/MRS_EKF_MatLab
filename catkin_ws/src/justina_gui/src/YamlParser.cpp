#include "YamlParser.h"

YamlParser::YamlParser()
{
}

YamlParser::~YamlParser()
{
}

void YamlParser::loadLaPredefinedPositions(std::string file)
{
    std::cout << "YamlParser.->Trying to parse file: " << file << std::endl;
    nodeLaPredefined = YAML::LoadFile(file);
}

void YamlParser::loadRaPredefinedPositions(std::string file)
{
    std::cout << "YamlParser.->Trying to parse file: " << file << std::endl;
    nodeRaPredefined = YAML::LoadFile(file);
}

void YamlParser::loadHdPredefinedPositions(std::string file)
{
    std::cout << "YamlParser.->Trying to parse file: " << file << std::endl;
    nodeHdPredefined = YAML::LoadFile(file);
}
