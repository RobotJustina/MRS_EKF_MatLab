#include <iostream>
#include <QApplication>
#include "MainWindow.h"
#include "QtRosNode.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING JUSTINA GUI BY MARCOSOFT" << std::endl;
    ros::init(argc, argv, "justina_gui");
    ros::NodeHandle n;


    std::string la_predefined_file;
    std::string ra_predefined_file;
    std::string hd_predefined_file;
    if(ros::param::has("~la_predefined"))
        ros::param::get("~la_predefined", la_predefined_file);
    if(ros::param::has("~ra_predefined"))
        ros::param::get("~ra_predefined", ra_predefined_file);
    if(ros::param::has("~hd_predefined"))
        ros::param::get("~hd_predefined", hd_predefined_file);

    YamlParser yamlParser;
    yamlParser.loadLaPredefinedPositions(la_predefined_file);
    yamlParser.loadRaPredefinedPositions(ra_predefined_file);
    yamlParser.loadHdPredefinedPositions(hd_predefined_file);

    QtRosNode qtRosNode;
    qtRosNode.setNodeHandle(&n);
    qtRosNode.start();

    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.setRosNode(&qtRosNode);
    mainWindow.setYamlParser(&yamlParser);
        
    mainWindow.show();
    return app.exec();
}
