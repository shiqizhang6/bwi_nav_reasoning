
#ifndef DOMAINPARSER_HPP
#define DOMAINPARSER_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>
#include <math.h>
#include "StateAction.h"
#include "bwi_msgs/AvailableRobotWithLocationArray.h"
#include "bwi_msgs/AvailableRobotWithLocation.h"
#include "bwi_msgs/AvailableRobot.h"

#define SIM_GRID_SIZE (2.5)
#define NUM_OF_WALKERS (10)

class DomainParser {

public:

    DomainParser() {}

    DomainParser(ros::NodeHandle *nh, const std::string static_obs, const
        std::string sunny_cells, const std::string plog_facts, 
        const std::string file_yaml, std::string topic_walkers); 

    ros::NodeHandle *nh_; 

    std::string topic_walkers_; 

    std::string file_static_obstacle;
    std::string file_sunny_cells; 
    std::string file_plog_facts; 

    std::vector<std::vector<int> > vec_static_obstacles;
    std::vector<std::vector<int> > vec_dynamic_obstacles;
    std::vector<std::vector<int> > vec_sunny_cells;

    // for instance, [0, 2] -> [1.231, 2.345]
    std::vector<std::vector<std::vector<float> > > coordinates_2d; 

    YAML::Node ynode;

    int col_num; 
    int row_num; 

    std::map<std::vector<int>, State> states_map; 

    void walkerCallback(const bwi_msgs::AvailableRobotWithLocationArray::ConstPtr &);

    void parseFile(const std::string file, std::vector<std::vector<int> >& vec); 
    void updateDynamicObstacles(); 
    void writeToFile(const std::string file); 
}; 

#endif
