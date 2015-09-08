#include "DomainParser.h"
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>

DomainParser::DomainParser(ros::NodeHandle *nh, const std::string static_obs, 
        const std::string sunny_cells, const std::string plog_facts,
        const std::string file_yaml, const std::string topic_walkers) {

    nh_ = nh; 

    file_static_obstacle = static_obs; 
    file_sunny_cells = sunny_cells;
    file_plog_facts = plog_facts; 

    vec_static_obstacles = std::vector<std::vector<int> >(0, std::vector<int>(0,0));
    vec_dynamic_obstacles = std::vector<std::vector<int> >(0, std::vector<int>(0,0));
    vec_sunny_cells = std::vector<std::vector<int> >(0, std::vector<int>(0,0));



    parseFile(file_static_obstacle, vec_static_obstacles); 
    std::cout << "finished parsing: " << file_static_obstacle << std::endl;

    parseFile(file_sunny_cells, vec_sunny_cells); 
    std::cout << "finished parsing: " << file_sunny_cells << std::endl;



    coordinates_2d = std::vector<std::vector<std::vector<float> > > (
        vec_static_obstacles.size(), std::vector<std::vector<float> > (
            vec_static_obstacles[0].size(), std::vector<float>(2, 0))); 

    ynode=YAML::LoadFile(ros::package::getPath("bwi_nav_reasoning") + "/maps/" + file_yaml); 

    for (YAML::const_iterator row_pt = ynode["row_y"].begin();
            row_pt != ynode["row_y"].end(); row_pt++) {

        for (YAML::const_iterator col_pt = ynode["col_x"].begin();
                col_pt != ynode["col_x"].end(); col_pt++) {
            
            coordinates_2d[row_pt->first.as<int>()][col_pt->first.as<int>()][0] 
                = row_pt->second.as<float>();
            coordinates_2d[row_pt->first.as<int>()][col_pt->first.as<int>()][1] 
                = col_pt->second.as<float>();
        }
    }


    updateDynamicObstacles(vec_static_obstacles, vec_dynamic_obstacles, ynode, 
        topic_walkers); 
    std::cout << "updated dynamic obstacles" << std::endl;

    row_num = vec_static_obstacles.size();
    col_num = vec_static_obstacles[0].size(); 

    int cnt = 0; 
    std::vector<int> key(2, 0); 
    for (int i=0; i<row_num; i++) {
        for (int j=0; j<col_num; j++) {
            // when the cell is open
            if (vec_static_obstacles[i][j] == 1) {
                State s;
                s.row = i;
                s.col = j;
                s.index = cnt++; 
                s.under_sunlight = (vec_sunny_cells[i][j] == 1); 
                s.has_human = (vec_dynamic_obstacles[i][j] == 1); 
                key[0] = i;
                key[1] = j;
                states_map[key] = s; 
            }
        }
    }

    State s;
    s.row = s.col = -1;
    s.index = cnt;
    key[0] = -1;
    key[1] = -1;
    states_map[key] = s; 

    std::cout << "finished domain files parsing" << std::endl; 
    
    std::cout << "writing to file: " << file_plog_facts << std::endl;
    writeToFile(file_plog_facts); 
    std::cout << "writing to file finished" << std::endl; 
}

void DomainParser::walkerCallback(const bwi_msgs::AvailableRobotWithLocationArray::ConstPtr& msg) {
    
    ROS_INFO("I heard: [%s]", msg->data.c_str());

    // initializing with a walker-free setting
    for (int i=0; i<vec_dynamic_obstacles.size(); i++) {
        for (int j=0; j<vec_dynamic_obstacles[0].size(); j++) {
            vec_dynamic_obstacles[i][j] = 0; 
        }
    }

    float x, y; 
    for (int i=0; i<NUM_OF_WALKERS; i++) {

        x = msg->data[i].pose.position.x;
        y = msg->data[i].pose.position.y;

        int min_row = min_col = -1, min_dis = 100000; 
        for (int j=0; j<coordinates_2d.size(); j++) {
            for (int k=0; k<coordinates_2d[0].size(); k++) {
                float curr_dis = ( (y - coordinates_2d[j][k][0])^2 + 
                                   (x - coordinates_2d[j][k][1])^2 ) ^0.5; 
                if (curr_dis < min_dis) {
                    min_row = coordinates_2d[j][k][0];
                    min_col = coordinates_2d[j][k][1]; 
                    min_dis = curr_dis; 
                }
            }
        }

        if (min_row < 0 or min_col < 0) {
            std::cerr << "error in locate walkers" << std::endl;
        } else if (min_dis < SIM_GRID_SIZE) {
            vec_dynamic_obstacles[min_row][min_col] = 1; 
        }
    }
}

void DomainParser::updateDynamicObstacles(
    const std::vector<std::vector<int> > vec_static_obstacles,
    std::vector<std::vector<int> >& vec_dynamic_obstacles, 
    YAML::Node ynode, const std::string topic_walkers) {
 
    // dynamic vec has the same size as static vec
    vec_dynamic_obstacles = vec_static_obstacles; 
    
    ros::Subscriber sub = nh->subscribe(topic_walkers, 1000, &DomainParser::callback, this);

}

void DomainParser::parseFile(const std::string filename,
    std::vector<std::vector<int> >& ret) {
 
    ret.clear();    
    std::ifstream input_file(filename.c_str()); 
    int num_row, num_col; 

    if (!input_file) {
        std::cout << "could not open file: " << filename << std::endl; 
    }

    input_file >> num_row; 
    input_file >> num_col; 
    for (int i=0; i<num_row; i++) {
        std::vector<int> vec;
        for (int j=0; j<num_col; j++) {
            std::string str;
            input_file >> str; 
            vec.push_back(boost::lexical_cast<int>(str)); 
        }
        ret.push_back(vec);
    }

}

void DomainParser::writeToFile(const std::string filename) {

    std::string str(""); 
    int state_num = states_map.size();
    str += "state={0.." + boost::lexical_cast<std::string>(state_num-1) + "}.\n"; 
    str += "terminal(" + boost::lexical_cast<std::string>(state_num-1) + ").\n";

    std::string str_leftof, str_belowof, str_sunny, str_human; 

    for (std::map<std::vector<int>, State>::iterator it=states_map.begin(); 
        it!=states_map.end(); it++) {

        std::vector<int> vec(2, 0); 

        vec[0] = it->first[0]; 
        vec[1] = it->first[1]-1; // to find the one one its left

        if (states_map.find(vec) != states_map.end()) {
            int index_left = states_map.find(vec)->second.index;
            str_leftof += "leftof(" + 
                boost::lexical_cast<std::string>(index_left) + "," + 
                boost::lexical_cast<std::string>(it->second.index) + ").\n"; 
        }

        vec[0] = it->first[0]+1; // to find the one below it
        vec[1] = it->first[1]; 

        if (states_map.find(vec) != states_map.end()) {
            int index_below = states_map.find(vec)->second.index;
            str_belowof += "belowof(" + 
                boost::lexical_cast<std::string>(index_below) + "," +
                boost::lexical_cast<std::string>(it->second.index) + ").\n";
        }

        if (it->second.under_sunlight) {
            str_sunny += "sunny(" + 
                boost::lexical_cast<std::string>(it->second.index) + ").\n"; 
        }

        if (it->second.has_human) {
            str_human += "human(" + 
                boost::lexical_cast<std::string>(it->second.index) + ").\n"; 
        }
    }

    std::ofstream output_file(filename.c_str());
    str += str_leftof + str_belowof + str_sunny + str_human;

    if (output_file.is_open()) {
        output_file << str; 
    }
    output_file.close(); 

}

