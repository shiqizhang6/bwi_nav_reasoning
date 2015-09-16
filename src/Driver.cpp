
#include <math.h> // for pow(,)
#include "Driver.h"

#define MAX_FLOAT (10000.0)
#define MAX_INDEX (1000)
#define GRID_WIDTH (15.0)

Driver::Driver(ros::NodeHandle *node_handle, std::string file_yaml) {

    nh = node_handle; 
    std::string path = ros::package::getPath("bwi_nav_reasoning") + "/maps/"; 
    file_coordinates = path + file_yaml; 
    
    ynode = YAML::LoadFile(file_coordinates); 

    sub_amcl_pose = nh->subscribe("/amcl_pose", 100, &Driver::callbackUpdatePosition, this); 
    pub_simple_goal = nh->advertise<geometry_msgs::PoseStamped>("/move_base_interruptable_simple/goal", 100);
    ros::Duration(2.0).sleep(); // give sometime for setting up publishers

    curr_row = curr_col = MAX_INDEX; 
    ros::spinOnce(); 
}

void Driver::callbackUpdatePosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

    curr_position = *msg;
    int row, col, min_row = MAX_INDEX, min_col = MAX_INDEX; 
    float min_distance = MAX_FLOAT; 

    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    // std::cout << "float x: " << x << std::endl; 
    // std::cout << "float y: " << y << std::endl; 

    for (int i=0; i<ynode.size(); i++) {
        for (int j=0; j<ynode[i].size(); j++) {
            float dis = pow(pow(x - ynode[i][j][0].as<float>(), 2.0) + 
                            pow(y - ynode[i][j][1].as<float>(), 2.0), 0.5);
        
            if (dis <min_distance) {
                min_row = i;
                min_col = j; 
                min_distance = dis; 
            }
        }
    }

    curr_x = msg->pose.pose.position.x;
    curr_y = msg->pose.pose.position.y; 
    curr_row = min_row;
    curr_col = min_col; 
}

void Driver::updateCurrentState(State &state) {

    ros::spinOnce(); 
    if (curr_row == MAX_INDEX or curr_col == MAX_INDEX) {
        ROS_ERROR("bwi_nav_reasoning cannot get current position"); 
    } else {
        state.row = curr_row;
        state.col = curr_col; 
    }
}

bool Driver::moveToGoalState(const State &state) {

    float goal_x, goal_y; 

    geometry_msgs::PoseStamped msg_goal; 
    
    msg_goal.header.frame_id = "level_mux/map"; 
    msg_goal.pose.position.x = goal_x = ynode["col_x"][state.col].as<float>();
    msg_goal.pose.position.y = goal_y = ynode["row_y"][state.row].as<float>();
    msg_goal.pose.orientation.z = 0.0;
    msg_goal.pose.orientation.w = 1.0;
        
    bool has_arrived = false;

    while (ros::ok() and !has_arrived) {
        ros::spinOnce(); 
        float x = goal_x - curr_x;
        float y = goal_y - curr_y; 
        float dis = pow(x*x + y*y, 0.5); 

        // 0.8: to force robot to move toward the center of grid cells, after
        // the body get into the corresponding grid cells
        has_arrived = dis < 0.5; 

        pub_simple_goal.publish(msg_goal); 
        ros::Duration(1.0).sleep(); 
    }
}

