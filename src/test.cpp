
#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <bwi_rl/planning/PredictiveModel.h>
#include <bwi_rl/planning/ValueIteration.h>
#include <bwi_rl/planning/VITabularEstimator.h>

#include "NavMdp.h"
#include "Driver.h"

#define NUM_OF_TRIALS (10000)

int main(int argc, char **argv) {

    bool icorpp = true;

    ros::init(argc, argv, "bwi_nav_reasoning_node"); 
    ros::NodeHandle *nh = new ros::NodeHandle(); 

    State term, curr, next; 

    std::string path_static, path_sunny, path_coord; 
    std::string param_static_name("path_static"), param_sunny_name("path_sunny"), param_coord_name("path_coord"); 

    if (ros::param::has(param_static_name)) ros::param::get(param_static_name, path_static); 
    if (ros::param::has(param_sunny_name)) ros::param::get(param_sunny_name, path_sunny); 
    if (ros::param::has(param_coord_name)) ros::param::get(param_coord_name, path_coord); 

    boost::shared_ptr<VIEstimator<State, Action> > estimator(new VITabularEstimator<State, Action>); 

    Driver *driver = new Driver(nh, path_coord); 
    srand(time(NULL)); 
    std::ofstream logfile; 

    // std::cout << "creating nav model..." << std::endl; 
    // boost::shared_ptr<NavMdp> model(new NavMdp(nh, path_static, 
    //                                            path_sunny, "tmp/rl_domain/facts.plog", 
    //                                            term.row, term.col, path_coord)); 
    // model->dparser.updateDynamicObstacles(); 
    // ValueIteration<State, Action> vi(model, estimator);
    // std::cout << "Computing policy..." << std::endl;
    // vi.computePolicy(); 

    int last_rnum = -1, rnum; 
    for (int i=0; i<NUM_OF_TRIALS; i++) {


        if (ros::ok() == false) break;
        while (1) {
            rnum = rand() % 4; 

            if (rnum == last_rnum) continue;
            else last_rnum = rnum; 

            if (rnum == 0) {
                term.row = 0; term.col = 3;
            } else if (rnum == 1) {
                term.row = 2; term.col = 0;
            } else if (rnum == 2) {
                term.row = 4; term.col = 2;
            } else {
                term.row = 4; term.col = 4;
            }
        }

        logfile.open("/home/shiqi/Dropbox/Shared_space/20150911_shiqi/log.txt", std::ios::app); 
        logfile << ros::Time::now().toSec() << " " << term.row << " " << term.col << "\n"; 
        logfile.close(); 

        std::cout << "creating nav model..." << std::endl; 
        boost::shared_ptr<NavMdp> model(new NavMdp(nh, path_static, 
                                                   path_sunny, "tmp/rl_domain/facts.plog", 
                                                   term.row, term.col, path_coord)); 

        std::ostringstream policy_name;
        // policy_name << "/home/shiqi/Desktop/baseline_policy/" << rnum; 
        policy_name << "/home/shiqi/Dropbox/Shared_space/20150911_shiqi/"; 
        policy_name << rnum << model->dparser.walker_row << model->dparser.walker_col; 


        std::ifstream infile(policy_name.str().c_str()); 

        model->dparser.updateDynamicObstacles(); 
        if (infile.good() == false) {
            model->computeTransitionDynamics(); 
        }

        ValueIteration<State, Action> vi(model, estimator);

        if (infile.good()) {
            std::cout << "Loading policy: " << policy_name.str() << std::endl;
            vi.loadPolicy(policy_name.str()); 
        } else {
            std::cout << "Computing policy..." << std::endl;
            vi.computePolicy(); 
            vi.savePolicy(policy_name.str()); 
        }

        Action action; 
        while (ros::ok()) {

            ros::spinOnce(); 
            ros::Duration(1.0).sleep(); 

            driver->updateCurrentState(curr); 
            if (curr == term) break; 

            action = vi.getBestAction(curr); 
            next = curr;

            if (action == UP) next.row -= 1;
            else if (action == DOWN) next.row += 1;
            else if (action == LEFT) next.col -= 1;
            else if (action == RIGHT) next.col += 1; 
            else ROS_ERROR("action not recognized"); 

            std::cout << "curr: " << curr << " next: " << next << " action: " << action << std::endl; 

            driver->moveToGoalState(next); 
        }
    }
    logfile.close(); 

    ROS_INFO("task done!"); 
    return 0; 
}

