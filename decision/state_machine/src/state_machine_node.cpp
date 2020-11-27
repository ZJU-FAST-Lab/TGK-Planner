#include "state_machine/fsm.h"

#include <ros/ros.h>

using namespace tgk_planner;

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "state_machine_node");
    ros::NodeHandle nh("~");
    
    FSM fsm;
    fsm.init(nh);
    
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    // ros::spin();
    // return 0;
}