#include "rclcpp/rclcpp.hpp"

#include "mobile_robot_simulator/mobile_robot_simulator.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "mobile_robot_simulator");
    rclcpp::Node nh("~");
    rclcpp::Duration(0.1).sleep();
    
    MobileRobotSimulator mob_sim(&nh);
    ros::AsyncSpinner spinner(1);
    
    RCLCPP_INFO(rclcpp::get_logger("MobileRobotSimulator"), "--- Starting MobileRobot simulator");
    
    rclcpp::Duration(0.1).sleep();
     
    mob_sim.start();
    
    spinner.start();
    while (nh.ok()) {
        //rclcpp::spin_some(node);
        rclcpp::Duration(0.01).sleep();
    }
    spinner.stop();
    
    mob_sim.stop();
    
    return 0;
    
} // end main
