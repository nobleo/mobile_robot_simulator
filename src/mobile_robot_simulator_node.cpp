#include "rclcpp/rclcpp.hpp"

#include "mobile_robot_simulator/mobile_robot_simulator.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mobile_robot_simulator");
    
    MobileRobotSimulator mob_sim(node);
    
    RCLCPP_INFO(node->get_logger(), "--- Starting MobileRobot simulator");
         
    mob_sim.start();
    
    rclcpp::spin(node);
    
    mob_sim.stop();
    
    return 0;
    
} // end main
