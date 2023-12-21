#include "rclcpp/rclcpp.hpp"

#include "mobile_robot_simulator/mobile_robot_simulator.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("mobile_robot_simulator");
    
    MobileRobotSimulator mob_sim(nh);
    
    RCLCPP_INFO(nh->get_logger(), "--- Starting MobileRobot simulator");
         
    mob_sim.start();
    
    rclcpp::spin(nh);
    
    mob_sim.stop();
    
    return 0;
    
} // end main
