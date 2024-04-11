#include "rclcpp/rclcpp.hpp"

#include "mobile_robot_simulator/laser_simulator.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("laser_simulator");
        
    LaserScannerSimulator laser_sim(nh);
    
    RCLCPP_INFO(rclcpp::get_logger("MobileRobotSimulator"), "--- Starting LaserScanner simulator");
        
    laser_sim.start();
    
    rclcpp::spin(nh);
    
    laser_sim.stop();
    
    return 0;
    
} // end main
