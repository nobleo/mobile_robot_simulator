#include "rclcpp/rclcpp.hpp"

#include "mobile_robot_simulator/laser_simulator.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "laser_simulator");
    rclcpp::Node nh("~");
        
    LaserScannerSimulator laser_sim(&nh);
    ros::AsyncSpinner spinner(1);
    
    RCLCPP_INFO(rclcpp::get_logger("MobileRobotSimulator"), "--- Starting LaserScanner simulator");
    
    rclcpp::Duration(0.5).sleep();
    
    laser_sim.start();
    
    spinner.start();
    while (nh.ok()) {
        //rclcpp::spin_some(node);
        rclcpp::Duration(0.01).sleep();
    }
    spinner.stop();
    
    laser_sim.stop();
    
    return 0;
    
} // end main
