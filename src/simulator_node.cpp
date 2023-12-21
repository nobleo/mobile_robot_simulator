#include "rclcpp/rclcpp.hpp"

#include "mobile_robot_simulator/mobile_robot_simulator.h"
#include "mobile_robot_simulator/laser_simulator.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "mobile_robot_simulator");
    rclcpp::Node nh;
    
    // global rate
    float rate = 10.0;
    
    MobileRobotSimulator mob_sim(&nh);
    LaserScannerSimulator laser_sim(&nh);
    
    RCLCPP_INFO(rclcpp::get_logger("MobileRobotSimulator"), "--- Starting simulator");
    
    rclcpp::Duration(1.0).sleep();
    ros::AsyncSpinner spinner(2);
    
    mob_sim.publish_map_transform = true;
    mob_sim.start();
    laser_sim.start();
    
    rclcpp::Time tic = rclcpp::Time::now();
    
    spinner.start();
    while (nh.ok() && rclcpp::Time::now()-tic<rclcpp::Duration(10.0)) {
        //rclcpp::spin_some(node);
        rclcpp::Duration(0.01).sleep();
    }
    spinner.stop();
    
    RCLCPP_INFO(rclcpp::get_logger("MobileRobotSimulator"), "--- Stopping simulator");
    
    mob_sim.stop();
    laser_sim.stop();
    
    tic = rclcpp::Time::now();
    while (nh.ok() && rclcpp::Time::now()-tic<rclcpp::Duration(5.0)) {
        rclcpp::spin_some(node);
        rclcpp::Duration(0.01).sleep();
    }
    
    return 0;
    
} // end main
