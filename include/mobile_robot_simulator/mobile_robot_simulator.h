#include "rclcpp/rclcpp.hpp"
#include "ros/console.h"

#include <geometry_msgs/msg/twist.hpp>
#include "tf/LinearMath/Transform.h"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <functional>

#ifndef MOBILE_ROBOT_SIMULATOR
#define MOBILE_ROBOT_SIMULATOR

class MobileRobotSimulator {

public:

    MobileRobotSimulator(rclcpp::Node *nh); // default constructor
    ~MobileRobotSimulator(); // default destructor

    /*! start the simulation loop */
    void start(); //

    /*! stop everything */
    void stop();

    bool publish_map_transform; // whether or not to publish the map transform


private:

    /*! gets parameters from the parameter server */
    void get_params();

    /*! main update loop */
    void update_loop(const rclcpp::TimerEvent& event);

    /*! update the odometry info based on velocity and duration */
    void update_odom_from_vel(geometry_msgs::msg::Twist vel, rclcpp::Duration time_diff);

    /*! generate transform from odom */
    void get_tf_from_odom(nav_msgs::msg::Odometry odom);

    /*! callback function for velocity */
    void vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);

    /*! initial pose callback function */
    void init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);

    double publish_rate;

    nav_msgs::msg::Odometry odom; // odometry message
    tf2::StampedTransform odom_trans; // odometry transform
    tf2::StampedTransform map_trans; // transformation from odom to map

    rclcpp::Time last_vel; // last incoming velocity command
    rclcpp::Time last_update; // last time the odom was published
    rclcpp::Time measure_time; // this incoming velocity command
    bool message_received = false;
    rclcpp::Node * nh_ptr;

    bool is_running;

    // ROS interfaces
    ros::Publisher odom_pub;
    ros::Subscriber vel_sub;
    ros::Subscriber init_pose_sub;
    tf::TransformBroadcaster tf_broadcaster;

    //Topics
    std::string velocity_topic;
    std::string odometry_topic;
    std::string base_link_frame;

    rclcpp::Timer loop_timer; // timer for the update loop

    double th = 0.0; // current pose (only need yaw, rest is calculated)

}; // end class

#endif
