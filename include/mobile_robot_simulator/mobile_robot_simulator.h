#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "tf2/LinearMath/Transform.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2/transform_datatypes.hpp>

#include <functional>

#ifndef MOBILE_ROBOT_SIMULATOR
#define MOBILE_ROBOT_SIMULATOR

class MobileRobotSimulator {

public:

    MobileRobotSimulator(const rclcpp::Node::SharedPtr &node); // default constructor
    ~MobileRobotSimulator(); // default destructor

    /*! start the simulation loop */
    void start(); //

    /*! stop everything */
    void stop();

    bool publish_map_transform = false; // whether or not to publish the map transform


private:

    /*! gets parameters from the parameter server */
    void get_params();

    /*! main update loop */
    void update_loop();

    /*! update the odometry info based on velocity and duration */
    void update_odom_from_vel(geometry_msgs::msg::Twist vel, rclcpp::Duration time_diff);

    /*! generate transform from odom */
    void get_tf_from_odom(nav_msgs::msg::Odometry odom);

    /*! callback function for velocity */
    void vel_callback(const geometry_msgs::msg::Twist& vel);
    void vel_stamped_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);

    /*! initial pose callback function */
    void init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);

    /*! get current time (sim_time or wall clock) */
    rclcpp::Time now();

    double publish_rate = 10;
    double speed_factor_ = 1.0;

    rclcpp::Logger logger_;

    nav_msgs::msg::Odometry odom; // odometry message
    tf2::Stamped<tf2::Transform> odom_trans; // odometry transform
    geometry_msgs::msg::TransformStamped map_trans; // transformation from odom to map

    rclcpp::Time last_vel; // last incoming velocity command
    rclcpp::Time last_update; // last time the odom was published
    rclcpp::Time measure_time; // this incoming velocity command
    bool message_received = false;
    rclcpp::Node::SharedPtr node_;

    bool is_running;

    // ROS interfaces
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::ConstSharedPtr vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::ConstSharedPtr vel_stamped_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr init_pose_sub;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    std::string base_link_frame = "base_link";

    rclcpp::TimerBase::SharedPtr loop_timer; // timer for the update loop
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    std::chrono::steady_clock::time_point wall_time_origin_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    double th = 0.0; // current pose (only need yaw, rest is calculated)

}; // end class

#endif
