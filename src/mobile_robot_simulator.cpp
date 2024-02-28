#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer_interface.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "mobile_robot_simulator/mobile_robot_simulator.h"

MobileRobotSimulator::MobileRobotSimulator(const rclcpp::Node::SharedPtr& nh) :
  logger_(nh->get_logger()),
  clock_(nh->get_clock()),
  tf_broadcaster(*nh)
{
    nh_ptr = nh;
    // get parameters
    get_params();
    odom_pub = nh_ptr->create_publisher<nav_msgs::msg::Odometry>("/odom",50); // odometry publisher
    vel_sub = nh_ptr->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",5,[this](geometry_msgs::msg::Twist::ConstSharedPtr msg){vel_callback(msg);}); // velocity subscriber

    // initialize timers
    last_update = steady_clock_.now();
    last_vel = last_update - rclcpp::Duration::from_seconds(0.1);
    // initialize forst odom message
    update_odom_from_vel(geometry_msgs::msg::Twist(), rclcpp::Duration::from_seconds(0.1));
    odom.header.stamp = last_update;
    get_tf_from_odom(odom);
    // Initialize tf from map to odom
    if (publish_map_transform)
    {
        init_pose_sub = nh_ptr->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",5,[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg){init_pose_callback(msg);}); // initial pose callback
        map_trans.header.frame_id = "/map";
        map_trans.header.stamp = last_update;
        map_trans.child_frame_id = "/odom";
        tf2::convert(tf2::Transform::getIdentity(), map_trans.transform);
    }

    RCLCPP_INFO(logger_, "Initialized mobile robot simulator");

}

MobileRobotSimulator::~MobileRobotSimulator()
{
    if (is_running) stop();
}

void MobileRobotSimulator::get_params()
{
     publish_map_transform = nh_ptr->declare_parameter("publish_map_transform", publish_map_transform);
     publish_rate  = nh_ptr->declare_parameter("publish_rate", publish_rate);
     base_link_frame  = nh_ptr->declare_parameter("base_link_frame", base_link_frame);
}


void MobileRobotSimulator::start()
{
    loop_timer = nh_ptr->create_wall_timer(std::chrono::duration<double>(1.0/publish_rate), std::bind(&MobileRobotSimulator::update_loop, this));
    is_running = true;
    RCLCPP_INFO(logger_, "Started mobile robot simulator update loop, listening on cmd_vel topic");
}

void MobileRobotSimulator::stop()
{
    loop_timer->cancel();
    is_running = false;
    RCLCPP_INFO(logger_, "Stopped mobile robot simulator");
}

void MobileRobotSimulator::update_loop()
{
    // RCLCPP_INFO(logger_, "MobileRobotSimulator::update_loop");

    last_update = steady_clock_.now();
    // If we didn't receive a message, send the old odometry info with a new timestamp
    if (!message_received)
    {
        odom.header.stamp = last_update;
        odom_trans.stamp_ = tf2_ros::fromRclcpp(last_update);
    }
    // publish odometry and tf
    odom_pub->publish(odom);
    get_tf_from_odom(odom);
    geometry_msgs::msg::TransformStamped odom_trans_msg = tf2::toMsg(odom_trans);
    odom_trans_msg.child_frame_id = odom.child_frame_id;
    tf_broadcaster.sendTransform(odom_trans_msg); // odom -> base_link_frame
    message_received = false;
    // should we publish the map transform?
    if (!publish_map_transform) return;
    map_trans.header.stamp = last_update;
    tf_broadcaster.sendTransform(map_trans); // map -> odom
}

void MobileRobotSimulator::update_odom_from_vel(geometry_msgs::msg::Twist vel, rclcpp::Duration time_diff)
{
    RCLCPP_DEBUG_STREAM(logger_, "Velocity - x: " << vel.linear.x << " y: " << vel.linear.y << " th: " << vel.angular.z);
    //compute odometry in a typical way given the velocities of the robot
    double delta_x = (vel.linear.x * cos(th) - vel.linear.y * sin(th)) * time_diff.seconds();
    double delta_y = (vel.linear.x * sin(th) + vel.linear.y * cos(th)) * time_diff.seconds();
    double delta_th = vel.angular.z * time_diff.seconds();
    RCLCPP_DEBUG_STREAM(logger_, "Delta - x: " << delta_x << " y: " << delta_y << " th: " << delta_th);

    // update odometry
    odom.header.stamp = measure_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x += delta_x;
    odom.pose.pose.position.y += delta_y;
    // generate quaternion based on current yaw
    th += delta_th;
    tf2::Quaternion q;
    q.setRPY(0, 0, th);
    odom.pose.pose.orientation = tf2::toMsg(q);
    // set velocity
    odom.child_frame_id = base_link_frame;
    odom.twist.twist = vel;
    RCLCPP_DEBUG_STREAM(logger_, "Odometry - x: " << odom.pose.pose.position.x << " y: " << odom.pose.pose.position.y << " th: " << th);
}

void MobileRobotSimulator::get_tf_from_odom(nav_msgs::msg::Odometry odom)
{
    geometry_msgs::msg::TransformStamped odom_tmp;
    // copy from odmoetry message
    odom_tmp.header = odom.header;
    odom_tmp.child_frame_id = odom.child_frame_id;
    odom_tmp.transform.translation.x = odom.pose.pose.position.x;
    odom_tmp.transform.translation.y = odom.pose.pose.position.y;
    odom_tmp.transform.translation.z = 0.0;
    odom_tmp.transform.rotation = odom.pose.pose.orientation;
    // convert and update
    tf2::convert(odom_tmp, odom_trans);
}

void MobileRobotSimulator::vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
    RCLCPP_DEBUG(logger_, "Received message on cmd_vel");
    measure_time = steady_clock_.now();
    rclcpp::Duration dt = measure_time - last_vel;
    last_vel = measure_time;
    if (dt >= rclcpp::Duration::from_seconds(0.5)) dt = rclcpp::Duration::from_seconds(0.1);
    message_received = true;
    geometry_msgs::msg::Twist vel = *msg;

    RCLCPP_INFO(logger_, "Received cmd_vel(%.3f, %.3f, %.3f), dt: %f", vel.linear.x, vel.linear.y, vel.angular.z, dt.seconds());
    update_odom_from_vel(vel,dt);
}

void MobileRobotSimulator::init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
{
    if (msg->header.frame_id != "map") {
        RCLCPP_ERROR(logger_, "Initial pose not specified in map frame, ignoring");
        return;
    }
    RCLCPP_INFO(logger_, "Received pose estimate of mobile base");

    // msg is map -> base_link_frame
    tf2::Stamped<tf2::Transform> msg_t;
    msg_t.setOrigin(tf2::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
    msg_t.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));
    RCLCPP_DEBUG_STREAM(logger_, "map -> base_link_frame - x: " << msg_t.getOrigin().getX() << " y: " << msg_t.getOrigin().getY());
    // get odom -> base_link_frame
    RCLCPP_DEBUG_STREAM(logger_, "odom -> base_link_frame - x: " << odom_trans.getOrigin().getX() << " y: " << odom_trans.getOrigin().getY());
    // calculate map -> odom and save as stamped
    auto map_t = msg_t * odom_trans.inverse();
    RCLCPP_DEBUG_STREAM(logger_, "map -> odom - x: " << map_t.getOrigin().getX() << " y: " << map_t.getOrigin().getY());
    map_trans.header.stamp = msg->header.stamp;
    tf2::convert(map_t, map_trans.transform);
}



