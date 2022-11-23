#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "Iir.h"
#include <algorithm>

#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#define PORT     44100

using boost::asio::ip::udp;


class azrael_driver : public rclcpp::Node
{
  public:

    azrael_driver();
    

  private:
    void call_odom();
    void timer_udp_receive();
    void timer_udp_send();


    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    const double radius = 0.1016;
    const double lxy    = 0.71;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_cmd_ ;

    
    double v_wheels_[4]  = {0.0,0.0,0.0,0.0};
    double v_robot_[3]   = {0.0,0.0,0.0};


    double velx_odom = 0.0;
    double vely_odom = 0.0;
    double velw_odom = 0.0;
 
    double posx_odom = 0.0;
    double posy_odom = 0.0;
    double posw_odom = 0.0;

    Iir::Butterworth::LowPass<2> fx;
    Iir::Butterworth::LowPass<2> fy;
    Iir::Butterworth::LowPass<2> fw;

    double vx_max_ = 0.4;
    double vy_max_ = 0.4;
    double vw_max_ = 1.0;

    // int sockfd_;
    // struct sockaddr_in servaddr_, cliaddr_;

    // int  n_out_;
    // unsigned int len_addr_ = sizeof(cliaddr_) ;

    boost::asio::io_context io_context_;
    udp::socket socket_(io_context_, udp::endpoint(udp::v4(), PORT));
    udp::endpoint remote_endpoint_;


    rclcpp::TimerBase::SharedPtr                               timer_odom_, timer_rec, timer_send;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::mutex v_robot_mutex_,v_wheels_mutex_;

    std::chrono::time_point<std::chrono::high_resolution_clock> current_time ;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time    ;

    nav_msgs::msg::Odometry message_odom ;

    rclcpp::TimerBase::SharedPtr timer_;

    std::thread t1;
    std::thread t2;

};