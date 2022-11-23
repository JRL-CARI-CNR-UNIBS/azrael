#include <azrael_driver_udp/driver.h>

using std::placeholders::_1;
using namespace std::chrono_literals;



azrael_driver::azrael_driver() : Node("azrael_driver")
{

    RCLCPP_INFO(this->get_logger(), "Constructor init");

    // if ( (sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
    //     perror("socket creation failed");
    //     exit(EXIT_FAILURE);
    // }

    // memset(&servaddr_, 0, sizeof(servaddr_));
    // memset(&cliaddr_ , 0, sizeof(cliaddr_));

    // // Filling server information 
    // servaddr_.sin_family    = AF_INET; // IPv4 
    // servaddr_.sin_addr.s_addr = inet_addr("192.169.1.2");
    // servaddr_.sin_port = htons(PORT);

    // cliaddr_.sin_family    = AF_INET; // IPv4 
    // cliaddr_.sin_addr.s_addr = inet_addr("192.169.1.1");
    // cliaddr_.sin_port = htons(PORT);

    // if ( bind(sockfd_, (const struct sockaddr *)&servaddr_,
    //         sizeof(servaddr_)) < 0 )
    // {
    //     perror("bind failed");
    //     exit(EXIT_FAILURE);
    // }

    socket = new udp::socket(io_context);
    remote_endpoint = udp::endpoint(address::from_string(IPADDRESS_REMOTE), UDP_PORT);
    local_endpoint = udp::endpoint(address::from_string(IPADDRESS_LOCAL), UDP_PORT);
    socket->open(udp::v4());
    // socket->bind(udp::endpoint(address::from_string(IPADDRESS), UDP_PORT));
    socket->bind(local_endpoint);
    io_context.run();
    

    const float samplingrate     = 50; // Hz
    const float cutoff_frequency = 10; // Hz
    fx.setup (samplingrate, cutoff_frequency);
    fy.setup (samplingrate, cutoff_frequency);
    fw.setup (samplingrate, cutoff_frequency);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    //example1.cpp
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(
        qos_profile.history,
        qos_profile.depth
        ),
        qos_profile);

    odom_pub_    = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
    timer_odom_  = this->create_wall_timer(20ms, std::bind(&azrael_driver::call_odom, this));
    timer_send   = this->create_wall_timer(20ms, std::bind(&azrael_driver::timer_udp_send, this));
    timer_rec    = this->create_wall_timer(20ms, std::bind(&azrael_driver::timer_udp_receive, this));
    
    // std::thread t1(&azrael_driver::timer_udp_send, this);
    // std::thread t2(&azrael_driver::timer_udp_receive, this);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&azrael_driver::cmd_vel_callback, this, _1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    current_time = std::chrono::high_resolution_clock::now();
    last_time    = std::chrono::high_resolution_clock::now();

    // t1.join();
    // t2.join();

    RCLCPP_INFO(this->get_logger(), "Constructor End");
}
    
void azrael_driver::call_odom()
{
    // auto message = nav_msgs::msg::Odometry();
    current_time = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time-last_time).count() / 1e9;
    {
        std::unique_lock<std::mutex> lock1(v_wheels_mutex_);
        this->velx_odom = ( -1 * this->v_wheels_[0] + this->v_wheels_[1] - this->v_wheels_[2] + this->v_wheels_[3] ) * (radius * 0.25);
        this->vely_odom = (      this->v_wheels_[0] + this->v_wheels_[1] + this->v_wheels_[2] + this->v_wheels_[3] ) * (radius * 0.25);
        this->velw_odom = (      this->v_wheels_[0] - this->v_wheels_[1] - this->v_wheels_[2] + this->v_wheels_[3] ) * (radius / ( 4 * lxy));
    }


    this->posx_odom += (this->velx_odom * cos(this->posw_odom) - this->vely_odom * sin(this->posw_odom)) * dt;
    this->posy_odom += (this->velx_odom * sin(this->posw_odom) + this->vely_odom * cos(this->posw_odom)) * dt;
    this->posw_odom += this->velw_odom * dt;

    this->last_time = this->current_time;

    message_odom.header.stamp =  this->get_clock()->now();
    message_odom.child_frame_id  = "azrael/base_footprint";
    message_odom.header.frame_id = "azrael/odom";
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, this->posw_odom);

    message_odom.pose.pose.orientation.x = q.x();
    message_odom.pose.pose.orientation.y = q.y();
    message_odom.pose.pose.orientation.z = q.z();
    message_odom.pose.pose.orientation.w = q.w();

    message_odom.pose.pose.position.x = this->posx_odom;
    message_odom.pose.pose.position.y = this->posy_odom;

    message_odom.twist.twist.linear.x  = this->velx_odom;
    message_odom.twist.twist.linear.y  = this->vely_odom;
    message_odom.twist.twist.angular.z = this->velw_odom;

    odom_pub_->publish(message_odom);

    geometry_msgs::msg::TransformStamped t;
    
    t.header.stamp = this->get_clock()->now();
    t.child_frame_id = "azrael/base_footprint";
    t.header.frame_id = "azrael/odom";
    

    t.transform.translation.x = this->posx_odom;
    t.transform.translation.y = this->posy_odom;

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);


}

void azrael_driver::timer_udp_receive()
{
    // while(rclcpp::ok())
    // {
    //     len_addr_ = sizeof(cliaddr_);
    //     {
    //         std::unique_lock<std::mutex> lock1(v_wheels_mutex_);
    //         n_out_ = recvfrom(sockfd_, (void *)v_wheels_, sizeof(double)*4, MSG_WAITALL, ( struct sockaddr *) &cliaddr_,  &len_addr_);
    //     }
    // }
    std::unique_lock<std::mutex> lock1(v_wheels_mutex_);
    socket->receive_from(boost::asio::buffer(v_wheels_), local_endpoint);
}

void azrael_driver::timer_udp_send()
{

    // while(rclcpp::ok())
    // {
        {
            std::unique_lock<std::mutex> lock2(v_robot_mutex_);
            if((std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-last_cmd_).count() / 1e6) < 200)
            {
                v_robot_[0] = std::clamp(v_robot_[0], -1 * vx_max_, vx_max_);
                v_robot_[1] = std::clamp(v_robot_[1], -1 * vy_max_, vy_max_);
                v_robot_[2] = std::clamp(v_robot_[2], -1 * vw_max_, vw_max_);
            }
            else if((std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now()-last_cmd_).count() / 1e6) > 1000)
            {
                // RCLCPP_INFO_STREAM(this->get_logger(), "stopped\n");
                v_robot_[0] = 0.0;
                v_robot_[1] = 0.0;
                v_robot_[2] = 0.0;
            }
            else
            {
                // RCLCPP_INFO_STREAM(this->get_logger(), "Cmd too slow\n");
                v_robot_[0] = v_robot_[0]/1.05;
                v_robot_[1] = v_robot_[1]/1.05;
                v_robot_[2] = v_robot_[2]/1.05;
            }

            // sendto(sockfd_, (const void *)v_robot_, sizeof(double)*3, MSG_WAITALL, (const struct sockaddr *) &cliaddr_, len_addr_);
            boost::system::error_code err;
            std::cout << "Sending upd \n";
            boost::array<double, 3> send_buf  = {{ 0.1,0.1,0.1 }};

            auto sent = socket->send_to(boost::asio::buffer(send_buf), remote_endpoint, 0, err);
        }
        // std::this_thread::sleep_for(std::chrono::microseconds(20000));
    // }
}

void azrael_driver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_cmd_ = std::chrono::high_resolution_clock::now();
    std::unique_lock<std::mutex> lock3(v_robot_mutex_);
    this->v_robot_[0] = msg->linear.x;
    this->v_robot_[1] = msg->linear.y;
    this->v_robot_[2] = msg->angular.z;
    // this->v_robot_[0] = fx.filter(msg->linear.x);
    // this->v_robot_[1] = fy.filter(msg->linear.y);
    // this->v_robot_[2] = fw.filter(msg->angular.z);
    // RCLCPP_INFO_STREAM(this->get_logger(), "cme " << this->v_robot_[0] << " " << this->v_robot_[1] << " " << this->v_robot_[2] << "\n");
}




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::cout << "Init\n" << std::flush;

//   rclcpp::executors::MultiThreadedExecutor exec ;
rclcpp::executors::StaticSingleThreadedExecutor exec ;
  rclcpp::Node::SharedPtr node1 = std::make_shared<azrael_driver>();

  exec.add_node(node1);
  exec.spin();

//   rclcpp::spin(std::make_shared<azrael_driver>());
  rclcpp::shutdown();
  return 0;
}