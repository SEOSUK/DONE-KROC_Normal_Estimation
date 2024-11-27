#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ign_pubsub : public rclcpp::Node
{
  public:
    ign_pubsub()
    : Node("ign_pubsub"), count_(0)
    {
      cmd_joint1_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_1/command", 10);
      cmd_joint2_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_2/command", 10);      
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/manipulator/joint_states", 10,
            std::bind(&ign_pubsub::manipulator_subsciber_callback, this, std::placeholders::_1));
		
      timer_ = this->create_wall_timer(
      10ms, std::bind(&ign_pubsub::timer_callback, this));
    }

  private:
	    void timer_callback()
	    {	//main loop, 100Hz
		set_command();
		data_publish();     
	    }
	    void set_command()
	    {	//command generator
		time_cnt++;
	    	time = time_cnt / 100;
	    	sine = 1.4 * std::sin(2 * M_PI * 0.1 * time);
	    	joint_1_cmd = sine;
		joint_2_cmd = sine;    
	    }
	    void data_publish()
	    {	// publish!!
	      joint_1_cmd_msg.data = joint_1_cmd;
	      joint_2_cmd_msg.data = joint_2_cmd;      
	      RCLCPP_INFO(this->get_logger(), "joint_1_cmd: '%lf' joint_2_cmd: '%lf'", joint_1_cmd_msg.data, joint_2_cmd_msg.data);
	      RCLCPP_INFO(this->get_logger(), "joint_1_meas: '%lf' joint_2_meas: '%lf'", joint_1_meas_angle, joint_2_meas_angle);
	      RCLCPP_INFO(this->get_logger(), "joint_1_error: '%lf' joint_2_error: '%lf'", joint_1_cmd_msg.data - joint_1_meas_angle, joint_2_cmd_msg.data - joint_2_meas_angle);	      

	      cmd_joint1_publisher_->publish(joint_1_cmd_msg);
	      cmd_joint2_publisher_->publish(joint_2_cmd_msg);          
	    }
 
 	    void manipulator_subsciber_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
	    {
	    joint_1_meas_angle = msg->position[0];
	    joint_2_meas_angle = msg->position[1];
	    joint_1_meas_torque = msg->effort[0];
	    joint_2_meas_torque = msg->effort[1];	    
	    }
 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint1_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint2_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;    
    size_t count_;
    std_msgs::msg::Float64 joint_1_cmd_msg;
    std_msgs::msg::Float64 joint_2_cmd_msg;
    double joint_1_cmd;
    double joint_2_cmd;
    double joint_1_meas_angle;
    double joint_2_meas_angle;
    double joint_1_meas_torque;
    double joint_2_meas_torque;
    double time;
    double time_cnt;
    double sine;
};

int main(int argc, char * argv[])
{
//ros2 topic pub /joint_1/command std_msgs/msg/Float64 "{data: 1.0}"
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ign_pubsub>());
  rclcpp::shutdown();
  return 0;
}
