#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
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
      x_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/x_axis/command", 10);                  
      z_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/z_axis/command", 10);            
      y_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/y_axis/command", 10);            
      roll_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/roll_axis/command", 10);            
      pitch_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/pitch_axis/command", 10);                  
      yaw_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_axis/command", 10);            
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/manipulator/joint_states", 10,
            std::bind(&ign_pubsub::joint_state_subsciber_callback, this, std::placeholders::_1));
        body_state_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/dynamic_pose/info", 10,
            std::bind(&ign_pubsub::body_state_subscriber_callback, this, std::placeholders::_1));

		
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
    time = time_cnt / 100.0; // 100Hz 기준 시간 계산

    if (time < 5.0)
    {
        // 0~5초: 초기 대기 상태
        z_axis_cmd = 0.0;
        y_axis_cmd = 0.0;
        yaw_axis_cmd = 0.0;
        joint_1_cmd = 0.0;
        joint_2_cmd = 0.0;
    }
    else if (time < 8.0)
    {
        // 5~8초: Z축 3m 선형 상승
        z_axis_cmd = (time - 5.0) / 3.0 * 3.0; // 3초 동안 0 -> 3m
        y_axis_cmd = 0.0;
        yaw_axis_cmd = 0.0;
        joint_1_cmd = 0.0;
        joint_2_cmd = 0.0;
    }
    else if (time < 10.0)
    {
        // 8~10초: 정지 상태
        z_axis_cmd = 3.0; // 유지
        y_axis_cmd = 0.0; // 유지
        yaw_axis_cmd = 0.0; // 유지
        joint_1_cmd = 0.0;
        joint_2_cmd = 0.0;
    }
    else if (time < 15.0)
    {
        // 10~15초: Y축 +3m 선형 이동
        z_axis_cmd = 3.0; // 유지
        y_axis_cmd = (time - 10.0) / 5.0 * 3.0; // 5초 동안 0 -> 3m
        yaw_axis_cmd = 0.0; // 유지
        joint_1_cmd = 0.0;
        joint_2_cmd = 0.0;
    }
    else if (time < 20.0)
    {
        // 15~20초: Y축 0m로 복귀
        z_axis_cmd = 3.0; // 유지
        y_axis_cmd = 3.0 - (time - 15.0) / 5.0 * 3.0; // 5초 동안 3m -> 0m
        yaw_axis_cmd = 0.0; // 유지
        joint_1_cmd = 0.0;
        joint_2_cmd = 0.0;
    }
    else if (time < 25.0)
    {
        // 20~25초: 대기
        z_axis_cmd = 3.0; // 유지
        y_axis_cmd = 0.0; // 유지
        yaw_axis_cmd = 0.0; // 유지
        joint_1_cmd = 0.0;
        joint_2_cmd = 0.0;
    }
    else if (time < 30.0)
    {
        // 25~30초: Yaw 방향 45도(π/4) 회전
        z_axis_cmd = 3.0; // 유지
        y_axis_cmd = 0.0; // 유지
        yaw_axis_cmd = (time - 25.0) / 5.0 * (M_PI / 4); // 5초 동안 0 -> π/4
        joint_1_cmd = 0.0;
        joint_2_cmd = 0.0;
    }
    else if (time < 45.0)
    {
        // 30~45초: Joint 1, 2 Sine 파동
        z_axis_cmd = 3.0; // 유지
        y_axis_cmd = 0.0; // 유지
        yaw_axis_cmd = M_PI / 4; // 유지
        joint_1_cmd = 1.57 * std::sin(2 * M_PI * (time - 30.0) / 2.0); // Joint 1: 주기 2초, 진폭 1.57
        joint_2_cmd = 1.57 * std::sin(2 * M_PI * (time - 30.0) / 3.0); // Joint 2: 주기 3초, 진폭 1.57
    }
    else if (time < 50.0)
    {
        // 45~50초: Z축 착륙
        z_axis_cmd = 3.0 - (time - 45.0) / 5.0 * 3.0; // 5초 동안 3m -> 0m
        y_axis_cmd = 0.0; // 유지
        yaw_axis_cmd = M_PI / 4; // 유지
        joint_1_cmd = 0.0; // 정지
        joint_2_cmd = 0.0; // 정지
    }
    else
    {
        // 50초 이후: 모든 축 및 조인트 정지
        z_axis_cmd = 0.0;
        y_axis_cmd = 0.0;
        yaw_axis_cmd = M_PI / 4; // 유지
        joint_1_cmd = 0.0;
        joint_2_cmd = 0.0;
    }




		//TODO!!!
	    }
	    void data_publish()
	    {	// publish!!
	      joint_1_cmd_msg.data = joint_1_cmd;
	      joint_2_cmd_msg.data = joint_2_cmd;
	      roll_axis_msg.data = 0;	      
	      pitch_axis_msg.data = 0;	      
	      x_axis_msg.data = 0;	      
	      z_axis_msg.data = z_axis_cmd;
	      y_axis_msg.data = y_axis_cmd;
	      yaw_axis_msg.data = yaw_axis_cmd;
//	      RCLCPP_INFO(this->get_logger(), "joint_1_cmd: '%lf' joint_2_cmd: '%lf'", joint_1_cmd_msg.data, joint_2_cmd_msg.data);
//	      RCLCPP_INFO(this->get_logger(), "joint_1_meas: '%lf' joint_2_meas: '%lf'", joint_1_meas_angle, joint_2_meas_angle);
//	      RCLCPP_INFO(this->get_logger(), "joint_1_error: '%lf' joint_2_error: '%lf'", joint_1_cmd_msg.data - joint_1_meas_angle, joint_2_cmd_msg.data - joint_2_meas_angle);	      

	      cmd_joint1_publisher_->publish(joint_1_cmd_msg);
	      cmd_joint2_publisher_->publish(joint_2_cmd_msg);       
	      x_axis_publisher_->publish(x_axis_msg);   	      
	      z_axis_publisher_->publish(z_axis_msg);   
	      y_axis_publisher_->publish(y_axis_msg);   
	      roll_axis_publisher_->publish(roll_axis_msg);   	
	      pitch_axis_publisher_->publish(pitch_axis_msg);   	            
	      yaw_axis_publisher_->publish(yaw_axis_msg);   
	    }
 
 	    void joint_state_subsciber_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
	    {
	    joint_1_meas_angle = msg->position[7];
	    joint_2_meas_angle = msg->position[8];
	    joint_1_meas_torque = msg->effort[7];
	    joint_2_meas_torque = msg->effort[8];	    
	    }
 
	     void body_state_subscriber_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
	    {
		for (size_t i = 0; i < msg->poses.size(); ++i)
		{
		    // Assuming pose name can be identified (adjust according to your custom message type)
		    // Example assumes a pose name string field (you may need a custom message)
		    std::string pose_name = ""; // Replace with the actual way to access pose name if available

		    if (pose_name == "drone_body")
		    {
		        const auto &pose = msg->poses[i];

		        // Extract position and orientation
		        const auto &position = pose.position;
		        const auto &orientation = pose.orientation;

		        RCLCPP_INFO(this->get_logger(), "Drone Body Pose:");
		        RCLCPP_INFO(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f",
		                    position.x, position.y, position.z);
		        RCLCPP_INFO(this->get_logger(), "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
		                    orientation.x, orientation.y, orientation.z, orientation.w);

		        break; // Exit loop after finding the desired link
		    }
		}
	    }

 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint1_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint2_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr x_axis_publisher_;        
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_axis_publisher_;    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr y_axis_publisher_;    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_axis_publisher_;     
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_axis_publisher_;           
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_axis_publisher_;    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr body_state_subscriber_;

    size_t count_;
    std_msgs::msg::Float64 joint_1_cmd_msg;
    std_msgs::msg::Float64 joint_2_cmd_msg;
    std_msgs::msg::Float64 x_axis_msg;      
    std_msgs::msg::Float64 z_axis_msg;    
    std_msgs::msg::Float64 y_axis_msg;  
    std_msgs::msg::Float64 roll_axis_msg;        
    std_msgs::msg::Float64 pitch_axis_msg;                      
    std_msgs::msg::Float64 yaw_axis_msg;        
    double joint_1_cmd;
    double joint_2_cmd;
    double x_axis_cmd;
    double y_axis_cmd;    
    double z_axis_cmd;
    double yaw_axis_cmd;
    

    
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
