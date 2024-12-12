#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/imu.hpp"  // IMU 메시지 타입 헤더
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
      cmd_joint3_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_3/command", 10);      
      x_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_x/command", 10);            
      y_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_y/command", 10);                        
      z_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_z/command", 10);            
      roll_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_r/command", 10);            
      pitch_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_p/command", 10);                  
      yaw_axis_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_yaw/command", 10);            
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/manipulator/joint_states", 10,
            std::bind(&ign_pubsub::joint_state_subsciber_callback, this, std::placeholders::_1));
        link_yaw_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/manipulator/imu", 10,
            std::bind(&ign_pubsub::imu_subscriber_callback, this, std::placeholders::_1));
        position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/manipulator/pose_info", 10,
            std::bind(&ign_pubsub::global_pose_callback, this, std::placeholders::_1));            
		
      timer_ = this->create_wall_timer(
      10ms, std::bind(&ign_pubsub::timer_callback, this));
    }

  private:
	    void timer_callback()
	    {	//main loop, 100Hz
		set_command();
		PID_controller();		
		data_publish();     	    
    }

  double saturation(double max, double min, double value){
  if (value > max) value = max;
  else if (value < min) value = min;
  return value;
  }

	void PID_controller()
	{
  x_error_integral += (x_axis_cmd - x_axis_meas);
  y_error_integral += (y_axis_cmd - y_axis_meas);
  z_error_integral += (z_axis_cmd - z_axis_meas);

  // x_axis_force_cmd  = 10* (x_axis_cmd - x_axis_meas) + 0. * x_error_integral;
  // y_axis_force_cmd  = 10* (y_axis_cmd - y_axis_meas) + 0. * y_error_integral;
  z_axis_force_cmd  = 40 * (z_axis_cmd - z_axis_meas) + 0. * z_error_integral;
	//error
	roll_tau_cmd = 100 * (roll_axis_cmd - roll_meas) -10 * roll_vel_meas;
	pitch_tau_cmd = 100 * (pitch_axis_cmd - pitch_meas) - 10 * pitch_vel_meas;
	yaw_tau_cmd = 100 * (yaw_axis_cmd - yaw_meas) - 10 * yaw_vel_meas;

 RCLCPP_INFO(this->get_logger(), "Fx: '%lf' Fy: '%lf' Fz: '%lf'", x_axis_force_cmd, y_axis_force_cmd, z_axis_force_cmd);
 RCLCPP_INFO(this->get_logger(), "Fr: '%lf' Fp: '%lf' Fyaw: '%lf'", roll_tau_cmd, pitch_tau_cmd, yaw_tau_cmd);


	}



void set_command()
{
    // 시간 증가 (100Hz 기준, 매 호출마다 0.01초 증가)
    time_cnt++;
    double time = time_cnt / 100.0;



    // Phase 1: Move in the z-direction (5초 ~ 10초, 선형적으로 5m 상승)
    if (time >= 5.0 && time < 10.0)
    {
        double t_normalized = (time - 5.0) / (10.0 - 5.0); // Normalized time (0 to 1)
        z_axis_cmd = t_normalized * 5.0;                   // Linear profile (0 to 5m)
    }

    // Phase 2: Rotate in roll direction (10초 ~ 15초, 선형적으로 10도 회전)
    else if (time >= 10.0 && time < 15.0)
    {
        z_axis_cmd = 5.0;                                 // 유지된 z 위치
        double t_normalized = (time - 10.0) / (15.0 - 10.0); // Normalized time (0 to 1)
        yaw_axis_cmd = t_normalized * - 60.0 * (M_PI / 180.0); // Linear profile (0 to 10 deg in radians)
    }

    // Phase 3: Rotate in yaw direction (15초 ~ 20초, 선형적으로 45도 회전)
    else if (time >= 15.0 && time < 20.0)
    {
        z_axis_cmd = 5.0;                                 // 유지된 z 위치
        yaw_axis_cmd = - 60.0 * (M_PI / 180.0);            // 유지된 roll 위치
        double t_normalized = (time - 15.0) / (20.0 - 15.0); // Normalized time (0 to 1)
        roll_axis_cmd = t_normalized * 45.0 * (M_PI / 180.0); // Linear profile (0 to 45 deg in radians)
    }

x_axis_force_cmd = 5;
}





	    void data_publish()
	    {	// publish!!
	      joint_1_cmd_msg.data = joint_1_cmd;
	      joint_2_cmd_msg.data = joint_2_cmd;
	      joint_3_cmd_msg.data = joint_3_cmd;	      
	      roll_axis_msg.data = roll_tau_cmd;	      
	      pitch_axis_msg.data = pitch_tau_cmd;	      
	      yaw_axis_msg.data = yaw_tau_cmd;
	      x_axis_msg.data = x_axis_force_cmd;	      
	      y_axis_msg.data = y_axis_force_cmd;
	      z_axis_msg.data = z_axis_force_cmd;
	    //  RCLCPP_INFO(this->get_logger(), "joint_1_meas: '%lf' joint_2_meas: '%lf'", joint_1_meas_angle, joint_2_meas_angle);

	      cmd_joint1_publisher_->publish(joint_1_cmd_msg);
	      cmd_joint2_publisher_->publish(joint_2_cmd_msg);       
	      cmd_joint3_publisher_->publish(joint_3_cmd_msg);       	      
	      x_axis_publisher_->publish(x_axis_msg);   
	      y_axis_publisher_->publish(y_axis_msg);   	      	      
	      z_axis_publisher_->publish(z_axis_msg);   
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
 
     void imu_subscriber_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
    // RCLCPP_INFO(this->get_logger(), "Received IMU data:");

    // 쿼터니언 값 가져오기
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;

    // Roll, Pitch, Yaw 계산
    roll_meas = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    pitch_meas = std::asin(2.0 * (qw * qy - qz * qx));
    yaw_meas = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

   roll_vel_meas = msg->angular_velocity.x;
   pitch_vel_meas = msg->angular_velocity.y;
   yaw_vel_meas = msg->angular_velocity.z;

    // RPY 출력
  //  RCLCPP_INFO(this->get_logger(), "Orientation (RPY) - Roll: %.6f, Pitch: %.6f, Yaw: %.6f",
  //             roll_meas, pitch_meas, yaw_meas);

    // 각속도와 선가속도 출력
  //  RCLCPP_INFO(this->get_logger(), "Angular Velocity - x: %.6f, y: %.6f, z: %.6f",
  //              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  //  RCLCPP_INFO(this->get_logger(), "Linear Acceleration - x: %.6f, y: %.6f, z: %.6f",
  //              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    }
	    
	    
    void global_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received PoseArray message with %ld poses", msg->poses.size());

        // link_yaw의 id는 7로 고정
        const int link_yaw_id = 7;

        if (link_yaw_id < msg->poses.size())
        {
            const auto &pose = msg->poses[link_yaw_id];
     //       RCLCPP_INFO(this->get_logger(), "link_yaw Pose:");
     //       RCLCPP_INFO(this->get_logger(), "Position - x: %.6f, y: %.6f, z: %.6f",
     //                   pose.position.x, pose.position.y, pose.position.z);
     //       RCLCPP_INFO(this->get_logger(), "Orientation - x: %.6f, y: %.6f, z: %.6f, w: %.6f",
     //                   pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	x_axis_meas = pose.position.x;
	y_axis_meas = pose.position.y;
	z_axis_meas = pose.position.z;                        
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "link_yaw id (17) is out of bounds in PoseArray.");
        }
    }
	    
	    

 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint1_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint2_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint3_publisher_;    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr x_axis_publisher_;        
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_axis_publisher_;    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr y_axis_publisher_;    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_axis_publisher_;     
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_axis_publisher_;           
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_axis_publisher_;    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr link_yaw_imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr position_subscriber_;
    size_t count_;
    std_msgs::msg::Float64 joint_1_cmd_msg;
    std_msgs::msg::Float64 joint_2_cmd_msg;
    std_msgs::msg::Float64 joint_3_cmd_msg;    
    std_msgs::msg::Float64 x_axis_msg;      
    std_msgs::msg::Float64 y_axis_msg;  
    std_msgs::msg::Float64 z_axis_msg;    
    std_msgs::msg::Float64 roll_axis_msg;        
    std_msgs::msg::Float64 pitch_axis_msg;                      
    std_msgs::msg::Float64 yaw_axis_msg;        
    double joint_1_cmd;
    double joint_2_cmd;
    double joint_3_cmd;    
    double x_axis_force_cmd;
    double y_axis_force_cmd;    
    double z_axis_force_cmd;
    double roll_tau_cmd;
    double pitch_tau_cmd;
    double yaw_tau_cmd;

    double x_axis_cmd;
    double y_axis_cmd;    
    double z_axis_cmd;
    double roll_axis_cmd;
    double pitch_axis_cmd;    
    double yaw_axis_cmd;
    

    double x_axis_meas;
    double y_axis_meas;
    double z_axis_meas;
    double joint_1_meas_angle;
    double joint_2_meas_angle;
    double joint_1_meas_torque;
    double joint_2_meas_torque;
    double roll_meas;
    double pitch_meas;
    double yaw_meas;
    double roll_vel_meas;
    double pitch_vel_meas;
    double yaw_vel_meas;
    
    double x_error_integral;
    double y_error_integral;
    double z_error_integral;
    
    double joint_1_temp;
    double joint_2_temp;
    double joint_3_temp;


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
