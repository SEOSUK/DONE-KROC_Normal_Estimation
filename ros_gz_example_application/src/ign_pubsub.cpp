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
#include <ros_gz_interfaces/msg/entity_wrench.hpp>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

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
      cmd_joint3_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_3/command", 10);            wrench_publisher_ = this->create_publisher<ros_gz_interfaces::msg::EntityWrench>("/link_drone/wrench", 10);
      
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




    body_xyz_P.diagonal() << 4.0, 4.0, 3.0;
    body_xyz_I.diagonal() << 0.001, 0.001, 0.001;
    body_rpy_P.diagonal() << 0.1, 0.1, 0.002;
    body_rpy_D.diagonal() << 0.000, 0.000, 0.000;
      wrench_msg.entity.name = "link_drone"; // 링크 이름
      wrench_msg.entity.type = ros_gz_interfaces::msg::Entity::LINK; // 엔티티 유형: LINK

    }

  private:
	    void timer_callback()
	    {	//main loop, 100Hz
		set_traj();
		// PID_controller();		
		data_publish();     	    
    }


  double saturation(double max, double min, double value){
  if (value > max) value = max;
  else if (value < min) value = min;
  return value;
  }

	void PID_controller()
	{

  global_xyz_error = global_xyz_cmd - global_xyz_meas;
  global_xyz_error_integral += global_xyz_error;  

//
  body_xyz_error = Rot_G2D(global_xyz_error, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
  body_xyz_error_integral = Rot_G2D(global_xyz_error_integral, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);

  body_force_cmd = body_xyz_P * body_xyz_error + body_xyz_I * body_xyz_error_integral;

  global_force_cmd = Rot_D2G(body_force_cmd, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);

  


  body_rpy_cmd = Rot_G2D(global_rpy_cmd, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);

  body_rpy_error = body_rpy_cmd - body_rpy_meas;
  body_rpy_error_d = body_rpy_vel_meas;

  body_torque_cmd = body_rpy_P * body_rpy_error + body_rpy_D * body_rpy_error_d;

  global_torque_cmd = Rot_D2G(body_torque_cmd, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);


	}



void set_traj()
{
    // 시간 증가 (100Hz 기준, 매 호출마다 0.01초 증가)
    time_cnt++;
    double time = time_cnt / 100.0;


RCLCPP_INFO(this->get_logger(), "Fx: '%lf' Fy: '%lf' Fz: '%lf'", global_xyz_meas[0], global_xyz_meas[1], global_xyz_meas[2]);
RCLCPP_INFO(this->get_logger(), "Fr: '%lf' Fp: '%lf' Fyaw: '%lf'", body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);




if (global_xyz_cmd[2] < 5) global_xyz_cmd[2] = time / 2;

}

Eigen::Matrix3d get_rotation_matrix(double roll, double pitch, double yaw) {
    // Z-axis (Yaw) rotation
    Eigen::Matrix3d Rz;
    Rz << std::cos(yaw), -std::sin(yaw), 0,
          std::sin(yaw),  std::cos(yaw), 0,
          0, 0, 1;

    // Y-axis (Pitch) rotation
    Eigen::Matrix3d Ry;
    Ry << std::cos(pitch), 0, std::sin(pitch),
          0, 1, 0,
         -std::sin(pitch), 0, std::cos(pitch);

    // X-axis (Roll) rotation
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, std::cos(roll), -std::sin(roll),
          0, std::sin(roll),  std::cos(roll);

    // Combined rotation: R = Rx * Ry * Rz
    return Rx * Ry * Rz;
}

Eigen::Vector3d Rot_G2D(const Eigen::Vector3d& global_vector, double roll, double pitch, double yaw) {
    Eigen::Matrix3d R = get_rotation_matrix(roll, pitch, yaw);
    return R * global_vector;
}

Eigen::Vector3d Rot_D2G(const Eigen::Vector3d& body_vector, double roll, double pitch, double yaw) {
    Eigen::Matrix3d R = get_rotation_matrix(roll, pitch, yaw);
    return R.transpose() * body_vector; // Transpose for inverse
}



	    void data_publish()
	    {	// publish!!
	      joint_1_cmd_msg.data = joint_angle_cmd[0];
	      joint_2_cmd_msg.data = joint_angle_cmd[1];
	      joint_3_cmd_msg.data = joint_angle_cmd[2];	      
        wrench_msg.wrench.force.x = global_force_cmd[0];
        wrench_msg.wrench.force.y = global_force_cmd[1];
        wrench_msg.wrench.force.z = global_force_cmd[2];  // 500 N 힘 적용
        wrench_msg.wrench.torque.x = global_torque_cmd[0];
        wrench_msg.wrench.torque.y = global_torque_cmd[1];
        wrench_msg.wrench.torque.z = global_torque_cmd[2];
	    //  RCLCPP_INFO(this->get_logger(), "joint_1_meas: '%lf' joint_2_meas: '%lf'", joint_1_meas_angle, joint_2_meas_angle);

	      cmd_joint1_publisher_->publish(joint_1_cmd_msg);
	      cmd_joint2_publisher_->publish(joint_2_cmd_msg);       
	      cmd_joint3_publisher_->publish(joint_3_cmd_msg);       	      
        wrench_publisher_->publish(wrench_msg);

	    }
 
 	    void joint_state_subsciber_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
	    {
	    joint_angle_meas[0] = msg->position[0];
	    joint_angle_meas[1] = msg->position[1];
	    joint_angle_meas[2] = msg->position[2];
	    joint_effort_meas[0] = msg->effort[0];
	    joint_effort_meas[1] = msg->effort[1];
	    joint_effort_meas[2] = msg->effort[2];
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
    body_rpy_meas[0] = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    body_rpy_meas[1] = std::asin(2.0 * (qw * qy - qz * qx));
    body_rpy_meas[2] = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

   body_rpy_vel_meas[0] = msg->angular_velocity.x;
   body_rpy_vel_meas[1] = msg->angular_velocity.y;
   body_rpy_vel_meas[2] = msg->angular_velocity.z;

    // RPY 출력
    // RCLCPP_INFO(this->get_logger(), "Orientation_IMU (RPY) - Roll: %.6f, Pitch: %.6f, Yaw: %.6f",
              //  body_rpy_vel_meas[0], body_rpy_vel_meas[1], body_rpy_vel_meas[2]);

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
        const int link_yaw_id = 1;

        if (link_yaw_id < msg->poses.size())
        {
            const auto &pose = msg->poses[link_yaw_id];
          //  RCLCPP_INFO(this->get_logger(), "link_yaw Pose:");
          //  RCLCPP_INFO(this->get_logger(), "Position - x: %.6f, y: %.6f, z: %.6f",
                      //  pose.position.x, pose.position.y, pose.position.z);
          //  RCLCPP_INFO(this->get_logger(), "Orientation - x: %.6f, y: %.6f, z: %.6f, w: %.6f",
                      //  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
          global_xyz_meas[0] = pose.position.x;
          global_xyz_meas[1] = pose.position.y;
          global_xyz_meas[2] = pose.position.z;                        
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "link_yaw id (17) is out of bounds in PoseArray.");
        }
    }
	    
	    

 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr wrench_timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint1_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint2_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint3_publisher_;    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr x_axis_publisher_;        
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_axis_publisher_;    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr y_axis_publisher_;    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_axis_publisher_;     
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_axis_publisher_;           
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_axis_publisher_;    
    rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr wrench_publisher_;    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr link_yaw_imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr position_subscriber_;
    size_t count_;
    std_msgs::msg::Float64 joint_1_cmd_msg;
    std_msgs::msg::Float64 joint_2_cmd_msg;
    std_msgs::msg::Float64 joint_3_cmd_msg;    
    //TODO:: 아래 세 줄 정의 제대로 하기
    ros_gz_interfaces::msg::EntityWrench wrench_msg;
    // msg.entity.name = "link_drone";
    // msg.entity.type = ros_gz_interfaces::msg::Entity::LINK;


  Eigen::Vector3d global_xyz_meas;
  Eigen::Vector3d global_xyz_cmd;
  Eigen::Vector3d global_xyz_error;
  Eigen::Vector3d global_xyz_error_integral;
  Eigen::Vector3d global_xyz_error_d;
  Eigen::Vector3d body_xyz_error;
  Eigen::Vector3d body_xyz_error_integral;
  Eigen::Vector3d body_force_cmd;
  Eigen::Vector3d global_force_cmd;  
  Eigen::Matrix3d body_xyz_P = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d body_xyz_I = Eigen::Matrix3d::Zero();



  Eigen::Vector3d global_rpy_cmd;
  Eigen::Vector3d body_rpy_meas;
  Eigen::Vector3d body_rpy_cmd;
  Eigen::Vector3d body_rpy_error;
  Eigen::Vector3d body_rpy_error_integral;
  Eigen::Vector3d body_rpy_vel_meas;
  Eigen::Vector3d body_rpy_error_d;  
  Eigen::Vector3d body_torque_cmd;  
  Eigen::Vector3d global_torque_cmd;  
  Eigen::Matrix3d body_rpy_P = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d body_rpy_D = Eigen::Matrix3d::Zero();



  Eigen::Vector3d joint_angle_cmd;
  Eigen::Vector3d joint_angle_meas;
  Eigen::Vector3d joint_effort_meas;

    

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
