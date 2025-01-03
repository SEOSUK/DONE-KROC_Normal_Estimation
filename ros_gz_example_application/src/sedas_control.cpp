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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2::Quaternion 추가
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include "sedas_rot.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class sedas_control : public rclcpp::Node
{
  public:
    sedas_control()
      : Node("sedas_control"), tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)), count_(0)
    {      
      // QoS 설정
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


      command_publsiher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/command_input_U", qos_settings);

      state_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/state_vector", qos_settings,
          std::bind(&sedas_control::state_subscriber_callback, this, std::placeholders::_1));
      body_rpy_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/body_rpy", qos_settings,
          std::bind(&sedas_control::body_rpy_subscriber_callback, this, std::placeholders::_1));



      timer_ = this->create_wall_timer(
      5ms, std::bind(&sedas_control::timer_callback, this));

    body_xyz_P.diagonal() << 8, 8, 8.0;
    body_xyz_I.diagonal() << 0.000, 0.000, 0.0;
    body_xyz_D.diagonal() << 1, 1, 1;
    body_rpy_P.diagonal() << 0.3, 0.3, 0.03;
    body_rpy_D.diagonal() << 0.06, 0.06, 0.003;

    }

  private:
	    void timer_callback()
    {	//main loop, 100Hz
    // 현재 시간 계산
      set_traj();
      PID_controller();		

      data_publish();     	    
    }



  double saturation(double max, double min, double value){
  if (value > max) value = max;
  else if (value < min) value = min;
  return value;
  }


void set_traj()
{
    // 시간 증가 (100Hz 기준, 매 호출마다 0.01초 증가)
    time_cnt++;
    double time = time_cnt * delta_time - 5;

    // Joint 명령의 누적 각도를 저장할 변수 (연속성을 보장하기 위해)
    static double joint1_accum_angle = 0.0;
    static double joint2_accum_angle = 0.0;
    static double joint3_accum_angle = 0.0;

    // 명령 생성
    if (time <= 5.0) {
        // 초기 상태, 명령 없음
        global_xyz_cmd.setZero();
        joint_angle_cmd.setZero();
        global_rpy_cmd.setZero();
    } else if (time > 5.0 && time <= 10.0) {
        // 5초부터 10초까지 Z축으로 2미터 상승
        global_xyz_cmd[2] = 2 * ((time - 5.0) / 5.0);
    } else if (time > 10.0 && time <= 15.0) {
        // 10초부터 15초까지 고정
        global_xyz_cmd[2] = 2.0;
    } else if (time > 15.0 && time <= 20.0) {
        // 15초부터 20초까지 X 방향으로 1미터 이동 및 Joint 명령 생성
        global_xyz_cmd[0] = 1 * ((time - 15.0) / 5.0);
        global_xyz_cmd[2] = 2.0; // Z축 고정
        global_rpy_cmd[2] = (30 * M_PI / 180) * ((time - 15.0) / 5.0);

        joint1_accum_angle += delta_time * 45 * M_PI / 180 * std::cos(2 * M_PI * (time - 15.0) / 5);
        joint2_accum_angle += delta_time * 30 * M_PI / 180 * std::cos(2 * M_PI * (time - 15.0) / 4);
        joint3_accum_angle += delta_time * 25 * M_PI / 180 * std::cos(2 * M_PI * (time - 15.0) / 3);
    } else if (time > 20.0 && time <= 25.0) {
        // 20초부터 25초까지 Yaw 방향으로 30도 회전
        global_xyz_cmd[0] = 1.0; // X축 고정
        global_xyz_cmd[2] = 2.0; // Z축 고정
        global_rpy_cmd[2] = (30 * M_PI / 180) * (1 - (time - 20.0) / 5.0);

        joint1_accum_angle += delta_time * 45 * M_PI / 180 * std::cos(2 * M_PI * (time - 20.0) / 5);
        joint2_accum_angle += delta_time * 30 * M_PI / 180 * std::cos(2 * M_PI * (time - 20.0) / 4);
        joint3_accum_angle += delta_time * 25 * M_PI / 180 * std::cos(2 * M_PI * (time - 20.0) / 3);
    } else if (time > 25.0 && time <= 30.0) {
        // 25초부터 30초까지 X 방향으로 원점 복귀
        global_xyz_cmd[0] = 1.0 - 1.0 * ((time - 25.0) / 5.0);
        global_xyz_cmd[2] = 2.0; // Z축 고정

        joint1_accum_angle += delta_time * 45 * M_PI / 180 * std::cos(2 * M_PI * (time - 25.0) / 5);
        joint2_accum_angle += delta_time * 30 * M_PI / 180 * std::cos(2 * M_PI * (time - 25.0) / 4);
        joint3_accum_angle += delta_time * 25 * M_PI / 180 * std::cos(2 * M_PI * (time - 25.0) / 3);
    } else if (time > 30.0 && time <= 40.0) {
        // 30초부터 40초까지 Joint1, Joint2, Joint3 명령을 연속적으로 sin 파로 줌
        double t = time - 30.0;
        joint1_accum_angle += delta_time * 45 * M_PI / 180 * std::cos(2 * M_PI * t / 5);
        joint2_accum_angle += delta_time * 30 * M_PI / 180 * std::cos(2 * M_PI * t / 4);
        joint3_accum_angle += delta_time * 25 * M_PI / 180 * std::cos(2 * M_PI * t / 3);
    }

    joint_angle_cmd[0] = joint1_accum_angle;
    joint_angle_cmd[1] = joint2_accum_angle;
    joint_angle_cmd[2] = joint3_accum_angle;

    // 디버깅 정보 출력
    // RCLCPP_INFO(this->get_logger(), "Xdes: '%lf', Ydes: '%lf', Zdes: '%lf'", global_xyz_cmd[0], global_xyz_cmd[1], global_xyz_cmd[2]);
    // RCLCPP_INFO(this->get_logger(), "YawCmd: '%lf'", global_rpy_cmd[2]);
    // RCLCPP_INFO(this->get_logger(), "JointCmd: q1='%lf', q2='%lf', q3='%lf'", joint_angle_cmd[0], joint_angle_cmd[1], joint_angle_cmd[2]);
}

void PID_controller()
{

  global_xyz_error = global_xyz_cmd - global_xyz_meas;
  body_xyz_error = Rot_G2D(global_xyz_error, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
  body_xyz_error_integral += body_xyz_error * delta_time;
  body_xyz_error_d = (body_xyz_error - prev_body_xyz_error) / delta_time;
  prev_body_xyz_error = body_xyz_error;


  body_force_cmd = body_xyz_P * body_xyz_error + body_xyz_I * body_xyz_error_integral + body_xyz_D * body_xyz_error_d;

  global_force_cmd = Rot_D2G(body_force_cmd, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);


  body_rpy_cmd = Rot_G2D(global_rpy_cmd, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);

  body_rpy_error = body_rpy_cmd - body_rpy_meas;
  // Wrapping to [-π, π] range
  body_rpy_error[2] = std::atan2(std::sin(body_rpy_error[2]), std::cos(body_rpy_error[2]));
  body_rpy_error_integral += body_rpy_error * delta_time;
  body_rpy_error_d = - body_rpy_vel_meas;

  body_torque_cmd = body_rpy_P * body_rpy_error + body_rpy_D * body_rpy_error_d;


  global_torque_cmd = Rot_D2G(body_torque_cmd, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);

      RCLCPP_INFO(this->get_logger(), "meas xyz - x: %.6f, y: %.6f, z: %.6f",
                  global_torque_cmd[0], global_torque_cmd[1], global_torque_cmd[2]);


}



void data_publish()
{	// publish!!
  std_msgs::msg::Float64MultiArray command_input_U_msg;

command_input_U_msg.data.push_back(global_force_cmd[0]);
command_input_U_msg.data.push_back(global_force_cmd[1]);
command_input_U_msg.data.push_back(global_force_cmd[2]);
command_input_U_msg.data.push_back(global_torque_cmd[0]);
command_input_U_msg.data.push_back(global_torque_cmd[1]);
command_input_U_msg.data.push_back(global_torque_cmd[2]);
command_input_U_msg.data.push_back(joint_angle_cmd[0]);
command_input_U_msg.data.push_back(joint_angle_cmd[1]);
command_input_U_msg.data.push_back(joint_angle_cmd[2]);


//  RCLCPP_INFO(this->get_logger(), "joint_1_meas: '%lf' joint_2_meas: '%lf'", joint_1_meas_angle, joint_2_meas_angle);
   command_publsiher_->publish(command_input_U_msg);
}


void state_subscriber_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){

      for (size_t i = 0; i < msg->data.size(); ++i) {
        state_meas[i] = msg->data[i];
      }

  global_xyz_meas[0] = state_meas[0];
  global_xyz_meas[1] = state_meas[1];
  global_xyz_meas[2] = state_meas[2];

  global_rpy_meas[0] = state_meas[3];
  global_rpy_meas[1] = state_meas[4];
  global_rpy_meas[2] = state_meas[5];

  joint_angle_meas[0] = state_meas[6];
  joint_angle_meas[1] = state_meas[7];
  joint_angle_meas[2] = state_meas[8];

}

void body_rpy_subscriber_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){

  body_rpy_meas[0] = msg->data[0];
  body_rpy_meas[1] = msg->data[1];
  body_rpy_meas[2] = msg->data[2];



}



  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_visual;
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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publsiher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;    
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_subscriber;    
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr body_rpy_subscriber;    

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr link_yaw_imu_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr position_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_publisher_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    

  size_t count_;
  std_msgs::msg::Float64 joint_1_cmd_msg;
  std_msgs::msg::Float64 joint_2_cmd_msg;
  std_msgs::msg::Float64 joint_3_cmd_msg;    
  //TODO:: 아래 세 줄 정의 제대로 하기
  ros_gz_interfaces::msg::EntityWrench wrench_msg;
  // msg.entity.name = "link_drone";
  // msg.entity.type = ros_gz_interfaces::msg::Entity::LINK;


  Eigen::Vector3d global_xyz_meas;
  Eigen::Vector3d global_xyz_cmd = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_xyz_error;
  Eigen::Vector3d global_xyz_error_integral;
  Eigen::Vector3d global_xyz_error_d;
  Eigen::Vector3d global_xyz_ddot_meas;
  Eigen::Vector3d body_xyz_error;
  Eigen::Vector3d body_xyz_error_integral = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_xyz_error_d = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_body_xyz_error = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_force_cmd;
  Eigen::Vector3d global_force_cmd;  
  Eigen::Matrix3d body_xyz_P = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d body_xyz_I = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d body_xyz_D = Eigen::Matrix3d::Zero();



  Eigen::Vector3d global_rpy_cmd;
  Eigen::Vector3d global_rpy_meas;
  Eigen::Vector3d body_rpy_meas;
  Eigen::Vector3d body_rpy_cmd;
  Eigen::Vector3d body_rpy_error;
  Eigen::Vector3d body_rpy_error_integral = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_rpy_vel_meas;
  Eigen::Vector3d global_rpy_vel_meas;
  Eigen::Vector3d body_rpy_error_d = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_torque_cmd;  
  Eigen::Vector3d global_torque_cmd;  
  Eigen::Matrix3d body_rpy_P = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d body_rpy_I = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d body_rpy_D = Eigen::Matrix3d::Zero();


  Eigen::Vector3d joint_angle_cmd;
  Eigen::Vector3d joint_angle_meas;
  Eigen::Vector3d joint_angle_dot_meas;  
  Eigen::Vector3d joint_effort_meas;
  Eigen::VectorXd State = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd State_prev = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd State_dot = Eigen::VectorXd::Zero(9);
    
  Eigen::VectorXd FK_EE_Pos = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Tw1_Pos = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Tw2_Pos = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Tw3_Pos = Eigen::VectorXd::Zero(6);
  Eigen::Matrix4d T_w1, T_w2, T_w3, T_w0;
    Eigen::Matrix4d T_01 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_12 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_23 = Eigen::Matrix4d::Identity();



  Eigen::VectorXd state_meas = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd command_input_U = Eigen::VectorXd::Zero(9);



    double time;
    double time_cnt;
    double sine;

    double delta_time = 0.005;

    double l1 = 0.1;
    double l2 = 0.2;
    double l3 = 0.2;
};

int main(int argc, char * argv[])
{
//ros2 topic pub /joint_1/command std_msgs/msg/Float64 "{data: 1.0}"
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sedas_control>());
  rclcpp::shutdown();
  return 0;
}
