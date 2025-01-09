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
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "ButterworthFilter.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ign_pubsub : public rclcpp::Node
{
  public:
    ign_pubsub()
      : Node("ign_pubsub"), 
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)), 
      count_(0),
    bwf_global_force_cmd_x(5.0, 0.005), // Cutoff frequency = 20 Hz, Sampling time = 0.005 s
    bwf_global_force_cmd_y(5.0, 0.005),
    bwf_global_force_cmd_z(5.0, 0.005)      
    {      
      // QoS 설정
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

      cmd_joint1_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_1/command", qos_settings);
      cmd_joint2_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_2/command", qos_settings);      
      cmd_joint3_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_3/command", qos_settings);            
      wrench_publisher_ = this->create_publisher<ros_gz_interfaces::msg::EntityWrench>("/link_drone/wrench", qos_settings);
      velocity_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/velocity_marker", qos_settings);
      state_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_vector", qos_settings);            
      commanded_publsiher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/commanded_input_U", qos_settings);
      state_U_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_U", qos_settings);



      joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/manipulator/joint_states", qos_settings,
          std::bind(&ign_pubsub::joint_state_subsciber_callback, this, std::placeholders::_1));
      link_yaw_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/manipulator/imu", qos_settings,
          std::bind(&ign_pubsub::imu_subscriber_callback, this, std::placeholders::_1));

      position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
          "/manipulator/pose_info", qos_settings,
          std::bind(&ign_pubsub::global_pose_callback, this, std::placeholders::_1));            
	

      // Joint 1 Subscriber
      joint_1_torque_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "/force_torque_joint1", 10,  // Topic name and QoS depth
          std::bind(&ign_pubsub::joint1_torque_Callback, this, std::placeholders::_1));

      // Joint 2 Subscriber
      joint_2_torque_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "/force_torque_joint2", 10,  // Topic name and QoS depth
          std::bind(&ign_pubsub::joint2_torque_Callback, this, std::placeholders::_1));

      // Joint 3 Subscriber
      joint_3_torque_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "/force_torque_joint3", 10,  // Topic name and QoS depth
          std::bind(&ign_pubsub::joint3_torque_Callback, this, std::placeholders::_1));




      timer_ = this->create_wall_timer(
      5ms, std::bind(&ign_pubsub::timer_callback, this));

      timer_visual = this->create_wall_timer(
      100ms, std::bind(&ign_pubsub::slower_callback, this));


    body_xyz_P.diagonal() << 20, 20, 50;
    body_xyz_I.diagonal() << 0.1, 0.1, 2;
    body_xyz_D.diagonal() << 1, 1, 5;
    body_rpy_P.diagonal() << 20, 20, 5;
    body_rpy_D.diagonal() << 3, 3, 0.5;
      wrench_msg.entity.name = "link_drone"; // 링크 이름
      wrench_msg.entity.type = ros_gz_interfaces::msg::Entity::LINK; // 엔티티 유형: LINK


    }

  private:
	    void timer_callback()
    {	//main loop, 100Hz
    // 현재 시간 계산
      set_state_FK();      
      set_state_dot_FK();
      set_traj();
      PID_controller();		

      data_publish();     	    
    }

    void slower_callback()
    {
      tf_publish();      
    }



  double saturation(double max, double min, double value){
  if (value > max) value = max;
  else if (value < min) value = min;
  return value;
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
        // 10초부터 15초까지 Joint 1이 0도에서 160도로 천천히 상승
        joint_angle_cmd[0] = (160 * M_PI / 180) * ((time - 10.0) / 5.0); // Joint 1 각도
    } else if (time > 15.0 && time <= 20.0) {
        // 15초부터 20초까지 고정
        global_xyz_cmd[2] = 2.0; // Z축 고정
    } else if (time > 20.0 && time <= 25.0) {
        // 20초부터 25초까지 Joint 2가 0도에서 180도로 천천히 상승
        joint_angle_cmd[1] = (70 * M_PI / 180) * ((time - 20.0) / 5.0); // Joint 2 각도
    } else if (time > 25.0 && time <= 30.0) {
        // 25초부터 30초까지 Z축 고정, X축 원점 복귀
        global_xyz_cmd[2] = 2.0; // Z축 고정
    } else if (time > 30.0) {
        // 30초 이후 Joint 명령을 연속적으로 sin 파형으로 설정
        double t = time - 30.0;
        joint1_accum_angle += delta_time * 10 * M_PI / 180 * std::cos(2 * M_PI * t / 5);
        joint2_accum_angle += delta_time * 20 * M_PI / 180 * std::cos(2 * M_PI * t / 4);
        joint3_accum_angle += delta_time * 10 * M_PI / 180 * std::cos(2 * M_PI * t / 3);
    }


    joint_angle_cmd[0] = joint1_accum_angle + 10 * M_PI / 180;
    joint_angle_cmd[1] = joint2_accum_angle + 20 * M_PI / 180;
    joint_angle_cmd[2] = joint3_accum_angle + 10 * M_PI / 180;
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

  cmd_joint1_publisher_->publish(joint_1_cmd_msg);
  cmd_joint2_publisher_->publish(joint_2_cmd_msg);       
  cmd_joint3_publisher_->publish(joint_3_cmd_msg);       	      
  wrench_publisher_->publish(wrench_msg);






  std_msgs::msg::Float64MultiArray state_msg;
  state_msg.data.push_back(global_xyz_meas[0]);
  state_msg.data.push_back(global_xyz_meas[1]);
  state_msg.data.push_back(global_xyz_meas[2]);
  state_msg.data.push_back(quat_meas[0]);
  state_msg.data.push_back(quat_meas[1]);
  state_msg.data.push_back(quat_meas[2]);
  state_msg.data.push_back(quat_meas[3]);
  state_msg.data.push_back(joint_angle_meas[0]);
  state_msg.data.push_back(joint_angle_meas[1]);
  state_msg.data.push_back(joint_angle_meas[2]);
  state_publisher_->publish(state_msg);


  std_msgs::msg::Float64MultiArray state_U_msg;
  state_U_msg.data.push_back(global_force_cmd[0]);
  state_U_msg.data.push_back(global_force_cmd[1]);
  state_U_msg.data.push_back(global_force_cmd[2]);
  state_U_msg.data.push_back(joint_effort_meas[0]);
  state_U_msg.data.push_back(joint_effort_meas[1]);
  state_U_msg.data.push_back(joint_effort_meas[2]);
  state_U_msg.data.push_back(bwf_global_force_cmd_x.Filter(joint_effort_meas[0]));
  state_U_msg.data.push_back(bwf_global_force_cmd_y.Filter(joint_effort_meas[1]));
  state_U_msg.data.push_back(bwf_global_force_cmd_z.Filter(joint_effort_meas[2]));
  state_U_publisher_->publish(state_U_msg);



  std_msgs::msg::Float64MultiArray commanded_input;
  commanded_input.data.push_back(global_force_cmd[0]);
  commanded_input.data.push_back(global_force_cmd[1]);
  commanded_input.data.push_back(global_force_cmd[2]);
  commanded_input.data.push_back(global_torque_cmd[0]);
  commanded_input.data.push_back(global_torque_cmd[1]);
  commanded_input.data.push_back(global_torque_cmd[2]);
  commanded_input.data.push_back(joint_angle_cmd[0]);
  commanded_input.data.push_back(joint_angle_cmd[1]);
  commanded_input.data.push_back(joint_angle_cmd[2]);
  commanded_publsiher_->publish(commanded_input);


}
 
void joint_state_subsciber_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  joint_angle_dot_meas[0] = msg->velocity[0];
  joint_angle_dot_meas[1] = msg->velocity[1];
  joint_angle_dot_meas[2] = msg->velocity[2];
  joint_angle_meas[0] = msg->position[0]; // D-H Parameter!!
  joint_angle_meas[1] = msg->position[1]; 
  joint_angle_meas[2] = msg->position[2];
}
 
void imu_subscriber_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{

    // 쿼터니언 값 가져오기
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;

  quat_meas[0] = qx;
  quat_meas[1] = qy;
  quat_meas[2] = qz;
  quat_meas[3] = qw;

    // Roll, Pitch, Yaw 계산
    body_rpy_meas[0] = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    body_rpy_meas[1] = std::asin(2.0 * (qw * qy - qz * qx));
    body_rpy_meas[2] = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    body_rpy_vel_meas[0] = msg->angular_velocity.x;
    body_rpy_vel_meas[1] = msg->angular_velocity.y;
    body_rpy_vel_meas[2] = msg->angular_velocity.z;

    global_xyz_ddot_meas[0] = msg->linear_acceleration.x;
    global_xyz_ddot_meas[1] = msg->linear_acceleration.y;
    global_xyz_ddot_meas[2] = msg->linear_acceleration.z;


  global_rpy_meas = Rot_D2G(body_rpy_meas, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
  global_rpy_vel_meas = Rot_D2G(body_rpy_vel_meas, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);


}

	    
	    
void global_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{

    // link_yaw의 id는 7로 고정
    const int link_yaw_id = 1;

    if (link_yaw_id < msg->poses.size())
    {
        const auto &pose = msg->poses[link_yaw_id];
      global_xyz_meas[0] = pose.position.x;
      global_xyz_meas[1] = pose.position.y;
      global_xyz_meas[2] = pose.position.z;                        
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "link_yaw id (17) is out of bounds in PoseArray.");
    }


}


void joint1_torque_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    joint_effort_meas[0] = msg->wrench.torque.z;
        RCLCPP_WARN(this->get_logger(), "CALLBACK!!");
}

void joint2_torque_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    joint_effort_meas[1] = msg->wrench.torque.z;
}

void joint3_torque_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    joint_effort_meas[2] = msg->wrench.torque.z;
}



void set_state_FK()
{
 
    State << global_xyz_meas[0], global_xyz_meas[1], global_xyz_meas[2], 
    global_rpy_meas[0], global_rpy_meas[1], global_rpy_meas[2], 
    joint_angle_meas[0], joint_angle_meas[1], joint_angle_meas[2];
 
    // 기본적으로 드론의 Global Frame 기준 위치 및 자세를 기반으로 변환 행렬 T_w0 계산
    Eigen::Matrix3d R_B = get_rotation_matrix(global_rpy_meas[0], global_rpy_meas[1], global_rpy_meas[2]);
    T_w0.block<3, 3>(0, 0) = R_B;
    T_w0.block<3, 1>(0, 3) = global_xyz_meas;

    // DH 파라미터 기반의 변환 행렬 정의
    T_01 << std::cos(joint_angle_meas[0]), 0, std::sin(joint_angle_meas[0]), 0,
            std::sin(joint_angle_meas[0]), 0, -std::cos(joint_angle_meas[0]), 0,
            0, 1, 0, l1,
            0, 0, 0, 1;

    T_12 << std::cos(joint_angle_meas[1]), -std::sin(joint_angle_meas[1]), 0, l2 * std::cos(joint_angle_meas[1]),
            std::sin(joint_angle_meas[1]), std::cos(joint_angle_meas[1]), 0, l2 * std::sin(joint_angle_meas[1]),
            0, 0, 1, 0,
            0, 0, 0, 1;

    T_23 << std::cos(joint_angle_meas[2]), -std::sin(joint_angle_meas[2]), 0, l3 * std::cos(joint_angle_meas[2]),
            std::sin(joint_angle_meas[2]), std::cos(joint_angle_meas[2]), 0, l3 * std::sin(joint_angle_meas[2]),
            0, 0, 1, 0,
            0, 0, 0, 1;

    // Forward Kinematics 계산
    Eigen::Matrix4d T_w1 = T_w0 * T_01;
    Eigen::Matrix4d T_w2 = T_w1 * T_12;
    Eigen::Matrix4d T_w3 = T_w2 * T_23;

    // 엔드 이펙터의 위치 및 자세 추출
    Eigen::Vector3d p_E = T_w3.block<3, 1>(0, 3); // 엔드 이펙터 위치
    Eigen::Matrix3d R_E = T_w3.block<3, 3>(0, 0); // 엔드 이펙터 자세

    FK_EE_Pos[0] = p_E[0];
    FK_EE_Pos[1] = p_E[1];
    FK_EE_Pos[2] = p_E[2];

    // Global 기준 r, p, y angle 추출
    FK_EE_Pos[3] = std::atan2(R_E(2, 1), R_E(2, 2));
    FK_EE_Pos[4] = std::asin(-R_E(2, 0));
    FK_EE_Pos[5] = std::atan2(R_E(1, 0), R_E(0, 0));

    // T_w1 위치 및 자세 추출
    Eigen::Vector3d p_w1 = T_w1.block<3, 1>(0, 3);
    Eigen::Matrix3d R_w1 = T_w1.block<3, 3>(0, 0);
    Tw1_Pos[0] = p_w1[0];
    Tw1_Pos[1] = p_w1[1];
    Tw1_Pos[2] = p_w1[2];
    Tw1_Pos[3] = std::atan2(R_w1(2, 1), R_w1(2, 2));
    Tw1_Pos[4] = std::asin(-R_w1(2, 0));
    Tw1_Pos[5] = std::atan2(R_w1(1, 0), R_w1(0, 0));

    // T_w2 위치 및 자세 추출
    Eigen::Vector3d p_w2 = T_w2.block<3, 1>(0, 3);
    Eigen::Matrix3d R_w2 = T_w2.block<3, 3>(0, 0);
    Tw2_Pos[0] = p_w2[0];
    Tw2_Pos[1] = p_w2[1];
    Tw2_Pos[2] = p_w2[2];
    Tw2_Pos[3] = std::atan2(R_w2(2, 1), R_w2(2, 2));
    Tw2_Pos[4] = std::asin(-R_w2(2, 0));
    Tw2_Pos[5] = std::atan2(R_w2(1, 0), R_w2(0, 0));

    // T_w3 위치 및 자세 추출
    Eigen::Vector3d p_w3 = T_w3.block<3, 1>(0, 3);
    Eigen::Matrix3d R_w3 = T_w3.block<3, 3>(0, 0);
    Tw3_Pos[0] = p_w3[0];
    Tw3_Pos[1] = p_w3[1];
    Tw3_Pos[2] = p_w3[2];
    Tw3_Pos[3] = std::atan2(R_w3(2, 1), R_w3(2, 2));
    Tw3_Pos[4] = std::asin(-R_w3(2, 0));
    Tw3_Pos[5] = std::atan2(R_w3(1, 0), R_w3(0, 0));
    // State 벡터 출력 (디버깅용)
    // std::cout << "State: \n" << State_dot << std::endl;


    // T_01의 위치 벡터 및 자세 추출
    Eigen::Vector3d p_01 = T_01.block<3, 1>(0, 3);
    Eigen::Matrix3d R_01 = T_01.block<3, 3>(0, 0);
    T01_Pos[0] = p_01[0];
    T01_Pos[1] = p_01[1];
    T01_Pos[2] = p_01[2];

    T01_Pos[3] = std::atan2(R_01(2, 1), R_01(2, 2));
    T01_Pos[4] = std::asin(-R_01(2, 0));
    T01_Pos[5] = std::atan2(R_01(1, 0), R_01(0, 0));

    // T_12의 위치 벡터 및 자세 추출
    Eigen::Vector3d p_12 = T_12.block<3, 1>(0, 3);
    Eigen::Matrix3d R_12 = T_12.block<3, 3>(0, 0);
    T12_Pos[0] = p_12[0];
    T12_Pos[1] = p_12[1];
    T12_Pos[2] = p_12[2];    
    
    T12_Pos[3] = std::atan2(R_12(2, 1), R_12(2, 2));
    T12_Pos[4] = std::asin(-R_12(2, 0));
    T12_Pos[5] = std::atan2(R_12(1, 0), R_12(0, 0));

    // T_23의 위치 벡터 및 자세 추출
    Eigen::Vector3d p_23 = T_23.block<3, 1>(0, 3);
    Eigen::Matrix3d R_23 = T_23.block<3, 3>(0, 0);
    T23_Pos[0] = p_23[0];
    T23_Pos[1] = p_23[1];
    T23_Pos[2] = p_23[2];    

    T23_Pos[3] = std::atan2(R_23(2, 1), R_23(2, 2));
    T23_Pos[4] = std::asin(-R_23(2, 0));
    T23_Pos[5] = std::atan2(R_23(1, 0), R_23(0, 0));

}


void set_state_dot_FK()
{
    // 수치미분 계산
    Eigen::VectorXd raw_State_dot = (State - State_prev) / delta_time;
    State_prev = State;

    // Low Pass Filter를 적용하기 위한 파라미터 계산
    double cutoff_freq = 40.0; // Hz
    double alpha = 1.0 / (1.0 + (1.0 / (2.0 * M_PI * cutoff_freq * delta_time)));

    // 필터 적용
    for (int i = 0; i < State_dot.size(); i++) {
        State_dot[i] = alpha * raw_State_dot[i] + (1.0 - alpha) * State_dot[i];
    }

    // 외부에서 측정한 속도값 적용 (필터를 사용하지 않음)
    State_dot[3] = global_rpy_vel_meas[0];
    State_dot[4] = global_rpy_vel_meas[1];
    State_dot[5] = global_rpy_vel_meas[2];
    State_dot[6] = joint_angle_dot_meas[0];
    State_dot[7] = joint_angle_dot_meas[1];
    State_dot[8] = joint_angle_dot_meas[2];
}



void tf_publish()
{
  //TODO 1: Drone TF Publish
  //global_xyz_meas[0], global_xyz_meas[1], global_xyz_meas[2], 
  //global_rpy_meas[0], global_rpy_meas[1], global_rpy_meas[2], 

  //TODO 2: End Effector TF Publish
  //FK_EE_Pos[0], FK_EE_Pos[1], FK_EE_Pos[2],
  //FK_EE_Pos[3], FK_EE_Pos[4], FK_EE_Pos[5],


  //TODO 1: End Effector TF Publish
        geometry_msgs::msg::TransformStamped transform_EE;
        transform_EE.header.stamp = this->get_clock()->now();
        transform_EE.header.frame_id = "world"; // Parent frame
        transform_EE.child_frame_id = "tf/EE_FK";  // Child frame

        transform_EE.transform.translation.x = FK_EE_Pos[0];
        transform_EE.transform.translation.y = FK_EE_Pos[1];
        transform_EE.transform.translation.z = FK_EE_Pos[2];

        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion q_EE;
        q_EE.setRPY(FK_EE_Pos[3], FK_EE_Pos[4], FK_EE_Pos[5]);
        transform_EE.transform.rotation.x = q_EE.x();
        transform_EE.transform.rotation.y = q_EE.y();
        transform_EE.transform.rotation.z = q_EE.z();
        transform_EE.transform.rotation.w = q_EE.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_EE);


  //TODO 2: Drone TF Publish
        geometry_msgs::msg::TransformStamped transform_drone;
        transform_drone.header.stamp = this->get_clock()->now();
        transform_drone.header.frame_id = "world"; // Parent frame
        transform_drone.child_frame_id = "link_drone";  // Child frame

        transform_drone.transform.translation.x = global_xyz_meas[0];
        transform_drone.transform.translation.y = global_xyz_meas[1];
        transform_drone.transform.translation.z = global_xyz_meas[2];

        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion q_drone;
        q_drone.setRPY(global_rpy_meas[0], global_rpy_meas[1], global_rpy_meas[2]);
        transform_drone.transform.rotation.x = q_drone.x();
        transform_drone.transform.rotation.y = q_drone.y();
        transform_drone.transform.rotation.z = q_drone.z();
        transform_drone.transform.rotation.w = q_drone.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_drone);



  //Tw1 Pub
        geometry_msgs::msg::TransformStamped transform_T01;
        transform_T01.header.stamp = this->get_clock()->now();
        transform_T01.header.frame_id = "link_drone"; // Parent frame
        transform_T01.child_frame_id = "link_1";  // Child frame

        transform_T01.transform.translation.x = 0;
        transform_T01.transform.translation.y = 0;
        transform_T01.transform.translation.z = 0.2;

        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion q_T01;
        q_T01.setRPY(0, 0, joint_angle_cmd[0]);
        transform_T01.transform.rotation.x = q_T01.x();
        transform_T01.transform.rotation.y = q_T01.y();
        transform_T01.transform.rotation.z = q_T01.z();
        transform_T01.transform.rotation.w = q_T01.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_T01);


  //Tw2 Pub
        geometry_msgs::msg::TransformStamped transform_T12;
        transform_T12.header.stamp = this->get_clock()->now();
        transform_T12.header.frame_id = "link_1"; // Parent frame
        transform_T12.child_frame_id = "link_2";  // Child frame

        transform_T12.transform.translation.x = 0;
        transform_T12.transform.translation.y = 0.0;
        transform_T12.transform.translation.z = 0.05;

        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion q_T12;
        q_T12.setRPY(M_PI / 2, -joint_angle_cmd[1], 0);
        transform_T12.transform.rotation.x = q_T12.x();
        transform_T12.transform.rotation.y = q_T12.y();
        transform_T12.transform.rotation.z = q_T12.z();
        transform_T12.transform.rotation.w = q_T12.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_T12);


  //Tw3 Pub
        geometry_msgs::msg::TransformStamped transform_T23;
        transform_T23.header.stamp = this->get_clock()->now();
        transform_T23.header.frame_id = "link_2"; // Parent frame
        transform_T23.child_frame_id = "link_3";  // Child frame

        transform_T23.transform.translation.x = 0.2;
        transform_T23.transform.translation.y = 0;
        transform_T23.transform.translation.z = 0;

        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion q_T23;
        q_T23.setRPY(0, 0, joint_angle_cmd[2]);
        transform_T23.transform.rotation.x = q_T23.x();
        transform_T23.transform.rotation.y = q_T23.y();
        transform_T23.transform.rotation.z = q_T23.z();
        transform_T23.transform.rotation.w = q_T23.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_T23);

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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_U_publisher_;
  rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr wrench_publisher_;    
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_; 
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr joint_1_torque_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr joint_2_torque_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr joint_3_torque_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr link_yaw_imu_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr position_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commanded_publsiher_;

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
  Eigen::VectorXd quat_meas = Eigen::VectorXd::Zero(4);


  


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

  Eigen::VectorXd T01_Pos = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd T12_Pos = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd T23_Pos = Eigen::VectorXd::Zero(6);



    ButterworthFilter bwf_global_force_cmd_x;
    ButterworthFilter bwf_global_force_cmd_y;
    ButterworthFilter bwf_global_force_cmd_z;


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
  rclcpp::spin(std::make_shared<ign_pubsub>());
  rclcpp::shutdown();
  return 0;
}
