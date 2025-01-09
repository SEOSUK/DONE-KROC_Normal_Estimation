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
#include <iostream>
#include <std_msgs/msg/float64_multi_array.hpp>  // 다중 float64 배열 퍼블리시

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class sedas_rviz : public rclcpp::Node
{
  public:
    sedas_rviz()
      : Node("sedas_rviz"), 
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)), 
      count_(0)
    {      
      // QoS 설정
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

      joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/manipulator/joint_states", qos_settings,
          std::bind(&sedas_rviz::joint_state_subsciber_callback, this, std::placeholders::_1));
      link_yaw_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/manipulator/imu", qos_settings,
          std::bind(&sedas_rviz::imu_subscriber_callback, this, std::placeholders::_1));
      position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
          "/manipulator/pose_info", qos_settings,
          std::bind(&sedas_rviz::global_pose_callback, this, std::placeholders::_1));            
      EE_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pinnochio/EE_vel", qos_settings,
          std::bind(&sedas_rviz::EE_vel_callback, this, std::placeholders::_1)); 
      EE_pos_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pinnochio/EE_pos", qos_settings,
          std::bind(&sedas_rviz::EE_pos_callback, this, std::placeholders::_1)); 

      marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/visualization_marker", 10);


      timer_ = this->create_wall_timer(
      10ms, std::bind(&sedas_rviz::timer_callback, this));




    }

  private:
	    void timer_callback()
    {	//main loop, 100Hz
    // 현재 시간 계산
      Calc_FK();
      Robot_State_Publisher();
      End_Effector_Pos_Vel_Publisher();      
    }

    void Calc_FK()
    { 
    // 기본적으로 드론의 Global Frame 기준 위치 및 자세를 기반으로 변환 행렬 T_w0 계산
    Eigen::Matrix3d R_B = get_rotation_matrix(global_rpy_meas[0], global_rpy_meas[1], global_rpy_meas[2]);
    T_w0.block<3, 3>(0, 0) = R_B;
    T_w0.block<3, 1>(0, 3) = global_xyz_meas;

    // DH 파라미터 기반의 변환 행렬 정의
    T_01 << std::cos(joint_angle_meas[0]), 0, std::sin(joint_angle_meas[0]), 0,
            std::sin(joint_angle_meas[0]), 0, -std::cos(joint_angle_meas[0]), 0,
            0, 1, 0, l1 + 0.15,
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
    p_E = T_w3.block<3, 1>(0, 3); // 엔드 이펙터 위치
    R_E = T_w3.block<3, 3>(0, 0); // 엔드 이펙터 자세

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


    void Robot_State_Publisher()
    {
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
        q_T01.setRPY(0, 0, joint_angle_meas[0]);
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
        q_T12.setRPY(M_PI / 2, -joint_angle_meas[1], 0);
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
        q_T23.setRPY(0, 0, joint_angle_meas[2]);
        transform_T23.transform.rotation.x = q_T23.x();
        transform_T23.transform.rotation.y = q_T23.y();
        transform_T23.transform.rotation.z = q_T23.z();
        transform_T23.transform.rotation.w = q_T23.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_T23);
    }


  void End_Effector_Pos_Vel_Publisher()
  {
    // Rviz에서 시각화할 Marker 메시지 생성
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "ee_velocity";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 화살표의 시작점과 끝점 설정
    geometry_msgs::msg::Point start_point, end_point;
    start_point.x = FK_EE_Pos[0]; // 현재 위치
    start_point.y = FK_EE_Pos[1];
    start_point.z = FK_EE_Pos[2];

    end_point.x = start_point.x + EE_lin_vel[0]; // 속도 벡터 방향
    end_point.y = start_point.y + EE_lin_vel[1];
    end_point.z = start_point.z + EE_lin_vel[2];

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    // 화살표의 색상 및 크기 설정
    marker.scale.x = 0.02; // 화살표의 줄기 두께
    marker.scale.y = 0.05; // 화살표의 머리 크기
    marker.scale.z = 0.05;

    marker.color.a = 1.0; // 불투명도
    marker.color.r = 1.0; // 빨간색 (속도)
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // 퍼블리시
    marker_publisher_->publish(marker);

  }


void data_publish()
{	// publish!!




}
 
void joint_state_subsciber_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Manipulator의 상~ㅌ태!!
  
  joint_angle_dot_meas[0] = msg->velocity[0];
  joint_angle_dot_meas[1] = msg->velocity[1];
  joint_angle_dot_meas[2] = msg->velocity[2];
  joint_angle_meas[0] = msg->position[0]; // D-H Parameter!!
  joint_angle_meas[1] = msg->position[1]; 
  joint_angle_meas[2] = msg->position[2];

}
 
void imu_subscriber_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Drone의 상.태~
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

void EE_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{  

    EE_lin_vel[0] = msg->data[0];
    EE_lin_vel[1] = msg->data[1];
    EE_lin_vel[2] = msg->data[2];
    EE_ang_vel[0] = msg->data[3];
    EE_ang_vel[1] = msg->data[4];
    EE_ang_vel[2] = msg->data[5];

    EE_lin_vel_global = Rot_D2G(EE_lin_vel, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);


}

void EE_pos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  EE_lin_pos[0] = msg->data[0];
  EE_lin_pos[1] = msg->data[1];
  EE_lin_pos[2] = msg->data[2];
  EE_ang_pos[0] = msg->data[3];
  EE_ang_pos[1] = msg->data[4];
  EE_ang_pos[2] = msg->data[5];
}

void joint1_torque_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    joint_effort_meas[0] = msg->wrench.torque.z;
}

void joint2_torque_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    joint_effort_meas[1] = msg->wrench.torque.z;
}

void joint3_torque_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    joint_effort_meas[2] = msg->wrench.torque.z;
}


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_visual;



  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_; 
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr link_yaw_imu_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr position_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr EE_vel_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr EE_pos_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;






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
  Eigen::VectorXd State_quat = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd State_quat_prev = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd filtered_state_dot = Eigen::VectorXd::Zero(9);


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

  Eigen::Vector3d EE_lin_vel;
  Eigen::Vector3d EE_ang_vel;
  Eigen::Vector3d EE_lin_pos;
  Eigen::Vector3d EE_ang_pos;
  Eigen::Vector3d p_E;
  Eigen::Matrix3d R_E;


  Eigen::Vector3d EE_lin_vel_global;



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
  rclcpp::spin(std::make_shared<sedas_rviz>());
  rclcpp::shutdown();
  return 0;
}
