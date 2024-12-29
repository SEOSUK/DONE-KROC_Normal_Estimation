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


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ign_pubsub : public rclcpp::Node
{
  public:
    ign_pubsub()
      : Node("ign_pubsub"), tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)), count_(0)
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

      joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/manipulator/joint_states", qos_settings,
          std::bind(&ign_pubsub::joint_state_subsciber_callback, this, std::placeholders::_1));
      link_yaw_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/manipulator/imu", qos_settings,
          std::bind(&ign_pubsub::imu_subscriber_callback, this, std::placeholders::_1));


      position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
          "/manipulator/pose_info", qos_settings,
          std::bind(&ign_pubsub::global_pose_callback, this, std::placeholders::_1));            
	
      timer_ = this->create_wall_timer(
      10ms, std::bind(&ign_pubsub::timer_callback, this));



    body_xyz_P.diagonal() << 8, 8, 8.0;
    body_xyz_I.diagonal() << 0.000, 0.000, 0.0;
    body_xyz_D.diagonal() << 1, 1, 1;
    body_rpy_P.diagonal() << 0.3, 0.3, 0.03;
    body_rpy_D.diagonal() << 0.06, 0.06, 0.003;
      wrench_msg.entity.name = "link_drone"; // 링크 이름
      wrench_msg.entity.type = ros_gz_interfaces::msg::Entity::LINK; // 엔티티 유형: LINK

    }

  private:
	    void timer_callback()
    {	//main loop, 100Hz
    // 현재 시간 계산
      set_state_FK();      
      set_state_dot_FK();
      calculate_end_effector_velocity();
      set_traj();
      PID_controller();		

      data_publish();     	    
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
    double time = time_cnt * delta_time;

    // 명령 생성
    if (time <= 5.0) {
        // 0초부터 5초까지 Z축으로 5미터 상승
        global_xyz_cmd[2] = 5 * (time / 5.0);
    } else if (time > 5.0 && time <= 10.0) {
        // 5초부터 10초까지 고정
        global_xyz_cmd[2] = 5.0;
    } else if (time > 10.0 && time <= 15.0) {
        // 10초부터 15초까지 X 방향으로 3미터 이동
        global_xyz_cmd[0] = 3 * ((time - 10.0) / 5.0);
        global_xyz_cmd[2] = 5.0; // Z축 고정
    } else if (time > 15.0 && time <= 20.0) {
        // 15초부터 20초까지 Yaw 방향으로 30도 회전
        global_rpy_cmd[2] = (30 * M_PI / 180) * ((time - 15.0) / 5.0);
        global_xyz_cmd[0] = 3.0; // X축 고정
        global_xyz_cmd[2] = 5.0; // Z축 고정
    } else if (time > 20.0 && time <= 25.0) {
        // 20초부터 25초까지 X 방향으로 -3미터 이동 (원점 복귀)
        global_xyz_cmd[0] = 3.0 - 3.0 * ((time - 20.0) / 5.0);
        global_xyz_cmd[2] = 5.0; // Z축 고정
    } else if (time > 25.0 && time <= 35.0) {
        // 25초부터 35초까지 Joint1, Joint2, Joint3 명령을 sin 파로 줌
        double t = time - 25.0;
        joint_angle_cmd[0] = 45 * M_PI / 180 * std::sin(2 * M_PI * t / 5); // 진폭 0.5, 주기 5초
        joint_angle_cmd[1] = 30 * M_PI / 180 * std::sin(2 * M_PI * t / 4); // 진폭 0.3, 주기 10초
        joint_angle_cmd[2] = 25 * M_PI / 180 * std::sin(2 * M_PI * t / 3); // 진폭 0.4, 주기 약 6.67초
    }
    else
    {
      joint_angle_cmd[0] = 0;
      joint_angle_cmd[1] = 0;
      joint_angle_cmd[2] = 0;
    }

    // 디버깅 정보 출력
    // RCLCPP_INFO(this->get_logger(), "Xdes: '%lf', Ydes: '%lf', Zdes: '%lf'", global_xyz_cmd[0], global_xyz_cmd[1], global_xyz_cmd[2]);
    // RCLCPP_INFO(this->get_logger(), "YawCmd: '%lf'", global_rpy_cmd[2]);
    // RCLCPP_INFO(this->get_logger(), "JointCmd: q1='%lf', q2='%lf', q3='%lf'", joint_angle_cmd[0], joint_angle_cmd[1], joint_angle_cmd[2]);
}


Eigen::Matrix3d get_rotation_matrix(double roll, double pitch, double yaw) {
  // Global Frame 기준!!!
  
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
    return Rz * Ry * Rx;
}

Eigen::Vector3d Rot_G2D(const Eigen::Vector3d& global_vector, double roll, double pitch, double yaw) {
    Eigen::Matrix3d R = get_rotation_matrix(roll, pitch, yaw);

    return R.transpose() * global_vector;
}

Eigen::Vector3d Rot_D2G(const Eigen::Vector3d& body_vector, double roll, double pitch, double yaw) {
    Eigen::Matrix3d R = get_rotation_matrix(roll, pitch, yaw);

    return R * body_vector; // Transpose for inverse

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
  joint_angle_dot_meas[0] = msg->velocity[0];
  joint_angle_dot_meas[1] = msg->velocity[1];
  joint_angle_dot_meas[2] = msg->velocity[2];
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

    global_xyz_ddot_meas[0] = msg->linear_acceleration.x;
    global_xyz_ddot_meas[1] = msg->linear_acceleration.y;
    global_xyz_ddot_meas[2] = msg->linear_acceleration.z;
    // 각속도와 선가속도 출력
    //  RCLCPP_INFO(this->get_logger(), "Angular Velocity - x: %.6f, y: %.6f, z: %.6f",
    //              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    //  RCLCPP_INFO(this->get_logger(), "Linear Acceleration - x: %.6f, y: %.6f, z: %.6f",
    //              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);


  global_rpy_meas = Rot_D2G(body_rpy_meas, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
  global_rpy_vel_meas = Rot_D2G(body_rpy_vel_meas, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);


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


void set_state_FK()
{
 
    State << global_xyz_meas[0], global_xyz_meas[1], global_xyz_meas[2], 
    global_rpy_meas[0], global_rpy_meas[1], global_rpy_meas[2], 
    joint_angle_meas[0], joint_angle_meas[1], joint_angle_meas[2];

    // RCLCPP_INFO(this->get_logger(), "End State - x: %.3f, y: %.3f, z: %.3f, r: %lf p: %lf, y: %lf, q1: %.3f q2: %.3f, q3: %.3f", global_xyz_meas[0], global_xyz_meas[1], global_xyz_meas[2], global_rpy_meas[0], global_rpy_meas[1], global_rpy_meas[2], joint_angle_meas[0], joint_angle_meas[1], joint_angle_meas[2]);

 
    // 기본적으로 드론의 Global Frame 기준 위치 및 자세를 기반으로 변환 행렬 T_w0 계산
    Eigen::Matrix4d T_w0 = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_B = get_rotation_matrix(global_rpy_meas[0], global_rpy_meas[1], global_rpy_meas[2]);
    T_w0.block<3, 3>(0, 0) = R_B;
    T_w0.block<3, 1>(0, 3) = global_xyz_meas;

    // DH 파라미터 기반의 변환 행렬 정의
    Eigen::Matrix4d T_01 = Eigen::Matrix4d::Identity();
    T_01 << std::cos(joint_angle_meas[0] + M_PI / 2), 0, std::sin(joint_angle_meas[0] + M_PI / 2), 0,
            std::sin(joint_angle_meas[0] + M_PI / 2), 0, -std::cos(joint_angle_meas[0] + M_PI / 2), 0,
            0, 1, 0, l1,
            0, 0, 0, 1;

    Eigen::Matrix4d T_12 = Eigen::Matrix4d::Identity();
    T_12 << std::cos(joint_angle_meas[1]), -std::sin(joint_angle_meas[1]), 0, l2 * std::cos(joint_angle_meas[1]),
            std::sin(joint_angle_meas[1]), std::cos(joint_angle_meas[1]), 0, l2 * std::sin(joint_angle_meas[1]),
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix4d T_23 = Eigen::Matrix4d::Identity();
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

    // RCLCPP_INFO(this->get_logger(), "Global rpy angles - Roll: %.3f, Pitch: %.3f, Yaw: %.3f", roll, pitch, yaw);

    // State 벡터 출력 (디버깅용)
    std::cout << "State: \n" << State_dot << std::endl;
}


void set_state_dot_FK()
{
 State_dot = (State - State_prev) / delta_time;
 State_prev = State;

  State_dot[3] = global_rpy_vel_meas[0];
  State_dot[4] = global_rpy_vel_meas[1];
  State_dot[5] = global_rpy_vel_meas[2];
  State_dot[6] = joint_angle_dot_meas[0];
  State_dot[7] = joint_angle_dot_meas[1];
  State_dot[8] = joint_angle_dot_meas[2];


}


void visualize_velocity(const Eigen::Vector3d& velocity, const Eigen::Vector3d& position)
{
    visualization_msgs::msg::Marker velocity_marker;
    velocity_marker.header.frame_id = "world";
    velocity_marker.header.stamp = this->get_clock()->now();
    velocity_marker.ns = "velocity";
    velocity_marker.id = 0;
    velocity_marker.type = visualization_msgs::msg::Marker::ARROW;
    velocity_marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start, end;
    start.x = position[0];
    start.y = position[1];
    start.z = position[2];

    end.x = position[0] + velocity[0];
    end.y = position[1] + velocity[1];
    end.z = position[2] + velocity[2];

    velocity_marker.points.push_back(start);
    velocity_marker.points.push_back(end);
    velocity_marker.scale.x = 0.02; // Arrow shaft diameter
    velocity_marker.scale.y = 0.05; // Arrow head diameter

    velocity_marker.color.a = 1.0; // Opacity
    velocity_marker.color.r = 1.0; // Red

    velocity_publisher_->publish(velocity_marker);
}



void calculate_end_effector_velocity()
{
    // State_dot 계산을 통해 속도 구하기
    // Jacobian 행렬 계산
    Eigen::MatrixXd J(6, 9); // 6x9 Jacobian

    // Forward Kinematics 변환 행렬
    Eigen::Matrix4d T_w1, T_w2, T_w3, T_w0;
    Eigen::Vector3d p_E = T_w3.block<3, 1>(0, 3); // End-effector position
    Eigen::Matrix3d R_E = T_w3.block<3, 3>(0, 0); // End-effector rotation

    // 드론 상태의 영향을 반영
    Eigen::Vector3d Z0(0, 0, 1); // 드론의 기본 Z 축
    Eigen::Vector3d p_drone = T_w0.block<3, 1>(0, 3);

    J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // 드론 위치의 선속도 부분
    J.block<3, 3>(0, 3) = -R_E * skewSymmetric(p_E - p_drone); // 드론 자세의 선속도 부분
    J.block<3, 3>(3, 3) = R_E; // 드론 자세의 각속도 부분

    // 매니퓰레이터 조인트에 따른 엔드 이펙터 변화 반영
    Eigen::Vector3d Z1 = T_w1.block<3, 3>(0, 0) * Z0;
    Eigen::Vector3d Z2 = T_w2.block<3, 3>(0, 0) * Z0;
    Eigen::Vector3d Z3 = T_w3.block<3, 3>(0, 0) * Z0;

    Eigen::Vector3d p_1 = T_w1.block<3, 1>(0, 3);
    Eigen::Vector3d p_2 = T_w2.block<3, 1>(0, 3);
    Eigen::Vector3d p_3 = T_w3.block<3, 1>(0, 3);

    J.block<3, 1>(0, 6) = Z1.cross(p_E - p_1);
    J.block<3, 1>(0, 7) = Z2.cross(p_E - p_2);
    J.block<3, 1>(0, 8) = Z3.cross(p_E - p_3);

    J.block<3, 1>(3, 6) = Z1;
    J.block<3, 1>(3, 7) = Z2;
    J.block<3, 1>(3, 8) = Z3;

    // End-effector 속도 계산: V_E = J * State_dot
    Eigen::VectorXd V_E = J * State_dot;

    // 선속도와 각속도 분리
    Eigen::Vector3d linear_velocity = V_E.segment<3>(0);
    Eigen::Vector3d angular_velocity = V_E.segment<3>(3);

    // 결과 출력
    RCLCPP_INFO(this->get_logger(), "Linear Velocity: x=%.6f, y=%.6f, z=%.6f", 
                linear_velocity.x(), linear_velocity.y(), linear_velocity.z());
    RCLCPP_INFO(this->get_logger(), "Angular Velocity: roll=%.6f, pitch=%.6f, yaw=%.6f", 
                angular_velocity.x(), angular_velocity.y(), angular_velocity.z());

    visualize_velocity(linear_velocity, FK_EE_Pos.segment<3>(0));
}

// Skew symmetric matrix 생성 함수
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d skew;
    skew << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
    return skew;
}



void tf_publish()
{
  //TODO 1: Drone TF Publish
  //global_xyz_meas[0], global_xyz_meas[1], global_xyz_meas[2], 
  //global_rpy_meas[0], global_rpy_meas[1], global_rpy_meas[2], 

  //TODO 2: End Effector TF Publish
  //FK_EE_Pos[0], FK_EE_Pos[1], FK_EE_Pos[2],
  //FK_EE_Pos[3], FK_EE_Pos[4], FK_EE_Pos[5],

  //TODO 1: Drone TF Publish
        geometry_msgs::msg::TransformStamped transform_drone;
        transform_drone.header.stamp = this->get_clock()->now();
        transform_drone.header.frame_id = "world"; // Parent frame
        transform_drone.child_frame_id = "tf/drone";  // Child frame

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


  //TODO 2: End Effector TF Publish
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


  //Tw1 Pub
        geometry_msgs::msg::TransformStamped transform_Tw1;
        transform_Tw1.header.stamp = this->get_clock()->now();
        transform_Tw1.header.frame_id = "world"; // Parent frame
        transform_Tw1.child_frame_id = "tf/Tw1";  // Child frame

        transform_Tw1.transform.translation.x = Tw1_Pos[0];
        transform_Tw1.transform.translation.y = Tw1_Pos[1];
        transform_Tw1.transform.translation.z = Tw1_Pos[2];

        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion q_Tw1;
        q_Tw1.setRPY(Tw1_Pos[3], Tw1_Pos[4], Tw1_Pos[5]);
        transform_Tw1.transform.rotation.x = q_Tw1.x();
        transform_Tw1.transform.rotation.y = q_Tw1.y();
        transform_Tw1.transform.rotation.z = q_Tw1.z();
        transform_Tw1.transform.rotation.w = q_Tw1.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_Tw1);


  //Tw2 Pub
        geometry_msgs::msg::TransformStamped transform_Tw2;
        transform_Tw2.header.stamp = this->get_clock()->now();
        transform_Tw2.header.frame_id = "world"; // Parent frame
        transform_Tw2.child_frame_id = "tf/Tw2";  // Child frame

        transform_Tw2.transform.translation.x = Tw2_Pos[0];
        transform_Tw2.transform.translation.y = Tw2_Pos[1];
        transform_Tw2.transform.translation.z = Tw2_Pos[2];

        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion q_Tw2;
        q_Tw2.setRPY(Tw2_Pos[3], Tw2_Pos[4], Tw2_Pos[5]);
        transform_Tw2.transform.rotation.x = q_Tw2.x();
        transform_Tw2.transform.rotation.y = q_Tw2.y();
        transform_Tw2.transform.rotation.z = q_Tw2.z();
        transform_Tw2.transform.rotation.w = q_Tw2.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_Tw2);


  //Tw3 Pub
        geometry_msgs::msg::TransformStamped transform_Tw3;
        transform_Tw3.header.stamp = this->get_clock()->now();
        transform_Tw3.header.frame_id = "world"; // Parent frame
        transform_Tw3.child_frame_id = "tf/Tw3";  // Child frame

        transform_Tw3.transform.translation.x = Tw3_Pos[0];
        transform_Tw3.transform.translation.y = Tw3_Pos[1];
        transform_Tw3.transform.translation.z = Tw3_Pos[2];

        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion q_Tw3;
        q_Tw3.setRPY(Tw3_Pos[3], Tw3_Pos[4], Tw3_Pos[5]);
        transform_Tw3.transform.rotation.x = q_Tw3.x();
        transform_Tw3.transform.rotation.y = q_Tw3.y();
        transform_Tw3.transform.rotation.z = q_Tw3.z();
        transform_Tw3.transform.rotation.w = q_Tw3.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_Tw3);

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





    double time;
    double time_cnt;
    double sine;

    double delta_time = 0.01;

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