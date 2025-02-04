#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <std_msgs/msg/float64_multi_array.hpp>  // 다중 float64 배열 퍼블리시
#include <std_msgs/msg/string.hpp>  // 다중 float64 배열 퍼블리시
#include <string> // std::string 헤더 추가
#include "std_msgs/msg/float64.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ncurses.h> // ncurses 헤더

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class sedas_traj : public rclcpp::Node
{
  public:
    sedas_traj()
      : Node("sedas_traj"), 
      count_(0)
    {      
      // QoS 설정
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

      keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>(
          "keyboard_input", qos_settings,
          std::bind(&sedas_traj::keyboard_subsciber_callback, this, std::placeholders::_1));




    drone_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/manipulator/drone_cmd", 10);


      timer_ = this->create_wall_timer(
      10ms, std::bind(&sedas_traj::timer_callback, this));




    }

  private:
	    void timer_callback()
    {	//main loop, 100Hz
    // 현재 시간 계산
      traj_gen();
      data_publisher();
    }



    void data_publisher()
    {
      std_msgs::msg::Float64MultiArray drone_cmd;
      drone_cmd.data.push_back(drone_xyz_position_cmd[0]);
      drone_cmd.data.push_back(drone_xyz_position_cmd[1]);
      drone_cmd.data.push_back(drone_xyz_position_cmd[2]);

      drone_cmd_publisher_->publish(drone_cmd);
    }


    void traj_gen()
    {
    drone_xyz_position_cmd += drone_xyz_vel_cmd * 0.01;
    }


void keyboard_subsciber_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // 입력된 키를 문자열로 가져옴
    std::string input = msg->data;

    if (!input.empty()) // 입력 값이 비어있지 않을 경우
    {
        char input_char = input[0]; // 문자열의 첫 번째 문자만 사용

        if (input_char == 'w')
        {
            drone_xyz_vel_cmd[0] += 0.1;
        }
        else if (input_char == 's')
        {
            drone_xyz_vel_cmd[0] -= 0.1;
        }
        else if (input_char == 'a')
        {
            drone_xyz_vel_cmd[1] += 0.1;
        }
        else if (input_char == 'd')
        {
            drone_xyz_vel_cmd[1] -= 0.1;
        }
        else if (input_char == 'e')
        {
            drone_xyz_vel_cmd[2] += 0.1;
        }
        else if (input_char == 'q')
        {
            drone_xyz_vel_cmd[2] -= 0.1;
        }
        else if (input_char == 'x')
        {
            drone_xyz_vel_cmd[0] = 0;
            drone_xyz_vel_cmd[1] = 0;
            drone_xyz_vel_cmd[2] = 0;
        }

        // 현재 명령 출력
      //  RCLCPP_INFO(this->get_logger(), "cmd: [%lf] [%lf] [%lf]",
      //              drone_xyz_vel_cmd[0], drone_xyz_vel_cmd[1], drone_xyz_vel_cmd[2]);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "입력된 키가 없습니다!");
    }
}


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_visual;



  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_; 


  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr drone_cmd_publisher_;




  size_t count_;
  std_msgs::msg::Float64 joint_1_cmd_msg;
  std_msgs::msg::Float64 joint_2_cmd_msg;
  std_msgs::msg::Float64 joint_3_cmd_msg;    
  //TODO:: 아래 세 줄 정의 제대로 하기


  Eigen::Vector3d drone_xyz_position_cmd = Eigen::Vector3d::Zero();
  Eigen::Vector3d drone_xyz_vel_cmd = Eigen::Vector3d::Zero();


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
  rclcpp::spin(std::make_shared<sedas_traj>());
  rclcpp::shutdown();
  return 0;
}
