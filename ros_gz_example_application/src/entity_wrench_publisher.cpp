#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/entity_wrench.hpp"
#include <geometry_msgs/msg/vector3.hpp>

class WrenchPublisher : public rclcpp::Node
{
public:
    WrenchPublisher()
        : Node("wrench_publisher")
    {
        // ROS2 Publisher 생성
        publisher_ = this->create_publisher<ros_gz_interfaces::msg::EntityWrench>("/manipulator/wrench", 10);

        // 타이머 생성 (0.1초 간격으로 퍼블리시)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WrenchPublisher::publish_wrench, this));

        // 초기 값 설정
        force_.x = 0.0;
        force_.y = 0.0;
        force_.z = -100.0; // 기본 Z 방향 힘
        torque_.x = 0.0;
        torque_.y = 0.0;
        torque_.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "WrenchPublisher node initialized");
    }

    void set_force(double x, double y, double z)
    {
        force_.x = x;
        force_.y = y;
        force_.z = z;
    }

    void set_torque(double x, double y, double z)
    {
        torque_.x = x;
        torque_.y = y;
        torque_.z = z;
    }

private:
    void publish_wrench()
    {
        auto message = ros_gz_interfaces::msg::EntityWrench();

        // 메시지 구성
        message.header.stamp = this->now();
        message.header.frame_id = "world";
        message.entity.name = "manipulator::link_drone";
        message.entity.type = ros_gz_interfaces::msg::Entity::LINK;

        message.wrench.force = force_;
        message.wrench.torque = torque_;

        // 퍼블리시
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published wrench: force[%.2f, %.2f, %.2f], torque[%.2f, %.2f, %.2f]",
                    force_.x, force_.y, force_.z, torque_.x, torque_.y, torque_.z);
    }

    rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Vector3 force_;
    geometry_msgs::msg::Vector3 torque_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WrenchPublisher>();

    // 제어기를 통해 Wrench를 동적으로 변경 가능 (예: 입력값을 받을 수 있음)
    node->set_force(0.0, 0.0, -50.0); // 예제: Z 방향 힘 변경
    node->set_torque(0.0, 0.0, 10.0); // 예제: Z 방향 토크 변경

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

