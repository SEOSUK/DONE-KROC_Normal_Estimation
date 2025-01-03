#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>         // CRBA(관성 행렬 계산)
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <pinocchio/algorithm/jacobian.hpp>    // Jacobian 계산


class PinocchioHandler : public rclcpp::Node {
public:
    PinocchioHandler()
        : Node("pinocchio_handler"), model(), data(model) {
        // URDF 파일 경로

      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


        const std::string urdf_filename = "/home/mrlseuk/ros2_ws/src/ros_gz_project_template/ros_gz_example_description/models/manipulator/model.urdf";
        // Pinocchio 모델 초기화
        try {
            pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model);
            RCLCPP_INFO(this->get_logger(), "Model loaded successfully with nq=%d, nv=%d", model.nq, model.nv);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading URDF: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        data = pinocchio::Data(model);

        // 상태 벡터 및 입력 벡터 초기화
        state_data = Eigen::VectorXd::Zero(model.nq);
        input_data = Eigen::VectorXd::Zero(model.nv);

        // 타이머 설정 (디버깅용, 주기적으로 RNEA 계산)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz = 50ms
            std::bind(&PinocchioHandler::timerCallback, this));

      gravity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pinnochio/gravity", qos_settings);


        // ROS2 구독자 생성
        state_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/state_vector", 10,
            std::bind(&PinocchioHandler::stateCallback, this, std::placeholders::_1));

        input_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/input_vector", 10,
            std::bind(&PinocchioHandler::inputCallback, this, std::placeholders::_1));
    }

private:
    pinocchio::Model model;
    pinocchio::Data data;

    Eigen::VectorXd state_data;
    Eigen::VectorXd input_data;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr input_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_publisher_;






void timerCallback() {
    if (state_data.size() != model.nq || input_data.size() != model.nv) {
        RCLCPP_ERROR(this->get_logger(), "State or input vector size mismatch!");
        return;
    }

    // 속도 및 가속도 초기화
    Eigen::VectorXd velocity = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(model.nv);


    try {
        // RNEA 호출
        Eigen::VectorXd G = pinocchio::rnea(model, data, state_data, velocity, acceleration);

        // 관성 행렬(M) 계산
        pinocchio::crba(model, data, state_data);
        Eigen::MatrixXd M = data.M; // 관성 행렬

        // 결과 출력
        std::stringstream ss_g, ss_m, ss_state;
        ss_g << G;
        ss_state << "\n" << state_data;
        ss_m << "\n" << M;
        RCLCPP_INFO(this->get_logger(), "State vector: %s \n", ss_state.str().c_str());
        RCLCPP_INFO(this->get_logger(), "Gravity vector:\n %s \n", ss_g.str().c_str());
        RCLCPP_INFO(this->get_logger(), "Mass matrix M: %s \n", ss_m.str().c_str());


        std_msgs::msg::Float64MultiArray gravity_msg;

        for (int i =0; i<9; i++)
        {
        gravity_msg.data.push_back(G[i]);
        }

        gravity_publisher_->publish(gravity_msg);

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error during RNEA computation: %s", e.what());
    }
}



    void stateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != model.nq) {
            RCLCPP_WARN(this->get_logger(), "Received state vector size mismatch: %ld (expected %ld)",
                        msg->data.size(), model.nq);
            return;
        }

        for (size_t i = 0; i < msg->data.size(); ++i) {
            state_data[i] = msg->data[i];
        }
    }

    void inputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != model.nv) {
            RCLCPP_WARN(this->get_logger(), "Received input vector size mismatch: %ld (expected %ld)",
                        msg->data.size(), model.nv);
            return;
        }

        for (size_t i = 0; i < msg->data.size(); ++i) {
            input_data[i] = msg->data[i];
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PinocchioHandler>());
    rclcpp::shutdown();
    return 0;
}
