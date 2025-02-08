#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fstream>
#include <filesystem>  // C++17 파일 시스템 라이브러리
#include <csignal>  // CTRL+C 신호 처리

namespace fs = std::filesystem;

class KrocCSVLogger : public rclcpp::Node
{
public:
    KrocCSVLogger(const std::string &output_csv)
        : Node("kroc_csv_logger"), output_csv_(output_csv)
    {
        // 경로가 없으면 자동 생성
        fs::path dir_path = fs::path(output_csv_).parent_path();
        if (!fs::exists(dir_path))
        {
            fs::create_directories(dir_path);
            RCLCPP_INFO(this->get_logger(), "Created directory: %s", dir_path.c_str());
        }

        // CSV 파일 열기
        csv_file_.open(output_csv_, std::ios::out | std::ios::app);
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", output_csv_.c_str());
            return;
        }

        // CSV 헤더 작성 (처음 파일 생성 시만)
        if (fs::file_size(output_csv_) == 0)
        {
            csv_file_ << "Timestamp,FK_meas_0,FK_meas_1,FK_meas_2,Body_RPY_Yaw\n";
            csv_file_.flush();
        }

        // Float64MultiArray 메시지 구독
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/Normal_Vector", 10,
            std::bind(&KrocCSVLogger::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Logging to CSV: %s", output_csv_.c_str());
    }

    ~KrocCSVLogger()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV file closed: %s", output_csv_.c_str());
        }
    }

private:
    std::string output_csv_;
    std::ofstream csv_file_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;

    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // 배열 크기에 따라 기본값 설정
        double data_0 = (msg->data.size() > 0) ? msg->data[0] : 0.0;
        double data_1 = (msg->data.size() > 1) ? msg->data[1] : 0.0;
        double data_2 = (msg->data.size() > 2) ? msg->data[2] : 0.0;
        double data_3 = (msg->data.size() > 3) ? msg->data[3] : 0.0;

        // 현재 시간 기록
        auto timestamp = this->now().nanoseconds();

        // CSV 파일에 데이터 기록
        csv_file_ << timestamp << "," 
                << data_0 << "," 
                << data_1 << "," 
                << data_2 << "," 
                << data_3 << "\n";
        csv_file_.flush(); // 즉시 저장
    }

};

// CTRL+C 신호 감지 후 안전 종료
void signal_handler(int signum)
{
    RCLCPP_INFO(rclcpp::get_logger("kroc_csv_logger"), "Caught signal %d, shutting down.", signum);
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 기본 저장 경로 설정
    std::string default_csv_path = "/home/mrlseuk/kroc_data/kroc_data.csv";
    std::string csv_path = (argc >= 2) ? argv[1] : default_csv_path;

    // CTRL+C 핸들러 등록
    std::signal(SIGINT, signal_handler);

    auto node = std::make_shared<KrocCSVLogger>(csv_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
