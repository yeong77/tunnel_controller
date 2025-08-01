#include "rclcpp/rclcpp.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"

class RoverCentralNode : public rclcpp::Node
{
public:
    RoverCentralNode()
    : Node("rover_central_node")
    {
        // GCS 주소 및 포트 설정
        gcs_ip_ = "127.0.0.1"; //192.168.0.60";
        gcs_port_ = 40006;

        // UDP 소켓 생성
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        // GCS 주소 설정
        gcs_addr_.sin_family = AF_INET;
        gcs_addr_.sin_port = htons(gcs_port_);
        gcs_addr_.sin_addr.s_addr = inet_addr(gcs_ip_.c_str());

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensor/transformed_cloud", 10,
        std::bind(&RoverCentralNode::pointCloudCallback, this, std::placeholders::_1));
    }

    ~RoverCentralNode()
    {
        close(sock_fd_);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        uint32_t point_count = msg->width * msg->height;
        if (point_count == 0 || msg->data.empty()) return;

        const size_t point_step = 12;//16;  // x, y, z, intensity 각 4바이트

        size_t total_size = 4 + point_count * point_step;
        std::vector<uint8_t> buffer(total_size);

        // (1) 포인트 수 삽입 (앞 4바이트)
        std::memcpy(buffer.data(), &point_count, 4);

        // (2) 포인트 데이터 삽입 (x, y, z, i 각 4바이트 순서로)
        size_t write_index = 4;
        for (size_t i = 0; i < point_count; ++i)
        {
            const uint8_t* point_data = &msg->data[i * point_step];
            std::memcpy(buffer.data() + write_index, point_data, point_step);
            write_index += point_step;
        }

        // (3) UDP 전송
        ssize_t sent = sendto(sock_fd_, buffer.data(), buffer.size(), 0,
                            (struct sockaddr*)&gcs_addr_, sizeof(gcs_addr_));

        if (sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP send failed");
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ Sent %u points (%zu bytes)", point_count, buffer.size());
        }
    }

    int sock_fd_;
    std::string gcs_ip_;
    int gcs_port_;
    struct sockaddr_in gcs_addr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverCentralNode>());
    rclcpp::shutdown();
    return 0;
}
