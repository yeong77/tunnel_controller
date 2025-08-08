#include "rclcpp/rclcpp.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/bool.hpp>

#include <cstring>
#include <vector>
#include <thread>
#include <chrono>
#include <mqtt/async_client.h>

#include <jsoncpp/json/json.h>

enum class MqttTopic {
    UNKNOWN,
    CONTROL,
    AUTOCONTROL,
    START_INSPECTION,
    INSPECTION_DISTANCE,
    INTERVAL,
    SPEED,
    PARAMETERS,
    GIMBAL1_PITCH,
    GIMBAL2_PITCH,
    CAMERA_CAPTURE1,
    CAMERA_CAPTURE2
};

// class RoverCentralNode;
// void mqtt_listener(RoverCentralNode* node);

class RoverCentralNode : public rclcpp::Node
{
public:
    RoverCentralNode()
    : Node("rover_central_node"),
      mqtt_client_("tcp://localhost:1883", "rover_node_client")
    {
        // GCS 주소 및 포트 설정
        gcs_ip_ = "192.168.0.160"; //"127.0.0.1"; //192.168.0.60" or "localhost"
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

        start_inspection_pub_= this->create_publisher<std_msgs::msg::Bool>("start_inspection", 10);


        start_mqtt_sub();
        mqtt_publish_thread_ = std::thread(&RoverCentralNode::mqttPublish, this);
    }

    ~RoverCentralNode()
    {   
        mqtt_client_.stop_consuming();
        try { mqtt_client_.disconnect()->wait(); } catch (...) {}
        if (mqtt_subscribe_thread_.joinable()) mqtt_subscribe_thread_.join(); 
        if (mqtt_publish_thread_.joinable()) mqtt_publish_thread_.join();
        close(sock_fd_);
    }

    mqtt::async_client mqtt_client_;

private:
    MqttTopic getTopicEnum(const std::string& topic) {
        if (topic == "Rover/Control") return MqttTopic::CONTROL;
        else if (topic == "Rover/AutoControl") return MqttTopic::AUTOCONTROL;
        else if (topic == "Rover/StartInspection") return MqttTopic::START_INSPECTION;
        else if (topic == "Rover/InspectionDistance") return MqttTopic::INSPECTION_DISTANCE;
        else if (topic == "Rover/Interval") return MqttTopic::INTERVAL;
        else if (topic == "Rover/Speed") return MqttTopic::SPEED;
        else if (topic == "Rover/Parameters") return MqttTopic::PARAMETERS;
        else if (topic == "Gimbal/Gimbal1_Pitch") return MqttTopic::GIMBAL1_PITCH;
        else if (topic == "Gimbal/Gimbal2_Pitch") return MqttTopic::GIMBAL2_PITCH;
        else if (topic == "Camera/Capture1") return MqttTopic::CAMERA_CAPTURE1;
        else if (topic == "Camera/Capture2") return MqttTopic::CAMERA_CAPTURE2;
        else return MqttTopic::UNKNOWN;
    }

    void mqttSubscribe(const std::string& topic, const std::string& message) {
        Json::Reader reader;
        Json::Value root;

        if (!reader.parse(message, root)) {
            std::cerr << "Failed to parse JSON\n";
            return;
        }

        switch (getTopicEnum(topic)) {
            case MqttTopic::CONTROL: {
                float linear = root["linear_vel"].asFloat();
                float angular = root["angular_vel"].asFloat();
                std::cout << "Control - Linear: " << linear << ", Angular: " << angular << "\n";
                break;
            }
            case MqttTopic::AUTOCONTROL: {
                bool auto_control = root["auto_control"].asBool();
                std::cout << "Auto Mode: " << std::boolalpha << auto_control << "\n";
                break;
            }
            case MqttTopic::START_INSPECTION: {
                bool start = root["start_inspection"].asBool();

                std_msgs::msg::Bool msg;
                msg.data = start;
                start_inspection_pub_->publish(msg);

                std::cout << "Start Inspection: " << std::boolalpha << start << "\n";
                break;
            }
            case MqttTopic::INSPECTION_DISTANCE: {
                float distance = root["inspection_distance"].asFloat();
                std::cout << "Inspection Distance: " << distance << "\n";
                break;
            }
            case MqttTopic::INTERVAL: {
                float interval = root["interval"].asFloat();
                std::cout << "Capture Interval: " << interval << "\n";
                break;
            }
            case MqttTopic::SPEED: {
                float speed = root["speed"].asFloat();
                std::cout << "Rover Speed: " << speed << "\n";
                break;
            }
            case MqttTopic::PARAMETERS: {
                bool auto_control = root["auto_control"].asBool();
                float speed = root["rover_speed"].asFloat();
                float distance = root["inspection_distance"].asFloat();
                float interval = root["capture_interval"].asFloat();
                std::cout << "Parameters\n";
                std::cout << "   - Auto: " << auto_control << "\n";
                std::cout << "   - Speed: " << speed << "\n";
                std::cout << "   - Distance: " << distance << "\n";
                std::cout << "   - Interval: " << interval << "\n";
                break;
            }

            case MqttTopic::GIMBAL1_PITCH: {
                float pitch = root["gimbal_pitch_position"].asFloat();
                std::cout << "Gimbal 1 Pitch: " << pitch << " deg\n";
                break;
            }
            case MqttTopic::GIMBAL2_PITCH: {
                float pitch = root["gimbal_pitch_position"].asFloat();
                std::cout << "Gimbal 2 Pitch: " << pitch << " deg\n";
                break;
            }
            case MqttTopic::CAMERA_CAPTURE1: {
                bool capture = root["camera_capture"].asBool();
                float angle = root["gimbal_angle"].asFloat();
                std::cout << "Capture1: " << std::boolalpha << capture << " at " << angle << " deg\n";
                break;
            }
            case MqttTopic::CAMERA_CAPTURE2: {
                bool capture = root["camera_capture"].asBool();
                float angle = root["gimbal_angle"].asFloat();
                std::cout << "Capture2: " << std::boolalpha << capture << " at " << angle << " deg\n";
                break;
            }
            default:
                std::cerr << "Unknown topic: " << topic << "\n";
                break;
        }
    }

    void mqttPublish()
    {
        Json::StreamWriterBuilder writer;

        while (rclcpp::ok()) {
            // 1. Rover 상태 publish
            {
                Json::Value status;
                status["rover_connected"] = rover_connected_;
                // status["auto_control"] = auto_control_;
                // status["battery"] = battery_;
                // status["velocity"] = velocity_;
                // status["storage"] = storage_;
                // start_mqtt_pub("Rover/Status/Status", status);
            }

            // // 2. SLAM Pose publish
            // {
            //     Json::Value pose;
            //     pose["x"] = slamState_.position_enu.x();
            //     pose["y"] = slamState_.position_enu.y();
            //     pose["z"] = slamState_.position_enu.z();
            //     pose["qx"] = slamState_.orientation_enu.x();
            //     pose["qy"] = slamState_.orientation_enu.y();
            //     pose["qz"] = slamState_.orientation_enu.z();
            //     pose["qw"] = slamState_.orientation_enu.w();
            //     start_mqtt_pub("Rover/Status/Rover_Pose", pose);
            // }

            // // 3. Gimbal 상태 (예시)
            // {
            //     Json::Value gimbal1;
            //     gimbal1["pitch"] = gimbal1_pitch_deg_;
            //     start_mqtt_pub("Gimbal/Gimba1_Pitch", gimbal1);

            //     Json::Value gimbal2;
            //     gimbal2["pitch"] = gimbal2_pitch_deg_;
            //     start_mqtt_pub("Gimbal/Gimba2_Pitch", gimbal2);
            // }

            // // 4. Camera/CapturePositionArrived
            // {
            //     Json::Value arrived;
            //     arrived["capture_position_arrived"] = capture_position_arrived_;
            //     start_mqtt_pub("Camera/CapturePositionArrived", arrived);
            // }

            // // 5. Camera/ImageSaved
            // {
            //     Json::Value image;
            //     image["camera_num"]   = image_saved_info_.camera_num;
            //     image["pitch_angle"]  = image_saved_info_.pitch_angle;
            //     image["x"]            = image_saved_info_.x;
            //     image["y"]            = image_saved_info_.y;
            //     image["z"]            = image_saved_info_.z;
            //     image["roll"]         = image_saved_info_.roll;
            //     image["pitch"]        = image_saved_info_.pitch;
            //     image["yaw"]          = image_saved_info_.yaw;
            //     image["image_name"]   = image_saved_info_.image_name;
            //     image["image_path"]   = image_saved_info_.image_path;
            //     image["csv_path"]     = image_saved_info_.csv_path;
            //     start_mqtt_pub("Camera/ImageSaved", image);
            // }

            //  // 6. Sensor/IMU_Hz
            // {
            //     Json::Value imu;
            //     imu["Hz"] = imu_hz_;
            //     start_mqtt_pub("Sensor/IMU_Hz", imu);
            // }

            // // 7. Sensor/Lidar_Hz
            // {
            //     Json::Value lidar;
            //     lidar["Hz"] = lidar_hz_;
            //     start_mqtt_pub("Sensor/Lidar_Hz", lidar);
            // }

            // // 8. Sensor/Doppler_Lidar_Hz
            // {
            //     Json::Value doppler;
            //     doppler["Hz"] = doppler_lidar_hz_;
            //     start_mqtt_pub("Sensor/Doppler_Lidar_Hz", doppler);
            // }

            // // 9. Sensor/Distances
            // {
            //     Json::Value dist;
            //     dist["left"]  = sensor_distances_.left;
            //     dist["right"] = sensor_distances_.right;
            //     dist["up"]    = sensor_distances_.up;
            //     start_mqtt_pub("Sensor/Distances", dist);
            // }

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 1초마다 반복
        }
    }

    void start_mqtt_sub()
    {
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);

        try {
            mqtt_client_.connect(connOpts)->wait();
            mqtt_client_.start_consuming();

            mqtt_client_.subscribe("#", 1);
            mqtt_subscribe_thread_ = std::thread([this]() {
                while (rclcpp::ok()) {
                    auto msg = mqtt_client_.consume_message();
                    if (msg)
                        mqttSubscribe(msg->get_topic(), msg->to_string());
                    else
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            });

        } catch (const mqtt::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MQTT connection failed: %s", e.what());
        }
    }
    
    void start_mqtt_pub(const std::string& topic, const Json::Value& payload) {
        if (!mqtt_client_.is_connected()) {
            RCLCPP_WARN(this->get_logger(), "MQTT not connected, cannot publish");
            return;
        }

        Json::StreamWriterBuilder writer;
        std::string message = Json::writeString(writer, payload);

        mqtt::message_ptr pubmsg = mqtt::make_message(topic, message);
        pubmsg->set_qos(1);
        mqtt_client_.publish(pubmsg);
    }

    
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
            RCLCPP_INFO(this->get_logger(), "Sent %u points (%zu bytes)", point_count, buffer.size());
        }
    }

    int sock_fd_;
    std::string gcs_ip_;
    int gcs_port_;
    struct sockaddr_in gcs_addr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    std::thread mqtt_subscribe_thread_;
    std::thread mqtt_publish_thread_;

    bool rover_connected_ = true;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_inspection_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverCentralNode>());
    rclcpp::shutdown();
    return 0;
}
