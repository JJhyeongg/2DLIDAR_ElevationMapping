#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <fstream>
#include <cmath>
#include <chrono>

class MultiMapSubscriberNode : public rclcpp::Node
{
public:
  MultiMapSubscriberNode()
  : Node("multi_map_subscriber_node")
  {
    using std::placeholders::_1;
    using namespace std::chrono_literals;

    // 원본 맵 구독
    sub_dual_layer_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "/elevation_map_dual_layer", 10,
        std::bind(&MultiMapSubscriberNode::callbackDualLayer, this, _1));

    sub_max_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "/elevation_map_max", 10,
        std::bind(&MultiMapSubscriberNode::callbackMax, this, _1));

    sub_kalman_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "/elevation_map_kalman", 10,
        std::bind(&MultiMapSubscriberNode::callbackKalman, this, _1));

    // 재발행용 퍼블리셔 (원하는 토픽 이름으로 수정 가능)
    pub_dual_layer_repub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "/elevation_map_dual_layer_repub", rclcpp::QoS(1).transient_local());
    pub_max_repub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "/elevation_map_max_repub", rclcpp::QoS(1).transient_local());
    pub_kalman_repub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "/elevation_map_kalman_repub", rclcpp::QoS(1).transient_local());

    // 1초마다 CSV 저장 (기존 기능 유지)
    save_timer_ = this->create_wall_timer(
        1s,
        std::bind(&MultiMapSubscriberNode::saveAllMapsToFile, this));

    // 0.1초마다 재발행
    repub_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&MultiMapSubscriberNode::republishAllMaps, this));

    RCLCPP_INFO(this->get_logger(),
                "Subscribed to /elevation_map_dual_layer, /elevation_map_max, /elevation_map_kalman");
  }

private:
  // ----- 구독 콜백 -----
  void callbackDualLayer(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_map::GridMapRosConverter::fromMessage(*msg, global_map_dual_layer_);
    header_dual_layer_ = msg->header;
    map_dual_layer_received_ = true;
  }

  void callbackMax(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_map::GridMapRosConverter::fromMessage(*msg, global_map_max_);
    header_max_ = msg->header;
    map_max_received_ = true;
  }

  void callbackKalman(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_map::GridMapRosConverter::fromMessage(*msg, global_map_kalman_);
    header_kalman_ = msg->header;
    map_kalman_received_ = true;
  }

  // ----- 재발행 타이머 콜백 -----
  void republishAllMaps()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = this->now();

    if (!map_dual_layer_received_ && !map_max_received_ && !map_kalman_received_) {
      // 아직 아무 맵도 안 들어온 경우
      return;
    }

    if (map_dual_layer_received_) {
      auto msg = grid_map::GridMapRosConverter::toMessage(global_map_dual_layer_);
      //msg.header = header_dual_layer_;
      //msg.header.stamp = now;  // 타임스탬프 갱신
      pub_dual_layer_repub_->publish(*msg);
    }

    if (map_max_received_) {
        auto msg = grid_map::GridMapRosConverter::toMessage(global_map_max_);
      //msg.header = header_max_;
      //msg.header.stamp = now;
      pub_max_repub_->publish(*msg);
    }

    if (map_kalman_received_) {
      auto msg = grid_map::GridMapRosConverter::toMessage(global_map_kalman_);
      //msg.header = header_kalman_;
      //msg.header.stamp = now;
      pub_kalman_repub_->publish(*msg);
    }
  }

  // ----- CSV 저장 -----
  void saveAllMapsToFile()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!map_dual_layer_received_ && !map_max_received_ && !map_kalman_received_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "No map data received yet. CSV not saved.");
      return;
    }

    if (map_dual_layer_received_) {
      saveOneMapToCSV(global_map_dual_layer_, "global_map_data_dual_layer.csv");
    }
    if (map_max_received_) {
      saveOneMapToCSV(global_map_max_, "global_map_data_max.csv");
    }
    if (map_kalman_received_) {
      saveOneMapToCSV(global_map_kalman_, "global_map_data_kalman.csv");
    }
  }

  void saveOneMapToCSV(const grid_map::GridMap &map, const std::string &filename)
  {
    std::ofstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
      return;
    }

    // 헤더 정보 저장
    file << "resolution," << map.getResolution() << std::endl;
    file << "width,"      << map.getLength().x() << std::endl;
    file << "height,"     << map.getLength().y() << std::endl;
    file << "position_x," << map.getPosition().x() << std::endl;
    file << "position_y," << map.getPosition().y() << std::endl;

    // 데이터 헤더
    file << "x,y,elevation" << std::endl;

    // 데이터 저장
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      map.getPosition(*it, position);
      float elevation = map.at("elevation", *it);

      file << position.x() << "," << position.y() << ",";

      if (std::isnan(elevation)) {
        file << "nan";
      } else {
        file << elevation;
      }

      file << std::endl;
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Map data saved to %s", filename.c_str());
  }

  // ----- 멤버 변수 -----
  std::mutex mutex_;

  rclcpp::TimerBase::SharedPtr save_timer_;
  rclcpp::TimerBase::SharedPtr repub_timer_;

  grid_map::GridMap global_map_dual_layer_;
  grid_map::GridMap global_map_max_;
  grid_map::GridMap global_map_kalman_;

  std_msgs::msg::Header header_dual_layer_;
  std_msgs::msg::Header header_max_;
  std_msgs::msg::Header header_kalman_;

  bool map_dual_layer_received_{false};
  bool map_max_received_{false};
  bool map_kalman_received_{false};

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_dual_layer_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_max_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_kalman_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_dual_layer_repub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_max_repub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_kalman_repub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiMapSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
