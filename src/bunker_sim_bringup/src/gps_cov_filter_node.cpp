#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <array>
#include <cmath>

class GpsCovFilterNode : public rclcpp::Node
{
public:
  GpsCovFilterNode()
  : Node("gps_cov_filter_node")
  {
    // Declare parameters
    input_topic_  = this->declare_parameter<std::string>("input_topic",  "/ublox_gps_node/fix");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/ublox_gps_node/fix_filtered");
    cov_threshold_ = this->declare_parameter<double>("cov_threshold", 0.00023); // m^2

    // QoS: GPS는 보통 sensor QoS
    auto qos = rclcpp::SensorDataQoS();

    sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      input_topic_, qos,
      std::bind(&GpsCovFilterNode::onFix, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(output_topic_, qos);

    RCLCPP_INFO(get_logger(),
      "GPS Cov Filter started. threshold=%.6f, sub='%s', pub='%s'",
      cov_threshold_, input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void onFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    ++count_total_;

    // position_covariance: length must be 9 (row-major 3x3)
    if (msg->position_covariance.size() != 9) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000,
                           "Invalid covariance length: %zu", msg->position_covariance.size());
      return;
    }

    const double cxx = msg->position_covariance[0];
    const double cyy = msg->position_covariance[4];
    const double czz = msg->position_covariance[8];

    // NaN/inf 방지 및 음수 variance 방지
    if (!std::isfinite(cxx) || !std::isfinite(cyy) || !std::isfinite(czz) ||
        cxx < 0.0 || cyy < 0.0 || czz < 0.0)
    {
      // 드롭
      return;
    }

    // 하나라도 임계값 초과면 드롭
    if ( (cxx > cov_threshold_) || (cyy > cov_threshold_) || (czz > cov_threshold_) ) {
      return;
    }

    pub_->publish(*msg);
    ++count_pub_;

    if ((count_total_ % 100) == 0) {
      const double rate = (count_total_ > 0) ? (100.0 * count_pub_ / count_total_) : 0.0;
      RCLCPP_INFO(get_logger(), "Filtered publish rate: %.1f%% (%zu/%zu)",
                  rate, count_pub_, count_total_);
    }
  }

  // params
  std::string input_topic_;
  std::string output_topic_;
  double cov_threshold_{0.00023};

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;

  // stats
  size_t count_total_{0};
  size_t count_pub_{0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsCovFilterNode>());
  rclcpp::shutdown();
  return 0;
}
