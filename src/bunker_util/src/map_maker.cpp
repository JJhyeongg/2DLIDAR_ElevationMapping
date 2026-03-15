#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <Eigen/Dense>

class KalmanMapUpdater : public rclcpp::Node
{
public:
    KalmanMapUpdater() 
        : Node("kalman_map_updater")
    {
        // 맵의 해상도와 크기 매개변수 선언
        this->declare_parameter("map_resolution", 0.1);
        this->declare_parameter("map_width", 30.0);
        this->declare_parameter("map_height", 30.0);

        // 구독자와 퍼블리셔 초기화
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", 10, std::bind(&KalmanMapUpdater::pointCloudCallback, this, std::placeholders::_1));
        map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/current_elevation_map", 10);

        // 그리드 맵 초기화
        double resolution = this->get_parameter("map_resolution").as_double();
        double width = this->get_parameter("map_width").as_double();
        double height = this->get_parameter("map_height").as_double();

        grid_map_ = grid_map::GridMap({"elevation", "variance", "updated"});
        grid_map_.setFrameId("map");
        grid_map_.setGeometry(grid_map::Length(width, height), resolution);
        grid_map_["elevation"].setConstant(0.0);
        grid_map_["variance"].setConstant(1.0);  // 초기 분산

    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg)
    {
        // 각 포인트에 대해 반복
        for (sensor_msgs::PointCloud2ConstIterator<float> it_x(*pointcloud_msg, "x"),
             it_y(*pointcloud_msg, "y"), it_z(*pointcloud_msg, "z"), it_var(*pointcloud_msg, "intensity");
             it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_var)
        {
            if (std::isnan(*it_x) || std::isnan(*it_y) || std::isnan(*it_z)) {
                continue;  // 유효하지 않은 포인트 건너뜀
            }

            grid_map::Position position(*it_x, *it_y);
            if (grid_map_.isInside(position))
            {
                float h_pi = *it_z;              // 측정된 높이
                float variance_pi = *it_var;     // 측정된 분산

                float& h_ei = grid_map_.atPosition("elevation", position); // 맵의 높이 값
                float& variance_ei = grid_map_.atPosition("variance", position); // 맵의 분산 값
                if(h_pi==0){
                    continue;
                }

                if (variance_ei == 1.0)  // 초기 분산이 1.0일 때 NULL로 간주
                {
                    h_ei = h_pi;
                    variance_ei = variance_pi;

                }
                else
                {
                    // Mahalanobis 거리 계산
                    float mahalanobis_distance = std::abs(h_ei - h_pi) / std::sqrt(variance_ei);

                    // Mahalanobis 거리 임계값 비교
                    const float threshold = 0.2; // 임계값 설정
                    if (mahalanobis_distance > threshold && h_ei < h_pi)
                    {
                        // 측정값을 맵의 값으로 대체
                        h_ei = h_pi;
                        variance_ei = variance_pi;
      
                        RCLCPP_INFO(this->get_logger(), "Cell updated with new height%f",mahalanobis_distance);
                    }
                    else
                    {
                        // Kalman 필터 업데이트
                        float kalman_gain = variance_ei / (variance_ei + variance_pi);
                        h_ei = h_ei * (1 - kalman_gain) + kalman_gain * h_pi;
                        variance_ei = (variance_ei * variance_pi) / (variance_ei + variance_pi);
                        //RCLCPP_INFO(this->get_logger(), "Kalman filter update applied");
                    }
                }
            }
        }
        publishMap();
    }

    void publishMap()
    {
        auto map_msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
        map_pub_->publish(*map_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
    grid_map::GridMap grid_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanMapUpdater>());
    rclcpp::shutdown();
    return 0;
}
