#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <random>

class StaticMapPublisher : public rclcpp::Node
{
public:
    StaticMapPublisher()
        : Node("static_map_publisher")
    {
        this->declare_parameter("map_resolution", 0.1);
        this->declare_parameter("map_width", 20.0);
        this->declare_parameter("map_height", 20.0);

        map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/current_elevation_map", 10);

        double resolution = this->get_parameter("map_resolution").as_double();
        double width = this->get_parameter("map_width").as_double();
        double height = this->get_parameter("map_height").as_double();

        grid_map_ = grid_map::GridMap({"elevation"});
        grid_map_.setFrameId("map");
        grid_map_.setGeometry(grid_map::Length(width, height), resolution);

        initializeElevationValues();
        placeBoxes();
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&StaticMapPublisher::publishMap, this));
    }

private:
    void initializeElevationValues()
    {
        // 평평한 지형을 156.0으로 설정
        grid_map_["elevation"].setConstant(156.7);
    }

    void placeBoxes()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        
        // 박스의 좌표는 전체 맵 크기 내에서 무작위 배치 (박스 중심 위치)
        std::uniform_real_distribution<double> dist_x(-9.0, 9.0);  // 맵 크기 고려
        std::uniform_real_distribution<double> dist_y(-9.0, 9.0);
        
        double box_size_x = 0.5; // 박스 가로 크기
        double box_size_y = 0.4; // 박스 세로 크기
        double box_height = 0.5; // 박스 높이

        for (int i = 0; i < 20; i++)
        {
            double box_center_x = dist_x(gen);
            double box_center_y = dist_y(gen);

            for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
            {
                Eigen::Vector2d position;
                grid_map_.getPosition(*it, position);

                // 박스 범위 내인지 확인
                if (std::abs(position.x() - box_center_x) <= box_size_x / 2.0 &&
                    std::abs(position.y() - box_center_y) <= box_size_y / 2.0)
                {
                    grid_map_.at("elevation", *it) = 156.7 + box_height; // 박스 높이 반영
                }
            }
        }
    }

    void publishMap()
    {
        auto map_msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
        map_pub_->publish(*map_msg);
    }

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    grid_map::GridMap grid_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticMapPublisher>());
    rclcpp::shutdown();
    return 0;
}
