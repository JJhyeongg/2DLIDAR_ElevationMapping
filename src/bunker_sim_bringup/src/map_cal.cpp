#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

// 현장 캘리브레이션 상수 -- 임의 수정 금지
static constexpr double GPS_Z_OFFSET = -157.195 + 0.481;  // map_cal → map (z)
static constexpr double MAP_YAW_DEG  = 8.9;               // map → map_rot (yaw)

class StaticTFPublisher : public rclcpp::Node
{
public:
    StaticTFPublisher()
    : Node("static_tf_pub")
    {
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", true);
        }

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        broadcastTransforms();
    }

private:
    void broadcastTransforms()
    {
        const rclcpp::Time stamp = this->get_clock()->now();

        // 1) map_cal → map  (GPS 고도 오프셋 보정)
        geometry_msgs::msg::TransformStamped t_map;
        t_map.header.stamp       = stamp;
        t_map.header.frame_id    = "map_cal";
        t_map.child_frame_id     = "map";
        t_map.transform.translation.z = GPS_Z_OFFSET;
        t_map.transform.rotation.w    = 1.0;

        // 2) map → map_rot  (로컬 정렬을 위한 yaw 회전)
        geometry_msgs::msg::TransformStamped t_rot;
        t_rot.header.stamp    = stamp;
        t_rot.header.frame_id = "map";
        t_rot.child_frame_id  = "map_rot";

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, MAP_YAW_DEG * M_PI / 180.0);
        t_rot.transform.rotation.x = q.x();
        t_rot.transform.rotation.y = q.y();
        t_rot.transform.rotation.z = q.z();
        t_rot.transform.rotation.w = q.w();

        // 3) map_rot → map_elev  (고도 맵 전용 프레임, 오프셋 없음)
        geometry_msgs::msg::TransformStamped t_elev;
        t_elev.header.stamp    = stamp;
        t_elev.header.frame_id = "map_rot";
        t_elev.child_frame_id  = "map_elev";
        t_elev.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform({t_map, t_rot, t_elev});
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticTFPublisher>());
    rclcpp::shutdown();
    return 0;
}
