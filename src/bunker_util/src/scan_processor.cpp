/*
 * scan_processor.cpp
 *
 * 역할: LiDAR 스캔 + 오도메트리 동기화 → 3D 고도 맵 생성
 *
 * 알고리즘 선택 파라미터:
 *   enable_map_max  (bool, default: true)  - max-z 맵 발행
 *   enable_map_kalman      (bool, default: true)  - Kalman 맵 발행
 *   enable_map_dual_layer  (bool, default: true)  - DualLayer 맵 발행 (메인)
 *   enable_debug_csv (bool, default: false) - CSV 디버그 로그
 */

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <cstring>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <mutex>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <fstream>

// 알고리즘 헤더 (새 아키텍처)
#include "bunker_util/elevation_types.hpp"
#include "bunker_util/elevation_map_max.hpp"
#include "bunker_util/elevation_map_kalman.hpp"
#include "bunker_util/elevation_map_dual_layer.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using bunker_elevation::ElevPoint;
using bunker_elevation::GroundBand;
using bunker_elevation::MapParams;

class OdomScanSyncNode : public rclcpp::Node
{
public:
  OdomScanSyncNode() : Node("odom_scan_sync_node")
  {
    initializeParameters();
    initializeSubscribers();
    initializePublisher();
    initializeTransforms();
    initializeMaps();
  }

private:
  // ==================== 초기화 ====================

  void initializeParameters()
  {
    sync_limit_time_   = declare_parameter<double>("sync_limit_time", 0.05);

    // LiDAR extrinsic (lidar → base_footprint)
    bl_roll_  = declare_parameter<double>("bl_roll",  0.0);
    bl_pitch_ = declare_parameter<double>("bl_pitch", -M_PI / 6.0);
    bl_yaw_   = declare_parameter<double>("bl_yaw",   M_PI);
    bl_tx_    = declare_parameter<double>("bl_tx",    0.2817);
    bl_ty_    = declare_parameter<double>("bl_ty",    0.0);
    bl_tz_    = declare_parameter<double>("bl_tz",    0.3938);

    // odom z 오프셋 (GPS 고도 보정)
    odom_z_offset_ = declare_parameter<double>("odom_z_offset", -157.195 + 0.481);

    // 맵 기하
    map_resolution_   = declare_parameter<double>("map_resolution",   0.1);
    map_width_        = declare_parameter<double>("map_width",        50.0);
    map_height_       = declare_parameter<double>("map_height",       50.0);
    max_distance_     = declare_parameter<double>("max_distance",     2.0);
    default_meas_var_ = declare_parameter<double>("default_meas_var", 0.01);

    // 프레임 이름
    map_frame_         = declare_parameter<std::string>("map_frame",         "map");
    base_frame_        = declare_parameter<std::string>("base_frame",        "calibrated_basefootprint");
    rotated_map_frame_ = declare_parameter<std::string>("rotated_map_frame", "map_elev");

    // 어떤 맵을 활성화할지 선택
    enable_map_max_ = declare_parameter<bool>("enable_map_max", true);
    enable_map_kalman_  = declare_parameter<bool>("enable_map_kalman",  true);
    enable_map_dual_layer_  = declare_parameter<bool>("enable_map_dual_layer",  true);

    // CSV 디버그 로그
    enable_debug_csv_ = declare_parameter<bool>("enable_debug_csv", false);

    // EKF 안정화 대기
    odom_stable_threshold_ = declare_parameter<double>("odom_stable_threshold", 0.5);  // m/frame
    odom_stable_min_count_ = declare_parameter<int>("odom_stable_min_count", 10);      // 연속 안정 프레임 수

    if (enable_debug_csv_) {
      debug_file_.open("debug_all_scans.csv", std::ios::out);
      if (debug_file_.is_open()) {
        debug_file_ << "scan_id,theta,range,x_l,y_l,x,y,z,label\n";
        RCLCPP_INFO(get_logger(), "CSV 로그 활성화: debug_all_scans.csv");
      } else {
        RCLCPP_ERROR(get_logger(), "CSV 파일 열기 실패");
        enable_debug_csv_ = false;
      }
    }
  }

  void initializeSubscribers()
  {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odom_sub_.subscribe(this, "/odometry/global");
    scan_sub_.subscribe(this, "/tilt_lidar_scan");

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry,
        sensor_msgs::msg::LaserScan>;

    odom_scan_sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), odom_sub_, scan_sub_);
    odom_scan_sync_->setAgePenalty(0.1);
    odom_scan_sync_->registerCallback(&OdomScanSyncNode::onSyncCallback, this);
  }

  void initializePublisher()
  {
    tf_broadcaster_       = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    laser_scan_pub_       = create_publisher<sensor_msgs::msg::LaserScan>("/simulated_lidar_scan", 10);
    pointcloud_pub_       = create_publisher<sensor_msgs::msg::PointCloud2>("/pcl", 10);
    cluster_cloud_pub_    = create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_clusters", 10);
    elev_marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/elevation_markers_array", 10);

    if (enable_map_max_) {
      map_pub_max_ = create_publisher<grid_map_msgs::msg::GridMap>("/elevation_map_max", 10);
      RCLCPP_INFO(get_logger(), "맵 발행 활성화: /elevation_map_max");
    }
    if (enable_map_kalman_) {
      map_pub_kalman_ = create_publisher<grid_map_msgs::msg::GridMap>("/elevation_map_kalman", 10);
      RCLCPP_INFO(get_logger(), "맵 발행 활성화: /elevation_map_kalman");
    }
    if (enable_map_dual_layer_) {
      map_pub_dual_layer_ = create_publisher<grid_map_msgs::msg::GridMap>("/elevation_map_dual_layer", 10);
      RCLCPP_INFO(get_logger(), "맵 발행 활성화: /elevation_map_dual_layer");
    }
  }

  void initializeTransforms()
  {
    tf2::Quaternion q;
    q.setRPY(bl_roll_, bl_pitch_, bl_yaw_);
    T_lidar_to_base_.setOrigin(tf2::Vector3(bl_tx_, bl_ty_, bl_tz_));
    T_lidar_to_base_.setRotation(q);

    // map → rotated_map_frame TF 캐싱 (성공할 때까지 재시도)
    while (rclcpp::ok() && !has_T_map_to_rot_) {
      try {
        auto tf_stamped = tf_buffer_->lookupTransform(
            rotated_map_frame_, map_frame_,
            tf2::TimePointZero, tf2::durationFromSec(1.0));

        tf2::fromMsg(tf_stamped.transform.rotation, q);
        T_map_to_rot_.setOrigin(tf2::Vector3(
            tf_stamped.transform.translation.x,
            tf_stamped.transform.translation.y,
            tf_stamped.transform.translation.z));
        T_map_to_rot_.setRotation(q);
        has_T_map_to_rot_ = true;

        RCLCPP_INFO(get_logger(), "TF 캐시 완료: %s ← %s",
                    rotated_map_frame_.c_str(), map_frame_.c_str());
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "TF 대기 중: %s", ex.what());
        rclcpp::sleep_for(std::chrono::milliseconds(200));
      }
    }

    // RPLIDAR A3 각도 노이즈 (균등분포 모델)
    constexpr double angle_res_deg = 0.225;
    lidar_angle_sigma_ = (angle_res_deg * M_PI / 180.0) / std::sqrt(12.0);
  }

  void initializeMaps()
  {
    MapParams p;
    p.resolution      = map_resolution_;
    p.width           = map_width_;
    p.height          = map_height_;
    p.frame_id        = rotated_map_frame_;
    p.default_meas_var = default_meas_var_;

    if (enable_map_max_) map_max_ = std::make_unique<bunker_elevation::ElevationMapMax>(p);
    if (enable_map_kalman_)  map_kalman_  = std::make_unique<bunker_elevation::ElevationMapKalman>(p);
    if (enable_map_dual_layer_)  map_dual_layer_  = std::make_unique<bunker_elevation::ElevationMapDualLayer>(p);
  }

  // ==================== 메인 콜백 ====================

  void onSyncCallback(
      const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
      const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
  {
    // 1) 타임스탬프 차이 검사
    const double time_diff =
        (rclcpp::Time(scan->header.stamp) - rclcpp::Time(odom->header.stamp)).seconds();
    if (std::fabs(time_diff) > sync_limit_time_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "시간 차이 초과: %.4f s (한계=%.3f)", time_diff, sync_limit_time_);
      return;
    }

    // 1-b) EKF 안정화 확인
    if (!odom_ready_) {
      const tf2::Vector3 cur_pos(
          odom->pose.pose.position.x,
          odom->pose.pose.position.y,
          odom->pose.pose.position.z);

      if (has_prev_odom_pos_) {
        const double delta = (cur_pos - prev_odom_pos_).length();
        if (delta > odom_stable_threshold_) {
          odom_stable_count_ = 0;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "EKF 불안정 (delta=%.3f m) — 안정화 대기 중", delta);
        } else {
          ++odom_stable_count_;
          if (odom_stable_count_ >= odom_stable_min_count_) {
            odom_ready_ = true;
            RCLCPP_INFO(get_logger(), "EKF 안정화 완료 — 맵핑 시작");
          }
        }
      }
      prev_odom_pos_    = cur_pos;
      has_prev_odom_pos_ = true;
      return;
    }

    // 2) map ← base 변환 구성
    const auto & pos = odom->pose.pose.position;
    const auto & ori = odom->pose.pose.orientation;
    tf2::Quaternion q_base;
    tf2::fromMsg(ori, q_base);

    tf2::Transform T_map_base(
        q_base,
        tf2::Vector3(pos.x, pos.y, pos.z + odom_z_offset_));

    // 3) map ← lidar 변환
    tf2::Transform T_map_lidar = T_map_base * T_lidar_to_base_;

    // 4) LaserScan → ElevPoint 변환
    std::vector<ElevPoint> elev_points;
    std::vector<tf2::Vector3> map_points;  // map 프레임 (PCL 발행용)
    buildElevPoints(*scan, T_map_lidar, *odom, elev_points, map_points);

    // 5) 수평/수직 라벨 분류 (v2가 활성화된 경우)
    std::vector<int> labels;
    if (enable_map_dual_layer_) {
      labels = classifyVerticalHorizontal(elev_points, 5, 0.05);
    }

    // 6) 활성화된 맵 업데이트
    if (enable_map_max_ && map_max_) map_max_->update(elev_points, labels);
    if (enable_map_kalman_  && map_kalman_)  map_kalman_->update(elev_points, labels);
    if (enable_map_dual_layer_  && map_dual_layer_)  map_dual_layer_->update(elev_points, labels);

    // 7) 색상 PCL 발행 (라벨 시각화)
    publishClusterCloud(elev_points, labels, scan->header.stamp);

    // 8) map 프레임 PCL 발행
    publishPointCloud(map_points, scan->header.stamp);

    // 9) 보정된 base frame TF 발행
    publishCorrectedTF(T_map_base, scan->header.stamp);

    // 10) 입력 스캔 재발행 (프레임만 교체)
    auto scan_out = *scan;
    scan_out.header.frame_id = "calibrated_lidar";
    laser_scan_pub_->publish(scan_out);

    // 11) 활성화된 맵 발행
    const rclcpp::Time stamp = scan->header.stamp;
    if (enable_map_max_ && map_pub_max_ && map_max_) {
      map_pub_max_->publish(*grid_map::GridMapRosConverter::toMessage(map_max_->getMap()));
    }
    if (enable_map_kalman_ && map_pub_kalman_ && map_kalman_) {
      map_pub_kalman_->publish(*grid_map::GridMapRosConverter::toMessage(map_kalman_->getMap()));
    }
    if (enable_map_dual_layer_ && map_pub_dual_layer_ && map_dual_layer_) {
      map_pub_dual_layer_->publish(*grid_map::GridMapRosConverter::toMessage(map_dual_layer_->getMap()));
      publishElevationMarkerArray(map_dual_layer_->getMap(), stamp);
    }

    // 12) CSV 로그
    if (enable_debug_csv_) {
      writeDebugCSV(elev_points, labels);
    }
  }

  // ==================== 스캔 변환 ====================

  void buildElevPoints(
      const sensor_msgs::msg::LaserScan & scan,
      const tf2::Transform & T_map_lidar,
      const nav_msgs::msg::Odometry & odom,
      std::vector<ElevPoint> & elev_points,
      std::vector<tf2::Vector3> & map_points)
  {
    // 각도 재정렬: [90°~180°] 뒤 [-180°~-90°] (LiDAR 틸트 구조)
    std::vector<int> reorder_idx;
    reorder_idx.reserve(scan.ranges.size());

    float angle = scan.angle_min;
    std::vector<int> idx_pos, idx_neg;
    for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {
      if (angle >= M_PI / 2 - 0.1 && angle <= M_PI)  idx_pos.push_back(i);
      else if (angle <= -M_PI / 2 + 0.1 && angle >= -M_PI) idx_neg.push_back(i);
    }
    reorder_idx.insert(reorder_idx.end(), idx_pos.begin(), idx_pos.end());
    reorder_idx.insert(reorder_idx.end(), idx_neg.begin(), idx_neg.end());

    elev_points.reserve(reorder_idx.size());
    map_points.reserve(reorder_idx.size());

    for (int idx : reorder_idx) {
      float r     = scan.ranges[idx];
      float theta = scan.angle_min + idx * scan.angle_increment;

      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max || r > max_distance_) {
        continue;
      }

      float x_l = r * std::cos(theta);
      float y_l = r * std::sin(theta);

      tf2::Vector3 p_lidar(x_l, y_l, 0.0f);
      tf2::Vector3 p_map  = T_map_lidar * p_lidar;
      tf2::Vector3 p_rot  = T_map_to_rot_ * p_map;

      map_points.push_back(p_map);

      ElevPoint ep;
      ep.x     = p_rot.x();
      ep.y     = p_rot.y();
      ep.z     = p_rot.z();
      ep.r     = static_cast<double>(r);
      ep.theta = static_cast<double>(theta);
      ep.var_z = computeZVarianceFull(odom, ep.r, ep.theta);
      ep.x_l   = x_l;
      ep.y_l   = y_l;

      elev_points.push_back(ep);
    }
  }

  // ==================== 수평/수직 분류 ====================

  std::vector<double> computePhiInLidarXY_Window(
      const std::vector<ElevPoint> & pts,
      int K,
      double gap_thr)
  {
    const size_t N = pts.size();
    std::vector<double> phi(N, 0.0);
    if (N < 3) return phi;

    const double gap2 = gap_thr * gap_thr;

    auto dist2 = [&](size_t i, size_t j) {
      double dx = pts[j].x_l - pts[i].x_l;
      double dy = pts[j].y_l - pts[i].y_l;
      return dx * dx + dy * dy;
    };

    for (size_t i = 0; i < N; ++i) {
      int L = std::max(0, static_cast<int>(i) - K);
      int R = std::min(static_cast<int>(N) - 1, static_cast<int>(i) + K);

      // 세그먼트 내 gap 검사
      bool bad = false;
      for (int j = L; j < R; ++j) {
        if (dist2(j, j + 1) > gap2) { bad = true; break; }
      }
      if (bad || R - L + 1 < 2) { phi[i] = 0.0; continue; }

      int count = R - L + 1;
      double mx = 0.0, my = 0.0;
      for (int j = L; j <= R; ++j) { mx += pts[j].x_l; my += pts[j].y_l; }
      mx /= count; my /= count;

      double Sxx = 0.0, Syy = 0.0, Sxy = 0.0;
      for (int j = L; j <= R; ++j) {
        double dx = pts[j].x_l - mx;
        double dy = pts[j].y_l - my;
        Sxx += dx * dx;
        Syy += dy * dy;
        Sxy += dx * dy;
      }

      if (Sxx + Syy < 1e-12) { phi[i] = 0.0; continue; }

      phi[i] = 0.5 * std::atan2(2.0 * Sxy, Sxx - Syy);
    }

    return phi;
  }

  GroundBand estimateGroundBandFromHorizontalZ(
      const std::vector<ElevPoint> & pts,
      const std::vector<int> & ori_labels)
  {
    GroundBand gb;
    const size_t N = pts.size();
    if (N == 0 || ori_labels.size() != N) return gb;

    struct HPoint { double x, y, z; size_t idx; };
    std::vector<HPoint> horiz;
    horiz.reserve(N);

    double global_z_min =  std::numeric_limits<double>::infinity();
    double global_z_max = -std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < N; ++i) {
      if (ori_labels[i] != 1) continue;
      const auto & p = pts[i];
      horiz.push_back({p.x, p.y, p.z, i});
      if (p.z < global_z_min) global_z_min = p.z;
      if (p.z > global_z_max) global_z_max = p.z;
    }

    constexpr size_t MIN_HORIZ = 30;
    if (horiz.size() < MIN_HORIZ) return gb;

    // 2D 격자 기반 connected component (8-이웃 BFS)
    constexpr double CLUSTER_RES = 0.05;

    auto gridKey = [&](double x, double y) -> long long {
      int ix = static_cast<int>(std::floor(x / CLUSTER_RES));
      int iy = static_cast<int>(std::floor(y / CLUSTER_RES));
      return (static_cast<long long>(ix) << 32) ^
             static_cast<unsigned long long>(static_cast<unsigned int>(iy));
    };

    std::unordered_map<long long, std::vector<size_t>> cell_pts;
    cell_pts.reserve(horiz.size());
    for (size_t i = 0; i < horiz.size(); ++i) {
      cell_pts[gridKey(horiz[i].x, horiz[i].y)].push_back(i);
    }

    struct Patch { std::vector<size_t> h_indices; };
    std::vector<Patch> patches;
    std::unordered_set<long long> visited;
    std::queue<long long> bfs;

    const int DX[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
    const int DY[8] = {-1, 0, 1,-1, 1,-1, 0, 1};

    for (const auto & kv : cell_pts) {
      if (visited.count(kv.first)) continue;

      Patch patch;
      bfs.push(kv.first);
      visited.insert(kv.first);

      while (!bfs.empty()) {
        long long ck = bfs.front(); bfs.pop();

        auto it = cell_pts.find(ck);
        if (it != cell_pts.end()) {
          patch.h_indices.insert(patch.h_indices.end(),
                                 it->second.begin(), it->second.end());
        }

        int cx = static_cast<int>(ck >> 32);
        int cy = static_cast<int>(static_cast<unsigned int>(ck));
        for (int k = 0; k < 8; ++k) {
          long long nk = (static_cast<long long>(cx + DX[k]) << 32) ^
                         static_cast<unsigned long long>(
                             static_cast<unsigned int>(cy + DY[k]));
          if (!cell_pts.count(nk) || visited.count(nk)) continue;
          visited.insert(nk);
          bfs.push(nk);
        }
      }

      if (!patch.h_indices.empty()) patches.push_back(std::move(patch));
    }

    if (patches.empty()) return gb;

    // 각 패치 통계 → 가장 낮고 충분히 큰 패치 선택
    struct PStats { size_t count; double mean_z, min_z, max_z; };
    std::vector<PStats> stats;
    size_t max_count = 0;

    for (const auto & p : patches) {
      double sum = 0.0, zmin = 1e9, zmax = -1e9;
      for (size_t hi : p.h_indices) {
        double z = horiz[hi].z;
        sum += z;
        if (z < zmin) zmin = z;
        if (z > zmax) zmax = z;
      }
      stats.push_back({p.h_indices.size(), sum / p.h_indices.size(), zmin, zmax});
      if (p.h_indices.size() > max_count) max_count = p.h_indices.size();
    }

    constexpr double BIG_RATIO = 0.3;
    constexpr size_t MIN_MAIN  = 30;
    int best = -1;
    double best_z = std::numeric_limits<double>::infinity();

    for (size_t si = 0; si < stats.size(); ++si) {
      const auto & s = stats[si];
      if (s.count < MIN_MAIN) continue;
      if (s.count < static_cast<size_t>(BIG_RATIO * max_count)) continue;
      if (s.mean_z < best_z) { best_z = s.mean_z; best = static_cast<int>(si); }
    }

    if (best < 0) {
      for (size_t si = 0; si < stats.size(); ++si) {
        const auto & s = stats[si];
        if (s.count < MIN_MAIN) continue;
        if (s.mean_z < best_z) { best_z = s.mean_z; best = static_cast<int>(si); }
      }
    }

    if (best < 0) return gb;

    constexpr double MARGIN_UP = 0.01;
    double band_max = stats[best].max_z + MARGIN_UP;
    if (band_max <= global_z_min) return gb;

    gb.valid = true;
    gb.z_min = global_z_min;
    gb.z_max = band_max;
    return gb;
  }

  std::vector<int> classifyVerticalHorizontal(
      const std::vector<ElevPoint> & pts,
      int K,
      double gap_thr)
  {
    const size_t N = pts.size();
    std::vector<int> labels(N, -1);
    if (N == 0) return labels;

    auto phi = computePhiInLidarXY_Window(pts, K, gap_thr);

    // 1차: 기하적 수직/수평 판별
    std::vector<int> ori(N, -1);
    constexpr double TH_VERT_DEG = 45.0;
    for (size_t i = 0; i < N; ++i) {
      double d = std::fabs(phi[i] * 180.0 / M_PI);
      ori[i] = (d <= TH_VERT_DEG) ? 0 : 1;
    }

    // 2차: 수평점으로 지면 대역 추정
    GroundBand gb = estimateGroundBandFromHorizontalZ(pts, ori);

    if (!gb.valid) {
      for (size_t i = 0; i < N; ++i) {
        labels[i] = (ori[i] == 0) ? 0 : (ori[i] == 1 ? 1 : -1);
      }
      return labels;
    }

    // 3차: ground / upper 분리
    constexpr double GROUND_MARGIN = 0.04;
    constexpr double UPPER_GAP_Z   = 0.01;

    for (size_t i = 0; i < N; ++i) {
      if (ori[i] == 0) { labels[i] = 0; continue; }
      if (ori[i] == -1) { labels[i] = -1; continue; }

      double z = pts[i].z;
      if (z >= gb.z_min - GROUND_MARGIN && z <= gb.z_max + GROUND_MARGIN) {
        labels[i] = 1;
      } else if (z > gb.z_max + UPPER_GAP_Z) {
        labels[i] = 2;
      } else {
        labels[i] = 1;
      }
    }

    return labels;
  }

  // ==================== Z 분산 계산 ====================

  double computeRangeSigma(double r) const
  {
    double rel = (r <= 3.0) ? 0.01 : (r <= 5.0) ? 0.02 : 0.025;
    double sigma = rel * r;
    return (sigma < 0.01) ? 0.01 : sigma;
  }

  double computeZFromPose(double x, double y, double z,
                          double roll, double pitch, double yaw,
                          double r, double theta) const
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2::Transform T_map_base(q, tf2::Vector3(x, y, z + odom_z_offset_));
    tf2::Transform T_map_lidar = T_map_base * T_lidar_to_base_;

    tf2::Vector3 p_lidar(r * std::cos(theta), r * std::sin(theta), 0.0);
    tf2::Vector3 p_rot = T_map_to_rot_ * (T_map_lidar * p_lidar);
    return p_rot.z();
  }

  double computeZVarianceFull(const nav_msgs::msg::Odometry & odom,
                              double r, double theta) const
  {
    const auto & pos = odom.pose.pose.position;
    const auto & ori = odom.pose.pose.orientation;
    tf2::Quaternion q;
    tf2::fromMsg(ori, q);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double x = pos.x, y = pos.y, z = pos.z;

    // Jacobian: 포즈 6자유도 수치 미분
    double J[6];
    constexpr double dp = 1e-3, da = 1e-4;
    auto dz = [&](int idx, double delta) -> double {
      double xp = x, yp = y, zp = z, rp = roll, pp = pitch, wp = yaw;
      switch (idx) {
        case 0: xp += delta; break;
        case 1: yp += delta; break;
        case 2: zp += delta; break;
        case 3: rp += delta; break;
        case 4: pp += delta; break;
        case 5: wp += delta; break;
      }
      return computeZFromPose(xp, yp, zp, rp, pp, wp, r, theta);
    };
    for (int i = 0; i < 6; ++i) {
      double d = (i < 3) ? dp : da;
      J[i] = (dz(i, d) - dz(i, -d)) / (2.0 * d);
    }

    double var_pose = 0.0;
    const auto & cov = odom.pose.covariance;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        var_pose += J[i] * cov[i * 6 + j] * J[j];

    // 센서 노이즈 (range + angle)
    double sr = computeRangeSigma(r);
    double Js[2];
    Js[0] = (computeZFromPose(x,y,z,roll,pitch,yaw,r+1e-4,theta) -
             computeZFromPose(x,y,z,roll,pitch,yaw,r-1e-4,theta)) / (2e-4);
    Js[1] = (computeZFromPose(x,y,z,roll,pitch,yaw,r,theta+1e-4) -
             computeZFromPose(x,y,z,roll,pitch,yaw,r,theta-1e-4)) / (2e-4);

    double var_sensor = Js[0]*Js[0]*sr*sr + Js[1]*Js[1]*lidar_angle_sigma_*lidar_angle_sigma_;
    double var_total  = var_pose + var_sensor;
    return (var_total < 1e-8) ? 1e-8 : var_total;
  }

  // ==================== 퍼블리시 헬퍼 ====================

  void publishPointCloud(const std::vector<tf2::Vector3> & points,
                         const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = map_frame_;
    cloud.header.stamp    = stamp;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
    for (const auto & p : points) {
      *ix = static_cast<float>(p.x()); ++ix;
      *iy = static_cast<float>(p.y()); ++iy;
      *iz = static_cast<float>(p.z()); ++iz;
    }
    pointcloud_pub_->publish(cloud);
  }

  void publishClusterCloud(const std::vector<ElevPoint> & pts,
                           const std::vector<int> & labels,
                           const rclcpp::Time & stamp)
  {
    if (pts.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = rotated_map_frame_;
    cloud.header.stamp    = stamp;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(
        4,
        "x",   1, sensor_msgs::msg::PointField::FLOAT32,
        "y",   1, sensor_msgs::msg::PointField::FLOAT32,
        "z",   1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(pts.size());

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> irgb(cloud, "rgb");

    for (size_t i = 0; i < pts.size(); ++i, ++ix, ++iy, ++iz, ++irgb) {
      *ix = static_cast<float>(pts[i].x);
      *iy = static_cast<float>(pts[i].y);
      *iz = static_cast<float>(pts[i].z);

      uint8_t r = 255, g = 255, b = 255;
      if (i < labels.size()) {
        if      (labels[i] == 0) { r=255; g=0;   b=0;   }  // 수직: red
        else if (labels[i] == 1) { r=0;   g=255; b=0;   }  // 지면: green
        else if (labels[i] == 2) { r=0;   g=0;   b=255; }  // 상단: blue
        else                     { r=0;   g=255; b=255; }   // unknown: cyan
      }
      uint32_t rgb_val = (static_cast<uint32_t>(r) << 16 |
                          static_cast<uint32_t>(g) <<  8 |
                          static_cast<uint32_t>(b));
      float rgb_f;
      std::memcpy(&rgb_f, &rgb_val, sizeof(float));
      *irgb = rgb_f;
    }

    cluster_cloud_pub_->publish(cloud);
  }

  void publishCorrectedTF(const tf2::Transform & T_map_base,
                          const rclcpp::Time & stamp)
  {
    // map → calibrated_basefootprint
    geometry_msgs::msg::TransformStamped base_tf;
    base_tf.header.stamp    = stamp;
    base_tf.header.frame_id = map_frame_;
    base_tf.child_frame_id  = base_frame_;
    base_tf.transform.translation.x = T_map_base.getOrigin().x();
    base_tf.transform.translation.y = T_map_base.getOrigin().y();
    base_tf.transform.translation.z = T_map_base.getOrigin().z();
    base_tf.transform.rotation = tf2::toMsg(T_map_base.getRotation());
    tf_broadcaster_->sendTransform(base_tf);

    // calibrated_basefootprint → calibrated_lidar
    geometry_msgs::msg::TransformStamped lidar_tf;
    lidar_tf.header.stamp    = stamp;
    lidar_tf.header.frame_id = base_frame_;
    lidar_tf.child_frame_id  = "calibrated_lidar";
    lidar_tf.transform.translation.x = T_lidar_to_base_.getOrigin().x();
    lidar_tf.transform.translation.y = T_lidar_to_base_.getOrigin().y();
    lidar_tf.transform.translation.z = T_lidar_to_base_.getOrigin().z();
    lidar_tf.transform.rotation = tf2::toMsg(T_lidar_to_base_.getRotation());
    tf_broadcaster_->sendTransform(lidar_tf);
  }

  void publishElevationMarkerArray(const grid_map::GridMap & map,
                                   const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = map.getFrameId();
    clear.header.stamp    = stamp;
    clear.ns              = "elevation_cells";
    clear.action          = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int id = 1;
    constexpr float Z_MIN = -0.1f;
    constexpr float Z_MAX =  0.5f;

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      float z = map.at("elevation", *it);
      if (!std::isfinite(z)) continue;

      grid_map::Position pos;
      map.getPosition(*it, pos);

      visualization_msgs::msg::Marker cell;
      cell.header.frame_id = map.getFrameId();
      cell.header.stamp    = stamp;
      cell.ns              = "elevation_cells";
      cell.id              = id++;
      cell.type            = visualization_msgs::msg::Marker::CUBE;
      cell.action          = visualization_msgs::msg::Marker::ADD;

      float h = std::max(std::fabs(z), 0.01f);
      cell.pose.position.x = pos.x();
      cell.pose.position.y = pos.y();
      cell.pose.position.z = (z >= 0.0f) ? h * 0.5f : -h * 0.5f;
      cell.pose.orientation.w = 1.0;
      cell.scale.x = map_resolution_ * 0.98;
      cell.scale.y = map_resolution_ * 0.98;
      cell.scale.z = h;

      float norm = std::clamp((z - Z_MIN) / (Z_MAX - Z_MIN), 0.0f, 1.0f);
      cell.color.a = 0.9f;
      cell.color.r = norm;
      cell.color.g = 0.2f;
      cell.color.b = 1.0f - norm;

      arr.markers.push_back(cell);
    }

    elev_marker_array_pub_->publish(arr);
  }

  void writeDebugCSV(const std::vector<ElevPoint> & pts,
                     const std::vector<int> & labels)
  {
    std::lock_guard<std::mutex> lock(debug_file_mutex_);
    for (size_t i = 0; i < pts.size(); ++i) {
      const auto & p = pts[i];
      int lbl = (i < labels.size()) ? labels[i] : -1;
      debug_file_
          << debug_scan_id_ << "," << p.theta << "," << p.r << ","
          << p.x_l << "," << p.y_l << ","
          << p.x   << "," << p.y   << "," << p.z << "," << lbl << "\n";
    }
    debug_file_.flush();
    ++debug_scan_id_;
  }

  // ==================== 멤버 변수 ====================

  // 메시지 동기화
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> odom_scan_sync_;
  message_filters::Subscriber<nav_msgs::msg::Odometry>   odom_sub_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;

  // 퍼블리셔
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   laser_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr elev_marker_array_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_max_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_kalman_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_dual_layer_;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;

  // 고도 맵 객체 (파라미터로 활성화 제어)
  std::unique_ptr<bunker_elevation::ElevationMapMax> map_max_;
  std::unique_ptr<bunker_elevation::ElevationMapKalman>  map_kalman_;
  std::unique_ptr<bunker_elevation::ElevationMapDualLayer>  map_dual_layer_;

  // 파라미터
  double sync_limit_time_{0.05};
  double map_resolution_{0.1}, map_width_{50.0}, map_height_{50.0};
  double max_distance_{2.0}, default_meas_var_{0.01};
  double odom_z_offset_{-157.195 + 0.481};
  double bl_roll_{0.0}, bl_pitch_{-M_PI/6.0}, bl_yaw_{M_PI};
  double bl_tx_{0.2817}, bl_ty_{0.0}, bl_tz_{0.3938};
  double lidar_angle_sigma_{0.0};

  std::string map_frame_{"map"};
  std::string base_frame_{"calibrated_basefootprint"};
  std::string rotated_map_frame_{"map_elev"};

  bool enable_map_max_{true};
  bool enable_map_kalman_{true};
  bool enable_map_dual_layer_{true};
  bool enable_debug_csv_{false};

  // EKF 안정화 상태
  double       odom_stable_threshold_{0.5};
  int          odom_stable_min_count_{10};
  int          odom_stable_count_{0};
  bool         odom_ready_{false};
  bool         has_prev_odom_pos_{false};
  tf2::Vector3 prev_odom_pos_;

  // TF 캐시
  tf2::Transform T_lidar_to_base_;
  tf2::Transform T_map_to_rot_;
  bool has_T_map_to_rot_{false};

  // CSV 디버그
  std::ofstream debug_file_;
  std::mutex    debug_file_mutex_;
  size_t        debug_scan_id_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomScanSyncNode>());
  rclcpp::shutdown();
  return 0;
}
