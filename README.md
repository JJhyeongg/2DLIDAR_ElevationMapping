# Bunker Robot — GPS-Based Autonomous Navigation with 2D LiDAR Elevation Mapping

ROS2 (Humble) 기반 자율 이동 로봇 시스템.
2D LiDAR 단일 센서로 실시간 고도 맵(Elevation Map)을 생성합니다.

---

## 주요 기능

- **GPS 웨이포인트 자율 주행** — Navigation2 스택 기반
- **2D LiDAR 실시간 고도 맵 생성** — 틸트 마운팅된 RPLIDAR로 포인트 클라우드 재구성
- **UKF 기반 센서 퓨전** — GPS/IMU 융합

---

## 시스템 구성

```
ros2_ws/bunker_sim_v2_ws/src/
├── bunker_description/       # 로봇 URDF/Xacro 모델
├── bunker_sim_bringup/       # TF 퍼블리셔, GPS 필터 (C++)
├── bunker_nav2/              # GPS 웨이포인트 팔로워 (Python + YAML)
└── bunker_util/              # 고도 맵 생성 코어 (C++17)  ← 핵심
    ├── src/
    │   ├── scan_processor.cpp         # 메인 노드: LiDAR + 오도메트리 → 고도 맵
    │   ├── elevation_map_max.cpp      # Max-height 알고리즘
    │   ├── elevation_map_kalman.cpp   # 1D Kalman 필터 알고리즘
    │   ├── elevation_map_dual_layer.cpp  # DualLayer 알고리즘 (메인)
    │   └── map_sub.cpp                # 맵 구독 / 재발행 / CSV 저장
    ├── include/bunker_util/
    │   ├── elevation_types.hpp
    │   ├── elevation_map_max.hpp
    │   ├── elevation_map_kalman.hpp
    │   └── elevation_map_dual_layer.hpp
    ├── launch/
    │   └── system_start.launch.py
    └── config/
        └── scan_processor_params.yaml

```

---

## 고도 맵 알고리즘

| 알고리즘 | 토픽 | 설명 |
|----------|------|------|
| `max` | `/elevation_map_max` | 셀별 최대 높이 누적 |
| `kalman` | `/elevation_map_kalman` | 라벨 구분 없이 모든 포인트에 1D Kalman 필터 적용 |
| `dual_layer` | `/elevation_map_dual_layer` | 지면(low) / 장애물(high) 레이어 분리 후 융합 **(메인)** |

### DualLayer 알고리즘 개요
```
LiDAR 포인트 → 라벨 분류 (ground / upper / vertical)
    ↓
low 레이어: ground 포인트 → Kalman 업데이트 (양방향, 지면 추적)
high 레이어: upper + vertical 포인트 → 단조증가 업데이트 (장애물 영속성)
    ↓
hit 카운트 기반 융합 → 최종 elevation 결정
```

---

## TF 프레임 구조

```
map → map_rot → base_link → base_lidar
                           → gps
                           → imu
```

- `map`: GPS 기준 전역 좌표계
- `map_elev`: 고도 맵 전용 회전 프레임 (8.9° 보정)

---

## 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp`, `sensor_msgs`, `nav_msgs` | ROS2 코어 |
| `grid_map_ros` | 고도 그리드 맵 |
| `robot_localization` | UKF 센서 퓨전 |
| `nav2_*` | Navigation2 스택 |
| `tf2_ros`, `tf2_geometry_msgs` | 좌표 변환 |
| `message_filters` | ApproximateTime 동기화 |
| `OpenCV` | 이미지 처리 |

---

## 데이터셋 & RViz 설정

### RViz 설정

```
rviz_config/
└── elevation_mapping.rviz   # 고도 맵 시각화 기본 설정
```

- `/elevation_map_dual_layer`, `/elevation_map_kalman`, `/elevation_map_max` GridMap 레이어
- `/tilt_lidar_scan` LaserScan
- `/odometry/global` Odometry
- TF 트리 시각화 포함

### rosbag 데이터
```
rosbag/
└── 5/   # 실외 환경 주행 실험 데이터
    ├── 5_0.db3
    └── metadata.yaml
```
![demo](assets/demo_elevation_mapping.gif)

**포함 토픽:**

| 토픽 | 타입 |
|------|------|
| `/tilt_lidar_scan` | `sensor_msgs/LaserScan` |
| `/odometry/global` | `nav_msgs/Odometry` |
| `/wheel/odometry` | `nav_msgs/Odometry` |
| `/imu/data` | `sensor_msgs/Imu` |
| `/fix` | `sensor_msgs/NavSatFix` |

---

## 의존성 설치

`ros-humble-desktop` 설치 후 아래 패키지를 추가로 설치합니다.

```bash
# grid_map
sudo apt install ros-humble-grid-map

# robot_localization (UKF 센서 퓨전)
sudo apt install ros-humble-robot-localization

# Navigation2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup


# 기타 메시지 / TF 관련
sudo apt install \
    ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs \
    ros-humble-message-filters

```

---

## 빌드

```bash
cd ~/ros2_ws/bunker_sim_v2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 실행

### 전체 시스템

```bash
ros2 launch bunker_util system_start.launch.py
```

### 알고리즘 선택 옵션

```bash
# DualLayer만 활성화
ros2 launch bunker_util system_start.launch.py \
    enable_map_max:=false enable_map_kalman:=false enable_map_dual_layer:=true

# 모두 비활성화 후 kalman만 활성화
ros2 launch bunker_util system_start.launch.py \
    enable_map_max:=false enable_map_kalman:=true enable_map_dual_layer:=false
```

### rosbag 데이터로 테스트

```bash
# 준비 스크립트 (tmux 3분할 + source 자동 완료)
./run_mapping.sh [bag_dir]

# 또는 수동
# 터미널 1: 매핑 시스템
source install/setup.bash
ros2 launch bunker_util system_start.launch.py

# 터미널 2: bag 재생
ros2 bag play rosbag/5/ --clock

# 터미널 3: RViz
rviz2
```

---

## 파라미터 (`scan_processor_params.yaml`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `sync_limit_time` | `0.05` | LiDAR-오도메트리 동기화 허용 오차 (sec) |
| `map_resolution` | `0.1` | 맵 해상도 (m/cell) |
| `map_width` / `map_height` | `50.0` | 맵 크기 (m) |
| `max_distance` | `2.0` | LiDAR 유효 거리 (m) |
| `odom_z_offset` | `-156.714` | GPS 고도 오프셋 (현장 캘리브레이션 값) |
| `enable_map_max` | `true` | Max 맵 활성화 |
| `enable_map_kalman` | `true` | Kalman 맵 활성화 |
| `enable_map_dual_layer` | `true` | DualLayer 맵 활성화 |
| `odom_stable_threshold` | `0.5` | EKF 안정화 판단 임계값 (m/frame) |
| `odom_stable_min_count` | `10` | EKF 안정화 연속 프레임 수 |
| `enable_debug_csv` | `false` | 디버그 CSV 로그 출력 |

---

## 현장 캘리브레이션 상수 — 임의 수정 금지

| 상수 | 값 | 파일 |
|------|----|------|
| GPS 고도 오프셋 | `-157.195 + 0.481` | `map_cal.cpp` |
| 맵 요 회전 | `8.9°` | `map_cal.cpp` |
| LiDAR 마운팅 각도 | `pitch=-30°, yaw=180°` | `bunker_start.launch.py` |
| LiDAR 위치 오프셋 | `x=0.285m, z=0.068m` | `bunker_start.launch.py` |
| GPS 위치 오프셋 | `x=0.157m, z=0.155m` | `bunker_start.launch.py` |

---

## 데이터 수집에 사용된 하드웨어

| 센서 | 모델 |
|------|------|
| LiDAR | SLAMTEC RPLIDAR S-series |
| GPS | u-blox ZED-F9P (RTK) |
| IMU/INS | VectorNav |
| 로봇 플랫폼 | AgileX Bunker |
