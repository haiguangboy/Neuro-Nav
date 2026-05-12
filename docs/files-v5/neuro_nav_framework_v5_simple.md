```
**neuro-nav** (v5 - 整合架构审查)
│
│  ★ 核心改进:
│  1. Nav2定位为"执行内核"，Neuro-Nav做插件
│  2. 跨本体执行: 导航层(意图) + 运动层(执行)分离
│  3. 统一IEnvironmentModel (2D Costmap + 3D ESDF + 2.5D Elevation)
│  4. VLN明确契约 + 降级策略
│
├── common/                          # 公共基础层 (所有模块共用)
│   ├── math/                        # 变换/插值/优化/滤波
│   ├── geometry/                    # 碰撞/距离/足迹
│   ├── data/                        # Pose/Trajectory/Costmap
│   ├── time/                        # 时钟/同步
│   ├── concurrency/                 # 线程池/无锁队列
│   ├── logging/                     # 日志/指标
│   ├── config/                      # 配置管理
│   └── error/                       # Result<T,E>
│
├── robot/                           # 机器人抽象层
│   ├── interface/
│   │   ├── i_kinematics             # 运动学
│   │   ├── i_constraints            # 约束
│   │   ├── i_footprint              # 足迹
│   │   └── i_robot_profile          # ★ 机器人特征Profile
│   ├── wheeled/                     # differential/ackermann/tricycle/omni
│   ├── legged/                      # quadruped/bipedal
│   └── factory/
│
├── perception/                      # ★ 感知层 (重构)
│   │
│   ├── interface/
│   │   ├── i_local_mapper           # 局部建图
│   │   └── i_environment_model      # ★ 统一环境模型接口
│   │
│   ├── environment_model/           # ★ 环境模型实现
│   │   ├── environment_model        # IEnvironmentModel实现
│   │   ├── projection_rules         # 3D→2D投影规则
│   │   ├── traversability_mapper    # Traversability→代价映射
│   │   └── unknown_space_policy     # Unknown空间策略
│   │
│   ├── local_mapping/
│   │   ├── nvblox/                  # TSDF/ESDF/Costmap
│   │   ├── voxel_grid/              # 轻量级
│   │   └── elevation_map/           # 足式地形
│   │
│   ├── obstacle/                    # 障碍物处理
│   ├── traversability/              # 可通行性
│   └── sensor_fusion/               # 融合
│
├── core/                            # ★ 导航核心层 (重构)
│   │
│   ├── navigation_command/          # ★ 跨本体导航指令
│   │   ├── navigation_command       # 统一指令结构
│   │   ├── i_command_output         # 输出接口
│   │   ├── twist_output             # 轮式: cmd_vel
│   │   ├── body_trajectory_output   # 足式: 躯干轨迹
│   │   └── footstep_output          # 足式: 落脚点
│   │
│   ├── planner/                     # ★ Nav2插件导向
│   │   │
│   │   ├── nav2_plugins/            # Nav2插件实现 (核心!)
│   │   │   │
│   │   │   ├── planners/            # Global Planner Plugins
│   │   │   │   ├── vln_planner_plugin       # VLN
│   │   │   │   ├── gnm_planner_plugin       # GNM
│   │   │   │   └── hybrid_vln_plugin        # VLN+传统混合
│   │   │   │
│   │   │   ├── controllers/         # Controller Plugins
│   │   │   │   ├── esdf_mppi_plugin         # ESDF感知MPPI
│   │   │   │   ├── legged_controller_plugin # 足式
│   │   │   │   └── diffusion_controller     # 扩散策略
│   │   │   │
│   │   │   ├── costmap_layers/      # Costmap Layer Plugins
│   │   │   │   ├── nvblox_layer             # nvblox投影
│   │   │   │   ├── esdf_layer               # ESDF距离
│   │   │   │   └── traversability_layer     # 可通行性
│   │   │   │
│   │   │   └── bt_nodes/            # ★ BT Node Plugins
│   │   │       ├── vln_fallback_node        # VLN降级
│   │   │       ├── model_confidence_node    # 置信度检查
│   │   │       ├── locomotion_switch_node   # 运动切换
│   │   │       └── esdf_safety_node         # ESDF安全
│   │   │
│   │   ├── model_based/             # 模型规划器核心
│   │   │   ├── intern_nav/          # InternVLA-N1/NavDP/StreamVLN
│   │   │   └── foundation/          # GNM/ViNT/NoMaD
│   │   │
│   │   └── vln_integration/         # ★ VLN集成
│   │       ├── vln_contract          # 输出契约定义
│   │       ├── vln_path_adapter      # VLN→nav_msgs/Path
│   │       ├── vln_prior_fusion      # 软约束融合
│   │       └── vln_fallback          # 降级策略
│   │
│   ├── decision/                    # 决策 (简化)
│   │   ├── mode_selector/
│   │   └── recovery/                # → Nav2 BT节点
│   │
│   ├── trajectory/                  # 轨迹处理
│   │   ├── fusion/                  # 多源融合
│   │   ├── validation/              # ESDF验证
│   │   ├── optimization/            # ★ ESDF梯度优化
│   │   └── execution/
│   │
│   └── map_manager/                 # → 依赖IEnvironmentModel
│
├── interface/                       # 接口层
│   │
│   ├── upstream/                    # 定位/感知/模型
│   │
│   ├── downstream/                  # ★ 重构
│   │   ├── locomotion/              # ★ 运动层适配
│   │   │   ├── i_locomotion_interface       # 运动层接口
│   │   │   ├── wheeled_locomotion           # → cmd_vel
│   │   │   ├── legged_locomotion            # → 躯干+落脚点
│   │   │   └── humanoid_locomotion          # → 全身轨迹
│   │   └── output/
│   │
│   ├── communication/               # ROS2/MQTT/gRPC/ZeroMQ
│   └── sensor/
│
├── simulation/                      # Isaac Sim/Lab/Gazebo
│
├── app/
│   ├── ros2_bringup/                # ★ Nav2 + Neuro-Nav启动
│   │   ├── nav2_bringup.launch.py
│   │   └── full_system.launch.py
│   ├── sdk/
│   └── cli/
│
└── config/
    ├── robots/                      # 含RobotProfile
    ├── nav2/                        # ★ Nav2配置
    │   ├── nav2_params.yaml
    │   └── bt_navigator.xml         # 使用Neuro-Nav BT节点
    ├── perception/
    │   └── environment_model.yaml   # ★ 环境模型配置
    └── vln/
        ├── vln_contract.yaml        # ★ VLN契约
        └── fallback_policy.yaml     # 降级策略
```

---

## 核心架构决策

### 1. Nav2 vs Neuro-Nav 职责划分

```
┌──────────────────────────────────────────────────────────────────┐
│  Nav2 (执行内核)              │  Neuro-Nav (能力扩展)            │
├──────────────────────────────────────────────────────────────────┤
│  ✓ Lifecycle管理             │  ✓ 跨本体抽象                     │
│  ✓ BT编排 (bt_navigator)     │  ✓ 环境模型融合                   │
│  ✓ Action接口                │  ✓ 学习型规划器适配               │
│  ✓ Server框架                │  ✓ Nav2插件实现:                  │
│  ✓ Costmap2D框架             │     - Planner Plugins            │
│  ✓ Recovery框架              │     - Controller Plugins         │
│                              │     - Costmap Layers             │
│                              │     - BT Nodes                   │
└──────────────────────────────────────────────────────────────────┘

原则: 不重复实现Nav2已有功能，通过插件扩展
```

### 2. 跨本体执行架构

```
                     ┌──────────────────────┐
                     │   Navigation Layer   │  ← 统一导航意图
                     │   (Body Trajectory)  │
                     └──────────┬───────────┘
                                │
           ┌────────────────────┼────────────────────┐
           │                    │                    │
           ▼                    ▼                    ▼
   ┌───────────────┐    ┌───────────────┐    ┌───────────────┐
   │   Wheeled     │    │    Legged     │    │   Humanoid    │
   │  Locomotion   │    │  Locomotion   │    │  Locomotion   │
   │               │    │               │    │               │
   │  → cmd_vel    │    │  → Body Traj  │    │  → Whole Body │
   │  → wheel_cmd  │    │  + Footholds  │    │    Trajectory │
   └───────────────┘    └───────────────┘    └───────────────┘
```

### 3. 统一环境模型

```
                    IEnvironmentModel
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        ▼                 ▼                 ▼
 ┌─────────────┐  ┌─────────────┐  ┌─────────────┐
 │ 2D Costmap  │  │  3D ESDF    │  │ 2.5D Elev   │
 │  (Nav2用)   │  │ (轨迹优化) │  │ (足式地形) │
 └─────────────┘  └─────────────┘  └─────────────┘

投影规则:
• 3D→2D: 高度切片 + 地面分割 + 动态滤除
• Unknown策略: 轮式=障碍(保守) / 足式=渐变(激进)
• Traversability→代价: 按RobotProfile参数化
```

### 4. VLN集成 + 降级

```
┌─────────────────────────────────────────────────────────────────┐
│                      VLN Integration                             │
│                                                                  │
│  输入: 语言指令 + 当前位姿 + 环境模型                            │
│                     │                                            │
│                     ▼                                            │
│  ┌─────────────────────────────────┐                            │
│  │         VLN Model               │                            │
│  │  (InternVLA-N1 / StreamVLN)     │                            │
│  └─────────────────┬───────────────┘                            │
│                    │                                             │
│          ┌─────────▼─────────┐                                  │
│          │  VlnOutput        │                                  │
│          │  - path/traj      │                                  │
│          │  - confidence     │                                  │
│          │  - feasibility    │                                  │
│          └─────────┬─────────┘                                  │
│                    │                                             │
│     ┌──────────────┼──────────────┐                             │
│     │              │              │                             │
│     ▼              ▼              ▼                             │
│ confidence    ESDF check    collision                           │
│   > 0.6?       > 0.3m?       < 10%?                            │
│     │              │              │                             │
│     └──────────────┼──────────────┘                             │
│                    │                                             │
│         ┌─────────YES─────────┐                                 │
│         │                     │                                 │
│         ▼                     ▼                                 │
│  ┌─────────────┐      ┌─────────────┐                          │
│  │  Use VLN    │      │  Fallback   │                          │
│  │  Path       │      │  SmacHybrid │                          │
│  └─────────────┘      └─────────────┘                          │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 关键接口

### IEnvironmentModel

```cpp
class IEnvironmentModel {
    // 2D Costmap (Nav2)
    OccupancyGrid get_local_costmap_2d(pose, width, height, resolution);
    
    // 3D ESDF (轨迹优化)
    double query_distance(point);
    vector<double> query_distances(points);  // 批量
    Vector3d query_distance_gradient(point); // 梯度
    
    // 2.5D (足式)
    ElevationMap get_elevation_map();
    bool is_foothold_valid(position, constraints);
    
    // 配置
    void set_robot_profile(profile);
    void set_unknown_space_policy(policy);
};
```

### NavigationCommand

```cpp
struct NavigationCommand {
    CommandType type;  // TWIST_2D / BODY_TRAJECTORY / FOOTSTEP_PLAN
    
    Trajectory body_trajectory;     // 通用载体
    Twist2D twist;                  // 轮式
    vector<Foothold> footholds;     // 足式
    
    double confidence;
    string source;
};

class ICommandOutput {
    void publish(NavigationCommand cmd);
    vector<CommandType> supported_types();
};
```

### VLN契约

```cpp
struct VlnOutput {
    VlnOutputType type;      // WAYPOINTS / PATH / TRAJECTORY
    Path path;
    double confidence;
    bool self_assessed_feasible;
};

struct VlnFallbackPolicy {
    double min_confidence = 0.6;
    double min_esdf_clearance = 0.3;
    string fallback_planner = "SmacHybrid";
};
```

---

## RobotProfile配置

```yaml
# 轮式 (三轮叉车)
profile:
  traversability:
    max_slope: 0.15        # 严格
    max_step_height: 0.05
  unknown_space:
    policy: "obstacle"     # 保守
  locomotion:
    output_type: "twist_2d"

# 足式 (四足)
profile:
  traversability:
    max_slope: 0.5         # 宽松
    max_step_height: 0.25
  unknown_space:
    policy: "cost_gradient" # 激进
  locomotion:
    output_type: "body_trajectory"
```

---

## v4 → v5 改进对比

| 方面 | v4 | v5 |
|------|----|----|
| Nav2关系 | 边界模糊 | 明确为插件提供者 |
| 跨本体 | 只抽象运动学 | 增加Locomotion层 |
| 环境模型 | 分散 | 统一IEnvironmentModel |
| VLN集成 | 接口不明 | 契约+降级策略 |
| ESDF | 只做Costmap | 闭环到轨迹优化 |
| Unknown | 未定义 | 策略化 |
| BT | 自建 | Nav2原生+自定义节点 |
