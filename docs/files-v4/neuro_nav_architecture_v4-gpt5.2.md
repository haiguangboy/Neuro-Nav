# Neuro-Nav 导航框架架构设计 v4

## 设计原则

本框架严格遵循以下软件设计原则：

| 原则 | 体现 |
|------|------|
| **单一职责 (SRP)** | 每个模块只做一件事，如`nvblox_adapter`只负责3D重建 |
| **开闭原则 (OCP)** | 通过接口扩展新机器人/规划器，无需修改核心代码 |
| **接口隔离 (ISP)** | 细粒度接口，如`IKinematicsModel`与`IPathPlanner`分离 |
| **高内聚低耦合** | 模块内部紧密协作，模块间通过抽象接口通信 |
| **无环依赖** | 依赖方向单向：app → core → common，无循环 |
| **Linux哲学** | 小而专一的模块，通过组合解决复杂问题 |

---

## 这版设计的主要问题（对照你的需求）

你提出的目标是“跨机器人本体（轮式/四足/人形/三支点叉车等） + 尽量沿用Navigation2架构做 freespace规划/避障/可行驶区域地图 + 能融合最新VLN模型给出的轨迹”。对照这份v4，主要缺口在：

1. **与Nav2的边界不清**：目前既有 `core/decision/behavior_tree/bt_navigator.cpp`，又有 `core/planner/global/nav2_wrapper/*` 与 `nav2_plugins/*`，容易形成“两套Nav2”：一套自己实现的server/BT，一套Nav2原生server/BT。落地时会出现参数、生命周期、TF、action接口重复与漂移。
2. **“跨本体”的抽象停在运动学层**：`IKinematics/IConstraints/IFootprint` 对轮式覆盖较好，但对四足/人形的“执行接口”不成立（它们通常不消费 `cmd_vel`，而是消费躯干轨迹+落脚点/全身轨迹/接触序列）。当前 `interface/downstream/output/cmd_vel_publisher.cpp` 默认输出 `cmd_vel` 会把足式/人形强行压扁成轮式假设。
3. **可行驶区域（Traversability/Drivable Area）的“地图语义”没收敛**：有 `traversability/*` 与 `map_manager/*`，但没有定义一个统一的“可通行地图”数据模型与发布方式（2D costmap、2.5D elevation、3D ESDF/TSDF、语义mask之间的关系、优先级与投影规则）。
4. **freespace规划依赖的地图链路不完整**：nvblox能产出 TSDF/ESDF，但文档只把它当 costmap layer；缺少“ESDF→可通行代价/安全距离→规划/轨迹优化”的闭环定义，以及 unknown space 处理策略（对不同本体差异很大）。
5. **VLN轨迹如何进入Nav2执行链路不明确**：有 `vln_planner.cpp`、`trajectory_fusion.cpp` 的名字，但缺少接口契约：VLN输出是 `nav_msgs/Path` 还是带时间戳的轨迹？是 hard constraint 还是 soft prior？失败/不可行时如何降级到传统规划器？

下面给出一个“最小改动、但能落地”的对齐方案（保持你当前分层不推倒重来）。

---

## 推荐的落地架构：把Nav2当作“执行与编排内核”

建议明确：

- **Nav2负责**：Lifecycle、BT编排（bt_navigator）、Planner/Controller/Behavior Server、action接口（NavigateToPose/NavigateThroughPoses/FollowPath 等）。
- **Neuro-Nav负责**：跨本体能力抽象、地图表示融合（ESDF/Elevation/Traversability）、学习/模型规划器（VLN/扩散/GNM等）的适配与安全约束、Nav2插件实现（planner/controller/costmap layer/BT node）。

这样可以避免重复实现Nav2的server与BT。对应改动点：

- `core/decision/behavior_tree/*`：定位为 **Nav2 BT节点集合**（`nav2_plugins/bt_nodes`）而不是自建bt_navigator。
- `core/planner/global/nav2_wrapper/*`：更推荐改为 **Nav2 Planner Plugin**（包在 `nav2_plugins/planners` 下），由Nav2调用而非你调用Nav2。
- `app/ros2_node/navigation_node.cpp`：作为launch/组合节点入口，拉起Nav2 + 你自己的感知/模型桥接节点。

---

## 跨机器人本体：把“规划”和“执行”解耦成两层能力

为了同时覆盖轮式/叉车/四足/人形，建议在架构里显式区分两类接口：

1. **导航层（Nav Layer）**：输出“机体轨迹/路径意图”（body path / body trajectory），用于避障与到达。
2. **运动层（Locomotion Layer）**：把导航层意图转成各本体可执行指令（`cmd_vel` / 轮速 / 足步序列 / 全身轨迹）。

文档里建议新增一个下游输出抽象，替代默认“只发布cmd_vel”的假设：

```cpp
// interface/downstream/output/i_navigation_command.hpp
namespace neuro_nav::interface {

enum class CommandType {
    TWIST2D,           // 轮式/叉车常用 cmd_vel
    WHEEL_JOINTS,      // 轮速/舵角
    BODY_TRAJECTORY,   // 躯干SE2/SE3轨迹（足式/人形导航意图）
    FOOTSTEP_PLAN,     // 落脚点序列（足式）
    WHOLE_BODY_TRAJ    // 需要时（人形/复杂场景）
};

struct NavigationCommand {
    CommandType type;
    common::Trajectory body_traj;                 // 最通用的载体（可为空）
    std::vector<Foothold> footholds;              // 可选
    common::Twist2D twist;                        // 可选
};

class ICommandOutput {
public:
    virtual ~ICommandOutput() = default;
    virtual void publish(const NavigationCommand& cmd) = 0;
};

}  // namespace neuro_nav::interface
```

这能让“跨本体”的核心抽象更真实：Nav2 controller plugin 可以对轮式输出 `cmd_vel`，而对四足/人形输出“躯干轨迹+（可选）落脚点”，由下游 locomotion 接管。

---

## 可行驶区域/地图表示：建立统一的 Environment Model

建议把地图表示收敛成一个统一接口（同时服务 Nav2 costmap、ESDF轨迹优化、足式地形）：

```cpp
// perception/interface/i_environment_model.hpp
namespace neuro_nav::perception {

struct TraversabilityCell {
    float traversability;   // [0,1]
    float slope;            // rad or deg
    float roughness;
    float step_height;
};

class IEnvironmentModel {
public:
    virtual ~IEnvironmentModel() = default;

    // 2D costmap (Nav2)
    virtual common::OccupancyGrid get_local_costmap_2d(
        const common::Pose2D& robot_pose,
        double width, double height,
        double resolution) const = 0;

    // 3D ESDF (freespace/距离场)
    virtual EsdfGrid get_esdf_3d() const = 0;

    // 2.5D elevation + traversability (legged/humanoid)
    virtual ElevationMap get_elevation_map() const = 0;
    virtual TraversabilityGrid get_traversability_grid() const = 0;
};

}  // namespace neuro_nav::perception
```

并在 `map_manager/` 中明确三条“投影/融合规则”（建议写进文档作为约定）：

- **3D→2D投影规则**：ESDF/TSDF/点云如何投影到Nav2 costmap（高度阈值、地面分割、动态物体处理）。
- **Traversability→代价映射**：对轮式（坡度/台阶阈值严格）与足式（可接受更大坡度/台阶）使用不同 mapping（通过robot profile参数化）。
- **Footprint/接触模型的膨胀策略**：叉车货叉高度与外廓变化、足式的“机体占据区”与“落脚可行区”分离。

---

## freespace规划与避障：建议用“两级约束”打通 Nav2

为了尽可能沿用Nav2生态，推荐：

- **全局/局部规划仍走Nav2接口**（planner/controller plugins）。
- **freespace约束来自 ESDF + costmap**：
  - costmap用于2D占用/动态障碍
  - ESDF用于连续空间的“安全距离/梯度”（轨迹优化、速度采样评估）

落地到插件层可以是：

- `nav2_plugins/costmap_layers/nvblox_layer.cpp` 负责把3D融合结果投到2D costmap。
- `nav2_plugins/controllers/mppi|teb|custom` 在评分函数里额外查询ESDF距离（你已在 `NvbloxAdapter` 里提供 `query_distance(s)` 的基础）。

---

## VLN轨迹融合：把VLN当“路径先验/约束”，不是直接替代避障

建议定义一个清晰的VLN输出契约，并在Nav2链路里以两种方式接入（二选一或同时支持）：

1. **VLN作为全局路径提供者（推荐默认）**
   - VLN输出：`nav_msgs/Path`（无时间）或 `NavigateThroughPoses` 的waypoints
   - 进入点：Nav2 planner plugin（ComputePathToPose/ThroughPoses）
   - 之后由Nav2 controller + 你的ESDF/costmap约束负责避障与可行性

2. **VLN作为软约束（prior）融合到传统规划器/轨迹优化**
   - 在 `core/trajectory/fusion/trajectory_fusion.cpp` 里定义“prior cost”项：偏离VLN轨迹的代价 + 碰撞/动力学硬约束
   - 当VLN给出不安全路径时，优化会自动偏离但仍尽量贴近语义意图

无论哪种方式，都要有明确降级策略（建议写成BT条件）：

- VLN路径不可行/置信度低 → fallback 到 SMAC/NavFn/Theta*。
- 执行中局部代价过高/ESDF距离过小 → 触发 replanning 或 recovery。

## 模块依赖关系图

```
                    ┌─────────────────────────────────────────┐
                    │              应用层 (app)                │
                    │  SDK / CLI / ROS2 API / VLA Agent       │
                    └────────────────────┬────────────────────┘
                                         │
                    ┌────────────────────▼────────────────────┐
                    │              核心层 (core)               │
                    │  planner / decision / trajectory        │
                    └────────────────────┬────────────────────┘
                                         │
        ┌────────────────┬───────────────┼───────────────┬────────────────┐
        │                │               │               │                │
        ▼                ▼               ▼               ▼                ▼
┌───────────┐    ┌───────────┐   ┌───────────┐   ┌───────────┐    ┌───────────┐
│  robot    │    │perception │   │ interface │   │simulation │    │  common   │
│(运动学)   │    │(局部感知) │   │  (接口)   │   │  (仿真)   │    │ (公共库)  │
└─────┬─────┘    └─────┬─────┘   └─────┬─────┘   └─────┬─────┘    └─────┬─────┘
      │                │               │               │                │
      └────────────────┴───────────────┴───────────────┴────────────────┘
                                         │
                    ┌────────────────────▼────────────────────┐
                    │           common (公共基础层)            │
                    │  math / geometry / data / time / log    │
                    └─────────────────────────────────────────┘

依赖规则:
- 上层可依赖下层
- 同层模块通过接口通信
- common被所有模块依赖，但不依赖任何模块
- 无循环依赖
```

---

## 完整框架结构

```
**neuro-nav** (v4)
│
├── common/                          # ★ 公共基础层 (所有模块共用)
│   │
│   ├── math/                        # 数学工具
│   │   ├── transform.hpp            # 坐标变换 (SE2/SE3/四元数)
│   │   ├── interpolation.hpp        # 插值 (线性/样条/Slerp)
│   │   ├── optimization.hpp         # 优化器 (梯度下降/LM)
│   │   ├── filter.hpp               # 滤波器 (卡尔曼/低通)
│   │   └── numerical.hpp            # 数值计算 (积分/微分)
│   │
│   ├── geometry/                    # 几何工具
│   │   ├── collision.hpp            # 碰撞检测 (AABB/OBB/GJK)
│   │   ├── distance.hpp             # 距离计算 (点到线/面)
│   │   ├── polygon.hpp              # 多边形操作
│   │   ├── convex_hull.hpp          # 凸包算法
│   │   └── footprint.hpp            # 机器人足迹
│   │
│   ├── data/                        # 数据结构
│   │   ├── pose.hpp                 # 位姿 (Pose2D/Pose3D)
│   │   ├── twist.hpp                # 速度 (Twist2D/Twist3D)
│   │   ├── trajectory.hpp           # 轨迹容器
│   │   ├── path.hpp                 # 路径容器
│   │   ├── occupancy_grid.hpp       # 占用栅格
│   │   ├── costmap.hpp              # 代价地图
│   │   └── point_cloud.hpp          # 点云
│   │
│   ├── time/                        # 时间管理
│   │   ├── clock.hpp                # 时钟抽象 (ROS/System/Sim)
│   │   ├── rate.hpp                 # 频率控制
│   │   ├── timer.hpp                # 定时器
│   │   └── time_sync.hpp            # 时间同步
│   │
│   ├── concurrency/                 # 并发工具
│   │   ├── thread_pool.hpp          # 线程池
│   │   ├── executor.hpp             # 任务执行器
│   │   ├── lock_free_queue.hpp      # 无锁队列
│   │   └── async_helper.hpp         # 异步辅助
│   │
│   ├── logging/                     # 日志系统
│   │   ├── logger.hpp               # 日志接口
│   │   ├── log_sink.hpp             # 日志输出 (文件/控制台/ROS)
│   │   └── metrics.hpp              # 性能指标收集
│   │
│   ├── config/                      # 配置管理
│   │   ├── config_loader.hpp        # 配置加载 (YAML/JSON)
│   │   ├── param_server.hpp         # 参数服务器
│   │   └── validator.hpp            # 参数校验
│   │
│   └── error/                       # 错误处理
│       ├── result.hpp               # Result<T, E> 类型
│       ├── error_code.hpp           # 错误码定义
│       └── exception.hpp            # 异常类
│
├── robot/                           # ★ 机器人抽象层 (运动学/动力学)
│   │
│   ├── interface/                   # 机器人接口定义
│   │   ├── i_kinematics.hpp         # 运动学接口
│   │   ├── i_dynamics.hpp           # 动力学接口
│   │   ├── i_footprint.hpp          # 足迹接口
│   │   └── i_constraints.hpp        # 约束接口
│   │
│   ├── wheeled/                     # 轮式机器人
│   │   │
│   │   ├── differential/            # 差速底盘
│   │   │   ├── diff_kinematics.cpp
│   │   │   ├── diff_constraints.cpp
│   │   │   └── diff_config.yaml
│   │   │
│   │   ├── ackermann/               # 阿克曼底盘
│   │   │   ├── ackermann_kinematics.cpp
│   │   │   ├── ackermann_constraints.cpp
│   │   │   └── ackermann_config.yaml
│   │   │
│   │   ├── tricycle/                # 三轮叉车底盘
│   │   │   ├── tricycle_kinematics.cpp
│   │   │   ├── tricycle_constraints.cpp
│   │   │   ├── fork_kinematics.cpp  # 货叉运动学
│   │   │   └── tricycle_config.yaml
│   │   │
│   │   └── omnidirectional/         # 全向轮/麦克纳姆轮
│   │       ├── omni_kinematics.cpp
│   │       ├── mecanum_kinematics.cpp
│   │       └── omni_config.yaml
│   │
│   ├── legged/                      # 足式机器人
│   │   │
│   │   ├── quadruped/               # 四足机器人
│   │   │   ├── quad_kinematics.cpp
│   │   │   ├── quad_gait.cpp        # 步态生成
│   │   │   ├── quad_stability.cpp   # 稳定性约束
│   │   │   └── quad_config.yaml
│   │   │
│   │   └── bipedal/                 # 双足机器人
│   │       ├── biped_kinematics.cpp
│   │       ├── biped_balance.cpp    # 平衡控制
│   │       ├── biped_footstep.cpp   # 落脚点规划
│   │       └── biped_config.yaml
│   │
│   └── factory/                     # 机器人工厂
│       ├── robot_factory.hpp        # 工厂类
│       └── robot_registry.hpp       # 注册表
│
├── perception/                      # ★ 感知层 (局部环境感知)
│   │
│   ├── interface/                   # 感知接口
│   │   ├── i_local_mapper.hpp       # 局部建图接口
│   │   ├── i_obstacle_detector.hpp  # 障碍物检测接口
│   │   └── i_traversability.hpp     # 可通行性分析接口
│   │
│   ├── local_mapping/               # 局部建图
│   │   │
│   │   ├── nvblox/                  # ★ nvblox集成
│   │   │   ├── nvblox_adapter.cpp   # nvblox适配器
│   │   │   ├── tsdf_integrator.cpp  # TSDF集成
│   │   │   ├── esdf_generator.cpp   # ESDF生成
│   │   │   ├── mesh_extractor.cpp   # 网格提取
│   │   │   └── costmap_publisher.cpp # Nav2 Costmap发布
│   │   │
│   │   ├── voxel_grid/              # 体素栅格 (轻量级替代)
│   │   │   ├── voxel_grid_map.cpp
│   │   │   └── voxel_filter.cpp
│   │   │
│   │   └── elevation_map/           # 高程图 (足式机器人用)
│   │       ├── elevation_mapping.cpp
│   │       └── terrain_analysis.cpp
│   │
│   ├── obstacle/                    # 障碍物处理
│   │   ├── static_obstacle.cpp      # 静态障碍物
│   │   ├── dynamic_obstacle.cpp     # 动态障碍物追踪
│   │   ├── people_detector.cpp      # 行人检测 (nvblox people mode)
│   │   └── obstacle_predictor.cpp   # 障碍物轨迹预测
│   │
│   ├── traversability/              # 可通行性分析
│   │   ├── ground_segmentation.cpp  # 地面分割
│   │   ├── drivable_area.cpp        # 可行驶区域检测
│   │   ├── slope_analysis.cpp       # 坡度分析
│   │   └── roughness_analysis.cpp   # 粗糙度分析
│   │
│   └── sensor_fusion/               # 传感器融合
│       ├── depth_lidar_fusion.cpp   # 深度+雷达融合
│       ├── multi_sensor_sync.cpp    # 多传感器同步
│       └── odom_fusion.cpp          # 里程计融合
│
├── core/                            # ★ 导航核心层
│   │
│   ├── planner/                     # 路径规划器
│   │   │
│   │   ├── interface/               # 规划器接口
│   │   │   ├── i_global_planner.hpp
│   │   │   ├── i_local_planner.hpp
│   │   │   └── i_trajectory_planner.hpp
│   │   │
│   │   ├── global/                  # 全局规划器
│   │   │   ├── nav2_wrapper/        # Nav2封装
│   │   │   │   ├── navfn_wrapper.cpp
│   │   │   │   ├── smac_wrapper.cpp
│   │   │   │   └── theta_star_wrapper.cpp
│   │   │   │
│   │   │   ├── semantic/            # 语义规划器
│   │   │   │   ├── vln_planner.cpp  # VLN语言导航
│   │   │   │   └── topology_planner.cpp
│   │   │   │
│   │   │   └── hybrid/              # 混合规划器
│   │   │       └── semantic_global.cpp
│   │   │
│   │   ├── local/                   # 局部规划器
│   │   │   │
│   │   │   ├── wheeled/             # 轮式专用
│   │   │   │   ├── teb_wrapper.cpp  # TEB
│   │   │   │   ├── dwa_wrapper.cpp  # DWA
│   │   │   │   ├── mppi_wrapper.cpp # MPPI
│   │   │   │   └── rpp_wrapper.cpp  # Regulated Pure Pursuit
│   │   │   │
│   │   │   ├── legged/              # 足式专用
│   │   │   │   ├── footstep_planner.cpp  # 落脚点规划
│   │   │   │   └── body_path_planner.cpp # 躯干路径
│   │   │   │
│   │   │   └── learned/             # 学习型
│   │   │       ├── e2e_local.cpp    # 端到端
│   │   │       └── diffusion_local.cpp # 扩散策略
│   │   │
│   │   ├── model_based/             # 模型驱动规划器
│   │   │   │
│   │   │   ├── intern_nav/          # InternNav系列
│   │   │   │   ├── intern_vla_n1.cpp
│   │   │   │   ├── nav_dp.cpp
│   │   │   │   ├── stream_vln.cpp
│   │   │   │   └── intern_adapter.hpp
│   │   │   │
│   │   │   └── foundation/          # 基础模型
│   │   │       ├── gnm_planner.cpp
│   │   │       ├── vint_planner.cpp
│   │   │       └── nomad_planner.cpp
│   │   │
│   │   └── factory/                 # 规划器工厂
│   │       ├── planner_factory.hpp
│   │       └── planner_registry.hpp
│   │
│   ├── decision/                    # 决策引擎
│   │   ├── behavior_tree/           # 行为树
│   │   │   ├── bt_navigator.cpp
│   │   │   ├── bt_nodes/            # 自定义节点
│   │   │   └── bt_xml/              # 行为树XML
│   │   │
│   │   ├── mode_selector/           # 模式选择器
│   │   │   ├── mode_selector.cpp
│   │   │   ├── context_analyzer.cpp # 上下文分析
│   │   │   └── resource_checker.cpp # 资源检查
│   │   │
│   │   ├── state_machine/           # 状态机
│   │   │   └── nav_state_machine.cpp
│   │   │
│   │   └── recovery/                # 恢复策略
│   │       ├── recovery_manager.cpp
│   │       ├── spin_recovery.cpp
│   │       ├── backup_recovery.cpp
│   │       └── wait_recovery.cpp
│   │
│   ├── trajectory/                  # 轨迹处理
│   │   ├── fusion/                  # 轨迹融合
│   │   │   └── trajectory_fusion.cpp
│   │   │
│   │   ├── validation/              # 轨迹验证
│   │   │   ├── collision_checker.cpp
│   │   │   ├── kinematic_checker.cpp
│   │   │   └── dynamic_checker.cpp
│   │   │
│   │   ├── optimization/            # 轨迹优化
│   │   │   ├── time_optimal.cpp
│   │   │   ├── smoothing.cpp
│   │   │   └── shortcutting.cpp
│   │   │
│   │   └── execution/               # 轨迹执行
│   │       ├── trajectory_executor.cpp
│   │       └── progress_tracker.cpp
│   │
│   ├── map_manager/                 # 地图管理
│   │   ├── global_map/
│   │   │   ├── costmap_manager.cpp
│   │   │   ├── semantic_map.cpp
│   │   │   └── topology_map.cpp
│   │   │
│   │   ├── local_map/               # 局部地图
│   │   │   ├── rolling_costmap.cpp
│   │   │   └── nvblox_costmap.cpp   # nvblox输出
│   │   │
│   │   └── map_server/
│   │       └── map_io.cpp
│   │
│   └── stance/                      # 操作位姿
│       ├── stance_generator.cpp
│       └── manipulation_aware.cpp
│
├── interface/                       # ★ 外部接口层
│   │
│   ├── upstream/                    # 上游接口
│   │   ├── localization/            # 定位接口
│   │   │   ├── i_localization.hpp
│   │   │   ├── slam_adapter.cpp     # xyz-slam适配
│   │   │   └── amcl_adapter.cpp
│   │   │
│   │   ├── perception/              # 感知接口
│   │   │   ├── i_perception.hpp
│   │   │   └── perception_adapter.cpp
│   │   │
│   │   └── model_bridge/            # 模型桥接
│   │       ├── i_model_bridge.hpp
│   │       ├── intern_nav_bridge.cpp
│   │       └── vla_bridge.cpp
│   │
│   ├── downstream/                  # 下游接口
│   │   ├── controller/
│   │   │   ├── i_controller.hpp
│   │   │   ├── ros2_controller.cpp
│   │   │   └── direct_controller.cpp
│   │   │
│   │   └── output/
│   │       ├── cmd_vel_publisher.cpp
│   │       └── trajectory_publisher.cpp
│   │
│   ├── communication/               # 跨主机通信
│   │   ├── i_comm_channel.hpp       # 通信通道接口
│   │   │
│   │   ├── ros2/                    # ROS2本机
│   │   │   └── ros2_adapter.cpp
│   │   │
│   │   ├── mqtt/                    # MQTT跨主机
│   │   │   ├── mqtt_client.cpp
│   │   │   ├── mqtt_trajectory_pub.cpp
│   │   │   └── zhongli_protocol.cpp # 中力协议
│   │   │
│   │   ├── grpc/                    # gRPC
│   │   │   ├── grpc_client.cpp
│   │   │   └── model_inference.cpp
│   │   │
│   │   └── zeromq/                  # ZeroMQ
│   │       └── zmq_adapter.cpp
│   │
│   └── sensor/                      # 传感器接口
│       ├── i_sensor.hpp
│       ├── camera_adapter.cpp
│       ├── lidar_adapter.cpp
│       ├── depth_adapter.cpp
│       └── imu_adapter.cpp
│
├── simulation/                      # ★ 仿真层
│   │
│   ├── interface/                   # 仿真接口
│   │   ├── i_sim_adapter.hpp
│   │   ├── i_scene_manager.hpp
│   │   └── i_sensor_sim.hpp
│   │
│   ├── isaac_sim/                   # Isaac Sim
│   │   ├── isaac_sim_adapter.cpp
│   │   ├── ros2_bridge.cpp
│   │   └── scene_loader.cpp
│   │
│   ├── isaac_lab/                   # Isaac Lab (RL)
│   │   ├── isaac_lab_adapter.cpp
│   │   ├── gym_env_wrapper.cpp
│   │   └── policy_deployer.cpp
│   │
│   ├── gazebo/                      # Gazebo
│   │   └── gazebo_adapter.cpp
│   │
│   └── tools/                       # 仿真工具
│       ├── scene_builder.cpp
│       ├── trajectory_replay.cpp
│       └── benchmark_runner.cpp
│
├── app/                             # ★ 应用层
│   │
│   ├── sdk/                         # Python SDK
│   │   ├── neuro_nav/
│   │   │   ├── __init__.py
│   │   │   ├── agent.py             # MobilityAgent
│   │   │   ├── planner.py
│   │   │   └── simulation.py
│   │   └── setup.py
│   │
│   ├── ros2_node/                   # ROS2节点
│   │   ├── navigation_node.cpp
│   │   ├── perception_node.cpp
│   │   └── launch/
│   │
│   ├── cli/                         # 命令行工具
│   │   └── neuro_nav_cli.cpp
│   │
│   └── examples/                    # 示例
│       ├── forklift_nav.py
│       ├── quadruped_nav.py
│       └── vla_integration.py
│
├── nav2_plugins/                    # Nav2插件
│   ├── planners/
│   ├── controllers/
│   ├── costmap_layers/
│   │   └── nvblox_layer.cpp         # nvblox代价层
│   └── bt_nodes/
│
└── config/                          # 配置
    ├── robots/                      # 机器人配置
    │   ├── differential.yaml
    │   ├── ackermann.yaml
    │   ├── tricycle_forklift.yaml
    │   ├── omnidirectional.yaml
    │   ├── quadruped.yaml
    │   └── bipedal.yaml
    │
    ├── planners/                    # 规划器配置
    ├── perception/                  # 感知配置
    │   └── nvblox_config.yaml
    ├── communication/               # 通信配置
    └── simulation/                  # 仿真配置
```

---

## 核心接口设计

### 1. Common层 - 数据结构

```cpp
// common/data/pose.hpp
namespace neuro_nav::common {

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    
    Pose2D transform(const Pose2D& delta) const;
    Pose2D inverse() const;
    double distance_to(const Pose2D& other) const;
};

struct Pose3D {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    
    Pose3D transform(const Pose3D& delta) const;
    Pose2D to_2d() const;
};

// common/data/trajectory.hpp
struct TrajectoryPoint {
    Pose2D pose;
    Twist2D velocity;
    double timestamp;
    double curvature;
};

class Trajectory {
public:
    void add_point(const TrajectoryPoint& point);
    TrajectoryPoint interpolate(double t) const;
    double length() const;
    double duration() const;
    
    // 迭代器支持
    auto begin() { return points_.begin(); }
    auto end() { return points_.end(); }
    
private:
    std::vector<TrajectoryPoint> points_;
};

}  // namespace neuro_nav::common
```

### 2. 机器人接口 (ISP原则 - 接口隔离)

```cpp
// robot/interface/i_kinematics.hpp
namespace neuro_nav::robot {

// 运动学接口 - 只关注运动学计算
class IKinematics {
public:
    virtual ~IKinematics() = default;
    
    // 正运动学: 轮速/关节 → 机器人速度
    virtual Twist2D forward_kinematics(
        const std::vector<double>& joint_velocities) const = 0;
    
    // 逆运动学: 机器人速度 → 轮速/关节
    virtual std::vector<double> inverse_kinematics(
        const Twist2D& cmd_vel) const = 0;
    
    // 获取机器人类型
    virtual RobotType type() const = 0;
};

// 约束接口 - 只关注运动约束
class IConstraints {
public:
    virtual ~IConstraints() = default;
    
    // 速度约束
    virtual VelocityLimits velocity_limits() const = 0;
    
    // 加速度约束
    virtual AccelerationLimits acceleration_limits() const = 0;
    
    // 曲率约束 (转弯半径)
    virtual double min_turning_radius() const = 0;
    
    // 验证轨迹是否满足约束
    virtual bool is_feasible(const Trajectory& trajectory) const = 0;
};

// 足迹接口 - 只关注碰撞检测用足迹
class IFootprint {
public:
    virtual ~IFootprint() = default;
    
    // 获取多边形足迹
    virtual Polygon2D footprint() const = 0;
    
    // 获取给定位姿下的足迹
    virtual Polygon2D footprint_at(const Pose2D& pose) const = 0;
    
    // 碰撞检测半径
    virtual double inscribed_radius() const = 0;
    virtual double circumscribed_radius() const = 0;
};

}  // namespace neuro_nav::robot
```

### 3. 机器人类型实现 (OCP原则 - 开闭原则)

```cpp
// robot/wheeled/tricycle/tricycle_kinematics.cpp
namespace neuro_nav::robot::wheeled {

class TricycleKinematics : public IKinematics {
public:
    explicit TricycleKinematics(const TricycleConfig& config)
        : wheelbase_(config.wheelbase),
          max_steer_angle_(config.max_steer_angle) {}
    
    Twist2D forward_kinematics(
        const std::vector<double>& inputs) const override {
        // inputs: [rear_wheel_vel, steer_angle]
        double v = inputs[0];
        double delta = inputs[1];
        
        return Twist2D{
            .linear_x = v * std::cos(delta),
            .linear_y = 0.0,  // 非全向
            .angular_z = v * std::tan(delta) / wheelbase_
        };
    }
    
    std::vector<double> inverse_kinematics(
        const Twist2D& cmd_vel) const override {
        double v = cmd_vel.linear_x;
        double omega = cmd_vel.angular_z;
        
        // 计算转向角
        double delta = std::atan2(omega * wheelbase_, v);
        delta = std::clamp(delta, -max_steer_angle_, max_steer_angle_);
        
        return {v, delta};
    }
    
    RobotType type() const override { return RobotType::TRICYCLE; }
    
private:
    double wheelbase_;
    double max_steer_angle_;
};

}  // namespace neuro_nav::robot::wheeled
```

### 4. 机器人工厂 (工厂模式)

```cpp
// robot/factory/robot_factory.hpp
namespace neuro_nav::robot {

// 机器人类型枚举
enum class RobotType {
    // 轮式
    DIFFERENTIAL,
    ACKERMANN,
    TRICYCLE,
    OMNIDIRECTIONAL,
    MECANUM,
    // 足式
    QUADRUPED,
    BIPEDAL,
};

// 机器人组件包
struct RobotComponents {
    std::unique_ptr<IKinematics> kinematics;
    std::unique_ptr<IConstraints> constraints;
    std::unique_ptr<IFootprint> footprint;
};

// 工厂类
class RobotFactory {
public:
    static RobotFactory& instance();
    
    // 根据类型创建机器人组件
    RobotComponents create(RobotType type, const YAML::Node& config);
    
    // 根据配置文件创建
    RobotComponents create_from_config(const std::string& config_path);
    
    // 注册新机器人类型 (扩展用)
    template<typename KinematicsT, typename ConstraintsT, typename FootprintT>
    void register_robot(RobotType type);
    
private:
    std::map<RobotType, CreatorFunc> creators_;
};

// 使用示例
auto robot = RobotFactory::instance().create(
    RobotType::TRICYCLE, 
    YAML::LoadFile("config/robots/tricycle_forklift.yaml")
);
```

---

## 感知层设计 (nvblox集成)

### 局部感知架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Perception Layer                                  │
│                                                                          │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                 │
│  │   Camera    │    │   LiDAR     │    │   Depth     │                 │
│  │   Adapter   │    │   Adapter   │    │   Adapter   │                 │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘                 │
│         │                  │                  │                         │
│         └──────────────────┼──────────────────┘                         │
│                            ▼                                            │
│              ┌─────────────────────────┐                               │
│              │    Sensor Fusion        │                               │
│              │   (时间同步/配准)        │                               │
│              └────────────┬────────────┘                               │
│                           │                                             │
│         ┌─────────────────┼─────────────────┐                          │
│         ▼                 ▼                 ▼                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                    │
│  │  nvblox     │  │ Elevation   │  │  VoxelGrid  │                    │
│  │  Adapter    │  │  Mapping    │  │  (轻量级)   │                    │
│  │             │  │ (足式专用)  │  │             │                    │
│  │ TSDF→ESDF   │  │             │  │             │                    │
│  │ →Costmap    │  │             │  │             │                    │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘                    │
│         │                │                │                            │
│         └────────────────┼────────────────┘                            │
│                          ▼                                             │
│              ┌─────────────────────────┐                               │
│              │   Local Costmap Output  │                               │
│              │   (Nav2 Costmap2D)      │                               │
│              └─────────────────────────┘                               │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### nvblox适配器接口

```cpp
// perception/local_mapping/nvblox/nvblox_adapter.hpp
namespace neuro_nav::perception {

class NvbloxAdapter : public ILocalMapper {
public:
    struct Config {
        double voxel_size = 0.05;           // 体素大小
        double tsdf_truncation = 0.3;       // TSDF截断距离
        double esdf_max_distance = 2.0;     // ESDF最大距离
        bool enable_people_detection = true; // 行人检测
        int gpu_device_id = 0;
    };
    
    explicit NvbloxAdapter(const Config& config);
    
    // === ILocalMapper接口实现 ===
    
    // 集成深度图像
    void integrate_depth(
        const DepthImage& depth,
        const Pose3D& camera_pose,
        const CameraIntrinsics& intrinsics) override;
    
    // 集成点云 (LiDAR)
    void integrate_pointcloud(
        const PointCloud& cloud,
        const Pose3D& sensor_pose) override;
    
    // 获取局部代价地图 (用于Nav2)
    OccupancyGrid get_costmap(
        const Pose2D& robot_pose,
        double width, double height,
        double resolution) override;
    
    // 获取ESDF (用于轨迹优化)
    EsdfGrid get_esdf() const;
    
    // 获取网格 (可视化)
    Mesh get_mesh() const;
    
    // === 扩展功能 ===
    
    // 查询指定点的距离场值
    double query_distance(const Eigen::Vector3d& point) const;
    
    // 批量查询 (轨迹碰撞检测)
    std::vector<double> query_distances(
        const std::vector<Eigen::Vector3d>& points) const;
    
    // 清除动态障碍物
    void clear_dynamic_objects();
    
private:
    std::unique_ptr<nvblox::Mapper> mapper_;
    std::unique_ptr<nvblox::EsdfIntegrator> esdf_integrator_;
    Config config_;
};

}  // namespace neuro_nav::perception
```

### nvblox与Nav2集成

```cpp
// nav2_plugins/costmap_layers/nvblox_layer.cpp
namespace neuro_nav::nav2_plugins {

class NvbloxCostmapLayer : public nav2_costmap_2d::CostmapLayer {
public:
    void onInitialize() override {
        // 初始化nvblox适配器
        nvblox_adapter_ = std::make_unique<perception::NvbloxAdapter>(config_);
        
        // 订阅深度图像
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "depth", 10, 
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                handle_depth(msg);
            });
        
        // 订阅位姿
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                current_pose_ = msg;
            });
    }
    
    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                      double* min_x, double* min_y,
                      double* max_x, double* max_y) override {
        // 更新局部地图边界
        *min_x = robot_x - update_radius_;
        *max_x = robot_x + update_radius_;
        *min_y = robot_y - update_radius_;
        *max_y = robot_y + update_radius_;
    }
    
    void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                     int min_i, int min_j,
                     int max_i, int max_j) override {
        // 从nvblox获取代价地图
        auto costmap = nvblox_adapter_->get_costmap(
            Pose2D{robot_x_, robot_y_, robot_yaw_},
            update_radius_ * 2, update_radius_ * 2,
            resolution_);
        
        // 更新到master costmap
        for (int j = min_j; j < max_j; ++j) {
            for (int i = min_i; i < max_i; ++i) {
                unsigned char cost = costmap.at(i - min_i, j - min_j);
                master_grid.setCost(i, j, cost);
            }
        }
    }
    
private:
    std::unique_ptr<perception::NvbloxAdapter> nvblox_adapter_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    double update_radius_ = 5.0;
};

}  // namespace neuro_nav::nav2_plugins
```

---

## 规划器分类设计

### 规划器选择策略

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      Planner Selection Strategy                          │
│                                                                          │
│                       ┌─────────────────┐                               │
│                       │  Mode Selector  │                               │
│                       └────────┬────────┘                               │
│                                │                                         │
│         ┌──────────────────────┼──────────────────────┐                 │
│         │                      │                      │                 │
│         ▼                      ▼                      ▼                 │
│  ┌─────────────┐       ┌─────────────┐       ┌─────────────┐           │
│  │ Robot Type  │       │   Scene     │       │  Resources  │           │
│  │  Checker    │       │  Analyzer   │       │   Checker   │           │
│  └──────┬──────┘       └──────┬──────┘       └──────┬──────┘           │
│         │                      │                      │                 │
│         └──────────────────────┼──────────────────────┘                 │
│                                ▼                                         │
│                   ┌────────────────────────┐                            │
│                   │    Planner Selector    │                            │
│                   └────────────┬───────────┘                            │
│                                │                                         │
│    ┌───────────────────────────┼───────────────────────────┐            │
│    │                           │                           │            │
│    ▼                           ▼                           ▼            │
│ ┌─────────┐              ┌─────────┐              ┌─────────┐          │
│ │ Wheeled │              │ Legged  │              │ Model   │          │
│ │Planners │              │Planners │              │ Based   │          │
│ └────┬────┘              └────┬────┘              └────┬────┘          │
│      │                        │                        │                │
│  ┌───┴───┐              ┌─────┴─────┐            ┌─────┴─────┐         │
│  │       │              │           │            │           │         │
│  ▼       ▼              ▼           ▼            ▼           ▼         │
│ TEB    MPPI         Footstep    BodyPath     InternNav    GNM/ViNT    │
│ DWA    RPP          Planner     Planner      VLA-N1       NoMaD       │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 规划器接口层次

```cpp
// core/planner/interface/i_local_planner.hpp
namespace neuro_nav::core::planner {

// 基础局部规划器接口
class ILocalPlanner {
public:
    virtual ~ILocalPlanner() = default;
    
    // 配置
    virtual void configure(const YAML::Node& config) = 0;
    
    // 设置全局路径
    virtual void set_global_path(const Path& path) = 0;
    
    // 计算速度指令
    virtual Twist2D compute_velocity(
        const Pose2D& current_pose,
        const Twist2D& current_velocity) = 0;
    
    // 是否到达目标
    virtual bool is_goal_reached() const = 0;
    
    // 获取规划轨迹 (可视化用)
    virtual Trajectory get_planned_trajectory() const = 0;
};

// 轮式机器人局部规划器 (扩展接口)
class IWheeledLocalPlanner : public ILocalPlanner {
public:
    // 设置机器人运动学模型
    virtual void set_kinematics(
        std::shared_ptr<robot::IKinematics> kinematics) = 0;
    
    // 设置机器人约束
    virtual void set_constraints(
        std::shared_ptr<robot::IConstraints> constraints) = 0;
};

// 足式机器人局部规划器 (扩展接口)
class ILeggedLocalPlanner : public ILocalPlanner {
public:
    // 设置落脚点约束
    virtual void set_foothold_constraints(
        const FootholdConstraints& constraints) = 0;
    
    // 获取落脚点序列
    virtual std::vector<Foothold> get_footholds() const = 0;
    
    // 设置地形信息
    virtual void set_terrain(
        std::shared_ptr<perception::ElevationMap> terrain) = 0;
};

// 模型驱动规划器接口
class IModelBasedPlanner : public ILocalPlanner {
public:
    // 设置观测数据
    virtual void set_observation(const Observation& obs) = 0;
    
    // 获取系统模式 (S1/S2/Dual)
    virtual SystemMode get_system_mode() const = 0;
    
    // 是否需要地图
    virtual bool requires_map() const = 0;
};

}  // namespace neuro_nav::core::planner
```

### 规划器工厂

```cpp
// core/planner/factory/planner_factory.hpp
namespace neuro_nav::core::planner {

class PlannerFactory {
public:
    static PlannerFactory& instance();
    
    // 创建适合特定机器人类型的规划器
    std::unique_ptr<ILocalPlanner> create_local_planner(
        robot::RobotType robot_type,
        PlannerType planner_type,
        const YAML::Node& config);
    
    // 创建全局规划器
    std::unique_ptr<IGlobalPlanner> create_global_planner(
        PlannerType planner_type,
        const YAML::Node& config);
    
    // 获取机器人类型支持的规划器列表
    std::vector<PlannerType> supported_planners(
        robot::RobotType robot_type) const;
    
private:
    // 规划器类型 → 机器人类型 兼容性映射
    std::map<PlannerType, std::set<robot::RobotType>> compatibility_;
};

// 规划器类型枚举
enum class PlannerType {
    // 全局
    NAVFN, SMAC_2D, SMAC_HYBRID, THETA_STAR,
    VLN, TOPOLOGY,
    
    // 局部 - 轮式
    TEB, DWA, MPPI, RPP,
    
    // 局部 - 足式
    FOOTSTEP, BODY_PATH,
    
    // 局部 - 模型驱动
    INTERN_VLA_N1, NAV_DP, GNM, VINT, NOMAD,
};

}  // namespace neuro_nav::core::planner
```

---

## 配置示例

### 机器人配置 (三轮叉车)

```yaml
# config/robots/tricycle_forklift.yaml
robot:
  name: "zhongli_forklift"
  type: "tricycle"
  
  kinematics:
    wheelbase: 1.5          # 轴距 (m)
    wheel_radius: 0.25      # 驱动轮半径
    max_steer_angle: 1.57   # 最大转向角 (rad, 90°)
    
  constraints:
    max_linear_velocity: 2.0      # m/s
    max_angular_velocity: 1.0     # rad/s
    max_linear_acceleration: 1.0  # m/s²
    max_angular_acceleration: 2.0 # rad/s²
    min_turning_radius: 1.5       # m
    
  footprint:
    type: "polygon"
    vertices:
      - [1.5, 0.6]
      - [1.5, -0.6]
      - [-0.5, -0.6]
      - [-0.5, 0.6]
    inscribed_radius: 0.5
    circumscribed_radius: 1.6
    
  fork:  # 叉车特有
    lift_range: [0.0, 3.0]  # 升降范围 (m)
    fork_length: 1.2        # 货叉长度
    fork_width: 0.8         # 货叉宽度
```

### nvblox配置

```yaml
# config/perception/nvblox_config.yaml
nvblox:
  # 体素参数
  voxel_size: 0.05        # 5cm分辨率
  
  # TSDF参数
  tsdf:
    truncation_distance: 0.3
    max_weight: 100.0
    
  # ESDF参数
  esdf:
    max_distance: 2.0
    
  # 传感器配置
  sensors:
    depth_camera:
      enabled: true
      topic: "/camera/depth/image_raw"
      max_range: 5.0
      
    lidar:
      enabled: true
      topic: "/scan"
      
  # 动态障碍物
  dynamic:
    people_detection: true
    people_segmentation_topic: "/people_mask"
    
  # 代价地图输出
  costmap:
    update_rate: 10.0
    publish_topic: "/nvblox_costmap"
```

---

## 使用示例

### Python SDK

```python
from neuro_nav import MobilityAgent, RobotType, PlannerType

# 创建叉车导航代理
agent = MobilityAgent(
    robot_type=RobotType.TRICYCLE,
    config_path="config/robots/tricycle_forklift.yaml"
)

# 配置感知 (使用nvblox)
agent.configure_perception(
    local_mapper="nvblox",
    config_path="config/perception/nvblox_config.yaml"
)

# 配置规划器
agent.configure_planner(
    global_planner=PlannerType.SMAC_HYBRID,  # 混合A*
    local_planner=PlannerType.TEB,           # TEB (适合三轮叉车)
)

# 导航到目标
result = agent.go_to(x=5.0, y=3.0, theta=0.0)

# === 切换到四足机器人 ===
quad_agent = MobilityAgent(
    robot_type=RobotType.QUADRUPED,
    config_path="config/robots/quadruped.yaml"
)

quad_agent.configure_perception(
    local_mapper="elevation_map",  # 高程图 (适合足式)
)

quad_agent.configure_planner(
    local_planner=PlannerType.FOOTSTEP,  # 落脚点规划
)

# === 使用模型驱动规划器 ===
agent.configure_planner(
    local_planner=PlannerType.NAV_DP,  # NavDP扩散策略
    model_config={
        "inference_mode": "local",  # 本地TensorRT推理
        "model_path": "/models/nav_dp.onnx"
    }
)

agent.go_to(instruction="去货架A-03旁边")
```

---

## 实施路线 (v4)

### Phase 1: Common + Robot抽象 (2周)
- [ ] common模块实现
- [ ] 机器人接口定义
- [ ] 三轮叉车/差速/阿克曼实现
- [ ] 机器人工厂

### Phase 2: 感知层 + nvblox (2周)
- [ ] 感知接口定义
- [ ] nvblox适配器
- [ ] Nav2 Costmap Layer
- [ ] 传感器融合

### Phase 3: 规划器分类 (3周)
- [ ] 规划器接口层次
- [ ] 轮式规划器封装
- [ ] 足式规划器框架
- [ ] 规划器工厂

### Phase 4: 决策 + 通信 + 仿真 (3周)
- [ ] 决策引擎
- [ ] 跨主机通信
- [ ] 仿真适配层

### Phase 5: 模型驱动 + 集成 (2周)
- [ ] InternNav集成
- [ ] SDK + 示例
- [ ] 文档
