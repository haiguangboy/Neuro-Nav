# Neuro-Nav 导航框架架构设计 v5

## 版本演进
- v1-v3: 基础框架、通信层、仿真层
- v4: 软件设计原则重构、common层、nvblox集成
- **v5: 整合架构审查意见，明确Nav2边界、跨本体执行接口、环境模型收敛**

---

## 核心设计决策

### 决策1: Nav2定位 — "执行与编排内核"

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    Nav2 vs Neuro-Nav 职责划分                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                         Nav2 负责                                │   │
│  │                                                                   │   │
│  │  • Lifecycle管理 (节点生命周期)                                   │   │
│  │  • BT编排 (bt_navigator)                                         │   │
│  │  • Action接口 (NavigateToPose/ThroughPoses/FollowPath)           │   │
│  │  • Planner/Controller/Behavior Server框架                        │   │
│  │  • Costmap2D框架                                                  │   │
│  │  • Recovery行为框架                                               │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                    │                                    │
│                                    ▼                                    │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                      Neuro-Nav 负责                              │   │
│  │                                                                   │   │
│  │  • 跨本体能力抽象 (轮式/足式/人形)                                │   │
│  │  • 环境模型融合 (ESDF/Elevation/Traversability)                  │   │
│  │  • 学习型规划器适配 (VLN/扩散/GNM/InternNav)                     │   │
│  │  • Nav2插件实现:                                                  │   │
│  │      - Planner Plugins (VLN/GNM/Hybrid等)                        │   │
│  │      - Controller Plugins (ESDF-aware MPPI等)                    │   │
│  │      - Costmap Layers (nvblox_layer等)                           │   │
│  │      - BT Nodes (VLN降级/模式切换等)                             │   │
│  │  • 跨主机通信 (MQTT/gRPC)                                        │   │
│  │  • 仿真适配                                                       │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  原则: 不重复实现Nav2已有的Server/BT框架，而是通过插件扩展              │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 决策2: 跨本体执行接口 — "导航层 + 运动层"分离

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      跨本体执行架构                                       │
│                                                                          │
│                    ┌─────────────────────┐                              │
│                    │    Navigation Layer │  ← Neuro-Nav核心              │
│                    │   (导航意图生成)     │                              │
│                    │                     │                              │
│                    │  输出: Body Path /  │                              │
│                    │       Body Trajectory│                              │
│                    └──────────┬──────────┘                              │
│                               │                                          │
│              ┌────────────────┼────────────────┐                        │
│              │                │                │                        │
│              ▼                ▼                ▼                        │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐              │
│  │ Wheeled       │  │ Legged        │  │ Humanoid      │              │
│  │ Locomotion    │  │ Locomotion    │  │ Locomotion    │              │
│  │               │  │               │  │               │              │
│  │ cmd_vel /     │  │ Body Traj +   │  │ Whole Body    │              │
│  │ wheel_joints  │  │ Footholds     │  │ Trajectory    │              │
│  └───────────────┘  └───────────────┘  └───────────────┘              │
│                                                                          │
│  关键: 导航层统一输出"机体轨迹意图"，运动层按本体类型转换为执行指令      │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 决策3: 环境模型收敛 — "统一IEnvironmentModel"

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      Environment Model 统一接口                          │
│                                                                          │
│                         IEnvironmentModel                               │
│                               │                                          │
│         ┌─────────────────────┼─────────────────────┐                   │
│         │                     │                     │                   │
│         ▼                     ▼                     ▼                   │
│  ┌─────────────┐      ┌─────────────┐      ┌─────────────┐            │
│  │ 2D Costmap  │      │ 3D ESDF     │      │ 2.5D Elev   │            │
│  │ (Nav2用)    │      │ (轨迹优化)  │      │ (足式地形)  │            │
│  └─────────────┘      └─────────────┘      └─────────────┘            │
│         │                     │                     │                   │
│         └─────────────────────┼─────────────────────┘                   │
│                               │                                          │
│                     ┌─────────▼─────────┐                               │
│                     │   数据源融合       │                               │
│                     │                   │                               │
│                     │  nvblox (TSDF)    │                               │
│                     │  LiDAR点云        │                               │
│                     │  深度相机         │                               │
│                     └───────────────────┘                               │
│                                                                          │
│  投影规则:                                                               │
│  • 3D→2D: 高度阈值切片 + 地面分割 + 动态物体处理                        │
│  • Traversability→代价: 按机器人Profile参数化 (轮式严格/足式宽松)       │
│  • Unknown Space: 轮式=障碍(保守) / 足式=可通行(激进) / 可配置           │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 完整框架结构 (v5)

```
**neuro-nav** (v5)
│
├── common/                          # 公共基础层 (unchanged)
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
│   │
│   ├── interface/                   # 机器人接口
│   │   ├── i_kinematics.hpp         # 运动学
│   │   ├── i_dynamics.hpp           # 动力学
│   │   ├── i_footprint.hpp          # 足迹
│   │   ├── i_constraints.hpp        # 约束
│   │   └── i_robot_profile.hpp      # ★ 新增: 机器人特征Profile
│   │
│   ├── wheeled/                     # 轮式 (differential/ackermann/tricycle/omni)
│   ├── legged/                      # 足式 (quadruped/bipedal)
│   └── factory/                     # 工厂
│
├── perception/                      # ★ 感知层 (重构)
│   │
│   ├── interface/
│   │   ├── i_local_mapper.hpp       # 局部建图接口
│   │   └── i_environment_model.hpp  # ★ 新增: 统一环境模型接口
│   │
│   ├── environment_model/           # ★ 新增: 环境模型实现
│   │   ├── environment_model.cpp    # IEnvironmentModel实现
│   │   ├── projection_rules.cpp     # 3D→2D投影规则
│   │   ├── traversability_mapper.cpp # Traversability→代价映射
│   │   └── unknown_space_policy.cpp # Unknown空间处理策略
│   │
│   ├── local_mapping/
│   │   ├── nvblox/                  # nvblox集成
│   │   ├── voxel_grid/              # 轻量级体素
│   │   └── elevation_map/           # 高程图
│   │
│   ├── obstacle/                    # 障碍物处理
│   ├── traversability/              # 可通行性分析
│   └── sensor_fusion/               # 传感器融合
│
├── core/                            # ★ 导航核心层 (重构)
│   │
│   ├── navigation_command/          # ★ 新增: 跨本体导航指令
│   │   ├── i_command_output.hpp     # 输出接口
│   │   ├── navigation_command.hpp   # 导航指令结构
│   │   ├── twist_output.cpp         # 轮式: cmd_vel
│   │   ├── body_trajectory_output.cpp # 足式: 躯干轨迹
│   │   └── footstep_output.cpp      # 足式: 落脚点序列
│   │
│   ├── planner/                     # 路径规划 (重构为Nav2插件导向)
│   │   │
│   │   ├── interface/
│   │   │   ├── i_nav2_planner_plugin.hpp  # ★ Nav2 Planner插件接口
│   │   │   └── i_nav2_controller_plugin.hpp # ★ Nav2 Controller插件接口
│   │   │
│   │   ├── nav2_plugins/            # ★ 移动到此: Nav2插件实现
│   │   │   │
│   │   │   ├── planners/            # Global Planner Plugins
│   │   │   │   ├── vln_planner_plugin.cpp      # VLN全局规划
│   │   │   │   ├── gnm_planner_plugin.cpp      # GNM规划
│   │   │   │   └── hybrid_vln_plugin.cpp       # VLN+传统混合
│   │   │   │
│   │   │   ├── controllers/         # Local Controller Plugins
│   │   │   │   ├── esdf_mppi_plugin.cpp        # ESDF感知的MPPI
│   │   │   │   ├── legged_controller_plugin.cpp # 足式控制器
│   │   │   │   └── diffusion_controller_plugin.cpp # 扩散策略
│   │   │   │
│   │   │   ├── costmap_layers/      # Costmap Layer Plugins
│   │   │   │   ├── nvblox_layer.cpp            # nvblox 2D投影
│   │   │   │   ├── esdf_layer.cpp              # ESDF距离层
│   │   │   │   └── traversability_layer.cpp    # 可通行性层
│   │   │   │
│   │   │   └── bt_nodes/            # BT Node Plugins
│   │   │       ├── vln_fallback_node.cpp       # ★ VLN降级节点
│   │   │       ├── model_confidence_node.cpp   # 模型置信度检查
│   │   │       ├── locomotion_switch_node.cpp  # 运动模式切换
│   │   │       └── esdf_safety_node.cpp        # ESDF安全检查
│   │   │
│   │   ├── model_based/             # 模型驱动规划器核心逻辑
│   │   │   ├── intern_nav/          # InternNav系列
│   │   │   │   ├── intern_vla_n1.cpp
│   │   │   │   ├── nav_dp.cpp
│   │   │   │   └── stream_vln.cpp
│   │   │   └── foundation/          # GNM/ViNT/NoMaD
│   │   │
│   │   └── vln_integration/         # ★ 新增: VLN集成模块
│   │       ├── vln_contract.hpp     # VLN输出契约定义
│   │       ├── vln_path_adapter.cpp # VLN→nav_msgs/Path适配
│   │       ├── vln_prior_fusion.cpp # VLN作为软约束融合
│   │       └── vln_fallback.cpp     # 降级策略实现
│   │
│   ├── decision/                    # 决策引擎 (简化)
│   │   ├── mode_selector/           # 模式选择器
│   │   └── recovery/                # 恢复策略 (作为Nav2 BT节点)
│   │
│   ├── trajectory/                  # 轨迹处理
│   │   ├── fusion/                  # 多源融合 + VLN prior
│   │   ├── validation/              # 可行性验证 (使用ESDF)
│   │   ├── optimization/            # 轨迹优化 (ESDF梯度)
│   │   └── execution/               # 执行跟踪
│   │
│   └── map_manager/                 # 地图管理 (简化，依赖IEnvironmentModel)
│
├── interface/                       # 外部接口层
│   │
│   ├── upstream/                    # 上游 (定位/感知/模型)
│   │
│   ├── downstream/                  # ★ 下游 (重构)
│   │   │
│   │   ├── locomotion/              # ★ 新增: 运动层适配
│   │   │   ├── i_locomotion_interface.hpp  # 运动层接口
│   │   │   ├── wheeled_locomotion.cpp      # 轮式: cmd_vel发布
│   │   │   ├── legged_locomotion.cpp       # 足式: 躯干+落脚点
│   │   │   └── humanoid_locomotion.cpp     # 人形: 全身轨迹
│   │   │
│   │   └── output/                  # 原有输出 (保留兼容)
│   │
│   ├── communication/               # 跨主机通信 (unchanged)
│   └── sensor/                      # 传感器接口 (unchanged)
│
├── simulation/                      # 仿真层 (unchanged)
│
├── app/                             # 应用层
│   │
│   ├── ros2_bringup/                # ★ 重命名: ROS2启动
│   │   ├── nav2_bringup.launch.py   # 拉起Nav2 + Neuro-Nav插件
│   │   ├── perception_bringup.launch.py
│   │   └── full_system.launch.py
│   │
│   ├── sdk/                         # Python SDK
│   └── cli/                         # 命令行
│
└── config/
    ├── robots/                      # 机器人配置 (增加profile)
    ├── nav2/                        # ★ 新增: Nav2配置
    │   ├── nav2_params.yaml         # Nav2参数
    │   ├── bt_navigator.xml         # BT定义 (使用Neuro-Nav节点)
    │   └── controller_params.yaml
    ├── perception/
    │   ├── nvblox_config.yaml
    │   └── environment_model.yaml   # ★ 新增: 环境模型配置
    └── vln/                         # ★ 新增: VLN配置
        ├── vln_contract.yaml        # VLN契约配置
        └── fallback_policy.yaml     # 降级策略配置
```

---

## 核心接口定义

### 1. 统一环境模型接口

```cpp
// perception/interface/i_environment_model.hpp
namespace neuro_nav::perception {

// 可通行性单元 (用于足式机器人)
struct TraversabilityCell {
    float traversability;   // [0,1] 可通行度
    float slope;            // 坡度 (rad)
    float roughness;        // 粗糙度
    float step_height;      // 台阶高度
    bool is_unknown;        // 是否未知区域
};

// Unknown空间处理策略
enum class UnknownSpacePolicy {
    OBSTACLE,       // 保守: 当作障碍 (轮式默认)
    FREE,           // 激进: 当作可通行 (足式探索)
    COST_GRADIENT,  // 渐变: 距离已知区域越远代价越高
    CONFIGURABLE    // 可配置阈值
};

// 统一环境模型接口
class IEnvironmentModel {
public:
    virtual ~IEnvironmentModel() = default;

    // === 2D Costmap (Nav2 costmap_2d 兼容) ===
    virtual common::OccupancyGrid get_local_costmap_2d(
        const common::Pose2D& robot_pose,
        double width, double height,
        double resolution) const = 0;

    // === 3D ESDF (轨迹优化/碰撞检测) ===
    virtual EsdfGrid get_esdf_3d() const = 0;
    
    // 单点距离查询
    virtual double query_distance(const Eigen::Vector3d& point) const = 0;
    
    // 批量距离查询 (轨迹验证用)
    virtual std::vector<double> query_distances(
        const std::vector<Eigen::Vector3d>& points) const = 0;
    
    // 距离梯度查询 (轨迹优化用)
    virtual Eigen::Vector3d query_distance_gradient(
        const Eigen::Vector3d& point) const = 0;

    // === 2.5D Elevation + Traversability (足式机器人) ===
    virtual ElevationMap get_elevation_map() const = 0;
    virtual TraversabilityGrid get_traversability_grid() const = 0;
    
    // 落脚点可行性查询
    virtual bool is_foothold_valid(
        const Eigen::Vector3d& position,
        const FootholdConstraints& constraints) const = 0;

    // === 配置 ===
    virtual void set_robot_profile(const RobotProfile& profile) = 0;
    virtual void set_unknown_space_policy(UnknownSpacePolicy policy) = 0;
};

}  // namespace neuro_nav::perception
```

### 2. 跨本体导航指令接口

```cpp
// core/navigation_command/navigation_command.hpp
namespace neuro_nav::core {

// 导航指令类型
enum class CommandType {
    TWIST_2D,           // 轮式: geometry_msgs/Twist
    WHEEL_JOINTS,       // 轮式: 轮速/舵角
    BODY_TRAJECTORY,    // 足式/人形: 躯干SE3轨迹
    FOOTSTEP_PLAN,      // 足式: 落脚点序列
    WHOLE_BODY_TRAJ,    // 人形: 全身关节轨迹
};

// 落脚点定义
struct Foothold {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    int leg_id;                    // 腿编号
    double timestamp;              // 时间戳
    double terrain_height;         // 地形高度
    double terrain_normal[3];      // 地形法向量
};

// 统一导航指令
struct NavigationCommand {
    CommandType type;
    
    // 通用载体 (导航意图)
    common::Trajectory body_trajectory;  // 躯干轨迹 (SE2/SE3)
    
    // 轮式专用
    common::Twist2D twist;               // cmd_vel
    std::vector<double> wheel_commands;  // 轮速/舵角
    
    // 足式专用
    std::vector<Foothold> footholds;     // 落脚点序列
    
    // 人形专用
    std::vector<std::vector<double>> joint_trajectory;  // 全身关节轨迹
    
    // 元信息
    double timestamp;
    double confidence;                   // 置信度 (模型输出)
    std::string source;                  // 来源 (nav2/vln/diffusion等)
};

// 导航指令输出接口
class ICommandOutput {
public:
    virtual ~ICommandOutput() = default;
    
    // 发布导航指令
    virtual void publish(const NavigationCommand& cmd) = 0;
    
    // 获取支持的指令类型
    virtual std::vector<CommandType> supported_types() const = 0;
    
    // 是否可以接受该指令
    virtual bool can_accept(CommandType type) const = 0;
};

}  // namespace neuro_nav::core
```

### 3. 运动层接口 (Locomotion Layer)

```cpp
// interface/downstream/locomotion/i_locomotion_interface.hpp
namespace neuro_nav::interface {

// 运动层接口 — 将导航指令转换为本体可执行指令
class ILocomotionInterface {
public:
    virtual ~ILocomotionInterface() = default;
    
    // 获取本体类型
    virtual robot::RobotType robot_type() const = 0;
    
    // 接受的导航指令类型
    virtual std::vector<core::CommandType> accepted_command_types() const = 0;
    
    // 执行导航指令
    virtual bool execute(const core::NavigationCommand& cmd) = 0;
    
    // 获取执行状态
    virtual LocomotionStatus get_status() const = 0;
    
    // 紧急停止
    virtual void emergency_stop() = 0;
};

// 轮式运动层
class WheeledLocomotion : public ILocomotionInterface {
public:
    robot::RobotType robot_type() const override { 
        return robot::RobotType::WHEELED; 
    }
    
    std::vector<core::CommandType> accepted_command_types() const override {
        return {CommandType::TWIST_2D, CommandType::WHEEL_JOINTS, 
                CommandType::BODY_TRAJECTORY};  // 躯干轨迹可转换为cmd_vel
    }
    
    bool execute(const core::NavigationCommand& cmd) override {
        if (cmd.type == CommandType::BODY_TRAJECTORY) {
            // 从躯干轨迹提取cmd_vel
            auto twist = trajectory_to_twist(cmd.body_trajectory);
            publish_cmd_vel(twist);
        } else if (cmd.type == CommandType::TWIST_2D) {
            publish_cmd_vel(cmd.twist);
        }
        return true;
    }
};

// 足式运动层
class LeggedLocomotion : public ILocomotionInterface {
public:
    robot::RobotType robot_type() const override { 
        return robot::RobotType::LEGGED; 
    }
    
    std::vector<core::CommandType> accepted_command_types() const override {
        return {CommandType::BODY_TRAJECTORY, CommandType::FOOTSTEP_PLAN};
    }
    
    bool execute(const core::NavigationCommand& cmd) override {
        // 发布躯干轨迹 + 落脚点到运动控制器
        publish_body_trajectory(cmd.body_trajectory);
        if (!cmd.footholds.empty()) {
            publish_footholds(cmd.footholds);
        }
        return true;
    }
};

}  // namespace neuro_nav::interface
```

### 4. VLN集成契约

```cpp
// core/planner/vln_integration/vln_contract.hpp
namespace neuro_nav::core::vln {

// VLN输出类型
enum class VlnOutputType {
    WAYPOINTS,          // 离散航点 (NavigateThroughPoses)
    PATH,               // 路径 (nav_msgs/Path)
    TIMED_TRAJECTORY,   // 带时间的轨迹
    ACTION_SEQUENCE,    // 动作序列 (前进/左转/右转)
};

// VLN输出契约
struct VlnOutput {
    VlnOutputType type;
    
    // 路径/轨迹输出
    common::Path path;                    // 无时间戳路径
    common::Trajectory trajectory;        // 带时间戳轨迹
    
    // 元信息
    double confidence;                    // 置信度 [0,1]
    double expected_success_rate;         // 预期成功率
    std::string language_instruction;     // 原始语言指令
    
    // 可行性预评估
    bool self_assessed_feasible;          // 模型自评可行性
    std::vector<int> risky_segment_ids;   // 高风险段索引
};

// VLN集成模式
enum class VlnIntegrationMode {
    // 模式1: VLN作为全局路径提供者 (推荐默认)
    GLOBAL_PATH_PROVIDER,
    
    // 模式2: VLN作为软约束融合到轨迹优化
    SOFT_PRIOR_FUSION,
    
    // 模式3: 混合 — 高置信度用模式1，低置信度用模式2
    ADAPTIVE,
};

// VLN降级策略
struct VlnFallbackPolicy {
    // 触发降级的条件
    double min_confidence = 0.6;          // 最低置信度
    double max_collision_ratio = 0.1;     // 最大碰撞比例
    double min_esdf_clearance = 0.3;      // 最小ESDF距离
    
    // 降级目标
    std::string fallback_planner = "SmacHybrid";  // 降级到的规划器
    
    // 恢复策略
    bool allow_recovery_to_vln = true;    // 允许恢复到VLN
    double recovery_confidence = 0.8;     // 恢复所需置信度
};

// VLN适配器接口
class IVlnAdapter {
public:
    virtual ~IVlnAdapter() = default;
    
    // 从语言指令生成导航输出
    virtual VlnOutput generate(
        const std::string& instruction,
        const common::Pose2D& current_pose,
        const perception::IEnvironmentModel& env) = 0;
    
    // 验证VLN输出可行性
    virtual bool validate(
        const VlnOutput& output,
        const perception::IEnvironmentModel& env) = 0;
    
    // 转换为Nav2兼容格式
    virtual nav_msgs::msg::Path to_nav2_path(const VlnOutput& output) = 0;
};

}  // namespace neuro_nav::core::vln
```

### 5. VLN降级BT节点

```cpp
// core/planner/nav2_plugins/bt_nodes/vln_fallback_node.cpp
namespace neuro_nav::nav2_plugins {

// VLN降级条件节点
class VlnFallbackCondition : public BT::ConditionNode {
public:
    VlnFallbackCondition(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}
    
    BT::NodeStatus tick() override {
        // 获取VLN输出
        auto vln_output = getInput<vln::VlnOutput>("vln_output");
        auto env_model = getInput<perception::IEnvironmentModel*>("env_model");
        auto policy = getInput<vln::VlnFallbackPolicy>("fallback_policy");
        
        // 检查置信度
        if (vln_output->confidence < policy.min_confidence) {
            RCLCPP_WARN(logger_, "VLN confidence %.2f < threshold %.2f, fallback",
                        vln_output->confidence, policy.min_confidence);
            return BT::NodeStatus::FAILURE;  // 触发降级
        }
        
        // 检查ESDF安全距离
        double min_clearance = check_esdf_clearance(vln_output->path, env_model);
        if (min_clearance < policy.min_esdf_clearance) {
            RCLCPP_WARN(logger_, "ESDF clearance %.2f < threshold %.2f, fallback",
                        min_clearance, policy.min_esdf_clearance);
            return BT::NodeStatus::FAILURE;  // 触发降级
        }
        
        return BT::NodeStatus::SUCCESS;  // VLN可用
    }
    
private:
    double check_esdf_clearance(const common::Path& path,
                                 perception::IEnvironmentModel* env) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& pose : path.poses) {
            Eigen::Vector3d point(pose.x, pose.y, 0.0);
            double dist = env->query_distance(point);
            min_dist = std::min(min_dist, dist);
        }
        return min_dist;
    }
};

}  // namespace neuro_nav::nav2_plugins
```

---

## BT示例: VLN导航 + 降级

```xml
<!-- config/nav2/bt_vln_navigation.xml -->
<root BTCPP_format="4">
  <BehaviorTree ID="VlnNavigationWithFallback">
    <Sequence>
      <!-- 1. 获取VLN规划结果 -->
      <VlnPlannerNode instruction="{instruction}" 
                      output="{vln_output}"/>
      
      <!-- 2. 检查是否需要降级 -->
      <Fallback>
        <!-- 分支A: VLN可用 -->
        <Sequence>
          <VlnFallbackCondition vln_output="{vln_output}"
                                env_model="{env_model}"
                                fallback_policy="{fallback_policy}"/>
          <VlnPathAdapter vln_output="{vln_output}" 
                          nav2_path="{path}"/>
          <FollowPath path="{path}" controller_id="EsdfMppiController"/>
        </Sequence>
        
        <!-- 分支B: 降级到传统规划器 -->
        <Sequence>
          <LogWarn message="VLN fallback to SmacHybrid"/>
          <ComputePathToPose goal="{goal}" path="{path}" 
                             planner_id="SmacHybrid"/>
          <FollowPath path="{path}" controller_id="EsdfMppiController"/>
        </Sequence>
      </Fallback>
      
      <!-- 3. 执行中ESDF安全检查 -->
      <ReactiveSequence>
        <EsdfSafetyNode env_model="{env_model}" 
                        min_clearance="0.3"
                        on_violation="Replan"/>
        <FollowPath path="{path}"/>
      </ReactiveSequence>
    </Sequence>
  </BehaviorTree>
</root>
```

---

## 环境模型配置示例

```yaml
# config/perception/environment_model.yaml
environment_model:
  # 数据源
  data_sources:
    nvblox:
      enabled: true
      config_file: "nvblox_config.yaml"
    lidar:
      enabled: true
      topic: "/scan"
    depth:
      enabled: true
      topic: "/camera/depth/image_raw"
  
  # 3D→2D投影规则
  projection:
    height_min: 0.1           # 最低高度 (过滤地面)
    height_max: 2.0           # 最高高度 (机器人高度相关)
    ground_segmentation:
      enabled: true
      method: "ransac"        # ransac / patchwork
      threshold: 0.1
    dynamic_object_filter:
      enabled: true
      use_nvblox_people: true
  
  # Unknown空间策略 (按机器人类型)
  unknown_space:
    wheeled:
      policy: "obstacle"      # 保守
      cost_value: 255
    legged:
      policy: "cost_gradient" # 渐变
      max_unknown_dist: 2.0   # 2m内可通行
      gradient_scale: 50      # 代价梯度
  
  # Traversability→代价映射
  traversability_mapping:
    wheeled:
      max_slope: 0.15         # ~8.5° (严格)
      max_step_height: 0.05   # 5cm
      roughness_weight: 1.0
    legged:
      max_slope: 0.5          # ~27° (宽松)
      max_step_height: 0.25   # 25cm
      roughness_weight: 0.3
```

---

## ESDF在轨迹优化中的闭环

```cpp
// core/trajectory/optimization/esdf_trajectory_optimizer.hpp
namespace neuro_nav::core::trajectory {

class EsdfTrajectoryOptimizer {
public:
    struct Config {
        double safety_margin = 0.3;       // 安全距离
        double esdf_weight = 10.0;        // ESDF代价权重
        double smoothness_weight = 1.0;   // 平滑代价权重
        double dynamic_weight = 5.0;      // 动力学代价权重
    };
    
    EsdfTrajectoryOptimizer(
        std::shared_ptr<perception::IEnvironmentModel> env_model,
        const Config& config);
    
    // 优化轨迹
    common::Trajectory optimize(const common::Trajectory& initial_traj) {
        // 目标函数 = 平滑代价 + ESDF代价 + 动力学代价
        auto cost_func = [this](const common::Trajectory& traj) {
            double cost = 0.0;
            
            // 1. ESDF代价: 距离障碍物越近代价越高
            for (const auto& point : traj.points) {
                Eigen::Vector3d pos(point.pose.x, point.pose.y, 0.0);
                double dist = env_model_->query_distance(pos);
                if (dist < config_.safety_margin) {
                    cost += config_.esdf_weight * 
                            std::pow(config_.safety_margin - dist, 2);
                }
            }
            
            // 2. 平滑代价
            cost += config_.smoothness_weight * compute_smoothness_cost(traj);
            
            // 3. 动力学代价
            cost += config_.dynamic_weight * compute_dynamics_cost(traj);
            
            return cost;
        };
        
        // 梯度: 使用ESDF梯度
        auto grad_func = [this](const common::Trajectory& traj) {
            std::vector<Eigen::Vector3d> gradients;
            for (const auto& point : traj.points) {
                Eigen::Vector3d pos(point.pose.x, point.pose.y, 0.0);
                gradients.push_back(env_model_->query_distance_gradient(pos));
            }
            return gradients;
        };
        
        // 优化求解
        return nlopt_optimize(initial_traj, cost_func, grad_func);
    }
    
private:
    std::shared_ptr<perception::IEnvironmentModel> env_model_;
    Config config_;
};

}  // namespace neuro_nav::core::trajectory
```

---

## 机器人Profile配置

```yaml
# config/robots/tricycle_forklift.yaml
robot:
  name: "zhongli_forklift"
  type: "tricycle"
  category: "wheeled"          # ★ 新增: 类别 (wheeled/legged/humanoid)
  
  # 运动学参数
  kinematics:
    wheelbase: 1.5
    wheel_radius: 0.25
    max_steer_angle: 1.57
    
  # 约束参数
  constraints:
    max_linear_velocity: 2.0
    max_angular_velocity: 1.0
    max_linear_acceleration: 1.0
    min_turning_radius: 1.5
    
  # 足迹
  footprint:
    type: "polygon"
    vertices: [[1.5, 0.6], [1.5, -0.6], [-0.5, -0.6], [-0.5, 0.6]]
    
  # ★ 新增: 机器人Profile (用于环境模型参数化)
  profile:
    traversability:
      max_slope: 0.15           # 最大坡度 (rad)
      max_step_height: 0.05     # 最大台阶 (m)
      roughness_tolerance: 0.5  # 粗糙度容忍度
    unknown_space:
      policy: "obstacle"        # 保守策略
    safety:
      min_esdf_clearance: 0.3   # 最小ESDF距离
      inflation_radius: 0.5     # 膨胀半径
      
  # ★ 新增: 导航指令输出类型
  locomotion:
    output_type: "twist_2d"     # twist_2d / wheel_joints
    cmd_vel_topic: "/cmd_vel"
```

```yaml
# config/robots/quadruped.yaml
robot:
  name: "spot_like"
  type: "quadruped"
  category: "legged"           # ★ 足式
  
  kinematics:
    body_length: 0.6
    body_width: 0.3
    leg_length: 0.4
    
  profile:
    traversability:
      max_slope: 0.5            # 足式可接受更大坡度
      max_step_height: 0.25     # 可跨越更高台阶
      roughness_tolerance: 0.8
    unknown_space:
      policy: "cost_gradient"   # 激进策略 (探索)
    safety:
      min_esdf_clearance: 0.2   # 更小安全距离
      
  # ★ 足式输出类型
  locomotion:
    output_type: "body_trajectory"  # 躯干轨迹 + 落脚点
    body_traj_topic: "/body_trajectory"
    foothold_topic: "/foothold_plan"
```

---

## 实施路线 (v5)

### Phase 1: Common + Robot抽象 (2周)
- [ ] common模块
- [ ] 机器人接口 + RobotProfile
- [ ] 机器人工厂

### Phase 2: 环境模型 + 感知 (2.5周)
- [ ] IEnvironmentModel接口
- [ ] nvblox集成
- [ ] 投影规则 + Unknown策略
- [ ] Nav2 Costmap Layer

### Phase 3: 跨本体执行接口 (1.5周)
- [ ] NavigationCommand定义
- [ ] ICommandOutput接口
- [ ] Locomotion适配器 (Wheeled/Legged)

### Phase 4: Nav2插件 (3周)
- [ ] Planner Plugins (VLN/GNM)
- [ ] Controller Plugins (ESDF-MPPI)
- [ ] BT Nodes (VLN降级等)

### Phase 5: VLN集成 (2周)
- [ ] VLN契约定义
- [ ] VLN→Nav2适配
- [ ] 降级策略 + BT

### Phase 6: 通信 + 仿真 + 集成 (2周)
- [ ] 跨主机通信
- [ ] 仿真适配
- [ ] 端到端测试

---

## 与v4对比

| 方面 | v4 | v5 |
|------|----|----|
| Nav2关系 | 边界模糊，有重复 | 明确为插件提供者 |
| 跨本体 | 只抽象运动学 | 增加Locomotion层 |
| 环境模型 | 分散 | 统一IEnvironmentModel |
| VLN集成 | 接口不明 | 明确契约+降级策略 |
| ESDF用法 | 只做Costmap | 闭环到轨迹优化 |
| Unknown空间 | 未定义 | 按机器人类型策略化 |
