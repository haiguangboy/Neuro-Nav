# Neuro-Nav 导航框架架构设计

## 设计理念

参考xyz-slam的分层解耦思想，将导航框架设计为**业务驱动、算法可插拔、接口标准化**的架构。

---

## 整体框架结构

```
**neuro-nav框架**
│
├── app: 业务入口层，适配不同平台和业务场景
│   │   # 支持多种业务入口：SDK调用、ROS2 Action、语义指令、VLA Agent等
│   │
│   ├── core: 导航核心逻辑
│   │   │
│   │   ├── planner: 路径规划器（可插拔设计）
│   │   │   ├── traditional          # 传统规划器
│   │   │   │   ├── global_planner   # NavFn/A*/Smac等
│   │   │   │   └── local_planner    # DWB/TEB/MPPI等
│   │   │   │
│   │   │   ├── learned              # 学习型规划器
│   │   │   │   ├── vln_planner      # VLN模型轨迹输出
│   │   │   │   ├── e2e_planner      # 端到端模型(GNM/ViNT/NoMaD)
│   │   │   │   └── vla_planner      # VLA模型移动指令
│   │   │   │
│   │   │   └── hybrid               # 混合规划器
│   │   │       └── hierarchical     # 分层规划（高层语义+底层轨迹）
│   │   │
│   │   ├── decision: 决策引擎
│   │   │   ├── behavior_tree        # 行为树调度
│   │   │   ├── state_machine        # 状态机管理
│   │   │   ├── mode_selector        # 规划模式选择器
│   │   │   └── recovery_handler     # 异常恢复策略
│   │   │
│   │   ├── map_manager: 地图管理
│   │   │   ├── global_map           # 全局规划地图
│   │   │   │   ├── costmap_2d       # 2D代价地图
│   │   │   │   ├── semantic_map     # 语义地图
│   │   │   │   └── topology_map     # 拓扑地图
│   │   │   │
│   │   │   ├── local_map            # 局部规划地图
│   │   │   │   ├── costmap_local    # 局部代价地图
│   │   │   │   └── elevation_map    # 高程图（可选）
│   │   │   │
│   │   │   └── map_converter        # 地图格式转换
│   │   │       ├── 3d_to_2d         # 点云转栅格
│   │   │       └── grid_to_topo     # 栅格转拓扑
│   │   │
│   │   ├── trajectory: 轨迹处理
│   │   │   ├── trajectory_fusion    # 多源轨迹融合
│   │   │   ├── trajectory_smooth    # 轨迹平滑
│   │   │   ├── trajectory_validate  # 轨迹可行性验证
│   │   │   └── trajectory_optimize  # 轨迹优化
│   │   │
│   │   └── stance_generator         # 操作位姿生成（特色功能）
│   │       └── manipulation_aware   # 考虑机械臂工作空间的位姿
│   │
│   ├── interface: 外部接口层
│   │   ├── upstream                 # 上游接口（输入）
│   │   │   ├── localization         # 定位模块接口
│   │   │   ├── perception           # 感知模块接口
│   │   │   ├── semantic_input       # 语义输入接口
│   │   │   └── vla_bridge           # VLA模型桥接
│   │   │
│   │   ├── downstream               # 下游接口（输出）
│   │   │   ├── controller           # 控制模块接口
│   │   │   ├── cmd_vel              # 速度指令输出
│   │   │   └── trajectory_out       # 轨迹输出
│   │   │
│   │   └── communication            # 通信适配
│   │       ├── ros2_adapter         # ROS2 Topic/Action/Service
│   │       ├── dds_adapter          # 原生DDS支持
│   │       └── grpc_adapter         # gRPC（跨语言调用）
│   │
│   ├── sensor: 传感器适配层
│   │   ├── camera_adapter           # 相机数据适配
│   │   ├── lidar_adapter            # 雷达数据适配
│   │   ├── depth_adapter            # 深度相机适配
│   │   └── odom_adapter             # 里程计适配
│   │
│   └── config: 配置管理
│       ├── robot_config             # 机器人参数配置
│       ├── planner_config           # 规划器配置
│       └── behavior_config          # 行为配置
│
├── sdk: Python SDK（给VLA开发者用）
│   ├── mobility_agent               # 移动代理封装
│   ├── semantic_api                 # 语义化API
│   └── async_client                 # 异步客户端
│
└── util: 通用工具
    ├── math_utils                   # 数学工具
    ├── geometry_utils               # 几何工具
    ├── logging                      # 日志系统
    └── visualization                # 可视化工具
```

---

## 核心设计详解

### 1. 规划器可插拔设计 (Planner Plugin System)

```
┌─────────────────────────────────────────────────────────────┐
│                    PlannerManager                            │
│  ┌─────────────────────────────────────────────────────┐    │
│  │              BasePlannerInterface                     │    │
│  │  + plan(start, goal) -> Trajectory                   │    │
│  │  + supports_map() -> bool                            │    │
│  │  + requires_sensor() -> List[SensorType]             │    │
│  └─────────────────────────────────────────────────────┘    │
│          ▲              ▲              ▲                     │
│          │              │              │                     │
│  ┌───────┴───┐  ┌───────┴───┐  ┌───────┴───┐               │
│  │Traditional │  │  Learned  │  │  Hybrid   │               │
│  │  Planner   │  │  Planner  │  │  Planner  │               │
│  │            │  │           │  │           │               │
│  │ Nav2 Plugin│  │ VLN/E2E   │  │ Semantic  │               │
│  │ TEB/MPPI   │  │ GNM/NoMaD │  │ + Local   │               │
│  └────────────┘  └───────────┘  └───────────┘               │
└─────────────────────────────────────────────────────────────┘
```

**关键点：**
- 所有规划器实现统一接口 `BasePlannerInterface`
- 通过 `supports_map()` 声明是否需要地图
- 通过 `requires_sensor()` 声明需要的传感器数据
- 端到端模型可以绕过地图直接输出轨迹

### 2. 决策引擎设计 (Decision Engine)

```
┌────────────────────────────────────────────────────────────┐
│                    Decision Engine                          │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              Behavior Tree (BT.CPP)                   │  │
│  │                                                       │  │
│  │    [Root: NavigateTo]                                 │  │
│  │         │                                             │  │
│  │    [Selector]                                         │  │
│  │    ├── [Sequence: VLN_Mode]                          │  │
│  │    │   ├── CheckVLNAvailable                         │  │
│  │    │   ├── GetVLNTrajectory                          │  │
│  │    │   └── ExecuteWithLocalPlanner                   │  │
│  │    │                                                  │  │
│  │    ├── [Sequence: Traditional_Mode]                  │  │
│  │    │   ├── GetGlobalPath                             │  │
│  │    │   └── FollowPath                                │  │
│  │    │                                                  │  │
│  │    └── [Sequence: E2E_Mode]                          │  │
│  │        ├── CheckE2EModel                             │  │
│  │        └── DirectE2EControl                          │  │
│  │                                                       │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              Mode Selector                            │  │
│  │                                                       │  │
│  │  Input Factors:                                       │  │
│  │  - 任务类型 (点对点/语义目标/跟随)                      │  │
│  │  - 环境复杂度 (开阔/狭窄/动态)                         │  │
│  │  - 可用资源 (地图/模型/传感器)                         │  │
│  │  - 置信度阈值                                         │  │
│  │                                                       │  │
│  │  Output: Selected Planning Mode                       │  │
│  └──────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘
```

### 3. 地图管理策略

```
┌─────────────────────────────────────────────────────────────┐
│                    Map Manager                               │
│                                                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │ Global Map  │  │ Local Map   │  │ No Map Mode │         │
│  │             │  │             │  │             │         │
│  │ - Costmap2D │  │ - Rolling   │  │ - E2E Model │         │
│  │ - Semantic  │  │   Costmap   │  │ - VLN Path  │         │
│  │ - Topology  │  │ - Elevation │  │ - Pure      │         │
│  │             │  │             │  │   Reactive  │         │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘         │
│         │                │                │                 │
│         └────────────────┼────────────────┘                 │
│                          ▼                                  │
│              ┌───────────────────┐                         │
│              │  Map Abstraction  │                         │
│              │     Layer         │                         │
│              └───────────────────┘                         │
│                                                              │
│  适配不同规划器的地图需求：                                    │
│  - Traditional Planner → 需要 Global + Local Costmap        │
│  - VLN Planner → 可选 Semantic/Topology Map                 │
│  - E2E Planner → 不需要地图，直接用传感器                     │
└─────────────────────────────────────────────────────────────┘
```

### 4. 轨迹融合与验证

```
┌─────────────────────────────────────────────────────────────┐
│                 Trajectory Pipeline                          │
│                                                              │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌──────────┐ │
│  │ Source  │───▶│ Validate│───▶│ Smooth  │───▶│ Execute  │ │
│  └─────────┘    └─────────┘    └─────────┘    └──────────┘ │
│                                                              │
│  Sources:                                                    │
│  - Nav2 Global Planner 输出                                  │
│  - VLN Model 输出的航点序列                                  │
│  - E2E Model 输出的速度/轨迹                                 │
│  - 手动遥控输入                                              │
│                                                              │
│  Validation:                                                 │
│  - 碰撞检测（有地图时）                                       │
│  - 运动学可行性检查                                          │
│  - 动力学约束验证                                            │
│  - 安全边界检查                                              │
│                                                              │
│  Smoothing:                                                  │
│  - B-Spline 平滑                                             │
│  - 时间参数化                                                │
│  - 速度/加速度约束                                           │
└─────────────────────────────────────────────────────────────┘
```

---

## 接口设计

### 上游接口（从其他模块接收数据）

```cpp
// 定位接口
class LocalizationInterface {
public:
    virtual Pose getCurrentPose() = 0;
    virtual Covariance getPoseCovariance() = 0;
    virtual bool isLocalized() = 0;
    virtual void subscribeOdom(OdomCallback cb) = 0;
};

// 感知接口
class PerceptionInterface {
public:
    virtual std::vector<Obstacle> getObstacles() = 0;
    virtual SemanticScene getSemanticScene() = 0;
    virtual PointCloud getPointCloud() = 0;
    virtual Image getRGBImage() = 0;
};

// VLA桥接接口
class VLABridgeInterface {
public:
    virtual void setNavigationGoal(SemanticGoal goal) = 0;
    virtual Trajectory getVLNTrajectory() = 0;
    virtual VelocityCmd getE2EVelocity() = 0;
};
```

### 下游接口（输出到其他模块）

```cpp
// 控制接口
class ControllerInterface {
public:
    virtual void sendTrajectory(Trajectory traj) = 0;
    virtual void sendVelocity(VelocityCmd cmd) = 0;
    virtual void emergencyStop() = 0;
    virtual ControllerStatus getStatus() = 0;
};
```

### SDK API (Python)

```python
from neuro_nav import MobilityAgent, PlanningMode

agent = MobilityAgent()

# 语义化导航（使用VLN）
agent.go_to(object="coffee_machine", intent="grasping")

# 传统点对点导航
agent.go_to(position=(x, y, theta), mode=PlanningMode.TRADITIONAL)

# 混合模式：VLN全局 + TEB局部
agent.go_to(
    object="kitchen_table", 
    mode=PlanningMode.HYBRID,
    global_planner="vln",
    local_planner="teb"
)

# 端到端模式（不需要地图）
agent.go_to(
    direction="forward_to_door",
    mode=PlanningMode.E2E,
    model="nomaD"
)
```

---

## 从多个维度的设计考量

### 业务角度

| 业务场景 | 规划模式 | 地图需求 | 传感器需求 |
|---------|---------|---------|-----------|
| 固定路线巡检 | Traditional | 全局+局部 | LiDAR |
| 语义目标导航 | Hybrid | 语义地图 | Camera+LiDAR |
| 开放环境探索 | E2E | 无 | Camera |
| 人跟随 | E2E | 无 | Camera+深度 |
| 精确物体抓取定位 | Traditional + Stance | 局部精细 | Camera+LiDAR |

### 开发角度

```
开发层级:

Level 3: VLA开发者
         └── 只用Python SDK，无需了解ROS2

Level 2: 机器人应用开发者  
         └── 配置行为树，选择规划器组合

Level 1: 算法开发者
         └── 实现新的规划器插件，地图处理等

Level 0: 框架维护者
         └── 核心架构，接口定义，ROS2集成
```

### 扩展性角度

```
新增功能的扩展点:

1. 新规划器
   └── 实现 BasePlannerInterface → 注册到 PlannerManager

2. 新传感器
   └── 实现 SensorAdapter → 注册到 SensorManager

3. 新业务逻辑
   └── 扩展 Behavior Tree 节点 → 组合成新行为

4. 新机器人类型
   └── 配置 robot_config → 自动适配运动学约束

5. 新模型接入
   └── 实现 VLABridgeInterface → 桥接模型输出
```

---

## 与Nav2的关系

```
┌────────────────────────────────────────────────────────────┐
│                     Neuro-Nav                               │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐ │
│  │              Neuro-Nav Extended Layer                  │ │
│  │                                                        │ │
│  │  - VLN Planner Plugin                                 │ │
│  │  - E2E Planner Plugin                                 │ │
│  │  - Semantic Map Layer                                 │ │
│  │  - Mode Selector                                      │ │
│  │  - Trajectory Fusion                                  │ │
│  │  - Stance Generator                                   │ │
│  │  - VLA Bridge                                         │ │
│  └───────────────────────────────────────────────────────┘ │
│                          │                                  │
│                          ▼                                  │
│  ┌───────────────────────────────────────────────────────┐ │
│  │              Nav2 (基座)                               │ │
│  │                                                        │ │
│  │  - BT Navigator                                       │ │
│  │  - Planner Server (插件化)                            │ │
│  │  - Controller Server (插件化)                         │ │
│  │  - Costmap 2D                                         │ │
│  │  - Recovery Server                                    │ │
│  │  - Lifecycle Management                               │ │
│  └───────────────────────────────────────────────────────┘ │
│                                                             │
└────────────────────────────────────────────────────────────┘

复用Nav2:
✓ Lifecycle管理
✓ 行为树框架
✓ Costmap基础设施
✓ Controller Server
✓ 恢复行为框架

扩展/替换:
+ Planner插件（VLN/E2E）
+ 地图层（语义/拓扑）
+ 决策逻辑（模式选择）
+ 轨迹处理（融合/验证）
+ 外部接口（VLA桥接）
```

---

## 目录结构建议

```
neuro-nav/
├── neuro_nav_core/              # 核心导航逻辑
│   ├── planner/
│   │   ├── base_planner.hpp     # 基类接口
│   │   ├── traditional/
│   │   ├── learned/
│   │   └── hybrid/
│   ├── decision/
│   │   ├── behavior_tree/
│   │   ├── mode_selector/
│   │   └── recovery/
│   ├── map_manager/
│   ├── trajectory/
│   └── stance/
│
├── neuro_nav_interface/         # 接口层
│   ├── upstream/
│   ├── downstream/
│   └── communication/
│
├── neuro_nav_sensor/            # 传感器适配
│
├── neuro_nav_nav2_plugins/      # Nav2插件
│   ├── planners/
│   ├── controllers/
│   └── bt_nodes/
│
├── neuro_nav_sdk/               # Python SDK
│   ├── mobility_agent.py
│   └── semantic_api.py
│
├── neuro_nav_sim/               # 仿真环境
│
├── neuro_nav_config/            # 配置文件
│   ├── robots/
│   ├── planners/
│   └── behaviors/
│
└── neuro_nav_bringup/           # 启动脚本
```

---

## 实施路线建议

### Phase 1: 框架搭建 (1-2周)
- 定义核心接口
- 搭建目录结构
- 集成Nav2基座
- 实现基础的 Traditional Planner 封装

### Phase 2: 决策引擎 (2-3周)
- 实现 Mode Selector
- 扩展 Behavior Tree 节点
- 实现 Recovery Handler

### Phase 3: 学习型规划器 (3-4周)
- 实现 VLN Planner Interface
- 实现 E2E Planner Interface
- 轨迹融合与验证

### Phase 4: 完善与测试 (2周)
- Python SDK
- 仿真测试
- 文档完善
