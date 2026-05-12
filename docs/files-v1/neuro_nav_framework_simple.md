```
**neuro-nav框架**
- app: 业务入口，适配SDK/ROS2/VLA Agent等多种接入方式
  - core: 导航核心逻辑
    - planner: 可插拔规划器
      - traditional      # Nav2原生规划器封装
        - global         # NavFn/A*/Smac
        - local          # DWB/TEB/MPPI/RPP
      - learned          # 学习型规划器
        - vln            # VLN模型轨迹
        - e2e            # 端到端(GNM/ViNT/NoMaD)
        - vla            # VLA移动指令
      - hybrid           # 混合规划
        - semantic_nav   # 语义全局+传统局部
    - decision: 决策引擎
      - behavior_tree    # BT.CPP行为树
      - mode_selector    # 规划模式选择器(根据场景/资源选模式)
      - state_machine    # 任务状态机
      - recovery         # 异常恢复策略
    - map_manager: 地图管理
      - global_map       # 全局地图
        - costmap_2d     # 代价地图
        - semantic       # 语义地图
        - topology       # 拓扑地图
      - local_map        # 局部地图
        - rolling_cost   # 滚动代价图
        - elevation      # 高程图(可选)
      - converter        # 地图转换
        - 3d_to_2d       # 点云转栅格
        - grid_to_topo   # 栅格转拓扑
      - no_map_mode      # 无地图模式(E2E专用)
    - trajectory: 轨迹处理
      - fusion           # 多源轨迹融合
      - validate         # 可行性验证(碰撞/运动学)
      - smooth           # 轨迹平滑(B-Spline)
      - optimize         # 时间/能量优化
    - stance             # 操作位姿生成(特色)
      - manipulation_aware  # 考虑机械臂可达性
  - interface: 接口适配层
    - upstream           # 上游接口(输入)
      - localization     # 定位模块(接xyz-slam)
      - perception       # 感知模块(障碍物/语义)
      - vla_bridge       # VLA模型桥接
    - downstream         # 下游接口(输出)
      - controller       # 控制模块接口
      - cmd_vel          # 速度指令
      - trajectory_out   # 轨迹输出
    - communication      # 通信适配
      - ros2_adapter     # ROS2 Topic/Action/Service
      - dds_adapter      # 原生DDS
      - grpc_adapter     # gRPC(跨语言)
  - sensor: 传感器适配
    - camera             # RGB相机
    - lidar              # 激光雷达
    - depth              # 深度相机
    - odom               # 里程计
  - config: 配置管理
    - robot_params       # 机器人运动学参数
    - planner_params     # 规划器参数
    - behavior_params    # 行为树配置
- sdk: Python SDK (给VLA开发者)
  - mobility_agent       # 移动代理封装
  - semantic_api         # 语义化API
  - async_client         # 异步客户端
- nav2_plugins: Nav2扩展插件
  - planners             # 自定义规划器插件
  - controllers          # 自定义控制器插件
  - bt_nodes             # 自定义BT节点
- util: 通用工具
  - math                 # 数学工具
  - geometry             # 几何工具
  - logging              # 日志系统
  - visualization        # 可视化
```

---

## 核心设计要点

### 1. 规划模式选择器 (Mode Selector)

```
输入因子:
├── 任务类型: point_to_point | semantic_goal | following | exploration
├── 环境特征: open | narrow | dynamic | unknown
├── 可用资源: has_map | has_model | sensor_types
└── 置信度: localization_conf | perception_conf

输出决策:
├── TRADITIONAL  → Nav2 Global + Local Planner (需地图)
├── LEARNED_VLN  → VLN模型全局 + Traditional局部 (需语义)
├── LEARNED_E2E  → 端到端模型直出 (不需地图)
├── HYBRID       → 语义引导 + 传统规划 (混合)
└── MANUAL       → 遥控/手动介入
```

### 2. 轨迹来源与处理

```
轨迹来源               地图需求        输出格式
─────────────────────────────────────────────────
Nav2 GlobalPlanner  →  需要全局地图  →  Path (航点序列)
Nav2 LocalPlanner   →  需要局部地图  →  Trajectory (带时间)
VLN Model           →  可选语义地图  →  Waypoints (稀疏航点)
E2E Model (GNM)     →  不需要地图    →  Velocity (速度指令)
E2E Model (NoMaD)   →  不需要地图    →  Waypoints (短期航点)

                        ↓
              ┌─────────────────┐
              │ Trajectory Hub  │
              │   统一处理管道   │
              └─────────────────┘
                        ↓
        验证 → 平滑 → 时间参数化 → 输出到控制器
```

### 3. 与定位模块(xyz-slam)的对接

```cpp
// 定位接口定义 (与xyz-slam对接)
class LocalizationAdapter {
public:
    // 获取当前位姿
    Pose2D getCurrentPose();
    
    // 获取位姿协方差(用于规划置信度)
    Matrix3d getPoseCovariance();
    
    // 定位状态
    enum class Status { INITIALIZING, TRACKING, LOST, RELOCATING };
    Status getStatus();
    
    // 订阅里程计(高频)
    void subscribeOdom(std::function<void(Odometry)> callback);
    
    // 订阅定位(低频但精确)
    void subscribePose(std::function<void(PoseStamped)> callback);
    
    // 地图获取
    OccupancyGrid getStaticMap();
    PointCloud getPointCloudMap();
};
```

### 4. 行为树扩展节点

```
自定义BT节点:
├── Condition Nodes (条件)
│   ├── IsVLNModelAvailable
│   ├── IsE2EModelAvailable  
│   ├── HasValidMap
│   ├── IsLocalized
│   └── IsInManipulationRange
│
├── Action Nodes (动作)
│   ├── GetVLNTrajectory
│   ├── GetE2ECommand
│   ├── ComputeManipulationStance
│   ├── ValidateTrajectory
│   └── SwitchPlanningMode
│
└── Decorator Nodes (装饰)
    ├── RetryWithFallback
    ├── TimeoutSwitch
    └── ConfidenceGate
```

---

## 业务场景适配表

| 业务场景 | 规划模式 | 全局地图 | 局部地图 | 模型 | 传感器 |
|---------|---------|---------|---------|------|--------|
| 仓库固定路线 | Traditional | ✓ Costmap | ✓ Rolling | - | LiDAR |
| 语义导航"去厨房" | Hybrid | ✓ Semantic | ✓ Rolling | VLN | Camera+LiDAR |
| 开放空间探索 | E2E | ✗ | ✗ | GNM/NoMaD | Camera |
| 动态跟随人 | E2E | ✗ | ✗ | E2E+Tracker | Camera+Depth |
| 精确对接充电桩 | Traditional | ✓ | ✓ Fine | - | LiDAR+Camera |
| 操作前定位对象 | Hybrid+Stance | ✓ Semantic | ✓ | VLN | Camera+LiDAR |

---

## SDK使用示例

```python
from neuro_nav import MobilityAgent, PlanningMode, Intent

# 初始化
agent = MobilityAgent(robot="forklift_3w")

# === 传统导航 ===
agent.go_to(x=5.0, y=3.0, theta=0.0)

# === 语义导航 (VLN) ===
agent.go_to(
    target="咖啡机", 
    intent=Intent.APPROACH
)

# === 操作位姿导航 (Stance) ===
agent.go_to(
    target="货架A-03",
    intent=Intent.MANIPULATION,
    arm_reach=0.8  # 机械臂可达距离
)

# === 端到端导航 (无地图) ===
agent.go_to(
    direction="穿过那扇门",
    mode=PlanningMode.E2E,
    model="nomad"
)

# === 混合模式 ===
agent.go_to(
    target="会议室",
    mode=PlanningMode.HYBRID,
    global_planner="vln",      # VLN做全局语义引导
    local_planner="teb"        # TEB做局部避障
)

# === 查询状态 ===
status = agent.get_status()
print(f"Mode: {status.planning_mode}")
print(f"Progress: {status.progress:.1%}")
print(f"ETA: {status.eta}s")
```
