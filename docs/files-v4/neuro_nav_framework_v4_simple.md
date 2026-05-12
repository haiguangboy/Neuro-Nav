```
**neuro-nav** (v4 - 遵循软件设计原则)
│
├── common/                          # ★ 公共基础层 (所有模块依赖)
│   │
│   ├── math/                        # 数学工具
│   │   ├── transform                # SE2/SE3坐标变换
│   │   ├── interpolation            # 线性/样条/Slerp插值
│   │   ├── optimization             # 梯度下降/LM优化器
│   │   └── filter                   # 卡尔曼/低通滤波
│   │
│   ├── geometry/                    # 几何工具
│   │   ├── collision                # 碰撞检测 (AABB/OBB/GJK)
│   │   ├── distance                 # 距离计算
│   │   └── footprint                # 机器人足迹
│   │
│   ├── data/                        # 数据结构
│   │   ├── pose                     # Pose2D/Pose3D
│   │   ├── twist                    # Twist2D/Twist3D
│   │   ├── trajectory               # 轨迹容器
│   │   ├── path                     # 路径容器
│   │   └── costmap                  # 代价地图
│   │
│   ├── time/                        # 时间管理
│   │   ├── clock                    # 时钟抽象 (ROS/System/Sim)
│   │   └── time_sync                # 时间同步
│   │
│   ├── concurrency/                 # 并发工具
│   │   ├── thread_pool              # 线程池
│   │   └── lock_free_queue          # 无锁队列
│   │
│   ├── logging/                     # 日志系统
│   │   └── metrics                  # 性能指标
│   │
│   ├── config/                      # 配置管理
│   │   ├── config_loader            # YAML/JSON加载
│   │   └── param_server             # 参数服务器
│   │
│   └── error/                       # 错误处理
│       └── result                   # Result<T,E>类型
│
├── robot/                           # ★ 机器人抽象层 (运动学/动力学)
│   │
│   ├── interface/                   # 机器人接口 (ISP原则)
│   │   ├── i_kinematics             # 运动学接口
│   │   ├── i_dynamics               # 动力学接口
│   │   ├── i_footprint              # 足迹接口
│   │   └── i_constraints            # 约束接口
│   │
│   ├── wheeled/                     # 轮式机器人
│   │   ├── differential/            # 差速底盘
│   │   ├── ackermann/               # 阿克曼底盘
│   │   ├── tricycle/                # ★ 三轮叉车底盘
│   │   │   ├── tricycle_kinematics  # 运动学
│   │   │   ├── tricycle_constraints # 约束
│   │   │   └── fork_kinematics      # 货叉运动学
│   │   └── omnidirectional/         # 全向轮/麦轮
│   │
│   ├── legged/                      # 足式机器人
│   │   ├── quadruped/               # 四足机器人
│   │   │   ├── quad_kinematics      # 运动学
│   │   │   ├── quad_gait            # 步态生成
│   │   │   └── quad_stability       # 稳定性约束
│   │   └── bipedal/                 # 双足机器人
│   │       ├── biped_kinematics     # 运动学
│   │       ├── biped_balance        # 平衡控制
│   │       └── biped_footstep       # 落脚点规划
│   │
│   └── factory/                     # 机器人工厂 (OCP原则)
│       ├── robot_factory            # 工厂类
│       └── robot_registry           # 注册表
│
├── perception/                      # ★ 感知层 (局部环境感知)
│   │
│   ├── interface/                   # 感知接口
│   │   ├── i_local_mapper           # 局部建图接口
│   │   ├── i_obstacle_detector      # 障碍物检测接口
│   │   └── i_traversability         # 可通行性接口
│   │
│   ├── local_mapping/               # 局部建图
│   │   │
│   │   ├── nvblox/                  # ★ nvblox集成 (GPU加速)
│   │   │   ├── nvblox_adapter       # nvblox适配器
│   │   │   ├── tsdf_integrator      # TSDF集成
│   │   │   ├── esdf_generator       # ESDF生成 (距离场)
│   │   │   ├── mesh_extractor       # 网格提取
│   │   │   └── costmap_publisher    # Nav2代价地图发布
│   │   │
│   │   ├── voxel_grid/              # 体素栅格 (轻量级替代)
│   │   │
│   │   └── elevation_map/           # 高程图 (足式机器人用)
│   │       └── terrain_analysis     # 地形分析
│   │
│   ├── obstacle/                    # 障碍物处理
│   │   ├── static_obstacle          # 静态障碍物
│   │   ├── dynamic_obstacle         # 动态障碍物追踪
│   │   ├── people_detector          # 行人检测 (nvblox people mode)
│   │   └── obstacle_predictor       # 障碍物轨迹预测
│   │
│   ├── traversability/              # 可通行性分析
│   │   ├── ground_segmentation      # 地面分割
│   │   ├── drivable_area            # 可行驶区域
│   │   └── slope_analysis           # 坡度分析
│   │
│   └── sensor_fusion/               # 传感器融合
│       ├── depth_lidar_fusion       # 深度+雷达融合
│       └── multi_sensor_sync        # 多传感器同步
│
├── core/                            # ★ 导航核心层
│   │
│   ├── planner/                     # 路径规划器
│   │   │
│   │   ├── interface/               # 规划器接口 (接口层次)
│   │   │   ├── i_global_planner     # 全局规划器接口
│   │   │   ├── i_local_planner      # 局部规划器接口
│   │   │   ├── i_wheeled_planner    # 轮式规划器扩展接口
│   │   │   ├── i_legged_planner     # 足式规划器扩展接口
│   │   │   └── i_model_planner      # 模型驱动扩展接口
│   │   │
│   │   ├── global/                  # 全局规划器
│   │   │   ├── nav2_wrapper/        # Nav2封装 (NavFn/Smac/ThetaStar)
│   │   │   ├── semantic/            # 语义规划 (VLN/Topology)
│   │   │   └── hybrid/              # 混合规划
│   │   │
│   │   ├── local/                   # 局部规划器
│   │   │   │
│   │   │   ├── wheeled/             # ★ 轮式专用
│   │   │   │   ├── teb_wrapper      # TEB (时间弹性带)
│   │   │   │   ├── dwa_wrapper      # DWA (动态窗口)
│   │   │   │   ├── mppi_wrapper     # MPPI (模型预测路径积分)
│   │   │   │   └── rpp_wrapper      # RPP (调节纯追踪)
│   │   │   │
│   │   │   ├── legged/              # ★ 足式专用
│   │   │   │   ├── footstep_planner # 落脚点规划
│   │   │   │   └── body_path_planner # 躯干路径规划
│   │   │   │
│   │   │   └── learned/             # 学习型
│   │   │       ├── e2e_local        # 端到端
│   │   │       └── diffusion_local  # 扩散策略
│   │   │
│   │   ├── model_based/             # ★ 模型驱动规划器
│   │   │   ├── intern_nav/          # InternNav系列
│   │   │   │   ├── intern_vla_n1    # InternVLA-N1 (双系统)
│   │   │   │   ├── nav_dp           # NavDP (扩散策略)
│   │   │   │   └── stream_vln       # StreamVLN (流式)
│   │   │   └── foundation/          # 基础模型
│   │   │       ├── gnm_planner      # GNM
│   │   │       ├── vint_planner     # ViNT
│   │   │       └── nomad_planner    # NoMaD
│   │   │
│   │   └── factory/                 # 规划器工厂
│   │       └── planner_factory      # 根据机器人类型选择规划器
│   │
│   ├── decision/                    # 决策引擎
│   │   ├── behavior_tree/           # BT.CPP行为树
│   │   ├── mode_selector/           # 模式选择器
│   │   └── recovery/                # 恢复策略
│   │
│   ├── trajectory/                  # 轨迹处理
│   │   ├── fusion/                  # 多源轨迹融合
│   │   ├── validation/              # 可行性验证
│   │   ├── optimization/            # 轨迹优化
│   │   └── execution/               # 轨迹执行
│   │
│   ├── map_manager/                 # 地图管理
│   │   ├── global_map/              # 全局地图
│   │   └── local_map/               # 局部地图 (nvblox输出)
│   │
│   └── stance/                      # 操作位姿生成
│
├── interface/                       # ★ 外部接口层
│   │
│   ├── upstream/                    # 上游接口
│   │   ├── localization/            # 定位 (xyz-slam)
│   │   ├── perception/              # 感知
│   │   └── model_bridge/            # 模型桥接
│   │
│   ├── downstream/                  # 下游接口
│   │   ├── controller/              # 控制器
│   │   └── output/                  # 输出 (cmd_vel/trajectory)
│   │
│   ├── communication/               # ★ 跨主机通信
│   │   ├── ros2/                    # ROS2本机
│   │   ├── mqtt/                    # MQTT跨主机 (中力协议)
│   │   ├── grpc/                    # gRPC模型推理
│   │   └── zeromq/                  # ZeroMQ低延迟
│   │
│   └── sensor/                      # 传感器接口
│       ├── camera / lidar / depth / imu
│
├── simulation/                      # ★ 仿真层
│   │
│   ├── interface/                   # 仿真接口
│   ├── isaac_sim/                   # Isaac Sim (高保真)
│   ├── isaac_lab/                   # Isaac Lab (RL训练)
│   ├── gazebo/                      # Gazebo (轻量级)
│   └── tools/                       # 仿真工具
│
├── app/                             # 应用层
│   ├── sdk/                         # Python SDK
│   ├── ros2_node/                   # ROS2节点
│   └── cli/                         # 命令行
│
├── nav2_plugins/                    # Nav2插件
│   ├── costmap_layers/
│   │   └── nvblox_layer             # ★ nvblox代价层
│   └── bt_nodes/
│
└── config/                          # 配置
    ├── robots/                      # 机器人配置
    │   ├── differential.yaml
    │   ├── ackermann.yaml
    │   ├── tricycle_forklift.yaml   # ★ 叉车
    │   ├── omnidirectional.yaml
    │   ├── quadruped.yaml           # ★ 四足
    │   └── bipedal.yaml             # ★ 双足
    └── perception/
        └── nvblox_config.yaml       # ★ nvblox配置
```

---

## 设计原则落地

| 原则 | 具体体现 |
|------|---------|
| **SRP** | `nvblox_adapter`只做3D重建, `tricycle_kinematics`只做运动学 |
| **OCP** | 新增机器人类型只需实现接口+注册工厂，无需修改core |
| **ISP** | `IKinematics`/`IConstraints`/`IFootprint`分离，按需依赖 |
| **高内聚** | perception模块内部紧密协作完成局部感知 |
| **低耦合** | core通过接口依赖robot，不直接依赖具体实现 |
| **无环依赖** | app→core→robot/perception→common，单向依赖 |
| **小而专一** | 每个文件/类只做一件事，通过组合解决复杂问题 |

---

## 依赖关系图

```
                         ┌─────────┐
                         │   app   │
                         └────┬────┘
                              │
                         ┌────▼────┐
                         │  core   │
                         └────┬────┘
                              │
       ┌──────────┬───────────┼───────────┬──────────┐
       │          │           │           │          │
  ┌────▼────┐ ┌───▼───┐ ┌─────▼─────┐ ┌───▼───┐ ┌────▼────┐
  │  robot  │ │percep │ │ interface │ │  sim  │ │ nav2_   │
  │         │ │ tion  │ │           │ │       │ │ plugins │
  └────┬────┘ └───┬───┘ └─────┬─────┘ └───┬───┘ └────┬────┘
       │          │           │           │          │
       └──────────┴───────────┼───────────┴──────────┘
                              │
                         ┌────▼────┐
                         │ common  │ ← 所有模块共用
                         └─────────┘
```

---

## nvblox工作流

```
传感器数据                处理管道                    输出
─────────────────────────────────────────────────────────────
 Depth Camera ──┐
                │      ┌──────────────┐
                ├─────▶│   nvblox     │
                │      │              │
 LiDAR ─────────┤      │ TSDF→ESDF    │──▶ 2D Costmap ──▶ Nav2
                │      │              │
 Pose/Odom ─────┘      │ +People Det  │──▶ 3D Mesh ──▶ RViz
                       └──────────────┘
                              │
                              ▼
                    距离场查询 (轨迹碰撞检测)
```

---

## 机器人类型 × 规划器兼容矩阵

| 规划器 | 差速 | 阿克曼 | 三轮叉车 | 全向 | 四足 | 双足 |
|-------|-----|-------|---------|-----|-----|-----|
| TEB | ✓ | ✓ | ✓ | ✓ | - | - |
| DWA | ✓ | - | - | ✓ | - | - |
| MPPI | ✓ | ✓ | ✓ | ✓ | - | - |
| RPP | ✓ | ✓ | ✓ | ✓ | - | - |
| Footstep | - | - | - | - | ✓ | ✓ |
| BodyPath | - | - | - | - | ✓ | ✓ |
| NavDP | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| GNM/ViNT | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |

---

## 使用示例

```python
from neuro_nav import MobilityAgent, RobotType, PlannerType

# === 三轮叉车 ===
forklift = MobilityAgent(
    robot_type=RobotType.TRICYCLE,
    config="config/robots/tricycle_forklift.yaml"
)
forklift.perception.use("nvblox")           # GPU加速局部建图
forklift.planner.use(local=PlannerType.TEB) # TEB适合非完整约束
forklift.go_to(x=5.0, y=3.0)

# === 四足机器人 ===
quadruped = MobilityAgent(
    robot_type=RobotType.QUADRUPED,
    config="config/robots/quadruped.yaml"
)
quadruped.perception.use("elevation_map")         # 高程图
quadruped.planner.use(local=PlannerType.FOOTSTEP) # 落脚点规划
quadruped.go_to(x=10.0, y=0.0)

# === 模型驱动 (任意底盘) ===
agent = MobilityAgent(robot_type=RobotType.DIFFERENTIAL)
agent.planner.use(local=PlannerType.NAV_DP)  # NavDP扩散策略
agent.go_to(instruction="去厨房")
```
