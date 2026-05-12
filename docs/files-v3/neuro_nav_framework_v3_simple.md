```
**neuro-nav框架** (v3 - 完整版)
│
├── app: 业务入口，支持SDK/ROS2/VLA Agent等接入
│   │
│   └── core: 导航核心逻辑
│       ├── planner: 可插拔规划器
│       │   ├── traditional           # Nav2原生 (NavFn/TEB/MPPI)
│       │   ├── learned               # 学习型 (VLN/E2E/Diffusion)
│       │   ├── hybrid                # 混合规划 (双系统)
│       │   └── intern_nav            # InternNav模型适配
│       │       ├── intern_vla_n1     # InternVLA-N1
│       │       ├── nav_dp            # NavDP扩散策略
│       │       └── stream_vln        # StreamVLN流式
│       ├── decision: 决策引擎
│       │   ├── behavior_tree         # BT.CPP行为树
│       │   ├── mode_selector         # 规划模式选择器
│       │   └── recovery              # 异常恢复
│       ├── map_manager               # 地图管理
│       ├── trajectory                # 轨迹处理 + 协议转换
│       └── stance                    # 操作位姿生成
│
├── interface: 接口适配层
│   ├── upstream                      # 上游 (定位/感知/模型)
│   ├── downstream                    # 下游 (控制/轨迹)
│   └── communication                 # ★ 跨主机通信
│       ├── ros2_adapter              # ROS2本机
│       ├── mqtt_bridge               # MQTT跨主机 (中力协议Beta-3)
│       ├── grpc_adapter              # gRPC模型推理
│       └── zeromq_adapter            # ZeroMQ低延迟
│
├── simulation: ★ 仿真适配层
│   │
│   ├── sim_interface                 # 仿真统一接口
│   │   ├── base_sim_adapter          # 基类定义
│   │   ├── scene_manager             # 场景管理
│   │   ├── sensor_bridge             # 传感器桥接
│   │   └── robot_controller          # 机器人控制
│   │
│   ├── isaac_sim                     # ★ Isaac Sim适配 (高保真)
│   │   ├── isaac_sim_adapter         # 主适配器
│   │   ├── ros2_bridge               # ROS2 Bridge
│   │   ├── scene_loader              # USD场景加载
│   │   ├── sensor_publisher          # RTX传感器发布
│   │   │   ├── rtx_lidar             # RTX雷达
│   │   │   ├── rtx_camera            # RTX相机
│   │   │   └── imu_contact           # IMU/接触
│   │   ├── nav2_integration          # Nav2集成
│   │   └── occupancy_map_gen         # 占用地图生成
│   │
│   ├── isaac_lab                     # ★ Isaac Lab适配 (RL训练)
│   │   ├── isaac_lab_adapter         # 主适配器
│   │   ├── env_wrapper               # Gym环境封装
│   │   ├── rl_policy_deploy          # 策略部署
│   │   ├── observation_space         # 观测空间
│   │   ├── action_space              # 动作空间
│   │   └── reward_shaping            # 奖励函数
│   │
│   ├── gazebo                        # Gazebo适配 (轻量级)
│   │   ├── gazebo_adapter            # 主适配器
│   │   ├── world_loader              # SDF/URDF加载
│   │   └── ros2_integration          # ROS2集成
│   │
│   ├── intern_utopia                 # InternUtopia适配 (可选)
│   │
│   └── tools                         # 仿真工具
│       ├── scene_builder             # 场景构建器
│       ├── trajectory_replay         # 轨迹回放
│       ├── collision_checker         # 碰撞检测
│       ├── path_visualizer           # 路径可视化
│       └── benchmark_runner          # 基准测试
│
├── sensor: 传感器适配
│   ├── camera / lidar / depth / odom
│
├── protocol: 协议定义
│   ├── neuro_nav_protocol            # 内部协议
│   ├── zhongli_protocol              # 中力协议 (Beta-3)
│   └── intern_protocol               # InternNav协议
│
├── nav2_plugins: Nav2扩展
│
├── config: 配置管理
│   ├── robot_params                  # 机器人参数
│   ├── planner_params                # 规划器参数
│   ├── comm_params                   # 通信参数
│   ├── sim_params                    # ★ 仿真参数
│   └── behavior_params               # 行为配置
│
├── sdk: Python SDK (VLA开发者)
│
└── util: 通用工具
```

---

## 仿真统一接口

```python
class BaseSimAdapter(ABC):
    """仿真适配器统一接口"""
    
    # 场景管理
    def load_scene(scene_path: str) -> bool
    def reset_scene() -> None
    def spawn_robot(robot_type: str, pose: Pose) -> str
    def add_obstacle(config: ObstacleConfig) -> str
    
    # 传感器数据
    def get_sensor_data(robot_id: str) -> SensorData
    def get_ground_truth_pose(robot_id: str) -> Pose
    
    # 机器人控制
    def set_velocity(robot_id: str, linear: float, angular: float)
    def set_trajectory(robot_id: str, trajectory: Trajectory)
    
    # 仿真控制
    def step(dt: float) -> None
    def get_sim_time() -> float
    
    # 验证工具
    def check_collision(trajectory: Trajectory) -> CollisionResult
    def get_occupancy_map() -> OccupancyGrid
```

---

## 仿真平台对比

| 平台 | 特点 | 适用场景 | ROS2支持 |
|------|------|---------|---------|
| **Isaac Sim** | 高保真RTX传感器,USD场景 | Sim2Real验证,复杂场景 | ROS2 Bridge |
| **Isaac Lab** | GPU并行,RL训练优化 | 策略训练,大规模实验 | 可选 |
| **Gazebo** | 轻量快速,社区丰富 | 快速原型,CI/CD | 原生支持 |
| **InternUtopia** | 大规模3D扫描场景 | 导航基准测试 | 支持 |

---

## 仿真工作流

```
开发验证流程:
┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐
│  算法   │───▶│ Gazebo  │───▶│ Isaac   │───▶│  实车   │
│  开发   │    │ 快速验证│    │ Sim验证 │    │  部署   │
└─────────┘    └─────────┘    └─────────┘    └─────────┘
                  轻量           高保真          Sim2Real

RL训练流程:
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ Isaac Lab   │────▶│  策略训练   │────▶│  ONNX导出   │
│ 环境定义    │     │  (PPO/SAC)  │     │  TensorRT   │
└─────────────┘     └─────────────┘     └──────┬──────┘
                                               │
                                               ▼
                                    ┌─────────────────┐
                                    │ Neuro-Nav部署   │
                                    │ (Orin实时推理)  │
                                    └─────────────────┘
```

---

## SDK使用示例

```python
from neuro_nav import MobilityAgent, SimType

# === 仿真验证 ===
agent = MobilityAgent(
    robot="forklift_3w",
    sim_type=SimType.ISAAC_SIM,  # 或 GAZEBO
    sim_config={
        "scene": "warehouse_small",
        "headless": False
    }
)

# 加载场景
agent.sim.load_scene("warehouse_with_forklifts.usd")

# 设置起点
agent.sim.set_robot_pose(x=0, y=0, theta=0)

# 导航到目标
result = agent.go_to(x=5.0, y=3.0, theta=0.0)

# 获取验证结果
print(f"成功: {result.success}")
print(f"路径长度: {result.path_length}m")
print(f"碰撞: {result.had_collision}")

# === 轨迹验证 ===
trajectory = agent.plan(start=(0,0,0), goal=(5,3,0))
collision = agent.sim.check_collision(trajectory)
print(f"碰撞点: {collision.collision_points}")

# === 基准测试 ===
from neuro_nav.simulation import BenchmarkRunner

runner = BenchmarkRunner(agent.sim)
results = runner.run_benchmark(
    planner=agent.planner,
    scenarios=["simple_corridor", "cluttered_warehouse"],
    num_trials=10
)
print(f"成功率: {results.success_rate:.1%}")
print(f"平均规划时间: {results.avg_planning_time:.3f}s")


# === 切换到实车 ===
agent_real = MobilityAgent(
    robot="forklift_3w",
    sim_type=None,  # 实车模式
    comm_config={
        "default": "mqtt",
        "mqtt_broker": "192.168.1.100:1883"
    }
)
agent_real.go_to(x=5.0, y=3.0)
```

---

## Isaac Sim ROS2话题

```
发布话题:
├── /tf                    # TF变换树
├── /odom                  # 里程计 (nav_msgs/Odometry)
├── /scan                  # 激光扫描 (sensor_msgs/LaserScan)
├── /point_cloud           # 点云 (sensor_msgs/PointCloud2)
├── /camera/rgb/image_raw  # RGB图像
├── /camera/depth/image_raw # 深度图像
└── /map                   # 占用地图 (nav_msgs/OccupancyGrid)

订阅话题:
├── /cmd_vel               # 速度指令 (geometry_msgs/Twist)
├── /goal_pose             # 目标位姿
├── /trajectory            # 轨迹 (nav_msgs/Path)
└── /initialpose           # 初始位姿
```

---

## Isaac Lab RL环境定义

```python
class NavigationEnv(IsaacEnv):
    """导航强化学习环境"""
    
    observation_space = {
        "lidar_scan": (360,),           # 雷达扫描
        "rgb_image": (H, W, 3),         # RGB图像
        "depth_image": (H, W),          # 深度图像
        "goal_relative": (3,),          # 相对目标 (dx, dy, dtheta)
        "velocity": (3,),               # 当前速度 (vx, vy, omega)
    }
    
    action_space = {
        "continuous": (2,),  # [linear_vel, angular_vel]
        # 或
        "discrete": (4,),    # [forward, backward, left, right]
    }
    
    def reward_function(self):
        return (
            + distance_progress * 1.0      # 接近目标奖励
            - collision_penalty * 10.0     # 碰撞惩罚
            - time_penalty * 0.01          # 时间惩罚
            + goal_reached * 100.0         # 到达目标奖励
            + smoothness_bonus * 0.1       # 平滑奖励
        )
```

---

## 完整通信架构图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Neuro-Nav 完整架构                                │
│                                                                          │
│                         ┌─────────────────┐                             │
│                         │    Python SDK   │                             │
│                         └────────┬────────┘                             │
│                                  │                                       │
│                                  ▼                                       │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                         Core (导航核心)                            │  │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐              │  │
│  │  │Planner  │  │Decision │  │Trajectory│ │Map Mgr  │              │  │
│  │  │Plugin   │  │Engine   │  │Handler  │  │         │              │  │
│  │  └─────────┘  └─────────┘  └─────────┘  └─────────┘              │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│         │                    │                    │                      │
│         ▼                    ▼                    ▼                      │
│  ┌─────────────┐      ┌─────────────┐      ┌─────────────┐             │
│  │Communication│      │ Simulation  │      │   Sensor    │             │
│  │   Layer     │      │   Layer     │      │   Layer     │             │
│  │             │      │             │      │             │             │
│  │ ROS2/MQTT/  │      │ Isaac Sim/  │      │ Camera/     │             │
│  │ gRPC/ZeroMQ │      │ Lab/Gazebo  │      │ LiDAR/Depth │             │
│  └──────┬──────┘      └──────┬──────┘      └──────┬──────┘             │
│         │                    │                    │                      │
└─────────┼────────────────────┼────────────────────┼──────────────────────┘
          │                    │                    │
          ▼                    ▼                    ▼
   ┌─────────────┐      ┌─────────────┐      ┌─────────────┐
   │ 控制主机    │      │ 仿真环境    │      │ 真实传感器  │
   │ (PLC/Orin)  │      │ (渲染/物理) │      │ (硬件)      │
   └─────────────┘      └─────────────┘      └─────────────┘
```
