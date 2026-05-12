# Neuro-Nav 导航框架架构设计 v3

## 更新说明
- v1: 基础框架 + Nav2集成
- v2: 跨主机通信 + InternNav模型接口
- **v3: 仿真适配层 (Isaac Sim / Isaac Lab / Gazebo)**

---

## 整体框架结构

```
**neuro-nav框架** (v3)
│
├── app: 业务入口层
│   ├── sdk                      # Python SDK (VLA开发者)
│   ├── ros2_api                 # ROS2 Action/Service
│   ├── vla_agent                # VLA Agent接入
│   └── cli                      # 命令行工具
│
├── core: 导航核心逻辑
│   ├── planner                  # 可插拔规划器
│   │   ├── traditional          # Nav2原生
│   │   ├── learned              # 学习型 (VLN/E2E/Diffusion)
│   │   ├── hybrid               # 混合规划
│   │   └── intern_nav           # InternNav模型适配
│   ├── decision                 # 决策引擎
│   ├── map_manager              # 地图管理
│   ├── trajectory               # 轨迹处理
│   └── stance                   # 操作位姿生成
│
├── interface: 接口适配层
│   ├── upstream                 # 上游接口 (定位/感知/模型)
│   ├── downstream               # 下游接口 (控制/轨迹输出)
│   └── communication            # 跨主机通信 (MQTT/gRPC/ZeroMQ)
│
├── simulation: ★ 仿真适配层 (新增)
│   │
│   ├── sim_interface            # 仿真统一接口
│   │   ├── base_sim_adapter     # 基类定义
│   │   ├── scene_manager        # 场景管理接口
│   │   ├── sensor_bridge        # 传感器数据桥接
│   │   ├── robot_controller     # 机器人控制接口
│   │   └── world_state          # 世界状态查询
│   │
│   ├── isaac_sim                # ★ NVIDIA Isaac Sim 适配
│   │   ├── isaac_sim_adapter    # Isaac Sim主适配器
│   │   ├── ros2_bridge          # ROS2 Bridge集成
│   │   ├── scene_loader         # USD场景加载
│   │   ├── sensor_publisher     # 传感器发布
│   │   │   ├── rtx_lidar        # RTX雷达
│   │   │   ├── rtx_camera       # RTX相机
│   │   │   └── imu_contact      # IMU/接触传感器
│   │   ├── nav2_integration     # Nav2集成
│   │   └── occupancy_map_gen    # 占用地图生成
│   │
│   ├── isaac_lab                # ★ Isaac Lab 适配 (RL训练)
│   │   ├── isaac_lab_adapter    # Isaac Lab主适配器
│   │   ├── env_wrapper          # 环境封装 (Gym接口)
│   │   ├── rl_policy_deploy     # RL策略部署
│   │   ├── observation_space    # 观测空间定义
│   │   ├── action_space         # 动作空间定义
│   │   └── reward_shaping       # 奖励函数
│   │
│   ├── gazebo                   # Gazebo 适配
│   │   ├── gazebo_adapter       # Gazebo主适配器
│   │   ├── world_loader         # SDF/URDF加载
│   │   ├── sensor_plugins       # 传感器插件
│   │   └── ros2_integration     # ROS2集成
│   │
│   ├── intern_utopia            # InternUtopia适配 (可选)
│   │   └── utopia_adapter       # InternUtopia适配器
│   │
│   └── tools                    # 仿真工具
│       ├── scene_builder        # 场景构建器
│       ├── trajectory_replay    # 轨迹回放
│       ├── collision_checker    # 碰撞检测
│       ├── path_visualizer      # 路径可视化
│       └── benchmark_runner     # 基准测试运行器
│
├── sensor: 传感器适配
│   ├── camera                   # RGB相机
│   ├── lidar                    # 激光雷达
│   ├── depth                    # 深度相机
│   └── odom                     # 里程计
│
├── protocol: 协议定义
│   ├── neuro_nav_protocol       # 内部协议
│   ├── zhongli_protocol         # 中力协议
│   └── intern_protocol          # InternNav协议
│
├── nav2_plugins: Nav2扩展
│
├── config: 配置管理
│   ├── robot_params             # 机器人参数
│   ├── planner_params           # 规划器参数
│   ├── comm_params              # 通信参数
│   ├── sim_params               # ★ 仿真参数
│   └── behavior_params          # 行为配置
│
└── util: 通用工具
```

---

## 仿真适配层详细设计

### 1. 仿真统一接口 (SimInterface)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        SimInterface (统一仿真接口)                        │
│                                                                          │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                    BaseSimAdapter (基类)                           │  │
│  │                                                                    │  │
│  │  场景管理:                                                         │  │
│  │  + load_scene(scene_path) -> bool                                 │  │
│  │  + spawn_robot(robot_type, pose) -> RobotHandle                   │  │
│  │  + add_obstacle(obstacle_config) -> ObstacleHandle               │  │
│  │  + reset_scene() -> void                                          │  │
│  │                                                                    │  │
│  │  传感器数据:                                                       │  │
│  │  + get_camera_image(camera_id) -> Image                          │  │
│  │  + get_depth_image(camera_id) -> DepthImage                      │  │
│  │  + get_lidar_scan(lidar_id) -> PointCloud                        │  │
│  │  + get_odom() -> Odometry                                         │  │
│  │                                                                    │  │
│  │  机器人控制:                                                       │  │
│  │  + set_velocity(linear, angular) -> void                         │  │
│  │  + set_trajectory(trajectory) -> void                            │  │
│  │  + get_robot_state() -> RobotState                                │  │
│  │                                                                    │  │
│  │  世界状态:                                                         │  │
│  │  + get_ground_truth_pose() -> Pose                               │  │
│  │  + check_collision(trajectory) -> CollisionResult                │  │
│  │  + get_distance_to_goal() -> float                               │  │
│  │                                                                    │  │
│  │  仿真控制:                                                         │  │
│  │  + step(dt) -> void                                               │  │
│  │  + pause() -> void                                                │  │
│  │  + resume() -> void                                               │  │
│  │  + get_sim_time() -> float                                        │  │
│  │                                                                    │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│          ▲              ▲              ▲              ▲                 │
│          │              │              │              │                 │
│  ┌───────┴───┐  ┌───────┴───┐  ┌───────┴───┐  ┌───────┴───┐           │
│  │Isaac Sim  │  │Isaac Lab  │  │  Gazebo   │  │  Intern   │           │
│  │ Adapter   │  │ Adapter   │  │  Adapter  │  │  Utopia   │           │
│  │           │  │           │  │           │  │  Adapter  │           │
│  │高保真仿真  │  │RL训练     │  │轻量级     │  │大规模场景 │           │
│  │ROS2 Bridge│  │Gym接口    │  │ROS2原生   │  │          │           │
│  └───────────┘  └───────────┘  └───────────┘  └───────────┘           │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2. Isaac Sim 适配详情

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Isaac Sim Adapter                                │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    ROS2 Bridge Integration                        │   │
│  │                                                                   │   │
│  │  发布话题:                            订阅话题:                    │   │
│  │  /tf                                 /cmd_vel                    │   │
│  │  /odom                               /goal_pose                  │   │
│  │  /scan (LaserScan)                   /initialpose                │   │
│  │  /point_cloud (PointCloud2)          /trajectory                 │   │
│  │  /camera/rgb/image_raw                                           │   │
│  │  /camera/depth/image_raw                                         │   │
│  │  /map (OccupancyGrid)                                            │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    Scene Management                               │   │
│  │                                                                   │   │
│  │  场景加载:                                                        │   │
│  │  - USD场景文件加载 (仓库/工厂/室内)                                │   │
│  │  - URDF机器人模型导入                                             │   │
│  │  - 动态障碍物生成 (人/叉车/托盘)                                   │   │
│  │                                                                   │   │
│  │  预置场景:                                                        │   │
│  │  - warehouse_small.usd     # 小型仓库                             │   │
│  │  - warehouse_large.usd     # 大型仓库                             │   │
│  │  - factory_floor.usd       # 工厂地面                             │   │
│  │  - office_indoor.usd       # 室内办公                             │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    RTX Sensor Simulation                          │   │
│  │                                                                   │   │
│  │  RTX Lidar:                                                       │   │
│  │  - 真实物理模拟                                                   │   │
│  │  - 材质反射特性                                                   │   │
│  │  - 雨雾天气影响                                                   │   │
│  │                                                                   │   │
│  │  RTX Camera:                                                      │   │
│  │  - 真实光照渲染                                                   │   │
│  │  - 运动模糊                                                       │   │
│  │  - 镜头畸变                                                       │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    Nav2 Integration                               │   │
│  │                                                                   │   │
│  │  - Occupancy Map Generator (生成2D栅格地图)                       │   │
│  │  - AMCL定位支持                                                   │   │
│  │  - Nav2 Goal发送                                                  │   │
│  │  - Waypoint Follower                                              │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3. Isaac Lab 适配 (RL训练)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Isaac Lab Adapter                                │
│                                                                          │
│  用途: 导航策略的强化学习训练                                             │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    Gym Environment Wrapper                        │   │
│  │                                                                   │   │
│  │  class NavigationEnv(IsaacEnv):                                  │   │
│  │      observation_space:                                           │   │
│  │          - lidar_scan: (360,)                                    │   │
│  │          - rgb_image: (H, W, 3)                                  │   │
│  │          - depth_image: (H, W)                                   │   │
│  │          - goal_relative: (3,)  # dx, dy, dtheta                 │   │
│  │          - velocity: (3,)       # vx, vy, omega                  │   │
│  │                                                                   │   │
│  │      action_space:                                                │   │
│  │          - continuous: (2,)     # linear_vel, angular_vel        │   │
│  │          - discrete: (4,)       # forward/back/left/right        │   │
│  │                                                                   │   │
│  │      reward_function:                                             │   │
│  │          - distance_to_goal_reward                               │   │
│  │          - collision_penalty                                      │   │
│  │          - smoothness_reward                                      │   │
│  │          - time_penalty                                           │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    RL Policy Deployment                           │   │
│  │                                                                   │   │
│  │  训练 → 导出 → 部署流程:                                          │   │
│  │                                                                   │   │
│  │  Isaac Lab (训练)                                                 │   │
│  │       │                                                           │   │
│  │       ▼ 导出ONNX/TorchScript                                      │   │
│  │  Neuro-Nav (集成)                                                 │   │
│  │       │                                                           │   │
│  │       ▼ TensorRT优化                                              │   │
│  │  实车部署 (Orin)                                                  │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  支持的RL框架:                                                           │
│  - RSL RL                                                               │
│  - SKRL                                                                 │
│  - RL Games                                                             │
│  - Stable Baselines3                                                    │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 4. Gazebo 适配 (轻量级)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Gazebo Adapter                                   │
│                                                                          │
│  用途: 快速原型验证、CI/CD测试                                            │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    Features                                       │   │
│  │                                                                   │   │
│  │  优点:                                                            │   │
│  │  - 轻量级，启动快                                                 │   │
│  │  - ROS2原生支持                                                   │   │
│  │  - 社区资源丰富                                                   │   │
│  │  - 适合CI/CD流水线                                                │   │
│  │                                                                   │   │
│  │  适用场景:                                                        │   │
│  │  - 算法快速迭代                                                   │   │
│  │  - 单元测试                                                       │   │
│  │  - 基础功能验证                                                   │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    Integration                                    │   │
│  │                                                                   │   │
│  │  - ros_gz_bridge: Gazebo ↔ ROS2 消息桥接                         │   │
│  │  - Nav2 Bringup: 标准Nav2启动                                     │   │
│  │  - URDF/SDF: 机器人模型支持                                       │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 仿真工具集

### 1. 场景构建器 (Scene Builder)

```python
class SceneBuilder:
    """统一的场景构建接口"""
    
    def __init__(self, sim_type: SimType):
        """
        Args:
            sim_type: ISAAC_SIM | ISAAC_LAB | GAZEBO
        """
        self.adapter = self._create_adapter(sim_type)
    
    # 场景加载
    def load_predefined_scene(self, scene_name: str) -> bool:
        """加载预定义场景: warehouse_small, factory_floor, etc."""
        pass
    
    def load_custom_scene(self, scene_path: str) -> bool:
        """加载自定义场景文件 (USD/SDF/World)"""
        pass
    
    # 动态元素
    def spawn_robot(self, robot_type: str, pose: Pose) -> RobotHandle:
        """生成机器人: forklift_3w, carter, turtlebot, etc."""
        pass
    
    def add_static_obstacle(self, obstacle: ObstacleConfig) -> ObstacleHandle:
        """添加静态障碍物: 货架、墙壁、柱子等"""
        pass
    
    def add_dynamic_obstacle(self, obstacle: DynamicObstacleConfig) -> ObstacleHandle:
        """添加动态障碍物: 行人、其他车辆等"""
        pass
    
    # 随机化
    def randomize_obstacles(self, config: RandomizationConfig) -> None:
        """障碍物随机化 (用于训练数据增强)"""
        pass
    
    def randomize_lighting(self, config: LightingConfig) -> None:
        """光照随机化"""
        pass
```

### 2. 轨迹验证器 (Trajectory Validator)

```python
class TrajectoryValidator:
    """轨迹验证和分析工具"""
    
    def __init__(self, sim_adapter: BaseSimAdapter):
        self.sim = sim_adapter
    
    # 碰撞检测
    def check_collision(self, trajectory: Trajectory) -> CollisionResult:
        """检查轨迹是否与障碍物碰撞"""
        pass
    
    def check_dynamic_collision(self, trajectory: Trajectory, 
                                  prediction_horizon: float) -> CollisionResult:
        """考虑动态障碍物的碰撞检测"""
        pass
    
    # 可行性验证
    def check_kinematic_feasibility(self, trajectory: Trajectory,
                                     robot_config: RobotConfig) -> bool:
        """检查轨迹是否满足运动学约束"""
        pass
    
    def check_dynamic_feasibility(self, trajectory: Trajectory,
                                   robot_config: RobotConfig) -> bool:
        """检查轨迹是否满足动力学约束"""
        pass
    
    # 轨迹回放
    def replay_trajectory(self, trajectory: Trajectory,
                          speed_factor: float = 1.0) -> ReplayResult:
        """在仿真中回放轨迹"""
        pass
    
    # 指标计算
    def compute_metrics(self, trajectory: Trajectory,
                        ground_truth: Trajectory) -> TrajectoryMetrics:
        """计算轨迹质量指标"""
        return TrajectoryMetrics(
            path_length=...,
            execution_time=...,
            smoothness=...,
            clearance_min=...,  # 最小障碍物距离
            tracking_error=...,
        )
```

### 3. 基准测试运行器 (Benchmark Runner)

```python
class BenchmarkRunner:
    """导航算法基准测试"""
    
    def __init__(self, sim_adapter: BaseSimAdapter):
        self.sim = sim_adapter
    
    def run_benchmark(self, 
                      planner: BasePlanner,
                      scenarios: List[BenchmarkScenario],
                      num_trials: int = 10) -> BenchmarkResult:
        """运行基准测试"""
        results = []
        for scenario in scenarios:
            for trial in range(num_trials):
                # 重置场景
                self.sim.reset_scene()
                self.sim.load_scene(scenario.scene)
                
                # 设置起点终点
                self.sim.set_robot_pose(scenario.start_pose)
                goal = scenario.goal_pose
                
                # 运行规划
                start_time = time.time()
                trajectory = planner.plan(scenario.start_pose, goal)
                planning_time = time.time() - start_time
                
                # 执行并收集指标
                result = self.sim.execute_trajectory(trajectory)
                
                results.append(TrialResult(
                    scenario=scenario.name,
                    success=result.reached_goal,
                    planning_time=planning_time,
                    execution_time=result.execution_time,
                    path_length=result.path_length,
                    collision=result.had_collision,
                ))
        
        return BenchmarkResult(trials=results)
    
    # 预定义基准场景
    SCENARIOS = {
        "simple_corridor": BenchmarkScenario(...),
        "cluttered_warehouse": BenchmarkScenario(...),
        "dynamic_pedestrians": BenchmarkScenario(...),
        "narrow_passage": BenchmarkScenario(...),
        "u_turn": BenchmarkScenario(...),
    }
```

---

## 仿真工作流

### 1. 开发验证流程

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        开发验证工作流                                     │
│                                                                          │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐             │
│  │  算法   │───▶│ Gazebo  │───▶│ Isaac   │───▶│  实车   │             │
│  │  开发   │    │ 快速验证│    │ Sim验证 │    │  部署   │             │
│  └─────────┘    └─────────┘    └─────────┘    └─────────┘             │
│                                                                          │
│  阶段1: Gazebo快速迭代                                                   │
│  - 轻量级启动                                                            │
│  - 基础功能验证                                                          │
│  - 单元测试                                                              │
│                                                                          │
│  阶段2: Isaac Sim高保真验证                                              │
│  - RTX传感器仿真                                                         │
│  - 真实光照/材质                                                         │
│  - 动态障碍物                                                            │
│  - 性能基准测试                                                          │
│                                                                          │
│  阶段3: 实车部署                                                         │
│  - Sim2Real迁移                                                          │
│  - 实际环境测试                                                          │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2. RL训练流程 (Isaac Lab)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        RL训练工作流                                       │
│                                                                          │
│  ┌─────────────┐                                                        │
│  │ Isaac Lab   │                                                        │
│  │ 环境定义    │                                                        │
│  │             │                                                        │
│  │ - 观测空间  │                                                        │
│  │ - 动作空间  │                                                        │
│  │ - 奖励函数  │                                                        │
│  └──────┬──────┘                                                        │
│         │                                                                │
│         ▼                                                                │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐              │
│  │  GPU并行   │────▶│  策略训练   │────▶│  策略导出   │              │
│  │  仿真      │     │  (PPO/SAC)  │     │  (ONNX)     │              │
│  └─────────────┘     └─────────────┘     └──────┬──────┘              │
│                                                  │                      │
│                                                  ▼                      │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    Neuro-Nav 集成                                │   │
│  │                                                                   │   │
│  │  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐        │   │
│  │  │  策略加载   │────▶│ TensorRT   │────▶│  实时推理   │        │   │
│  │  │  (ONNX)     │     │ 优化        │     │  (Orin)     │        │   │
│  │  └─────────────┘     └─────────────┘     └─────────────┘        │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 接口定义

### 仿真适配器基类 (Python)

```python
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, List
from enum import Enum

class SimType(Enum):
    ISAAC_SIM = "isaac_sim"
    ISAAC_LAB = "isaac_lab"
    GAZEBO = "gazebo"
    INTERN_UTOPIA = "intern_utopia"

@dataclass
class RobotState:
    pose: Pose
    velocity: Velocity
    joint_positions: Optional[List[float]] = None
    joint_velocities: Optional[List[float]] = None

@dataclass
class SensorData:
    timestamp: float
    rgb_image: Optional[np.ndarray] = None
    depth_image: Optional[np.ndarray] = None
    lidar_scan: Optional[np.ndarray] = None
    point_cloud: Optional[np.ndarray] = None
    imu: Optional[IMUData] = None

class BaseSimAdapter(ABC):
    """仿真适配器基类"""
    
    # === 生命周期 ===
    @abstractmethod
    def initialize(self, config: SimConfig) -> bool:
        """初始化仿真环境"""
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """关闭仿真环境"""
        pass
    
    # === 场景管理 ===
    @abstractmethod
    def load_scene(self, scene_path: str) -> bool:
        """加载场景"""
        pass
    
    @abstractmethod
    def reset_scene(self) -> None:
        """重置场景"""
        pass
    
    @abstractmethod
    def spawn_robot(self, robot_type: str, pose: Pose) -> str:
        """生成机器人，返回robot_id"""
        pass
    
    # === 传感器 ===
    @abstractmethod
    def get_sensor_data(self, robot_id: str) -> SensorData:
        """获取传感器数据"""
        pass
    
    @abstractmethod
    def get_ground_truth_pose(self, robot_id: str) -> Pose:
        """获取真值位姿"""
        pass
    
    # === 控制 ===
    @abstractmethod
    def set_velocity(self, robot_id: str, linear: float, angular: float) -> None:
        """设置速度指令"""
        pass
    
    @abstractmethod
    def set_trajectory(self, robot_id: str, trajectory: Trajectory) -> None:
        """设置轨迹"""
        pass
    
    # === 仿真控制 ===
    @abstractmethod
    def step(self, dt: float = None) -> None:
        """仿真步进"""
        pass
    
    @abstractmethod
    def get_sim_time(self) -> float:
        """获取仿真时间"""
        pass
    
    # === 验证工具 ===
    @abstractmethod
    def check_collision(self, trajectory: Trajectory) -> CollisionResult:
        """碰撞检测"""
        pass
    
    @abstractmethod
    def get_occupancy_map(self) -> OccupancyGrid:
        """获取占用地图"""
        pass
```

### ROS2 Bridge配置 (Isaac Sim)

```yaml
# config/isaac_sim_ros2_bridge.yaml
isaac_sim:
  ros2_bridge:
    enabled: true
    domain_id: 0
    
  publishers:
    - topic: /odom
      msg_type: nav_msgs/Odometry
      frame_id: odom
      child_frame_id: base_link
      rate: 50.0
      
    - topic: /scan
      msg_type: sensor_msgs/LaserScan
      frame_id: lidar_link
      rate: 10.0
      
    - topic: /camera/rgb/image_raw
      msg_type: sensor_msgs/Image
      frame_id: camera_link
      rate: 30.0
      
    - topic: /camera/depth/image_raw
      msg_type: sensor_msgs/Image
      frame_id: camera_link
      rate: 30.0
      
    - topic: /point_cloud
      msg_type: sensor_msgs/PointCloud2
      frame_id: lidar_link
      rate: 10.0
      
  subscribers:
    - topic: /cmd_vel
      msg_type: geometry_msgs/Twist
      
    - topic: /trajectory
      msg_type: nav_msgs/Path
      
  tf:
    publish_tf: true
    publish_odom_tf: true
```

---

## 仿真场景与平台选择矩阵

| 应用场景 | 推荐平台 | 原因 |
|---------|---------|------|
| 算法快速原型 | Gazebo | 启动快，迭代快 |
| CI/CD自动测试 | Gazebo | 轻量，易集成 |
| 传感器真实性验证 | Isaac Sim | RTX传感器仿真 |
| 复杂场景测试 | Isaac Sim | USD场景，高保真 |
| 动态障碍物测试 | Isaac Sim | 人/车辆仿真 |
| RL策略训练 | Isaac Lab | GPU并行，高效训练 |
| 大规模场景 | InternUtopia | 3D扫描场景支持 |
| Sim2Real验证 | Isaac Sim | 最接近真实 |

---

## 配置示例

### 仿真配置 `config/sim_config.yaml`

```yaml
simulation:
  # 默认仿真平台
  default_sim: "isaac_sim"  # isaac_sim / isaac_lab / gazebo
  
  # Isaac Sim 配置
  isaac_sim:
    headless: false
    gpu_id: 0
    physics_dt: 0.01667  # 60Hz
    rendering_dt: 0.03333  # 30Hz
    
    ros2_bridge:
      enabled: true
      domain_id: 0
    
    scene:
      default: "warehouse_small"
      assets_path: "/isaac-sim/assets"
    
    sensors:
      rtx_lidar:
        enabled: true
        config: "HESAI_Pandar128"
      rtx_camera:
        enabled: true
        resolution: [1280, 720]
        
  # Isaac Lab 配置 (RL训练)
  isaac_lab:
    num_envs: 4096  # 并行环境数
    env_spacing: 5.0
    device: "cuda:0"
    
    rl:
      algorithm: "PPO"
      policy_network: "MLP"
      learning_rate: 0.0003
      
  # Gazebo 配置
  gazebo:
    world_file: "nav2_world.sdf"
    physics_engine: "ode"
    real_time_factor: 1.0
    
    ros2:
      bridge_topics:
        - "/cmd_vel"
        - "/odom"
        - "/scan"
```

---

## 实施路线 (更新v3)

### Phase 1: 基础框架 + 通信层 (2周)
- [x] 核心目录结构
- [x] Nav2基座集成
- [x] MQTT Bridge
- [x] 基础ROS2适配

### Phase 2: 决策引擎 + 协议层 (2周)
- [ ] 行为树扩展
- [ ] Mode Selector
- [ ] 轨迹协议转换

### Phase 3: InternNav接口 (3周)
- [ ] InternNav基类接口
- [ ] 模型适配器

### Phase 4: 仿真适配层 (3周) ★ 新增
- [ ] 仿真统一接口定义
- [ ] Isaac Sim适配器
- [ ] Gazebo适配器
- [ ] 场景构建工具
- [ ] 轨迹验证工具

### Phase 5: Isaac Lab集成 (2周) ★ 新增
- [ ] Gym环境封装
- [ ] RL训练流水线
- [ ] 策略导出/部署

### Phase 6: 集成测试 (2周)
- [ ] 仿真→实车流程验证
- [ ] 端到端测试
- [ ] 文档完善
