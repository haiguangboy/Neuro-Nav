```
**neuro-nav框架** (v2)
- app: 业务入口，支持SDK/ROS2/VLA Agent等多种接入方式
  - core: 导航核心逻辑
    - planner: 可插拔规划器
      - base_interface        # 统一规划器接口
      - traditional           # 传统规划器
        - nav2_global         # NavFn/A*/Smac
        - nav2_local          # DWB/TEB/MPPI/RPP
      - learned               # 学习型规划器
        - vln_planner         # VLN语言导航(离散动作)
        - e2e_planner         # 端到端(GNM/ViNT/NoMaD)
        - diffusion_planner   # NavDP扩散策略
        - vla_planner         # VLA移动指令
      - hybrid                # 混合规划
        - dual_system         # 双系统(S1+S2)
        - semantic_local      # 语义全局+传统局部
      - intern_nav            # ★ InternNav模型适配
        - intern_vla_n1       # InternVLA-N1双系统
        - nav_dp              # NavDP扩散策略
        - stream_vln          # StreamVLN流式
        - intern_adapter      # 统一适配层
    - decision: 决策引擎
      - behavior_tree         # BT.CPP行为树
      - mode_selector         # 规划模式选择器
      - state_machine         # 任务状态机
      - recovery              # 异常恢复策略
    - map_manager: 地图管理
      - global_map            # 全局地图(Costmap/Semantic/Topo)
      - local_map             # 局部地图(Rolling Costmap)
      - no_map_mode           # 无地图模式(E2E专用)
    - trajectory: 轨迹处理
      - fusion                # 多源轨迹融合
      - validate              # 可行性验证
      - smooth                # 轨迹平滑
      - protocol_convert      # ★ 协议格式转换
    - stance                  # 操作位姿生成
  - interface: 接口适配层
    - upstream                # 上游接口(输入)
      - localization          # 定位模块(xyz-slam)
      - perception            # 感知模块
      - vla_bridge            # VLA模型桥接
      - intern_nav_bridge     # ★ InternNav桥接
    - downstream              # 下游接口(输出)
      - controller            # 控制模块接口
      - cmd_vel               # 速度指令(本地)
      - trajectory_out        # 轨迹输出(本地)
    - communication           # ★ 跨主机通信层
      - ros2_adapter          # ROS2 Topic/Action(本机)
      - mqtt_bridge           # ★ MQTT桥接(跨主机)
        - client              # MQTT客户端
        - trajectory_pub      # 轨迹发布
        - status_sub          # 状态订阅
        - protocol            # 协议编解码(Beta-3)
      - grpc_adapter          # ★ gRPC(跨主机/模型推理)
      - zeromq_adapter        # ★ ZeroMQ(低延迟)
      - dds_adapter           # 原生DDS
  - sensor: 传感器适配
    - camera                  # RGB相机
    - lidar                   # 激光雷达
    - depth                   # 深度相机
    - odom                    # 里程计
  - protocol                  # ★ 协议定义
    - neuro_nav_protocol      # Neuro-Nav内部协议
    - zhongli_protocol        # 中力协议(Beta-3)
    - intern_protocol         # InternNav协议
    - custom_protocol         # 自定义协议扩展
  - config: 配置管理
    - robot_params            # 机器人参数
    - planner_params          # 规划器参数
    - comm_params             # ★ 通信参数
    - behavior_params         # 行为配置
- sdk: Python SDK (VLA开发者)
  - mobility_agent            # 移动代理封装
  - semantic_api              # 语义化API
  - async_client              # 异步客户端
- nav2_plugins: Nav2扩展
  - planners                  # 自定义规划器插件
  - controllers               # 自定义控制器插件
  - bt_nodes                  # 自定义BT节点
- util: 通用工具
  - math                      # 数学工具
  - geometry                  # 几何工具
  - logging                   # 日志系统
  - visualization             # 可视化
```

---

## 关键接口定义

### 1. InternNav统一接口

```python
class InternNavInterface(ABC):
    """InternNav模型统一接口"""
    
    # 输入类型
    class ObsType(Enum):
        RGB = "rgb"
        RGBD = "rgbd"
        POINTCLOUD = "pointcloud"
    
    # 输出类型  
    class OutputType(Enum):
        DISCRETE_ACTION = "discrete"    # forward/left/right/stop
        CONTINUOUS_ACTION = "continuous" # vx, vy, omega
        WAYPOINTS = "waypoints"         # 稀疏航点
        TRAJECTORY = "trajectory"       # 密集轨迹
    
    # 系统模式
    class SystemMode(Enum):
        S1 = "system1"      # 视觉导航(快速响应)
        S2 = "system2"      # 语言导航(语义理解)
        DUAL = "dual"       # 双系统协同
    
    @abstractmethod
    def set_observation(self, obs: Observation) -> None: ...
    
    @abstractmethod
    def set_goal(self, goal: Goal) -> None: ...
    
    @abstractmethod
    def get_output(self) -> Output: ...
    
    @abstractmethod
    def get_system_mode(self) -> SystemMode: ...
```

### 2. 跨主机通信接口

```cpp
class CommunicationManager {
public:
    enum class Channel { AUTO, ROS2_LOCAL, MQTT, GRPC, ZEROMQ };
    
    // 发布轨迹 (自动选择通信方式)
    bool publishTrajectory(const Trajectory& traj, Channel ch = AUTO);
    
    // 订阅回调
    void subscribeOdom(OdomCallback callback);
    void subscribeControlStatus(CtrlStatusCallback callback);
    
    // 通信状态
    bool isConnected(Channel ch);
    int getLatencyMs(Channel ch);
};
```

### 3. 协议转换接口

```cpp
class TrajectoryProtocolConverter {
public:
    // Nav2 Path ↔ Neuro-Nav Trajectory
    Trajectory fromNav2Path(const nav_msgs::msg::Path& path);
    nav_msgs::msg::Path toNav2Path(const Trajectory& traj);
    
    // InternNav Output → Neuro-Nav Trajectory
    Trajectory fromInternNav(const InternNavOutput& output);
    
    // Neuro-Nav Trajectory → MQTT (zhongli Beta-3协议)
    MqttMsg toMqttMessage(const Trajectory& traj, const ProtocolConfig& cfg);
};
```

---

## 通信架构图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           Neuro-Nav 通信架构                             │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                      Communication Manager                        │   │
│  │                                                                   │   │
│  │   场景判断 → 自动选择通道:                                         │   │
│  │   - 本机ROS2节点 → ROS2 Local                                     │   │
│  │   - 远程控制器 → MQTT                                             │   │
│  │   - 模型推理服务 → gRPC                                           │   │
│  │   - 实时控制 → ZeroMQ                                             │   │
│  │                                                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│          │              │              │              │                 │
│          ▼              ▼              ▼              ▼                 │
│  ┌───────────┐  ┌───────────┐  ┌───────────┐  ┌───────────┐           │
│  │ROS2 Local │  │   MQTT    │  │   gRPC    │  │  ZeroMQ   │           │
│  │           │  │   Bridge  │  │  Client   │  │  Socket   │           │
│  │同主机     │  │跨主机     │  │模型推理   │  │低延迟     │           │
│  │Nav2原生   │  │中力协议   │  │InternNav  │  │实时控制   │           │
│  └─────┬─────┘  └─────┬─────┘  └─────┬─────┘  └─────┬─────┘           │
│        │              │              │              │                   │
│        ▼              ▼              ▼              ▼                   │
│  ┌───────────┐  ┌───────────┐  ┌───────────┐  ┌───────────┐           │
│  │Nav2 Stack │  │PLC控制器  │  │GPU Server │  │实时控制器 │           │
│  │(本机)     │  │(192.168.x)│  │(推理服务) │  │(底盘)     │           │
│  └───────────┘  └───────────┘  └───────────┘  └───────────┘           │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## InternNav模型适配策略

| 模型 | 系统模式 | 输入 | 输出 | 推理方式 | 地图需求 |
|------|---------|------|------|---------|---------|
| InternVLA-N1 | Dual | RGBD+Language | Traj/Action | Local+Remote | 可选 |
| InternVLA-N1 S1 | S1 | RGBD | Trajectory | Local (TensorRT) | 无 |
| InternVLA-N1 S2 | S2 | RGB+Language | Discrete | Remote (GPU) | 可选 |
| NavDP | S1 | RGBD | Trajectory | Local | 无 |
| StreamVLN | S2 | RGB+Language | Discrete | Remote | 可选 |
| GNM/ViNT/NoMaD | S1 | RGB | Waypoints | Local | 无 |

---

## SDK使用示例

```python
from neuro_nav import MobilityAgent, PlanningMode

# 初始化 (指定通信方式)
agent = MobilityAgent(
    robot="forklift_3w",
    comm_config={
        "default": "mqtt",
        "mqtt_broker": "192.168.1.100:1883",
        "grpc_server": "192.168.1.200:50051"
    }
)

# === 传统导航 (Nav2 + MQTT发布) ===
agent.go_to(x=5.0, y=3.0, theta=0.0)

# === InternVLA-N1 双系统导航 ===
agent.go_to(
    instruction="去咖啡机旁边",
    mode=PlanningMode.INTERN_VLA_N1,
    system_mode="dual"  # s1/s2/dual
)

# === NavDP 扩散策略导航 ===
agent.go_to(
    target=(5.0, 3.0),
    mode=PlanningMode.NAV_DP
)

# === 查询通信状态 ===
status = agent.get_comm_status()
print(f"MQTT: {status.mqtt_connected}, latency: {status.mqtt_latency_ms}ms")
print(f"gRPC: {status.grpc_connected}, latency: {status.grpc_latency_ms}ms")
```
