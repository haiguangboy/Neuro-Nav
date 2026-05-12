# Neuro-Nav 导航框架架构设计 v2

## 更新说明
- 新增跨主机通信支持（MQTT/gRPC/ZeroMQ）
- 预留InternNav/InternVLA-N1模型接口
- 参考zhongli-bridge的协议设计

---

## 整体框架结构

```
**neuro-nav框架**
│
├── app: 业务入口层
│   ├── sdk                      # Python SDK (VLA开发者)
│   ├── ros2_api                 # ROS2 Action/Service
│   ├── vla_agent                # VLA Agent接入
│   └── cli                      # 命令行工具
│
├── core: 导航核心逻辑
│   │
│   ├── planner: 规划器 (可插拔设计)
│   │   ├── base_interface       # 统一基类接口
│   │   │
│   │   ├── traditional          # 传统规划器
│   │   │   ├── nav2_global      # NavFn/A*/Smac
│   │   │   └── nav2_local       # DWB/TEB/MPPI/RPP
│   │   │
│   │   ├── learned              # 学习型规划器
│   │   │   ├── vln_planner      # VLN语言导航 (离散动作)
│   │   │   ├── e2e_planner      # 端到端 (GNM/ViNT/NoMaD)
│   │   │   ├── diffusion_planner # NavDP扩散策略
│   │   │   └── vla_planner      # VLA移动指令
│   │   │
│   │   ├── hybrid               # 混合规划
│   │   │   ├── dual_system      # 双系统(InternVLA-N1风格)
│   │   │   └── semantic_local   # 语义全局+传统局部
│   │   │
│   │   └── intern_nav           # ★ InternNav模型适配
│   │       ├── intern_vla_n1    # InternVLA-N1 (S1+S2)
│   │       ├── nav_dp           # NavDP扩散策略
│   │       ├── stream_vln       # StreamVLN流式
│   │       └── intern_adapter   # 统一适配层
│   │
│   ├── decision: 决策引擎
│   │   ├── behavior_tree        # BT.CPP行为树
│   │   ├── mode_selector        # 规划模式选择器
│   │   ├── state_machine        # 任务状态机
│   │   └── recovery             # 异常恢复策略
│   │
│   ├── map_manager: 地图管理
│   │   ├── global_map           # 全局地图
│   │   │   ├── costmap_2d       # 代价地图
│   │   │   ├── semantic         # 语义地图
│   │   │   └── topology         # 拓扑地图
│   │   ├── local_map            # 局部地图
│   │   └── no_map_mode          # 无地图模式(E2E专用)
│   │
│   ├── trajectory: 轨迹处理
│   │   ├── fusion               # 多源轨迹融合
│   │   ├── validate             # 可行性验证
│   │   ├── smooth               # 轨迹平滑
│   │   └── protocol_convert     # ★ 协议格式转换
│   │
│   └── stance                   # 操作位姿生成
│
├── interface: 接口适配层
│   │
│   ├── upstream: 上游接口 (输入)
│   │   ├── localization         # 定位模块 (xyz-slam)
│   │   ├── perception           # 感知模块
│   │   ├── vla_bridge           # VLA模型桥接
│   │   └── intern_nav_bridge    # ★ InternNav桥接
│   │
│   ├── downstream: 下游接口 (输出)
│   │   ├── controller           # 控制模块接口
│   │   ├── cmd_vel              # 速度指令 (本地)
│   │   └── trajectory_out       # 轨迹输出 (本地)
│   │
│   └── communication: ★ 跨主机通信层
│       ├── ros2_adapter         # ROS2 Topic/Action (本机)
│       ├── mqtt_bridge          # ★ MQTT桥接 (跨主机)
│       │   ├── client           # MQTT客户端
│       │   ├── trajectory_pub   # 轨迹发布
│       │   ├── status_sub       # 状态订阅
│       │   └── protocol         # 协议编解码
│       ├── grpc_adapter         # ★ gRPC (跨主机/跨语言)
│       ├── zeromq_adapter       # ★ ZeroMQ (低延迟)
│       └── dds_adapter          # 原生DDS
│
├── sensor: 传感器适配
│   ├── camera                   # RGB相机
│   ├── lidar                    # 激光雷达
│   ├── depth                    # 深度相机
│   └── odom                     # 里程计
│
├── protocol: ★ 协议定义
│   ├── neuro_nav_protocol       # Neuro-Nav内部协议
│   ├── zhongli_protocol         # 中力协议适配
│   ├── intern_protocol          # InternNav协议
│   └── custom_protocol          # 自定义协议扩展
│
├── nav2_plugins: Nav2扩展
│   ├── planners                 # 自定义规划器插件
│   ├── controllers              # 自定义控制器插件
│   └── bt_nodes                 # 自定义BT节点
│
├── config: 配置管理
│   ├── robot_params             # 机器人参数
│   ├── planner_params           # 规划器参数
│   ├── comm_params              # 通信参数
│   └── behavior_params          # 行为配置
│
└── util: 通用工具
    ├── math                     # 数学工具
    ├── geometry                 # 几何工具
    ├── logging                  # 日志系统
    └── visualization            # 可视化
```

---

## 核心设计详解

### 1. 跨主机通信架构 (参考zhongli-bridge)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Communication Layer                               │
│                                                                          │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐            │
│  │   ROS2 Local   │  │   MQTT Bridge  │  │  gRPC/ZeroMQ   │            │
│  │                │  │                │  │                │            │
│  │ - Topic/Action │  │ - Trajectory   │  │ - Model Infer  │            │
│  │ - Same Host    │  │ - Status/Odom  │  │ - Low Latency  │            │
│  │ - Nav2 Stack   │  │ - Cross Host   │  │ - Cross Lang   │            │
│  └───────┬────────┘  └───────┬────────┘  └───────┬────────┘            │
│          │                   │                   │                      │
│          └───────────────────┼───────────────────┘                      │
│                              ▼                                          │
│                   ┌──────────────────┐                                  │
│                   │  Comm Manager    │                                  │
│                   │  (路由选择)       │                                  │
│                   └──────────────────┘                                  │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘

应用场景:
┌──────────────────┐        MQTT/gRPC         ┌──────────────────┐
│  导航主机 (Orin) │ ◄─────────────────────► │   控制主机 (PLC) │
│                  │        轨迹/状态          │                  │
│  - Neuro-Nav     │                          │  - 运动控制器     │
│  - xyz-slam      │                          │  - 底盘驱动      │
│  - 感知模块      │                          │  - 安全逻辑      │
└──────────────────┘                          └──────────────────┘
```

### 2. MQTT Bridge 详细设计 (参考zhongli协议)

```cpp
// MQTT话题定义
namespace MqttTopics {
    // 发布 (导航 → 控制)
    const std::string TRAJECTORY = "neuro_nav/{robot_id}/trajectory";
    const std::string ACTION     = "neuro_nav/{robot_id}/action";
    const std::string STATUS     = "neuro_nav/{robot_id}/nav_status";
    
    // 订阅 (控制 → 导航)
    const std::string ODOM       = "neuro_nav/{robot_id}/odom";
    const std::string CTRL_STATUS = "neuro_nav/{robot_id}/ctrl_status";
    const std::string TASK_CMD   = "neuro_nav/{robot_id}/task_cmd";
}

// 轨迹消息格式 (参考zhongli协议扩展)
struct TrajectoryMessage {
    std::string trajectory_id;
    std::string task_id;
    int64_t timestamp_ms;
    
    // Beta-3协议兼容字段
    double orientation;      // 运动方向: 0.0前进, ±3.14后退
    int flag;               // 分支标志: 0非分支, 1进入分支
    
    // 轨迹点序列
    struct Point {
        double x, y, theta;
        double max_speed;
        double tolerance_dist;
        double tolerance_theta;
    };
    std::vector<Point> points;
    
    // 动作参数 (可选)
    std::optional<ActionParams> action_params;
};
```

### 3. InternNav 模型接口预留

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     InternNav Model Adapter                              │
│                                                                          │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                    InternNavInterface (基类)                       │  │
│  │                                                                    │  │
│  │  + init(config) -> bool                                           │  │
│  │  + set_goal(goal: SemanticGoal | PointGoal | ImageGoal)          │  │
│  │  + get_action() -> Action (离散) | Trajectory (连续)              │  │
│  │  + get_system_mode() -> System1 | System2 | DualSystem           │  │
│  │  + requires_observation() -> ObsType (RGB | RGBD | PointCloud)   │  │
│  │                                                                    │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│          ▲              ▲              ▲              ▲                 │
│          │              │              │              │                 │
│  ┌───────┴───┐  ┌───────┴───┐  ┌───────┴───┐  ┌───────┴───┐           │
│  │InternVLA  │  │  NavDP    │  │StreamVLN  │  │  GNM/ViNT │           │
│  │   N1      │  │           │  │           │  │  /NoMaD   │           │
│  │           │  │           │  │           │  │           │           │
│  │Dual-System│  │ Diffusion │  │ Streaming │  │  E2E Nav  │           │
│  │S1+S2      │  │ Policy    │  │ VLN       │  │           │           │
│  └───────────┘  └───────────┘  └───────────┘  └───────────┘           │
│                                                                          │
│  输入/输出适配:                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │ ObservationType:                                                  │   │
│  │   - RGB Image (Camera)                                           │   │
│  │   - RGBD (Camera + Depth)                                        │   │
│  │   - PointCloud (LiDAR)                                           │   │
│  │   - Semantic Features                                            │   │
│  │                                                                   │   │
│  │ OutputType:                                                       │   │
│  │   - DiscreteAction (forward/left/right/stop)  → 需要局部规划器   │   │
│  │   - ContinuousAction (vx, vy, omega)          → 直接控制         │   │
│  │   - Waypoints (稀疏航点序列)                   → 需要轨迹插值    │   │
│  │   - DenseTrajectory (密集轨迹)                 → 直接执行        │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 4. InternVLA-N1 双系统架构适配

```
InternVLA-N1 双系统设计:

┌─────────────────────────────────────────────────────────────────────────┐
│                        InternVLA-N1 Adapter                              │
│                                                                          │
│  ┌─────────────────────┐         ┌─────────────────────┐               │
│  │   System 1 (S1)     │         │   System 2 (S2)     │               │
│  │   Visual Navigator  │         │   VLN Planner       │               │
│  │                     │         │                     │               │
│  │ 输入: RGB/RGBD      │         │ 输入: RGB + 语言指令 │               │
│  │ 输出: 短期轨迹      │         │ 输出: 离散动作序列  │               │
│  │ 特点: 快速反应      │         │ 特点: 语义理解      │               │
│  │       避障能力强    │         │       长程规划      │               │
│  └──────────┬──────────┘         └──────────┬──────────┘               │
│             │                               │                           │
│             └───────────────┬───────────────┘                           │
│                             ▼                                           │
│                   ┌─────────────────┐                                   │
│                   │  Dual System    │                                   │
│                   │  Coordinator    │                                   │
│                   │                 │                                   │
│                   │ S2→高层语义规划 │                                   │
│                   │ S1→底层轨迹执行 │                                   │
│                   └─────────────────┘                                   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘

工作模式:
1. 纯S1模式: PointGoal导航，快速响应
2. 纯S2模式: 语言指令导航，离散动作
3. 双系统模式: S2提供高层指导，S1执行底层控制
```

### 5. 模型推理部署方案

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     Model Inference Deployment                           │
│                                                                          │
│  方案A: 本地推理 (Orin NX/AGX)                                           │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  Neuro-Nav ────► TensorRT/ONNX ────► InternVLA-N1 (量化)        │   │
│  │                                                                   │   │
│  │  优点: 低延迟, 无网络依赖                                         │   │
│  │  缺点: 算力受限, 模型需量化                                       │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  方案B: 远程推理 (gRPC/ZeroMQ)                                           │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  Neuro-Nav (Orin) ←─gRPC─→ Inference Server (GPU Server)         │   │
│  │                                                                   │   │
│  │  优点: 算力充足, 可用大模型                                       │   │
│  │  缺点: 网络延迟, 依赖连接                                         │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  方案C: 混合推理                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  S1 (本地) ←──协调──→ S2 (远程)                                   │   │
│  │                                                                   │   │
│  │  S1视觉导航: 本地TensorRT, 实时避障                               │   │
│  │  S2语言规划: 远程服务, 高层决策                                   │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 接口定义

### 1. InternNav模型接口 (Python/C++)

```python
# Python接口 (供InternNav模型使用)
class InternNavInterface(ABC):
    """InternNav模型统一接口"""
    
    @abstractmethod
    def init(self, config: Dict) -> bool:
        """初始化模型"""
        pass
    
    @abstractmethod
    def set_observation(self, obs: Observation) -> None:
        """设置观测数据"""
        pass
    
    @abstractmethod
    def set_goal(self, goal: Union[PointGoal, ImageGoal, LanguageGoal]) -> None:
        """设置目标"""
        pass
    
    @abstractmethod
    def get_output(self) -> Union[DiscreteAction, ContinuousAction, Trajectory]:
        """获取模型输出"""
        pass
    
    @abstractmethod
    def get_system_mode(self) -> SystemMode:
        """获取系统模式 (S1/S2/Dual)"""
        pass


# 观测数据类型
@dataclass
class Observation:
    rgb: Optional[np.ndarray] = None          # (H, W, 3)
    depth: Optional[np.ndarray] = None        # (H, W)
    pointcloud: Optional[np.ndarray] = None   # (N, 3)
    odom: Optional[Odometry] = None
    timestamp: float = 0.0


# 目标类型
@dataclass
class PointGoal:
    x: float
    y: float
    theta: Optional[float] = None

@dataclass
class ImageGoal:
    target_image: np.ndarray
    
@dataclass
class LanguageGoal:
    instruction: str
    language: str = "en"  # 或 "zh"


# 输出类型
@dataclass
class DiscreteAction:
    action: int  # 0:stop, 1:forward, 2:left, 3:right
    
@dataclass
class ContinuousAction:
    linear_x: float
    linear_y: float
    angular_z: float

@dataclass
class Trajectory:
    points: List[TrajectoryPoint]
    
    @dataclass
    class TrajectoryPoint:
        x: float
        y: float
        theta: float
        velocity: float
        timestamp: float
```

### 2. 跨主机通信接口

```cpp
// C++ 通信管理器接口
class CommunicationManager {
public:
    // 初始化
    bool init(const CommConfig& config);
    
    // 发布轨迹 (自动选择通信方式)
    bool publishTrajectory(const Trajectory& traj, CommChannel channel = AUTO);
    
    // 发布状态
    bool publishStatus(const NavStatus& status);
    
    // 订阅回调
    void subscribeOdom(std::function<void(const Odometry&)> callback);
    void subscribeControlStatus(std::function<void(const CtrlStatus&)> callback);
    
    // 通信通道
    enum class CommChannel {
        AUTO,           // 自动选择
        ROS2_LOCAL,     // ROS2本机
        MQTT,           // MQTT跨主机
        GRPC,           // gRPC
        ZEROMQ          // ZeroMQ
    };
    
private:
    std::unique_ptr<MqttBridge> mqtt_bridge_;
    std::unique_ptr<GrpcClient> grpc_client_;
    std::unique_ptr<ZmqSocket> zmq_socket_;
};


// MQTT Bridge (参考zhongli-bridge设计)
class MqttBridge {
public:
    bool connect(const std::string& broker, int port);
    bool publishTrajectory(const TrajectoryMsg& msg);
    bool publishAction(const ActionMsg& msg);
    void subscribeOdom(OdomCallback callback);
    
private:
    // 协议编解码
    std::string encodeTrajectory(const TrajectoryMsg& msg);
    TrajectoryMsg decodeTrajectory(const std::string& payload);
    
    // Beta-3协议支持
    void encodeOrientation(double orientation, std::string& frame_id);
    void encodeFlag(int flag, std::string& frame_id);
};
```

### 3. 轨迹协议转换

```cpp
// 轨迹协议转换器
class TrajectoryProtocolConverter {
public:
    // Nav2 Path → Neuro-Nav Trajectory
    Trajectory fromNav2Path(const nav_msgs::msg::Path& path);
    
    // InternNav Output → Neuro-Nav Trajectory  
    Trajectory fromInternNav(const InternNavOutput& output);
    
    // Neuro-Nav Trajectory → MQTT Message (zhongli协议)
    MqttTrajectoryMsg toMqttMessage(const Trajectory& traj, 
                                     const ProtocolConfig& config);
    
    // Neuro-Nav Trajectory → Nav2 Path
    nav_msgs::msg::Path toNav2Path(const Trajectory& traj);

private:
    // Beta-3协议编码
    std::string encodeBeta3FrameId(
        const std::string& frame,
        const std::string& action_type,
        double orientation,
        int flag,
        const std::optional<ContainerParams>& container
    );
};
```

---

## 业务场景与通信方式映射

| 场景 | 规划模式 | 通信方式 | 说明 |
|------|---------|---------|------|
| 本机开发测试 | Traditional | ROS2 Local | Nav2原生 |
| 实车部署(单主机) | Traditional/Hybrid | ROS2 Local | Orin+底盘一体 |
| 实车部署(分布式) | Traditional/Hybrid | MQTT | Orin导航 + PLC控制 |
| InternNav本地推理 | E2E/Dual | ROS2 Local | TensorRT本地 |
| InternNav远程推理 | E2E/Dual | gRPC | GPU服务器推理 |
| 仿真测试(IsaacSim) | Any | ROS2/gRPC | 仿真环境适配 |
| 叉车堆垛任务 | Traditional + Action | MQTT (zhongli) | 中力协议 |

---

## 配置示例

### 通信配置 `config/comm_config.yaml`

```yaml
communication:
  # 默认通信方式
  default_channel: "ros2_local"  # ros2_local / mqtt / grpc / zeromq
  
  # ROS2 配置
  ros2:
    trajectory_topic: "/neuro_nav/trajectory"
    cmd_vel_topic: "/cmd_vel"
    odom_topic: "/odom"
  
  # MQTT 配置 (跨主机)
  mqtt:
    enabled: true
    broker_host: "192.168.1.100"
    broker_port: 1883
    robot_id: "forklift-001"
    topics:
      trajectory: "neuro_nav/{robot_id}/trajectory"
      action: "neuro_nav/{robot_id}/action"
      status: "neuro_nav/{robot_id}/status"
      odom: "neuro_nav/{robot_id}/odom"
    # Beta-3协议支持
    protocol:
      version: "beta3"
      orientation_encoding: true
      flag_encoding: true
  
  # gRPC 配置 (模型推理)
  grpc:
    enabled: true
    inference_server: "192.168.1.200:50051"
    timeout_ms: 100
  
  # ZeroMQ 配置 (低延迟)
  zeromq:
    enabled: false
    pub_endpoint: "tcp://*:5555"
    sub_endpoint: "tcp://192.168.1.100:5556"
```

### InternNav模型配置 `config/intern_nav_config.yaml`

```yaml
intern_nav:
  # 模型选择
  model: "intern_vla_n1"  # intern_vla_n1 / nav_dp / stream_vln / gnm / vint / nomad
  
  # 推理方式
  inference:
    mode: "local"  # local / remote / hybrid
    local:
      model_path: "/models/intern_vla_n1_quantized.onnx"
      device: "tensorrt"  # tensorrt / cuda / cpu
    remote:
      server: "192.168.1.200:50051"
      protocol: "grpc"
  
  # InternVLA-N1 特定配置
  intern_vla_n1:
    system_mode: "dual"  # s1 / s2 / dual
    s1:
      # System1: 视觉导航
      input: "rgbd"
      output: "continuous_trajectory"
      horizon: 10  # 预测步数
    s2:
      # System2: 语言导航
      input: "rgb_language"
      output: "discrete_action"
      language: "zh"  # 中文指令支持
  
  # NavDP 配置
  nav_dp:
    diffusion_steps: 10
    horizon: 16
    privileged_info: true  # 使用特权信息
  
  # 观测配置
  observation:
    camera:
      width: 640
      height: 480
      fov: 90
    depth:
      enabled: true
      max_range: 10.0
```

---

## 实施路线 (更新)

### Phase 1: 基础框架 + 通信层 (2周)
- [ ] 核心目录结构搭建
- [ ] Nav2基座集成
- [ ] MQTT Bridge实现 (参考zhongli-bridge)
- [ ] 基础ROS2适配

### Phase 2: 决策引擎 + 协议层 (2周)
- [ ] 行为树扩展
- [ ] Mode Selector实现
- [ ] 轨迹协议转换器
- [ ] Beta-3协议支持

### Phase 3: InternNav接口 (3周)
- [ ] InternNav基类接口定义
- [ ] InternVLA-N1适配器
- [ ] NavDP适配器
- [ ] GNM/ViNT/NoMaD适配器
- [ ] 本地/远程推理支持

### Phase 4: 集成测试 (2周)
- [ ] 仿真环境测试 (Isaac Sim)
- [ ] 实车通信测试
- [ ] 端到端流程验证
- [ ] 文档完善
