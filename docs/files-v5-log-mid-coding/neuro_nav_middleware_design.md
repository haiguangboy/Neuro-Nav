# Neuro-Nav 中间件与数据流设计

> 从高级系统工程师视角分析系统数据交互的实时性、安全性、可靠性设计

---

## 1. 系统数据流全景

### 1.1 数据流拓扑

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              External Systems                                    │
│                                                                                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐ │
│  │ 感知模块 │  │ 定位模块 │  │ 控制器  │  │ 大模型   │  │ 上位机/云端      │ │
│  │ Percep.  │  │ Locali.  │  │ Contrl.  │  │ LLM/VLA  │  │ Fleet Manager   │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────┬─────────┘ │
│       │             │             │             │                  │            │
│       │ PointCloud  │ Pose        │ cmd_vel    │ Trajectory       │ Task      │
│       │ ~30Hz       │ ~100Hz      │ ~50Hz      │ ~1-10Hz          │ ~0.1Hz    │
│       │ ~1MB/msg    │ ~100B/msg   │ ~50B/msg   │ ~10KB/msg        │ ~1KB/msg  │
│       │             │             │             │                  │            │
└───────┼─────────────┼─────────────┼─────────────┼──────────────────┼────────────┘
        │             │             │             │                  │
        ▼             ▼             ▼             ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                                                                                  │
│                           Neuro-Nav System                                       │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                     Data Ingestion Layer                                 │   │
│  │                                                                          │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │   │
│  │  │ Sensor Hub  │  │  Pose Hub   │  │  Model Hub  │  │  Task Hub   │    │   │
│  │  │ (ZeroMQ)    │  │  (ROS2)     │  │  (gRPC)     │  │  (MQTT)     │    │   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘    │   │
│  │         │                │                │                │            │   │
│  └─────────┼────────────────┼────────────────┼────────────────┼────────────┘   │
│            │                │                │                │                 │
│            ▼                ▼                ▼                ▼                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                     Processing Layer                                     │   │
│  │                                                                          │   │
│  │  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐            │   │
│  │  │   Perception   │  │    Planner     │  │   Decision     │            │   │
│  │  │   Pipeline     │  │    Pipeline    │  │   Engine       │            │   │
│  │  │                │  │                │  │                │            │   │
│  │  │ nvblox→ESDF→   │  │ Global→Local→ │  │ BT→Mode→      │            │   │
│  │  │ Costmap        │  │ Trajectory    │  │ Recovery       │            │   │
│  │  └────────┬───────┘  └────────┬───────┘  └────────┬───────┘            │   │
│  │           │                   │                   │                     │   │
│  │           └───────────────────┼───────────────────┘                     │   │
│  │                               │                                         │   │
│  └───────────────────────────────┼─────────────────────────────────────────┘   │
│                                  │                                              │
│                                  ▼                                              │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                     Output Layer                                         │   │
│  │                                                                          │   │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │   │
│  │  │                  Navigation Command                              │   │   │
│  │  │                  (Twist / BodyTraj / Footsteps)                  │   │   │
│  │  └──────────────────────────────┬──────────────────────────────────┘   │   │
│  │                                 │                                       │   │
│  └─────────────────────────────────┼───────────────────────────────────────┘   │
│                                    │                                            │
└────────────────────────────────────┼────────────────────────────────────────────┘
                                     │
                                     ▼
                              ┌──────────────┐
                              │  Locomotion  │
                              │  Controller  │
                              └──────────────┘
```

### 1.2 数据流特性分析

| 数据流 | 频率 | 大小 | 延迟要求 | 可靠性 | 推荐协议 |
|--------|------|------|---------|--------|----------|
| 点云 | 10-30Hz | 0.5-2MB | <50ms | 可丢帧 | ZeroMQ/共享内存 |
| 位姿 | 50-200Hz | ~100B | <10ms | 高 | ROS2 DDS |
| 控制指令 | 20-100Hz | ~50B | <20ms | 最高 | ROS2 DDS |
| 模型推理请求 | 1-10Hz | 10KB-1MB | <500ms | 高 | gRPC |
| 任务指令 | ~0.1Hz | ~1KB | <1s | 最高 | MQTT QoS2 |
| 日志/诊断 | 1-10Hz | ~1KB | 无限制 | 中 | 异步文件 |

---

## 2. 实时性保证

### 2.1 优先级分层

```cpp
// common/concurrency/priority.hpp
namespace neuro_nav::concurrency {

// 线程优先级定义 (Linux SCHED_FIFO)
enum class ThreadPriority {
    REALTIME_CRITICAL = 99,  // 控制输出、安全监控
    REALTIME_HIGH = 90,      // 局部规划、避障
    REALTIME_NORMAL = 80,    // 感知处理
    NORMAL = 0,              // 普通任务
    BACKGROUND = -10,        // 日志、诊断
};

// 设置线程优先级
inline bool set_thread_priority(std::thread& t, ThreadPriority priority) {
    sched_param param;
    param.sched_priority = static_cast<int>(priority);
    
    int policy = (priority >= ThreadPriority::REALTIME_NORMAL) 
                 ? SCHED_FIFO : SCHED_OTHER;
    
    return pthread_setschedparam(t.native_handle(), policy, &param) == 0;
}

// 绑定CPU核心 (避免调度抖动)
inline bool bind_to_cpu(std::thread& t, int cpu_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    return pthread_setaffinity_np(t.native_handle(), sizeof(cpuset), &cpuset) == 0;
}

}  // namespace neuro_nav::concurrency
```

### 2.2 关键路径延迟分析

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                    Critical Path: 感知 → 规划 → 控制                            │
│                                                                                  │
│  时间预算: 50ms (20Hz控制频率)                                                   │
│                                                                                  │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────────┐   │
│  │   传感器     │────▶│   nvblox     │────▶│  局部规划    │────▶│  输出    │   │
│  │   接收       │     │   TSDF/ESDF  │     │  TEB/MPPI    │     │  cmd_vel │   │
│  │              │     │              │     │              │     │          │   │
│  │   <5ms       │     │   <15ms      │     │   <25ms      │     │   <5ms   │   │
│  └──────────────┘     └──────────────┘     └──────────────┘     └──────────┘   │
│                                                                                  │
│  关键优化点:                                                                     │
│  1. 传感器接收: 零拷贝/共享内存                                                  │
│  2. nvblox: GPU加速，流水线处理                                                  │
│  3. 规划器: 增量更新，warm start                                                 │
│  4. 输出: 无锁队列                                                               │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 2.3 延迟监控

```cpp
// common/time/latency_monitor.hpp
namespace neuro_nav::time {

class LatencyMonitor {
public:
    struct Stats {
        double min_ms;
        double max_ms;
        double avg_ms;
        double p99_ms;
        size_t count;
        size_t deadline_miss_count;
    };
    
    explicit LatencyMonitor(const std::string& name, double deadline_ms)
        : name_(name), deadline_ms_(deadline_ms) {}
    
    // RAII 计时
    class ScopedTimer {
    public:
        explicit ScopedTimer(LatencyMonitor& monitor)
            : monitor_(monitor), start_(std::chrono::steady_clock::now()) {}
        
        ~ScopedTimer() {
            auto elapsed = std::chrono::steady_clock::now() - start_;
            monitor_.record(std::chrono::duration<double, std::milli>(elapsed).count());
        }
        
    private:
        LatencyMonitor& monitor_;
        std::chrono::steady_clock::time_point start_;
    };
    
    ScopedTimer start_timer() { return ScopedTimer(*this); }
    
    void record(double latency_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        samples_.push_back(latency_ms);
        if (latency_ms > deadline_ms_) {
            deadline_misses_++;
            LOG_WARN_THROTTLE(1.0, "{} deadline miss: {:.2f}ms > {:.2f}ms",
                              name_, latency_ms, deadline_ms_);
        }
    }
    
    Stats get_stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (samples_.empty()) return {};
        
        auto sorted = samples_;
        std::sort(sorted.begin(), sorted.end());
        
        return {
            .min_ms = sorted.front(),
            .max_ms = sorted.back(),
            .avg_ms = std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size(),
            .p99_ms = sorted[static_cast<size_t>(sorted.size() * 0.99)],
            .count = sorted.size(),
            .deadline_miss_count = deadline_misses_
        };
    }
    
private:
    std::string name_;
    double deadline_ms_;
    mutable std::mutex mutex_;
    std::vector<double> samples_;
    size_t deadline_misses_ = 0;
};

// 使用示例
static LatencyMonitor planner_latency("local_planner", 25.0);

void LocalPlanner::compute() {
    auto timer = planner_latency.start_timer();
    // ... 规划逻辑 ...
}  // 自动记录延迟

}  // namespace neuro_nav::time
```

---

## 3. 零拷贝与共享内存

### 3.1 大数据传输策略

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                        大数据传输策略                                            │
│                                                                                  │
│  数据大小          推荐方式                      延迟         内存开销           │
│  ─────────────────────────────────────────────────────────────────────────────  │
│  < 1KB            ROS2 DDS (值传递)              ~1ms         低                │
│  1KB - 100KB      ROS2 DDS (zero-copy loan)      ~1ms         低                │
│  100KB - 10MB     共享内存 + 句柄传递            ~0.1ms       低                │
│  > 10MB           共享内存 + mmap                ~0.1ms       低                │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 共享内存实现

```cpp
// common/memory/shared_memory.hpp
#include <sys/mman.h>
#include <fcntl.h>

namespace neuro_nav::memory {

// 共享内存区域
template<typename T>
class SharedMemoryRegion {
public:
    SharedMemoryRegion(const std::string& name, size_t size, bool create = false)
        : name_(name), size_(size) 
    {
        int flags = create ? (O_CREAT | O_RDWR) : O_RDWR;
        fd_ = shm_open(name.c_str(), flags, 0666);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open shared memory: " + name);
        }
        
        if (create) {
            ftruncate(fd_, size);
        }
        
        ptr_ = static_cast<T*>(mmap(nullptr, size, PROT_READ | PROT_WRITE,
                                     MAP_SHARED, fd_, 0));
        if (ptr_ == MAP_FAILED) {
            close(fd_);
            throw std::runtime_error("Failed to mmap shared memory");
        }
    }
    
    ~SharedMemoryRegion() {
        if (ptr_ != MAP_FAILED) {
            munmap(ptr_, size_);
        }
        if (fd_ >= 0) {
            close(fd_);
        }
    }
    
    T* get() { return ptr_; }
    const T* get() const { return ptr_; }
    size_t size() const { return size_; }
    
private:
    std::string name_;
    size_t size_;
    int fd_ = -1;
    T* ptr_ = nullptr;
};

// 共享点云缓冲区
struct SharedPointCloudBuffer {
    std::atomic<uint64_t> sequence;      // 序列号
    std::atomic<uint64_t> timestamp_ns;  // 时间戳
    uint32_t num_points;                 // 点数
    uint32_t point_step;                 // 每点字节数
    float data[];                        // 点云数据 (flexible array)
};

// 生产者
class PointCloudProducer {
public:
    explicit PointCloudProducer(const std::string& shm_name, size_t max_points)
        : shm_(shm_name, sizeof(SharedPointCloudBuffer) + max_points * sizeof(float) * 4, true)
    {}
    
    void publish(const PointCloud& cloud) {
        auto* buf = shm_.get();
        
        // 拷贝数据到共享内存
        buf->num_points = cloud.size();
        buf->point_step = sizeof(float) * 4;  // x, y, z, intensity
        std::memcpy(buf->data, cloud.data(), cloud.size() * buf->point_step);
        
        // 更新时间戳和序列号 (原子操作，确保消费者看到一致数据)
        buf->timestamp_ns.store(now_ns(), std::memory_order_release);
        buf->sequence.fetch_add(1, std::memory_order_release);
    }
    
private:
    SharedMemoryRegion<SharedPointCloudBuffer> shm_;
};

// 消费者
class PointCloudConsumer {
public:
    explicit PointCloudConsumer(const std::string& shm_name, size_t max_points)
        : shm_(shm_name, sizeof(SharedPointCloudBuffer) + max_points * sizeof(float) * 4)
    {}
    
    std::optional<PointCloud> try_get_latest() {
        auto* buf = shm_.get();
        
        // 读取序列号
        uint64_t seq1 = buf->sequence.load(std::memory_order_acquire);
        if (seq1 == last_seq_) {
            return std::nullopt;  // 没有新数据
        }
        
        // 读取数据
        PointCloud cloud(buf->num_points);
        std::memcpy(cloud.data(), buf->data, buf->num_points * buf->point_step);
        
        // 再次检查序列号，确保读取期间数据没有被更新
        uint64_t seq2 = buf->sequence.load(std::memory_order_acquire);
        if (seq1 != seq2) {
            return std::nullopt;  // 数据被覆盖，丢弃
        }
        
        last_seq_ = seq1;
        return cloud;
    }
    
private:
    SharedMemoryRegion<SharedPointCloudBuffer> shm_;
    uint64_t last_seq_ = 0;
};

}  // namespace neuro_nav::memory
```

### 3.3 ROS2 零拷贝

```cpp
// 使用 ROS2 loaned messages (Humble+)
#include <rclcpp/rclcpp.hpp>

class ZeroCopyPublisher : public rclcpp::Node {
public:
    ZeroCopyPublisher() : Node("zero_copy_pub") {
        // 创建支持零拷贝的发布者
        auto qos = rclcpp::QoS(10);
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", qos);
    }
    
    void publish(const PointCloud& cloud) {
        // 借用消息内存 (零拷贝)
        auto loaned_msg = pub_->borrow_loaned_message();
        auto& msg = loaned_msg.get();
        
        // 直接填充借用的内存
        fill_point_cloud_msg(cloud, msg);
        
        // 发布 (无拷贝)
        pub_->publish(std::move(loaned_msg));
    }
    
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};
```

---

## 4. 无锁数据结构

### 4.1 无锁队列 (SPSC)

```cpp
// common/concurrency/spsc_queue.hpp
namespace neuro_nav::concurrency {

// 单生产者单消费者无锁队列
template<typename T, size_t Capacity>
class SPSCQueue {
public:
    bool try_push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next = (head + 1) % Capacity;
        
        if (next == tail_.load(std::memory_order_acquire)) {
            return false;  // 队列满
        }
        
        buffer_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }
    
    bool try_push(T&& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next = (head + 1) % Capacity;
        
        if (next == tail_.load(std::memory_order_acquire)) {
            return false;
        }
        
        buffer_[head] = std::move(item);
        head_.store(next, std::memory_order_release);
        return true;
    }
    
    std::optional<T> try_pop() {
        size_t tail = tail_.load(std::memory_order_relaxed);
        
        if (tail == head_.load(std::memory_order_acquire)) {
            return std::nullopt;  // 队列空
        }
        
        T item = std::move(buffer_[tail]);
        tail_.store((tail + 1) % Capacity, std::memory_order_release);
        return item;
    }
    
    bool empty() const {
        return head_.load(std::memory_order_acquire) == 
               tail_.load(std::memory_order_acquire);
    }
    
private:
    std::array<T, Capacity> buffer_;
    alignas(64) std::atomic<size_t> head_{0};  // 避免 false sharing
    alignas(64) std::atomic<size_t> tail_{0};
};

}  // namespace neuro_nav::concurrency
```

### 4.2 无锁最新值缓存 (用于高频数据)

```cpp
// common/concurrency/latest_value.hpp
namespace neuro_nav::concurrency {

// 只保留最新值，适用于高频更新的传感器数据
template<typename T>
class LatestValue {
public:
    void update(T value) {
        auto new_data = std::make_shared<T>(std::move(value));
        std::atomic_store(&data_, new_data);
        sequence_.fetch_add(1, std::memory_order_release);
    }
    
    std::shared_ptr<T> get() const {
        return std::atomic_load(&data_);
    }
    
    // 获取并检查是否有更新
    std::pair<std::shared_ptr<T>, bool> get_if_newer(uint64_t& last_seq) const {
        uint64_t current_seq = sequence_.load(std::memory_order_acquire);
        if (current_seq == last_seq) {
            return {nullptr, false};
        }
        last_seq = current_seq;
        return {std::atomic_load(&data_), true};
    }
    
private:
    std::shared_ptr<T> data_;
    std::atomic<uint64_t> sequence_{0};
};

// 使用示例: 位姿订阅
class PoseSubscriber {
public:
    void pose_callback(const Pose& pose) {
        latest_pose_.update(pose);
    }
    
    Pose get_latest_pose() const {
        auto ptr = latest_pose_.get();
        return ptr ? *ptr : Pose{};
    }
    
private:
    LatestValue<Pose> latest_pose_;
};

}  // namespace neuro_nav::concurrency
```

---

## 5. 跨主机通信设计

### 5.1 通信协议选择

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                         跨主机通信协议选择                                       │
│                                                                                  │
│  场景                    推荐协议        原因                                    │
│  ─────────────────────────────────────────────────────────────────────────────  │
│  传感器数据 (高频/大数据)  ZeroMQ        最低延迟，支持零拷贝                      │
│  位姿/控制 (高频/小数据)   ROS2 DDS      已有生态，QoS丰富                        │
│  模型推理 (RPC风格)        gRPC          双向流，protobuf高效                     │
│  任务调度 (可靠/低频)      MQTT QoS2     保证送达，支持离线                       │
│  配置同步                  HTTP/REST     简单，易调试                            │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 5.2 ZeroMQ 传输层

```cpp
// interface/communication/zeromq/zmq_transport.hpp
#include <zmq.hpp>

namespace neuro_nav::interface {

// ZeroMQ 点云传输
class ZmqPointCloudTransport {
public:
    // 发送端
    class Publisher {
    public:
        explicit Publisher(const std::string& endpoint)
            : context_(1), socket_(context_, ZMQ_PUB)
        {
            socket_.bind(endpoint);
            
            // 设置高水位，避免内存暴涨
            int hwm = 2;
            socket_.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));
        }
        
        void publish(const PointCloud& cloud) {
            // 序列化
            size_t size = cloud.size() * sizeof(Point);
            zmq::message_t msg(size);
            std::memcpy(msg.data(), cloud.data(), size);
            
            // 非阻塞发送
            socket_.send(msg, zmq::send_flags::dontwait);
        }
        
    private:
        zmq::context_t context_;
        zmq::socket_t socket_;
    };
    
    // 接收端
    class Subscriber {
    public:
        explicit Subscriber(const std::string& endpoint)
            : context_(1), socket_(context_, ZMQ_SUB)
        {
            socket_.connect(endpoint);
            socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
            
            // 只保留最新消息
            int conflate = 1;
            socket_.setsockopt(ZMQ_CONFLATE, &conflate, sizeof(conflate));
        }
        
        std::optional<PointCloud> receive(int timeout_ms = 0) {
            zmq::message_t msg;
            zmq::recv_flags flags = timeout_ms > 0 
                                   ? zmq::recv_flags::none 
                                   : zmq::recv_flags::dontwait;
            
            if (timeout_ms > 0) {
                socket_.setsockopt(ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
            }
            
            auto result = socket_.recv(msg, flags);
            if (!result) {
                return std::nullopt;
            }
            
            // 反序列化
            size_t num_points = msg.size() / sizeof(Point);
            PointCloud cloud(num_points);
            std::memcpy(cloud.data(), msg.data(), msg.size());
            return cloud;
        }
        
    private:
        zmq::context_t context_;
        zmq::socket_t socket_;
    };
};

}  // namespace neuro_nav::interface
```

### 5.3 gRPC 模型推理接口

```protobuf
// proto/model_inference.proto
syntax = "proto3";

package neuro_nav.inference;

service ModelInference {
    // 单次推理
    rpc Infer(InferRequest) returns (InferResponse);
    
    // 流式推理 (用于实时控制)
    rpc StreamInfer(stream InferRequest) returns (stream InferResponse);
}

message InferRequest {
    string model_name = 1;
    bytes observation = 2;        // 序列化的观测数据
    string instruction = 3;       // 语言指令 (VLN用)
    uint64 timestamp_ns = 4;
}

message InferResponse {
    bytes trajectory = 1;         // 序列化的轨迹
    float confidence = 2;
    uint64 inference_time_us = 3;
    string error_message = 4;
}
```

```cpp
// interface/communication/grpc/model_client.hpp
#include <grpcpp/grpcpp.h>
#include "model_inference.grpc.pb.h"

namespace neuro_nav::interface {

class ModelInferenceClient {
public:
    explicit ModelInferenceClient(const std::string& server_address)
        : stub_(ModelInference::NewStub(
              grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials())))
    {}
    
    Result<Trajectory, Error> infer(const Observation& obs, 
                                    const std::string& instruction,
                                    std::chrono::milliseconds timeout) 
    {
        InferRequest request;
        request.set_model_name("intern_vla_n1");
        request.set_observation(serialize(obs));
        request.set_instruction(instruction);
        request.set_timestamp_ns(now_ns());
        
        InferResponse response;
        grpc::ClientContext context;
        context.set_deadline(std::chrono::system_clock::now() + timeout);
        
        grpc::Status status = stub_->Infer(&context, request, &response);
        
        if (!status.ok()) {
            return Result<Trajectory, Error>::err(
                Error::make(ErrorCode::TIMEOUT, 
                            "Model inference failed: " + status.error_message()));
        }
        
        if (!response.error_message().empty()) {
            return Result<Trajectory, Error>::err(
                Error::make(ErrorCode::INFERENCE_ERROR, response.error_message()));
        }
        
        return Result<Trajectory, Error>::ok(deserialize<Trajectory>(response.trajectory()));
    }
    
    // 流式推理 (用于实时控制)
    class StreamSession {
    public:
        void send(const Observation& obs) {
            InferRequest request;
            // ... 填充 request
            stream_->Write(request);
        }
        
        std::optional<Trajectory> receive(std::chrono::milliseconds timeout) {
            InferResponse response;
            if (stream_->Read(&response)) {
                return deserialize<Trajectory>(response.trajectory());
            }
            return std::nullopt;
        }
        
    private:
        std::unique_ptr<grpc::ClientReaderWriter<InferRequest, InferResponse>> stream_;
    };
    
private:
    std::unique_ptr<ModelInference::Stub> stub_;
};

}  // namespace neuro_nav::interface
```

---

## 6. 内存泄漏防护

### 6.1 资源管理器

```cpp
// common/memory/resource_manager.hpp
namespace neuro_nav::memory {

// 资源管理器 - 跟踪所有动态资源
class ResourceManager {
public:
    static ResourceManager& instance() {
        static ResourceManager mgr;
        return mgr;
    }
    
    // 注册资源
    template<typename T>
    void register_resource(const std::string& name, std::shared_ptr<T> resource) {
        std::lock_guard<std::mutex> lock(mutex_);
        resources_[name] = resource;
        LOG_DEBUG("Resource registered: {}", name);
    }
    
    // 释放资源
    void release(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = resources_.find(name);
        if (it != resources_.end()) {
            resources_.erase(it);
            LOG_DEBUG("Resource released: {}", name);
        }
    }
    
    // 释放所有资源
    void release_all() {
        std::lock_guard<std::mutex> lock(mutex_);
        resources_.clear();
        LOG_INFO("All resources released");
    }
    
    // 诊断: 打印当前资源状态
    void dump_resources() const {
        std::lock_guard<std::mutex> lock(mutex_);
        LOG_INFO("=== Resource Manager Status ===");
        for (const auto& [name, ptr] : resources_) {
            LOG_INFO("  {}: use_count={}", name, ptr.use_count());
        }
    }
    
private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::shared_ptr<void>> resources_;
};

// RAII 资源注册
template<typename T>
class ScopedResource {
public:
    ScopedResource(const std::string& name, std::shared_ptr<T> resource)
        : name_(name) 
    {
        ResourceManager::instance().register_resource(name, resource);
    }
    
    ~ScopedResource() {
        ResourceManager::instance().release(name_);
    }
    
private:
    std::string name_;
};

}  // namespace neuro_nav::memory
```

### 6.2 内存使用监控

```cpp
// common/memory/memory_monitor.hpp
namespace neuro_nav::memory {

class MemoryMonitor {
public:
    struct Stats {
        size_t rss_bytes;           // 物理内存
        size_t virt_bytes;          // 虚拟内存
        size_t shared_bytes;        // 共享内存
        double memory_percent;      // 内存占比
    };
    
    static Stats get_process_memory() {
        Stats stats{};
        
        // 读取 /proc/self/statm
        std::ifstream statm("/proc/self/statm");
        if (statm) {
            size_t virt, rss, shared;
            statm >> virt >> rss >> shared;
            size_t page_size = sysconf(_SC_PAGESIZE);
            stats.virt_bytes = virt * page_size;
            stats.rss_bytes = rss * page_size;
            stats.shared_bytes = shared * page_size;
        }
        
        // 计算百分比
        size_t total_mem = sysconf(_SC_PHYS_PAGES) * sysconf(_SC_PAGESIZE);
        stats.memory_percent = 100.0 * stats.rss_bytes / total_mem;
        
        return stats;
    }
    
    // 周期性检查，超阈值告警
    void start_monitoring(double threshold_percent = 80.0, 
                          std::chrono::seconds interval = std::chrono::seconds(10)) 
    {
        monitor_thread_ = std::thread([this, threshold_percent, interval] {
            while (!stop_) {
                auto stats = get_process_memory();
                
                if (stats.memory_percent > threshold_percent) {
                    LOG_WARN("Memory usage high: {:.1f}% ({:.1f}MB RSS)",
                             stats.memory_percent, 
                             stats.rss_bytes / 1024.0 / 1024.0);
                    
                    // 触发诊断
                    ResourceManager::instance().dump_resources();
                }
                
                std::this_thread::sleep_for(interval);
            }
        });
    }
    
    void stop_monitoring() {
        stop_ = true;
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
    }
    
private:
    std::thread monitor_thread_;
    std::atomic<bool> stop_{false};
};

}  // namespace neuro_nav::memory
```

---

## 7. 数据阻塞防护

### 7.1 背压处理策略

```cpp
// common/concurrency/backpressure.hpp
namespace neuro_nav::concurrency {

// 背压策略
enum class BackpressurePolicy {
    BLOCK,           // 阻塞等待
    DROP_OLDEST,     // 丢弃最旧
    DROP_NEWEST,     // 丢弃最新
    SAMPLE,          // 采样 (每N个保留1个)
};

// 带背压处理的队列
template<typename T, size_t Capacity>
class BackpressureQueue {
public:
    explicit BackpressureQueue(BackpressurePolicy policy = BackpressurePolicy::DROP_OLDEST)
        : policy_(policy) {}
    
    bool push(T item) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (queue_.size() >= Capacity) {
            switch (policy_) {
                case BackpressurePolicy::BLOCK:
                    cv_space_.wait(lock, [this] { return queue_.size() < Capacity; });
                    break;
                    
                case BackpressurePolicy::DROP_OLDEST:
                    queue_.pop();
                    dropped_count_++;
                    LOG_WARN_THROTTLE(1.0, "Queue full, dropping oldest. Total dropped: {}", 
                                      dropped_count_.load());
                    break;
                    
                case BackpressurePolicy::DROP_NEWEST:
                    dropped_count_++;
                    return false;  // 丢弃当前
                    
                case BackpressurePolicy::SAMPLE:
                    if (++sample_counter_ % sample_rate_ != 0) {
                        return false;  // 采样丢弃
                    }
                    queue_.pop();
                    break;
            }
        }
        
        queue_.push(std::move(item));
        cv_data_.notify_one();
        return true;
    }
    
    std::optional<T> pop(std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (timeout.count() > 0) {
            if (!cv_data_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) {
                return std::nullopt;
            }
        } else if (queue_.empty()) {
            return std::nullopt;
        }
        
        T item = std::move(queue_.front());
        queue_.pop();
        cv_space_.notify_one();
        return item;
    }
    
    size_t dropped_count() const { return dropped_count_; }
    
private:
    BackpressurePolicy policy_;
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_data_;
    std::condition_variable cv_space_;
    std::atomic<size_t> dropped_count_{0};
    size_t sample_counter_ = 0;
    size_t sample_rate_ = 2;
};

}  // namespace neuro_nav::concurrency
```

### 7.2 超时保护

```cpp
// common/concurrency/timeout_guard.hpp
namespace neuro_nav::concurrency {

// 超时执行器
template<typename Func, typename... Args>
auto execute_with_timeout(std::chrono::milliseconds timeout, Func&& func, Args&&... args)
    -> std::optional<std::invoke_result_t<Func, Args...>>
{
    using ReturnType = std::invoke_result_t<Func, Args...>;
    
    std::promise<ReturnType> promise;
    auto future = promise.get_future();
    
    std::thread worker([&promise, &func, &args...] {
        try {
            if constexpr (std::is_void_v<ReturnType>) {
                std::invoke(std::forward<Func>(func), std::forward<Args>(args)...);
                promise.set_value();
            } else {
                promise.set_value(
                    std::invoke(std::forward<Func>(func), std::forward<Args>(args)...));
            }
        } catch (...) {
            promise.set_exception(std::current_exception());
        }
    });
    
    if (future.wait_for(timeout) == std::future_status::timeout) {
        worker.detach();  // 注意: 超时后线程继续运行
        LOG_WARN("Execution timeout after {}ms", timeout.count());
        return std::nullopt;
    }
    
    worker.join();
    return future.get();
}

// 使用示例
auto result = execute_with_timeout(
    std::chrono::milliseconds(100),
    [](const PointCloud& cloud) { return process_cloud(cloud); },
    point_cloud
);

if (!result) {
    LOG_ERROR("Point cloud processing timeout");
    // 降级处理
}

}  // namespace neuro_nav::concurrency
```

---

## 8. 诊断与监控

### 8.1 数据流健康检查

```cpp
// common/diagnostics/data_flow_monitor.hpp
namespace neuro_nav::diagnostics {

// 数据流健康状态
struct DataFlowHealth {
    std::string name;
    double expected_rate_hz;
    double actual_rate_hz;
    double max_latency_ms;
    double avg_latency_ms;
    size_t dropped_count;
    bool is_healthy;
};

class DataFlowMonitor {
public:
    void register_flow(const std::string& name, double expected_rate_hz) {
        std::lock_guard<std::mutex> lock(mutex_);
        flows_[name] = FlowStats{expected_rate_hz};
    }
    
    void record_message(const std::string& name, double latency_ms = 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& stats = flows_[name];
        stats.message_count++;
        stats.total_latency_ms += latency_ms;
        stats.max_latency_ms = std::max(stats.max_latency_ms, latency_ms);
    }
    
    void record_drop(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        flows_[name].dropped_count++;
    }
    
    std::vector<DataFlowHealth> get_health_report() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<DataFlowHealth> report;
        
        auto now = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(now - last_report_time_).count();
        
        for (auto& [name, stats] : flows_) {
            double actual_rate = stats.message_count / elapsed_sec;
            double avg_latency = stats.message_count > 0 
                                ? stats.total_latency_ms / stats.message_count : 0;
            
            bool is_healthy = actual_rate >= stats.expected_rate_hz * 0.9  // 允许10%波动
                           && stats.dropped_count == 0;
            
            report.push_back({
                .name = name,
                .expected_rate_hz = stats.expected_rate_hz,
                .actual_rate_hz = actual_rate,
                .max_latency_ms = stats.max_latency_ms,
                .avg_latency_ms = avg_latency,
                .dropped_count = stats.dropped_count,
                .is_healthy = is_healthy
            });
            
            // 重置计数
            stats.message_count = 0;
            stats.total_latency_ms = 0;
            stats.max_latency_ms = 0;
            stats.dropped_count = 0;
        }
        
        last_report_time_ = now;
        return report;
    }
    
private:
    struct FlowStats {
        double expected_rate_hz = 0;
        size_t message_count = 0;
        double total_latency_ms = 0;
        double max_latency_ms = 0;
        size_t dropped_count = 0;
    };
    
    mutable std::mutex mutex_;
    std::unordered_map<std::string, FlowStats> flows_;
    std::chrono::steady_clock::time_point last_report_time_ = 
        std::chrono::steady_clock::now();
};

}  // namespace neuro_nav::diagnostics
```

### 8.2 系统诊断发布

```cpp
// common/diagnostics/system_diagnostics.hpp
namespace neuro_nav::diagnostics {

class SystemDiagnostics {
public:
    void publish_diagnostics() {
        // 收集各项指标
        auto memory = MemoryMonitor::get_process_memory();
        auto data_flow = data_flow_monitor_.get_health_report();
        auto latencies = collect_latency_stats();
        
        // 发布到ROS2 diagnostics
        diagnostic_msgs::msg::DiagnosticArray msg;
        msg.header.stamp = now();
        
        // 内存状态
        auto& mem_status = msg.status.emplace_back();
        mem_status.name = "neuro_nav/memory";
        mem_status.level = memory.memory_percent > 90 
                          ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                          : diagnostic_msgs::msg::DiagnosticStatus::OK;
        add_kv(mem_status, "rss_mb", memory.rss_bytes / 1024.0 / 1024.0);
        add_kv(mem_status, "percent", memory.memory_percent);
        
        // 数据流状态
        for (const auto& flow : data_flow) {
            auto& status = msg.status.emplace_back();
            status.name = "neuro_nav/data_flow/" + flow.name;
            status.level = flow.is_healthy 
                          ? diagnostic_msgs::msg::DiagnosticStatus::OK
                          : diagnostic_msgs::msg::DiagnosticStatus::WARN;
            add_kv(status, "expected_hz", flow.expected_rate_hz);
            add_kv(status, "actual_hz", flow.actual_rate_hz);
            add_kv(status, "max_latency_ms", flow.max_latency_ms);
            add_kv(status, "dropped", flow.dropped_count);
        }
        
        diagnostics_pub_->publish(msg);
    }
    
private:
    DataFlowMonitor data_flow_monitor_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace neuro_nav::diagnostics
```

---

## 9. 关键设计检查清单

### 实时性
- [ ] 关键路径延迟预算已分配
- [ ] 高优先级任务绑定CPU核心
- [ ] 使用无锁数据结构避免阻塞
- [ ] 大数据使用零拷贝/共享内存

### 安全性
- [ ] 输入数据边界检查
- [ ] 超时保护
- [ ] 异常状态下的安全停止

### 可靠性
- [ ] 关键数据有QoS保证
- [ ] 模块故障隔离
- [ ] 降级策略已定义

### 资源管理
- [ ] 无裸指针/手动内存管理
- [ ] 队列有容量限制
- [ ] 背压策略已配置
- [ ] 内存使用监控

### 可观测性
- [ ] 延迟监控
- [ ] 数据流健康检查
- [ ] 资源使用统计
- [ ] 诊断信息发布

---

## 10. 配置示例

```yaml
# config/middleware/middleware_config.yaml
middleware:
  # 线程配置
  threading:
    control_thread:
      priority: 99           # SCHED_FIFO
      cpu_affinity: [0]      # 绑定CPU0
    planner_thread:
      priority: 90
      cpu_affinity: [1]
    perception_thread:
      priority: 80
      cpu_affinity: [2, 3]
      
  # 队列配置
  queues:
    sensor_queue:
      capacity: 10
      backpressure: "drop_oldest"
    command_queue:
      capacity: 5
      backpressure: "block"
      
  # 通信配置
  communication:
    point_cloud:
      protocol: "zmq"
      endpoint: "ipc:///tmp/neuro_nav_cloud"
    model_inference:
      protocol: "grpc"
      endpoint: "localhost:50051"
      timeout_ms: 500
    task_command:
      protocol: "mqtt"
      broker: "tcp://localhost:1883"
      qos: 2
      
  # 监控配置
  monitoring:
    memory_threshold_percent: 80
    latency_deadline_ms:
      perception: 20
      planner: 30
      control: 10
    health_check_interval_sec: 10
```
