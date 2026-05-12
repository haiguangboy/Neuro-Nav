# Neuro-Nav 代码实现指南

> 本文档面向使用 Claude Code、Codex、Cursor 等 AI 编码工具实现 Neuro-Nav 框架的开发者。
> 目标：确保生成的代码**可靠、实用、简洁**，减少歧义和反复修改。

---

## 1. 核心原则

### 1.1 代码质量优先级

```
正确性 > 可读性 > 性能 > 简洁性
```

- **正确性**: 首先确保功能正确，边界条件处理完善
- **可读性**: 代码是写给人看的，其次才是机器
- **性能**: 在热路径上优化，避免过早优化
- **简洁性**: 不过度设计，不写多余代码

### 1.2 必须遵守的约定

| 约定 | 说明 |
|------|------|
| **C++17** | 使用 C++17 标准，充分利用 `std::optional`, `std::variant`, structured bindings |
| **智能指针** | 禁止裸 `new/delete`，使用 `std::unique_ptr` / `std::shared_ptr` |
| **RAII** | 资源通过构造获取，析构释放 |
| **const正确性** | 能 const 的地方必须 const |
| **错误处理** | 使用 `Result<T, E>` 或 `std::expected`，不用异常做流程控制 |

---

## 2. 命名规范

### 2.1 文件命名

```
snake_case.hpp / snake_case.cpp

示例:
nvblox_adapter.hpp
nvblox_adapter.cpp
tricycle_kinematics.hpp
tricycle_kinematics.cpp
```

### 2.2 类型命名

```cpp
// 类/结构体: PascalCase
class NvbloxAdapter;
struct TrajectoryPoint;

// 接口: I前缀 + PascalCase
class ILocalMapper;
class IKinematics;

// 枚举: PascalCase，枚举值全大写
enum class RobotType {
    DIFFERENTIAL,
    TRICYCLE,
    QUADRUPED
};

// 类型别名: PascalCase
using Pose2D = Eigen::Vector3d;
using Trajectory = std::vector<TrajectoryPoint>;
```

### 2.3 变量/函数命名

```cpp
// 成员变量: snake_case + 下划线后缀
class Example {
    double voxel_size_;
    std::string robot_name_;
};

// 局部变量/参数: snake_case
void process(const PointCloud& input_cloud) {
    double min_distance = compute_min_distance(input_cloud);
}

// 函数: snake_case
double compute_distance(const Pose2D& a, const Pose2D& b);
bool is_valid() const;

// 常量: k前缀 + PascalCase 或 全大写
constexpr double kDefaultVoxelSize = 0.05;
constexpr int MAX_ITERATIONS = 100;
```

### 2.4 命名空间

```cpp
// 层次化命名空间
namespace neuro_nav::perception {
    class NvbloxAdapter;
}

namespace neuro_nav::core::planner {
    class TebWrapper;
}

// 禁止 using namespace 在头文件中
// 在 .cpp 中可以使用
using namespace neuro_nav::common;
```

---

## 3. 接口设计规范

### 3.1 接口定义模板

```cpp
// 接口文件: i_xxx.hpp
#pragma once

namespace neuro_nav::xxx {

/**
 * @brief 接口描述 (一句话说明职责)
 * 
 * 详细说明：
 * - 这个接口的设计意图
 * - 实现者需要注意什么
 * - 线程安全性要求
 */
class IExample {
public:
    virtual ~IExample() = default;
    
    // === 核心方法 (必须实现) ===
    
    /**
     * @brief 方法描述
     * @param input 参数说明
     * @return 返回值说明
     * @throws 如果使用异常，说明什么情况会抛出
     * @note 特殊说明 (线程安全性、性能考虑等)
     */
    virtual Result<Output, Error> process(const Input& input) = 0;
    
    // === 配置方法 ===
    virtual void configure(const Config& config) = 0;
    
    // === 查询方法 ===
    virtual bool is_ready() const = 0;
    
protected:
    // 保护成员供子类使用
    IExample() = default;
    
    // 禁止拷贝
    IExample(const IExample&) = delete;
    IExample& operator=(const IExample&) = delete;
};

}  // namespace neuro_nav::xxx
```

### 3.2 实现类模板

```cpp
// xxx_impl.hpp
#pragma once

#include "i_xxx.hpp"

namespace neuro_nav::xxx {

/**
 * @brief IExample 的具体实现
 * 
 * 实现细节说明...
 */
class ExampleImpl final : public IExample {
public:
    // 构造函数：显式，避免隐式转换
    explicit ExampleImpl(const Config& config);
    
    // 析构函数：只在需要时声明
    ~ExampleImpl() override = default;
    
    // === 接口实现 ===
    Result<Output, Error> process(const Input& input) override;
    void configure(const Config& config) override;
    bool is_ready() const override;
    
private:
    // 私有方法: 实现细节
    void internal_helper();
    
    // 私有成员
    Config config_;
    bool initialized_ = false;
};

}  // namespace neuro_nav::xxx
```

---

## 4. 错误处理规范

### 4.1 Result 类型

```cpp
// common/error/result.hpp
#pragma once

#include <variant>
#include <string>

namespace neuro_nav::common {

// 错误类型
struct Error {
    int code;
    std::string message;
    std::string source;  // 发生位置
    
    static Error make(int code, const std::string& msg, 
                      const char* file = __FILE__, int line = __LINE__) {
        return {code, msg, fmt::format("{}:{}", file, line)};
    }
};

// 预定义错误码
namespace ErrorCode {
    constexpr int OK = 0;
    constexpr int INVALID_ARGUMENT = 1;
    constexpr int NOT_INITIALIZED = 2;
    constexpr int TIMEOUT = 3;
    constexpr int IO_ERROR = 4;
    constexpr int NOT_FOUND = 5;
    constexpr int COLLISION = 6;
    constexpr int INFEASIBLE = 7;
}

// Result 类型
template<typename T>
class Result {
public:
    // 成功构造
    static Result ok(T value) { 
        return Result{std::move(value)}; 
    }
    
    // 失败构造
    static Result err(Error error) { 
        return Result{std::move(error)}; 
    }
    
    // 查询
    bool is_ok() const { return std::holds_alternative<T>(data_); }
    bool is_err() const { return !is_ok(); }
    
    // 获取值 (调用前必须检查 is_ok)
    T& value() { return std::get<T>(data_); }
    const T& value() const { return std::get<T>(data_); }
    
    // 获取错误 (调用前必须检查 is_err)
    Error& error() { return std::get<Error>(data_); }
    const Error& error() const { return std::get<Error>(data_); }
    
    // 带默认值获取
    T value_or(T default_value) const {
        return is_ok() ? value() : default_value;
    }
    
private:
    explicit Result(T value) : data_(std::move(value)) {}
    explicit Result(Error error) : data_(std::move(error)) {}
    
    std::variant<T, Error> data_;
};

// 无返回值的Result
using VoidResult = Result<std::monostate>;

}  // namespace neuro_nav::common
```

### 4.2 错误处理模式

```cpp
// ✅ 推荐: 使用 Result
Result<Trajectory, Error> TebWrapper::compute_trajectory(
    const Pose2D& start, const Pose2D& goal) 
{
    if (!initialized_) {
        return Result<Trajectory, Error>::err(
            Error::make(ErrorCode::NOT_INITIALIZED, "TEB not initialized"));
    }
    
    if (!is_valid_pose(start) || !is_valid_pose(goal)) {
        return Result<Trajectory, Error>::err(
            Error::make(ErrorCode::INVALID_ARGUMENT, "Invalid start or goal pose"));
    }
    
    auto traj = teb_planner_->optimize();
    if (traj.empty()) {
        return Result<Trajectory, Error>::err(
            Error::make(ErrorCode::INFEASIBLE, "No feasible trajectory found"));
    }
    
    return Result<Trajectory, Error>::ok(std::move(traj));
}

// 调用方
auto result = planner->compute_trajectory(start, goal);
if (result.is_err()) {
    LOG_ERROR("Planning failed: {}", result.error().message);
    return handle_planning_failure(result.error());
}
auto trajectory = std::move(result.value());
```

### 4.3 禁止的错误处理方式

```cpp
// ❌ 禁止: 返回空指针表示错误
Trajectory* compute();  // 不知道 nullptr 是错误还是无结果

// ❌ 禁止: 返回特殊值表示错误
double compute_distance();  // 返回 -1 表示错误？

// ❌ 禁止: 输出参数返回错误
bool compute(Trajectory& out);  // 失败时 out 是什么状态？

// ❌ 禁止: 用异常做流程控制
try {
    auto traj = compute();
} catch (const NoPathException&) {
    // 这是正常情况，不应该用异常
}
```

---

## 5. 内存管理规范

### 5.1 智能指针使用

```cpp
// ✅ unique_ptr: 独占所有权
class Planner {
    std::unique_ptr<ITebOptimizer> optimizer_;  // 独占
};

// ✅ shared_ptr: 共享所有权 (谨慎使用)
class TrajectoryExecutor {
    std::shared_ptr<IEnvironmentModel> env_;  // 多个模块共享环境模型
};

// ✅ weak_ptr: 观察者，不影响生命周期
class Visualizer {
    std::weak_ptr<IEnvironmentModel> env_;  // 只是观察
};

// ✅ 工厂函数返回 unique_ptr
std::unique_ptr<IRobot> RobotFactory::create(RobotType type);
```

### 5.2 避免内存问题

```cpp
// ❌ 禁止: 裸指针管理资源
class Bad {
    double* data_ = new double[100];  // 谁负责 delete？
};

// ✅ 推荐: RAII
class Good {
    std::vector<double> data_;  // 自动管理
};

// ❌ 禁止: 返回局部变量引用
const std::string& get_name() {
    std::string name = compute_name();
    return name;  // 悬垂引用!
}

// ✅ 推荐: 返回值或引用成员
std::string get_name() {
    return compute_name();  // 返回拷贝
}
const std::string& get_name() const {
    return name_;  // 引用成员变量
}
```

### 5.3 避免循环引用

```cpp
// ❌ 问题: 循环引用导致内存泄漏
class A {
    std::shared_ptr<B> b_;
};
class B {
    std::shared_ptr<A> a_;  // 循环!
};

// ✅ 解决: 一方使用 weak_ptr
class B {
    std::weak_ptr<A> a_;  // 打破循环
};
```

---

## 6. 线程安全规范

### 6.1 线程安全类模板

```cpp
// 线程安全容器
template<typename T>
class ThreadSafeQueue {
public:
    void push(T value) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(std::move(value));
        cv_.notify_one();
    }
    
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return std::nullopt;
        T value = std::move(queue_.front());
        queue_.pop();
        return value;
    }
    
    T wait_and_pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty(); });
        T value = std::move(queue_.front());
        queue_.pop();
        return value;
    }
    
private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
};
```

### 6.2 锁使用规范

```cpp
// ✅ 优先使用 lock_guard (简单锁定)
void simple_update() {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = new_data;
}

// ✅ 需要手动解锁时用 unique_lock
void complex_update() {
    std::unique_lock<std::mutex> lock(mutex_);
    // 处理...
    lock.unlock();  // 提前解锁
    // 后续不需要锁的操作
}

// ✅ 多锁用 scoped_lock 避免死锁
void transfer(Account& from, Account& to, double amount) {
    std::scoped_lock lock(from.mutex_, to.mutex_);  // 同时锁定
    from.balance_ -= amount;
    to.balance_ += amount;
}

// ❌ 禁止: 手动 lock/unlock
mutex_.lock();
// 如果这里抛异常，永远不会 unlock
mutex_.unlock();
```

### 6.3 文档化线程安全性

```cpp
/**
 * @brief 环境模型接口
 * 
 * @thread_safety 所有方法都是线程安全的，可以并发调用
 */
class IEnvironmentModel {
    // ...
};

/**
 * @brief 轨迹优化器
 * 
 * @thread_safety 非线程安全，调用方需要保证同一时间只有一个线程访问
 */
class TrajectoryOptimizer {
    // ...
};
```

---

## 7. 单元测试规范

### 7.1 测试文件组织

```
module/
├── src/
│   └── xxx.cpp
├── include/
│   └── xxx.hpp
└── test/
    ├── xxx_test.cpp          # 单元测试
    ├── xxx_integration_test.cpp  # 集成测试
    └── test_helpers.hpp      # 测试辅助
```

### 7.2 测试命名

```cpp
// 测试类名: {被测类}Test
class TricycleKinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        kinematics_ = std::make_unique<TricycleKinematics>(config_);
    }
    
    std::unique_ptr<TricycleKinematics> kinematics_;
    TricycleConfig config_{.wheelbase = 1.5, .max_steer_angle = 1.57};
};

// 测试用例名: {方法}_{场景}_{期望结果}
TEST_F(TricycleKinematicsTest, ForwardKinematics_ZeroVelocity_ReturnsZeroTwist) {
    auto twist = kinematics_->forward_kinematics(0.0, 0.0);
    EXPECT_DOUBLE_EQ(twist.linear, 0.0);
    EXPECT_DOUBLE_EQ(twist.angular, 0.0);
}

TEST_F(TricycleKinematicsTest, InverseKinematics_ExceedsMaxSteer_ClampsAngle) {
    Twist2D cmd{.linear = 1.0, .angular = 10.0};  // 过大的角速度
    auto result = kinematics_->inverse_kinematics(cmd);
    EXPECT_LE(std::abs(result.steer_angle), config_.max_steer_angle);
}

TEST_F(TricycleKinematicsTest, InverseKinematics_InvalidVelocity_ReturnsError) {
    Twist2D cmd{.linear = -100.0, .angular = 0.0};  // 负速度
    auto result = kinematics_->inverse_kinematics(cmd);
    EXPECT_TRUE(result.is_err());
}
```

### 7.3 测试覆盖要求

```cpp
// 每个公开方法必须测试:
// 1. 正常输入 → 正确输出
// 2. 边界条件 (最大值、最小值、零、空)
// 3. 错误输入 → 正确的错误处理
// 4. 状态变化 (如果有状态)

TEST_F(NvbloxAdapterTest, Integrate_NormalPointCloud_UpdatesTSDF) { ... }
TEST_F(NvbloxAdapterTest, Integrate_EmptyPointCloud_DoesNothing) { ... }
TEST_F(NvbloxAdapterTest, Integrate_InvalidPose_ReturnsError) { ... }
TEST_F(NvbloxAdapterTest, QueryDistance_BeforeIntegration_ReturnsError) { ... }
TEST_F(NvbloxAdapterTest, QueryDistance_AfterIntegration_ReturnsValidDistance) { ... }
```

---

## 8. 代码审查检查清单

### 在提交代码前，确保：

#### 正确性
- [ ] 所有公开接口都有单元测试
- [ ] 边界条件都已处理
- [ ] 错误路径都返回适当的错误
- [ ] 没有未处理的异常
- [ ] 没有资源泄漏

#### 可读性
- [ ] 命名清晰，符合规范
- [ ] 复杂逻辑有注释
- [ ] 公开接口有文档注释
- [ ] 没有魔法数字，都用命名常量

#### 安全性
- [ ] 输入都经过验证
- [ ] 没有缓冲区溢出风险
- [ ] 线程安全性已说明
- [ ] 没有悬垂指针/引用

#### 性能
- [ ] 大对象用引用传递
- [ ] 热路径没有不必要的内存分配
- [ ] 没有明显的 O(n²) 可以优化为 O(n)

---

## 9. 常见反模式（避免）

### 9.1 过度设计

```cpp
// ❌ 过度设计: 只有一个实现的抽象工厂
class AbstractWidgetFactoryFactory {
    virtual AbstractWidgetFactory* createFactory() = 0;
};

// ✅ 简单直接
std::unique_ptr<Widget> create_widget(WidgetType type);
```

### 9.2 God Class

```cpp
// ❌ God Class: 什么都做
class NavigationSystem {
    void plan_path();
    void control_robot();
    void process_sensors();
    void manage_map();
    void handle_communication();
    void log_data();
    // ... 几十个方法
};

// ✅ 分解职责
class PathPlanner { ... };
class RobotController { ... };
class SensorProcessor { ... };
```

### 9.3 复制粘贴编程

```cpp
// ❌ 复制粘贴
double compute_distance_2d(Point2D a, Point2D b) {
    return std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}
double compute_distance_3d(Point3D a, Point3D b) {
    return std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

// ✅ 泛化
template<typename Point>
double compute_distance(const Point& a, const Point& b) {
    return (a - b).norm();
}
```

### 9.4 注释代码

```cpp
// ❌ 禁止: 注释掉的代码
void process() {
    // old_method();  // TODO: remove after testing
    // another_old_method();
    new_method();
}

// ✅ 删除不用的代码，Git会记住历史
void process() {
    new_method();
}
```

---

## 10. AI 编码工具特别提示

### 10.1 给 Claude Code / Codex 的 Prompt 模板

```
请为 Neuro-Nav 框架实现 [模块名]。

要求:
1. 遵循 neuro_nav_coding_guide.md 中的规范
2. 使用 C++17
3. 使用 Result<T, Error> 处理错误，不使用异常
4. 使用智能指针，禁止裸 new/delete
5. 提供完整的单元测试
6. 类的公开方法必须有 doxygen 注释

接口定义见: [接口文件路径]
配置格式见: [配置文件路径]

请先确认理解，然后再开始实现。
```

### 10.2 分步实现策略

```
1. 先让 AI 生成接口定义，人工审核
2. 再让 AI 生成实现，强调错误处理
3. 再让 AI 生成单元测试
4. 最后让 AI 生成集成测试

不要一次性要求完整实现！
```

### 10.3 常见 AI 生成代码问题

| 问题 | 解决方案 |
|------|---------|
| 错误处理不完整 | 明确要求列出所有错误情况 |
| 缺少边界检查 | 明确要求处理空输入、最大值等 |
| 过度使用 shared_ptr | 明确要求优先 unique_ptr |
| 缺少线程安全说明 | 明确要求文档化线程安全性 |
| 测试用例太简单 | 明确要求测试边界和错误情况 |
| 命名不规范 | 提供命名规范示例 |

---

## 11. 参考资料

- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [ROS2 Developer Guide](https://docs.ros.org/en/humble/Contributing/Developer-Guide.html)
- [Effective Modern C++](https://www.oreilly.com/library/view/effective-modern-c/9781491908419/)
