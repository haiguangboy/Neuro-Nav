# Neuro-Nav 统一日志系统设计

## 1. 问题背景

当前系统存在多种日志来源：
- **ROS2 日志**: `rclcpp::get_logger()` / `RCLCPP_INFO()` 等
- **C++ 原生**: `std::cout` / `printf` / 自定义
- **Python**: `logging` 模块
- **目标**: spdlog 作为高性能后端

### 各日志系统特点对比

| 特性 | ROS2 rcl_logging | spdlog | Python logging |
|------|------------------|--------|----------------|
| 性能 | 中等 | 极高(异步) | 较低 |
| 异步支持 | 有限 | 原生支持 | 需配置 |
| 多Sink | 支持 | 原生支持 | 支持 |
| 格式化 | fmt-like | fmt原生 | %-style/{}  |
| 跨语言 | 仅C++/Python | 仅C++ | 仅Python |
| ROS集成 | 原生 | 需适配 | 需适配 |

---

## 2. 统一日志架构设计

### 2.1 分层架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         应用层 (Application)                             │
│                                                                          │
│   C++ Code              Python Code             ROS2 Nodes              │
│   LOG_INFO(...)         logger.info(...)        RCLCPP_INFO(...)        │
│       │                      │                       │                   │
└───────┼──────────────────────┼───────────────────────┼───────────────────┘
        │                      │                       │
        ▼                      ▼                       ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      统一日志接口层 (Unified Interface)                   │
│                                                                          │
│   ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐        │
│   │  C++ Logger     │  │  Python Logger  │  │  ROS2 Bridge    │        │
│   │  Wrapper        │  │  Adapter        │  │  Adapter        │        │
│   └────────┬────────┘  └────────┬────────┘  └────────┬────────┘        │
│            │                    │                    │                   │
│            └────────────────────┼────────────────────┘                   │
│                                 │                                        │
│                                 ▼                                        │
│                    ┌────────────────────────┐                           │
│                    │   ILoggerBackend       │  ← 抽象接口               │
│                    │   (Abstract Interface) │                           │
│                    └────────────┬───────────┘                           │
│                                 │                                        │
└─────────────────────────────────┼────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         后端实现层 (Backend)                             │
│                                                                          │
│   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐   │
│   │  spdlog     │  │  ROS2 rcl   │  │  File       │  │  Network    │   │
│   │  Backend    │  │  Backend    │  │  Backend    │  │  Backend    │   │
│   │  (推荐)     │  │  (兼容)     │  │  (备用)     │  │  (远程)     │   │
│   └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 核心接口设计

```cpp
// common/logging/i_logger_backend.hpp
namespace neuro_nav::logging {

// 日志级别 (与spdlog兼容)
enum class LogLevel {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    CRITICAL = 5,
    OFF = 6
};

// 日志元数据
struct LogMetadata {
    std::string module;           // 模块名
    std::string function;         // 函数名
    std::string file;             // 文件名
    int line;                     // 行号
    std::thread::id thread_id;    // 线程ID
    std::chrono::system_clock::time_point timestamp;
};

// 抽象日志后端接口
class ILoggerBackend {
public:
    virtual ~ILoggerBackend() = default;
    
    // 核心日志方法
    virtual void log(LogLevel level, 
                     const LogMetadata& meta,
                     const std::string& message) = 0;
    
    // 配置
    virtual void set_level(LogLevel level) = 0;
    virtual void set_pattern(const std::string& pattern) = 0;
    virtual void flush() = 0;
    
    // Sink管理
    virtual void add_file_sink(const std::string& path) = 0;
    virtual void add_console_sink() = 0;
};

}  // namespace neuro_nav::logging
```

### 2.3 spdlog 后端实现

```cpp
// common/logging/spdlog_backend.hpp
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/async.h>

namespace neuro_nav::logging {

class SpdlogBackend : public ILoggerBackend {
public:
    SpdlogBackend(const std::string& logger_name = "neuro_nav") {
        // 创建异步logger (高性能)
        spdlog::init_thread_pool(8192, 1);  // 队列大小, 线程数
        
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        
        logger_ = std::make_shared<spdlog::async_logger>(
            logger_name,
            console_sink,
            spdlog::thread_pool(),
            spdlog::async_overflow_policy::block
        );
        
        spdlog::register_logger(logger_);
    }
    
    void log(LogLevel level, const LogMetadata& meta, 
             const std::string& message) override {
        // 格式: [时间] [级别] [模块.函数] message
        logger_->log(
            to_spdlog_level(level),
            "[{}.{}:{}] {}",
            meta.module, meta.function, meta.line, message
        );
    }
    
    void add_file_sink(const std::string& path) override {
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            path, 
            10 * 1024 * 1024,  // 10MB per file
            5                   // 保留5个文件
        );
        logger_->sinks().push_back(file_sink);
    }
    
private:
    std::shared_ptr<spdlog::async_logger> logger_;
    
    static spdlog::level::level_enum to_spdlog_level(LogLevel level) {
        static const spdlog::level::level_enum mapping[] = {
            spdlog::level::trace,
            spdlog::level::debug,
            spdlog::level::info,
            spdlog::level::warn,
            spdlog::level::err,
            spdlog::level::critical,
            spdlog::level::off
        };
        return mapping[static_cast<int>(level)];
    }
};

}  // namespace neuro_nav::logging
```

---

## 3. 统一日志宏设计

### 3.1 C++ 宏定义

```cpp
// common/logging/log_macros.hpp
#pragma once

#include "logger_manager.hpp"

// 获取当前模块名 (编译时确定)
#ifndef NEURO_NAV_MODULE
#define NEURO_NAV_MODULE "unknown"
#endif

// 核心日志宏
#define NEURO_LOG(level, ...) \
    ::neuro_nav::logging::LoggerManager::instance().log( \
        level, \
        {NEURO_NAV_MODULE, __FUNCTION__, __FILE__, __LINE__, \
         std::this_thread::get_id(), std::chrono::system_clock::now()}, \
        fmt::format(__VA_ARGS__))

// 便捷宏
#define LOG_TRACE(...)    NEURO_LOG(::neuro_nav::logging::LogLevel::TRACE, __VA_ARGS__)
#define LOG_DEBUG(...)    NEURO_LOG(::neuro_nav::logging::LogLevel::DEBUG, __VA_ARGS__)
#define LOG_INFO(...)     NEURO_LOG(::neuro_nav::logging::LogLevel::INFO, __VA_ARGS__)
#define LOG_WARN(...)     NEURO_LOG(::neuro_nav::logging::LogLevel::WARN, __VA_ARGS__)
#define LOG_ERROR(...)    NEURO_LOG(::neuro_nav::logging::LogLevel::ERROR, __VA_ARGS__)
#define LOG_CRITICAL(...) NEURO_LOG(::neuro_nav::logging::LogLevel::CRITICAL, __VA_ARGS__)

// 条件日志 (避免不必要的字符串构造)
#define LOG_INFO_IF(cond, ...) \
    do { if (cond) LOG_INFO(__VA_ARGS__); } while(0)

// 频率限制日志 (每N秒最多一次)
#define LOG_INFO_THROTTLE(period_sec, ...) \
    do { \
        static auto last_log = std::chrono::steady_clock::time_point::min(); \
        auto now = std::chrono::steady_clock::now(); \
        if (now - last_log >= std::chrono::seconds(period_sec)) { \
            last_log = now; \
            LOG_INFO(__VA_ARGS__); \
        } \
    } while(0)
```

### 3.2 模块名定义方式

```cpp
// 在每个模块的 CMakeLists.txt 中定义
target_compile_definitions(perception_module PRIVATE
    NEURO_NAV_MODULE="perception"
)

target_compile_definitions(planner_module PRIVATE
    NEURO_NAV_MODULE="planner"
)

// 或在源文件开头定义
#define NEURO_NAV_MODULE "perception.nvblox"
#include "common/logging/log_macros.hpp"
```

---

## 4. ROS2 日志桥接

### 4.1 ROS2 → spdlog 桥接

```cpp
// common/logging/ros2_bridge.hpp
#include <rclcpp/logging.hpp>

namespace neuro_nav::logging {

// ROS2日志桥接器 - 将ROS2日志转发到统一后端
class Ros2LogBridge {
public:
    static void install() {
        // 设置ROS2日志输出处理器
        rcl_logging_ret_t ret = rcl_logging_configure_with_output_handler(
            &rcl_get_default_allocator(),
            ros2_to_spdlog_handler,
            nullptr
        );
    }
    
private:
    static void ros2_to_spdlog_handler(
        const rcutils_log_location_t* location,
        int severity,
        const char* name,
        rcutils_time_point_value_t timestamp,
        const char* format,
        va_list* args)
    {
        // 格式化消息
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, *args);
        
        // 转发到统一日志
        LogMetadata meta{
            .module = name ? name : "ros2",
            .function = location ? location->function_name : "",
            .file = location ? location->file_name : "",
            .line = location ? static_cast<int>(location->line_number) : 0,
            .thread_id = std::this_thread::get_id(),
            .timestamp = std::chrono::system_clock::now()
        };
        
        LoggerManager::instance().log(
            ros2_to_log_level(severity),
            meta,
            buffer
        );
    }
    
    static LogLevel ros2_to_log_level(int severity) {
        switch (severity) {
            case RCUTILS_LOG_SEVERITY_DEBUG: return LogLevel::DEBUG;
            case RCUTILS_LOG_SEVERITY_INFO:  return LogLevel::INFO;
            case RCUTILS_LOG_SEVERITY_WARN:  return LogLevel::WARN;
            case RCUTILS_LOG_SEVERITY_ERROR: return LogLevel::ERROR;
            case RCUTILS_LOG_SEVERITY_FATAL: return LogLevel::CRITICAL;
            default: return LogLevel::INFO;
        }
    }
};

}  // namespace neuro_nav::logging
```

### 4.2 保持ROS2宏兼容

```cpp
// 可选: 重定义ROS2宏指向统一日志
// common/logging/ros2_compat.hpp

// 如果希望RCLCPP_INFO等也走spdlog
#ifdef NEURO_NAV_REDIRECT_ROS2_LOG

#undef RCLCPP_INFO
#define RCLCPP_INFO(logger, ...) LOG_INFO(__VA_ARGS__)

#undef RCLCPP_WARN
#define RCLCPP_WARN(logger, ...) LOG_WARN(__VA_ARGS__)

#undef RCLCPP_ERROR
#define RCLCPP_ERROR(logger, ...) LOG_ERROR(__VA_ARGS__)

#endif
```

---

## 5. Python 日志适配

### 5.1 Python Logger Wrapper

```python
# common/logging/python/neuro_logger.py
import logging
import sys
from datetime import datetime
from typing import Optional
import threading

class NeuroNavLogFormatter(logging.Formatter):
    """统一日志格式化器"""
    
    def format(self, record: logging.LogRecord) -> str:
        # 格式: [时间] [级别] [模块.函数:行号] 消息
        timestamp = datetime.fromtimestamp(record.created).strftime(
            '%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        return (f"[{timestamp}] [{record.levelname:<5}] "
                f"[{record.module}.{record.funcName}:{record.lineno}] "
                f"{record.getMessage()}")


class NeuroNavLogger:
    """Neuro-Nav统一Python日志器"""
    
    _instances: dict = {}
    _lock = threading.Lock()
    
    def __init__(self, module_name: str, log_dir: Optional[str] = None):
        self.logger = logging.getLogger(f"neuro_nav.{module_name}")
        self.logger.setLevel(logging.DEBUG)
        
        # 避免重复添加handler
        if not self.logger.handlers:
            formatter = NeuroNavLogFormatter()
            
            # 控制台输出
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)
            
            # 文件输出
            if log_dir:
                from logging.handlers import RotatingFileHandler
                file_handler = RotatingFileHandler(
                    f"{log_dir}/neuro_nav_{module_name}.log",
                    maxBytes=10*1024*1024,  # 10MB
                    backupCount=5
                )
                file_handler.setFormatter(formatter)
                self.logger.addHandler(file_handler)
    
    @classmethod
    def get_logger(cls, module_name: str, log_dir: Optional[str] = None):
        """获取或创建logger (单例模式)"""
        with cls._lock:
            if module_name not in cls._instances:
                cls._instances[module_name] = cls(module_name, log_dir)
            return cls._instances[module_name].logger


# 便捷函数
def get_logger(module_name: str, log_dir: Optional[str] = None):
    return NeuroNavLogger.get_logger(module_name, log_dir)


# 使用示例
if __name__ == "__main__":
    logger = get_logger("perception", "/tmp/neuro_nav_logs")
    logger.info("This is an info message")
    logger.warning("This is a warning")
```

### 5.2 Python ↔ C++ 日志同步 (可选)

```python
# common/logging/python/cpp_bridge.py
"""通过ZeroMQ将Python日志发送到C++ spdlog后端"""

import zmq
import json
import logging
from datetime import datetime

class CppLogBridgeHandler(logging.Handler):
    """将Python日志转发到C++ spdlog后端"""
    
    def __init__(self, endpoint: str = "ipc:///tmp/neuro_nav_log"):
        super().__init__()
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.connect(endpoint)
    
    def emit(self, record: logging.LogRecord):
        try:
            log_entry = {
                "level": record.levelno,
                "module": record.module,
                "function": record.funcName,
                "file": record.pathname,
                "line": record.lineno,
                "message": record.getMessage(),
                "timestamp": datetime.fromtimestamp(record.created).isoformat()
            }
            self.socket.send_json(log_entry, zmq.NOBLOCK)
        except Exception:
            pass  # 日志不应影响主程序
```

---

## 6. 日志命名规范

### 6.1 模块命名层次

```
neuro_nav                          # 根模块
├── common                         # 公共模块
│   ├── common.math
│   ├── common.geometry
│   └── common.logging
├── perception                     # 感知模块
│   ├── perception.nvblox
│   ├── perception.obstacle
│   └── perception.traversability
├── robot                          # 机器人模块
│   ├── robot.tricycle
│   ├── robot.quadruped
│   └── robot.factory
├── core                           # 核心模块
│   ├── core.planner.global
│   ├── core.planner.local
│   ├── core.trajectory
│   └── core.decision
└── interface                      # 接口模块
    ├── interface.ros2
    ├── interface.mqtt
    └── interface.grpc
```

### 6.2 日志格式规范

```
[时间戳] [级别] [模块.函数:行号] [线程ID] 消息

示例:
[2025-01-15 14:32:15.123] [INFO ] [perception.nvblox.integrate:156] [0x7f8b] Integrated 1024 points
[2025-01-15 14:32:15.125] [WARN ] [core.planner.validate:89] [0x7f8c] Path too close to obstacle: 0.15m
[2025-01-15 14:32:15.130] [ERROR] [interface.grpc.call:45] [0x7f8d] Model inference timeout: 500ms
```

### 6.3 spdlog Pattern配置

```cpp
// 推荐的spdlog格式
logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] [%t] %v");

// 各占位符含义:
// %Y-%m-%d %H:%M:%S.%e  - 时间戳 (毫秒精度)
// %^%l%$                - 带颜色的日志级别
// %n                    - logger名称 (模块名)
// %t                    - 线程ID
// %v                    - 消息内容
```

---

## 7. 日志输出路径配置

### 7.1 配置文件

```yaml
# config/logging/logging_config.yaml
logging:
  # 全局配置
  global:
    level: INFO              # 默认级别
    pattern: "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] [%t] %v"
    async: true              # 异步日志
    async_queue_size: 8192
    
  # 输出路径
  output:
    base_dir: "/var/log/neuro_nav"  # 基础目录
    
    # 文件命名规则: {base_dir}/{date}/{module}_{time}.log
    file_pattern: "{date}/{module}_{hour}.log"
    
    # 轮转策略
    rotation:
      max_size: 10MB         # 单文件最大
      max_files: 10          # 保留文件数
      rotate_on_open: false
      
  # 模块级别配置
  modules:
    perception:
      level: DEBUG           # 感知模块更详细
    core.planner:
      level: INFO
    interface.grpc:
      level: WARN            # 接口层只记录警告以上
      
  # 控制台输出
  console:
    enabled: true
    colored: true
    level: INFO              # 控制台只显示INFO以上
```

### 7.2 Logger初始化

```cpp
// common/logging/logger_manager.hpp
namespace neuro_nav::logging {

class LoggerManager {
public:
    static LoggerManager& instance() {
        static LoggerManager mgr;
        return mgr;
    }
    
    void initialize(const std::string& config_path) {
        YAML::Node config = YAML::LoadFile(config_path);
        
        // 解析配置
        auto logging = config["logging"];
        base_dir_ = logging["output"]["base_dir"].as<std::string>();
        
        // 创建日期目录
        auto date_str = get_date_string();
        std::filesystem::create_directories(base_dir_ + "/" + date_str);
        
        // 初始化后端
        if (logging["global"]["async"].as<bool>(true)) {
            backend_ = std::make_unique<SpdlogBackend>("neuro_nav");
        }
        
        // 添加文件sink
        auto file_path = fmt::format("{}/{}/neuro_nav_{}.log",
                                     base_dir_, date_str, get_hour_string());
        backend_->add_file_sink(file_path);
        
        // 设置级别
        auto level_str = logging["global"]["level"].as<std::string>("INFO");
        backend_->set_level(string_to_level(level_str));
        
        initialized_ = true;
    }
    
    void log(LogLevel level, const LogMetadata& meta, 
             const std::string& message) {
        if (!initialized_) {
            // 未初始化时fallback到stderr
            std::cerr << message << std::endl;
            return;
        }
        backend_->log(level, meta, message);
    }
    
    // 创建模块级logger
    std::shared_ptr<spdlog::logger> get_module_logger(const std::string& module) {
        // 检查是否有模块特定配置
        // ...
    }
    
private:
    LoggerManager() = default;
    bool initialized_ = false;
    std::string base_dir_;
    std::unique_ptr<ILoggerBackend> backend_;
};

}  // namespace neuro_nav::logging
```

---

## 8. 性能考虑

### 8.1 异步日志

```cpp
// spdlog异步模式配置
spdlog::init_thread_pool(8192, 1);  // 队列大小8K, 1个后台线程

// 溢出策略选择
// block: 队列满时阻塞 (保证不丢日志)
// overrun_oldest: 丢弃最旧的 (保证不阻塞)
auto logger = spdlog::create_async<spdlog::sinks::rotating_file_sink_mt>(
    "async_logger",
    "logs/async.log",
    10 * 1024 * 1024,
    3,
    spdlog::async_overflow_policy::block  // 或 overrun_oldest
);
```

### 8.2 避免性能陷阱

```cpp
// ❌ 不好: 每次都构造字符串
LOG_DEBUG("Processing point cloud with {} points", cloud.size());

// ✅ 好: 使用条件日志
LOG_DEBUG_IF(is_debug_enabled(), "Processing point cloud with {} points", cloud.size());

// ✅ 好: 使用级别检查
if (logger->should_log(spdlog::level::debug)) {
    LOG_DEBUG("Expensive computation: {}", expensive_to_string(data));
}

// ✅ 好: 高频数据使用throttle
LOG_INFO_THROTTLE(1.0, "Odometry rate: {} Hz", odom_rate);  // 每秒最多一次
```

---

## 9. 迁移策略

### 阶段1: 引入统一接口 (不影响现有代码)
1. 实现 `ILoggerBackend` 和 `SpdlogBackend`
2. 实现统一宏 `LOG_INFO` 等
3. 新代码使用统一宏

### 阶段2: 桥接ROS2日志
1. 实现 `Ros2LogBridge`
2. 在main函数初始化时安装桥接
3. ROS2日志自动转发到spdlog

### 阶段3: 逐步替换
1. 模块逐个替换 `RCLCPP_INFO` → `LOG_INFO`
2. 使用脚本批量替换: `sed -i 's/RCLCPP_INFO/LOG_INFO/g'`

### 阶段4: Python集成
1. 实现Python logger wrapper
2. 可选: 通过ZMQ桥接到C++

---

## 10. 最终使用示例

```cpp
// perception/nvblox/nvblox_adapter.cpp
#define NEURO_NAV_MODULE "perception.nvblox"
#include "common/logging/log_macros.hpp"

void NvbloxAdapter::integrate_pointcloud(const PointCloud& cloud) {
    LOG_DEBUG("Integrating {} points", cloud.size());
    
    auto start = std::chrono::steady_clock::now();
    
    // ... 处理逻辑 ...
    
    auto elapsed = std::chrono::steady_clock::now() - start;
    LOG_INFO_THROTTLE(5.0, "Integration time: {:.2f}ms", 
                      std::chrono::duration<double, std::milli>(elapsed).count());
    
    if (integration_failed) {
        LOG_ERROR("Integration failed: {}", error_message);
    }
}
```

```python
# perception/obstacle/people_detector.py
from neuro_nav.logging import get_logger

logger = get_logger("perception.obstacle", log_dir="/var/log/neuro_nav")

def detect_people(image):
    logger.debug(f"Processing image: {image.shape}")
    
    # ... 处理逻辑 ...
    
    logger.info(f"Detected {len(people)} people")
    return people
```
