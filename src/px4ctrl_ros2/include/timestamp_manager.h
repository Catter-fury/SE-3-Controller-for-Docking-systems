#ifndef TIMESTAMP_MANAGER_H
#define TIMESTAMP_MANAGER_H

#include <atomic>
#include <inttypes.h>      // for PRIu64
#include <mutex>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 稳健化时间戳管理器
 * 
 * 核心功能：
 * 1. 统一基准：对外统一返回 PX4 boot 基准的微秒时间戳
 * 2. 稳健外推：线性外推保证每帧输出新鲜、单调的 PX4 μs
 * 3. 线程安全：全原子变量设计，支持多线程访问
 * 4. 超时处理：超时只影响有效标志，不切换时钟基准
 * 5. 单调保护：确保输出时间戳永不回退
 */
class TimestampManager
{
public:
    explicit TimestampManager(rclcpp::Node* node);

    /**
     * @brief 更新PX4时间戳样本（在任一PX4高频话题回调里调用）
     * @param px4_timestamp_us PX4 boot-time 微秒时间戳
     */
    void updatePX4Timestamp(uint64_t px4_timestamp_us);

    /**
     * @brief 主函数：返回"当前 PX4 μs"（外推/兜底）
     * @return 当前时间戳（微秒）
     *   - 有PX4样本：外推的PX4时间戳（PX4基准） 
     *   - 无PX4样本：ROS steady time转微秒（ROS基准，仅兜底用）
     */
    uint64_t getCurrentTimestampUs();

    /**
     * @brief 仅用于日志显示，不参与计算
     * @return rclcpp::Time 对象，仅供日志打印
     * @warning 不要用于时间差计算
     */
    rclcpp::Time getTimeForLogging();

    /**
     * @brief 检查PX4时钟有效性
     */
    bool isPX4ClockValid() const;

    /**
     * @brief 获取调试状态信息
     */
    std::string getClockStatus() const;

    // 公开持久时钟供THROTTLE宏使用
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME}; ///< 持久 throttle 时钟

private:
    // 时间参数
    static constexpr uint64_t MIN_PX4_TIMESTAMP = 1000000ULL;        // 1s
    static constexpr uint64_t MAX_TIMESTAMP_JUMP_US = 50000000ULL;   // 50s (放宽跳跃限制)
    static constexpr double   PX4_TIMEOUT_SEC = 1.0;                // 1000ms (放宽超时)

    /**
     * @brief 线性外推当前PX4时间戳
     * @param base_px4_us 基准PX4时间戳
     * @param base_steady_ns 基准steady时间
     * @return 外推的当前PX4时间戳
     */
    uint64_t extrapolatePX4Timestamp(uint64_t base_px4_us, uint64_t base_steady_ns);
    
    /**
     * @brief 检查PX4时间戳有效性
     * @param px4_timestamp_us PX4时间戳（微秒）
     * @return true如果时间戳有效
     */
    bool isValidPX4Timestamp(uint64_t px4_timestamp_us);

    // 成员变量
    rclcpp::Node* node_{nullptr};

    // 样本（最近一次 PX4 μs 以及采样时的 steady ns）
    std::atomic<uint64_t> last_px4_timestamp_us_{0};
    std::atomic<uint64_t> last_px4_update_steady_ns_{0};

    // 状态/统计
    std::atomic<bool>     px4_clock_valid_{false};
    std::atomic<uint32_t> px4_valid_count_{0};
    std::atomic<uint32_t> px4_invalid_count_{0};
    std::atomic<uint32_t> ros2_fallback_count_{0};

    // 单调保护：最后一次返回给外部的 μs
    std::atomic<uint64_t> last_returned_us_{0};

    mutable std::mutex status_mutex_;
};

#endif // TIMESTAMP_MANAGER_H