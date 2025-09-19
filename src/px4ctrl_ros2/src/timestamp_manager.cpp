#include "../include/timestamp_manager.h"

TimestampManager::TimestampManager(rclcpp::Node* node)
    : node_(node)
{
    if (!node_) {
        throw std::invalid_argument("TimestampManager: node pointer cannot be null");
    }
    
    RCLCPP_INFO(node_->get_logger(), "[TimestampManager] 稳健化时间戳管理器初始化完成");
}

void TimestampManager::updatePX4Timestamp(uint64_t px4_timestamp_us)
{
    const int64_t now_ns = steady_clock_.now().nanoseconds();

    if (isValidPX4Timestamp(px4_timestamp_us)) {
        last_px4_timestamp_us_.store(px4_timestamp_us, std::memory_order_relaxed);
        last_px4_update_steady_ns_.store(static_cast<uint64_t>(now_ns), std::memory_order_relaxed);
        px4_clock_valid_.store(true, std::memory_order_relaxed);
        px4_valid_count_.fetch_add(1, std::memory_order_relaxed);

        static uint32_t log_counter = 0;
        if (++log_counter % 1000 == 0) {
            RCLCPP_DEBUG(node_->get_logger(),
                        "[TimestampManager] valid=%u invalid=%u fallback=%u",
                        px4_valid_count_.load(), px4_invalid_count_.load(), ros2_fallback_count_.load());
        }
    } else {
        px4_invalid_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), steady_clock_, 5000,
                            "[TimestampManager] invalid PX4 ts: %" PRIu64, px4_timestamp_us);
    }
}

uint64_t TimestampManager::getCurrentTimestampUs()
{
    const uint64_t base_us = last_px4_timestamp_us_.load(std::memory_order_relaxed);
    const uint64_t base_ns = last_px4_update_steady_ns_.load(std::memory_order_relaxed);

    if (base_us != 0) {
        return extrapolatePX4Timestamp(base_us, base_ns);
    }

    // 真·无样本 → ROS steady μs 兜底（不同基准，仅为"非零/递增"）
    ros2_fallback_count_.fetch_add(1, std::memory_order_relaxed);
    uint64_t est = static_cast<uint64_t>(steady_clock_.now().nanoseconds() / 1000);
    uint64_t prev = last_returned_us_.load(std::memory_order_relaxed);
    if (est < prev) est = prev;
    last_returned_us_.store(est, std::memory_order_relaxed);
    return est;
}

rclcpp::Time TimestampManager::getTimeForLogging()
{
    // 仅用于日志展示：RCL_STEADY_TIME，不参与计算
    const uint64_t ts_us = getCurrentTimestampUs();
    return rclcpp::Time(ts_us * 1000ULL, RCL_STEADY_TIME);
}

uint64_t TimestampManager::extrapolatePX4Timestamp(uint64_t base_px4_us, uint64_t base_steady_ns)
{
    const int64_t now_ns = steady_clock_.now().nanoseconds();
    const int64_t dt_ns  = now_ns - static_cast<int64_t>(base_steady_ns);
    const uint64_t dt_us = static_cast<uint64_t>(dt_ns > 0 ? (dt_ns / 1000) : 0);

    uint64_t est = base_px4_us + dt_us;

    // 超时：仅影响"有效性标志"，不要切到 ROS 基准
    if (dt_ns > static_cast<int64_t>(PX4_TIMEOUT_SEC * 1e9)) {
        px4_clock_valid_.store(false, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), steady_clock_, 2000,
                            "[TimestampManager] PX4 clock timeout (%.3fs)",
                            dt_ns * 1e-9);
        // 可选：夹到 base + timeout_us，避免无限外推
        // est = base_px4_us + static_cast<uint64_t>(PX4_TIMEOUT_SEC * 1e6);
    }

    // 单调保护
    uint64_t prev = last_returned_us_.load(std::memory_order_relaxed);
    if (est < prev) est = prev;
    last_returned_us_.store(est, std::memory_order_relaxed);
    return est;
}

bool TimestampManager::isPX4ClockValid() const
{
    return px4_clock_valid_.load(std::memory_order_relaxed);
}

std::string TimestampManager::getClockStatus() const
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    std::stringstream status;
    status << "TimestampManager状态: ";
    status << "PX4时钟" << (px4_clock_valid_.load() ? "有效" : "无效");
    status << ", 有效次数: " << px4_valid_count_.load();
    status << ", 无效次数: " << px4_invalid_count_.load();
    status << ", ROS2回退次数: " << ros2_fallback_count_.load();
    
    uint64_t last_timestamp = last_px4_timestamp_us_.load();
    if (px4_clock_valid_.load() && last_timestamp > 0) {
        status << ", 最后PX4时间戳: " << last_timestamp << "us";
    }
    
    return status.str();
}

bool TimestampManager::isValidPX4Timestamp(uint64_t px4_timestamp_us)
{
    // 检查时间戳是否在合理范围内
    if (px4_timestamp_us < MIN_PX4_TIMESTAMP) {
        return false;
    }
    
    uint64_t last_timestamp = last_px4_timestamp_us_.load(std::memory_order_relaxed);
    
    // 如果这是第一个时间戳，直接认为有效
    if (last_timestamp == 0) {
        return true;
    }
    
    // 检查时间戳是否有异常跳跃 - 使用有符号差值避免下溢
    int64_t timestamp_diff = static_cast<int64_t>(px4_timestamp_us) - static_cast<int64_t>(last_timestamp);
    uint64_t abs_diff = static_cast<uint64_t>(timestamp_diff > 0 ? timestamp_diff : -timestamp_diff);
    
    if (abs_diff > MAX_TIMESTAMP_JUMP_US) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), steady_clock_, 2000,
                            "[TimestampManager] PX4时间戳跳跃过大: %" PRIu64 " us (当前: %" PRIu64 ", 上次: %" PRIu64 ")", 
                            abs_diff, px4_timestamp_us, last_timestamp);
        return false;
    }
    
    return true;
}