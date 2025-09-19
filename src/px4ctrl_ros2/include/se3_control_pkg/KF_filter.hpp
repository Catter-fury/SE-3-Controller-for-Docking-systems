    #ifndef KF_FILTER_HPP
    #define KF_FILTER_HPP

    #include <Eigen/Dense>
    #include <rclcpp/rclcpp.hpp>
    #include <tf2_msgs/msg/tf_message.hpp>
    #include "input.h"  // 共享参数配置文件
    #include "timestamp_manager.h"

    using namespace Eigen;
    
    // 前向声明
    namespace se3control {
        struct UAVState;
    }

    class KF_Filter : public rclcpp::Node {
    public:
        // 构造函数
        KF_Filter();
        
        // 初始化函数（从ROS参数服务器加载配置）
        void initializeWithROS2Params(rclcpp::Node::SharedPtr node);
        
        // TF 回调函数
        void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
        
        // 预测步骤
        void predict();

        // 使用TF数据进行校正
        void correctWithTF(const TF_Data_t& tf_data);
        
        // 获取当前状态估计
        VectorXd getState() const;
        
        // 获取UAV状态（用于控制器）
        se3control::UAVState getUAVState() const;
        
        // 获取协方差矩阵
        MatrixXd getCovariance() const;
        
        // 设置过程噪声协方差
        void setProcessNoise(const MatrixXd& Q);
        
        // 设置测量噪声协方差
        void setMeasurementNoise(const MatrixXd& R);
        
        // 重置滤波器
        void reset();
        
        // 设置初始状态
        void setInitialState(const VectorXd& x0, const MatrixXd& P0);
        
        // 添加获取最后更新时间的方法
        rclcpp::Time getLastUpdateTime() const {
        return last_update_time_;
    }
    private:
     
        // 参数结构体（与SE3控制器共享）
        Parameter_t param_;
        
        // 状态向量 (6维: 位置3, 速度3)
        VectorXd x_;
        
        // 协方差矩阵
        MatrixXd P_;

        //状态转移矩阵
        MatrixXd A_;
        
        // 过程噪声协方差
        MatrixXd Q_;
        
        // 测量噪声协方差
        MatrixXd R_;
        
        // 时间步长
        double dt_;
        
        // 滤波器状态标志
        bool initialized_;

        // 最后更新时间戳
        rclcpp::Time last_update_time_;
        
        // ROS2 订阅者和数据
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
        TF_Data_t tf_data_;
        std::unique_ptr<TimestampManager> timestamp_manager_;
    };

    #endif // KF_FILTER_HPP
