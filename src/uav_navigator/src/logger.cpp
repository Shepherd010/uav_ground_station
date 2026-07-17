#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <uav_navigator/NavigatorStatus.h>
#include <uav_navigator/ExperimentMetrics.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <cstdio>

namespace uav_navigator {

class Logger {
public:
    Logger(ros::NodeHandle& nh);
    void run();

private:
    ros::NodeHandle nh_;

    // 订阅者
    ros::Subscriber odom_sub_;
    ros::Subscriber mavros_state_sub_;
    ros::Subscriber nav_status_sub_;
    ros::Subscriber safety_alert_sub_;
    ros::Subscriber setpoint_sub_;
    ros::Subscriber metrics_sub_;
    ros::Subscriber config_loaded_sub_;

    // 数据缓存
    nav_msgs::Odometry current_odom_;
    mavros_msgs::State current_mavros_state_;
    uav_navigator::NavigatorStatus current_nav_status_;
    uav_navigator::ExperimentMetrics current_metrics_;
    bool has_odom_;
    bool has_mavros_state_;
    bool has_nav_status_;
    bool has_metrics_;

    // 配置
    struct Config {
        std::string odom_topic;
        std::string mavros_state_topic;
        std::string nav_status_topic;
        std::string safety_alert_topic;
        std::string setpoint_topic;
        std::string metrics_topic;
        std::string config_loaded_topic;
        bool print_odom;
        bool print_mavros_state;
        bool print_nav_status;
        bool print_safety_alert;
        bool print_setpoint;
        bool print_metrics;
        double print_rate;
    } config_;

    // 回调
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    void navStatusCallback(const uav_navigator::NavigatorStatus::ConstPtr& msg);
    void safetyAlertCallback(const std_msgs::String::ConstPtr& msg);
    void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void metricsCallback(const uav_navigator::ExperimentMetrics::ConstPtr& msg);
    void configLoadedCallback(const std_msgs::String::ConstPtr& msg);

    // 打印函数
    void printHeader();
    void printStatusLine();
    void printSummary();
    void printAlert(const std::string& alert);

    // 辅助
    std::string stateToString(uint8_t state);
    std::string stateToDisplay(uint8_t state);   // 格式化显示用
    std::string formatTime();
    void loadConfig();
    void clearScreenSafe();
};

Logger::Logger(ros::NodeHandle& nh) : nh_(nh), has_odom_(false), has_mavros_state_(false), has_nav_status_(false), has_metrics_(false) {
    loadConfig();

    odom_sub_ = nh_.subscribe(config_.odom_topic, 10, &Logger::odomCallback, this);
    mavros_state_sub_ = nh_.subscribe(config_.mavros_state_topic, 10, &Logger::mavrosStateCallback, this);
    nav_status_sub_ = nh_.subscribe(config_.nav_status_topic, 10, &Logger::navStatusCallback, this);
    safety_alert_sub_ = nh_.subscribe(config_.safety_alert_topic, 10, &Logger::safetyAlertCallback, this);
    setpoint_sub_ = nh_.subscribe(config_.setpoint_topic, 10, &Logger::setpointCallback, this);
    metrics_sub_ = nh_.subscribe(config_.metrics_topic, 10, &Logger::metricsCallback, this);
    config_loaded_sub_ = nh_.subscribe(config_.config_loaded_topic, 1, &Logger::configLoadedCallback, this);

    printHeader();
    ROS_INFO("[Logger] Initialization complete, print rate: %.1f Hz", config_.print_rate);
}

void Logger::loadConfig() {
    ros::NodeHandle global_nh;
    global_nh.param<std::string>("logger/odom_topic", config_.odom_topic, "mavros/local_position/odom");
    global_nh.param<std::string>("logger/mavros_state_topic", config_.mavros_state_topic, "mavros/state");
    global_nh.param<std::string>("logger/navigator_status_topic", config_.nav_status_topic, "uav/navigator/status");
    global_nh.param<std::string>("logger/safety_alert_topic", config_.safety_alert_topic, "uav/safety/alert");
    global_nh.param<std::string>("logger/setpoint_topic", config_.setpoint_topic, "mavros/setpoint_position/local");
    global_nh.param<std::string>("logger/metrics_topic", config_.metrics_topic, "uav/experiment/metrics");
    global_nh.param<std::string>("topics/config_loaded_topic", config_.config_loaded_topic, "uav/config/loaded");
    global_nh.param<bool>("logger/print_odom", config_.print_odom, false);
    global_nh.param<bool>("logger/print_mavros_state", config_.print_mavros_state, false);
    global_nh.param<bool>("logger/print_nav_status", config_.print_nav_status, true);
    global_nh.param<bool>("logger/print_safety_alert", config_.print_safety_alert, true);
    global_nh.param<bool>("logger/print_setpoint", config_.print_setpoint, false);
    global_nh.param<bool>("logger/print_metrics", config_.print_metrics, true);
    global_nh.param<double>("logger/print_rate", config_.print_rate, 2.0);
}

void Logger::clearScreenSafe() {
    // 终端友好的清屏（仅在交互式终端有效）
    std::cout << "\033[2J\033[H" << std::flush;
}

void Logger::printHeader() {
    clearScreenSafe();
    std::cout << "\n";
    std::cout << "┌──────────────────────────────────────────────────────────────────────┐\n";
    std::cout << "│              UAV Ground Station —  Flight Monitor                    │\n";
    std::cout << "├──────┬────────┬──────────────────────────┬──────────────────────────┤\n";
    std::cout << "│ Time │ State  │     Current Position     │      Target Position     │\n";
    std::cout << "├──────┼────────┼──────────────────────────┼──────────────────────────┤\n";
}

std::string Logger::formatTime() {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%H:%M:%S");
    return oss.str();
}

std::string Logger::stateToString(uint8_t state) {
    switch (state) {
        case 0: return "IDLE";
        case 1: return "PREFLT";
        case 2: return "ARMING";
        case 3: return "TAKEOF";
        case 4: return "NAVIGT";
        case 5: return "HOVER";
        case 6: return "LAND";
        case 7: return "LANDED";
        case 8: return "EMRGCY";
        case 9: return "RTH";
        default: return "UNKN";
    }
}

std::string Logger::stateToDisplay(uint8_t state) {
    switch (state) {
        case 0: return "\033[34m IDLE \033[0m";      // 蓝色
        case 1: return "\033[36mPREFLT\033[0m";      // 青色
        case 2: return "\033[33mARMING\033[0m";      // 黄色
        case 3: return "\033[33mTAKEOF\033[0m";      // 黄色
        case 4: return "\033[32mNAVIGT\033[0m";      // 绿色
        case 5: return "\033[36mHOVER \033[0m";      // 青色
        case 6: return "\033[33mLAND  \033[0m";      // 黄色
        case 7: return "\033[34mLANDED\033[0m";      // 蓝色
        case 8: return "\033[31mEMRGCY\033[0m";      // 红色
        case 9: return "\033[35m RTH  \033[0m";      // 紫色
        default: return "UNKN";
    }
}

void Logger::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_ = *msg;
    has_odom_ = true;
}

void Logger::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_mavros_state_ = *msg;
    has_mavros_state_ = true;
}

void Logger::navStatusCallback(const uav_navigator::NavigatorStatus::ConstPtr& msg) {
    current_nav_status_ = *msg;
    has_nav_status_ = true;
}

void Logger::safetyAlertCallback(const std_msgs::String::ConstPtr& msg) {
    printAlert(msg->data);
}

void Logger::setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // setpoint 日志静默，仅缓存数据
}

void Logger::metricsCallback(const uav_navigator::ExperimentMetrics::ConstPtr& msg) {
    current_metrics_ = *msg;
    has_metrics_ = true;
}

void Logger::configLoadedCallback(const std_msgs::String::ConstPtr& msg) {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║          CONFIG RELOADED — 配置已重新加载                     ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ " << msg->data << "\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
}

void Logger::printAlert(const std::string& alert) {
    std::cout << "\033[31m"  // 红色
              << "[" << formatTime() << "] ⚠ ALERT: " << alert
              << "\033[0m" << std::endl;
}

void Logger::printStatusLine() {
    if (!has_nav_status_) {
        // 等待第一条状态消息，不打印空行
        return;
    }

    // 格式化位置：处理 NaN 情况
    auto fmtCoord = [](float val) -> std::string {
        if (std::isnan(val)) return "  --  ";
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << std::setw(6) << val;
        return oss.str();
    };

    std::string time_str = formatTime();
    std::string state_str = stateToDisplay(current_nav_status_.state);

    // 当前位置
    std::string cur_str = fmtCoord(current_nav_status_.current_x) + " "
                        + fmtCoord(current_nav_status_.current_y) + " "
                        + fmtCoord(current_nav_status_.current_z);

    // 目标位置
    std::string tgt_str = fmtCoord(current_nav_status_.target_x) + " "
                        + fmtCoord(current_nav_status_.target_y) + " "
                        + fmtCoord(current_nav_status_.target_z);

    // 航点进度
    std::ostringstream wp_str;
    if (current_nav_status_.total_waypoints == 0) {
        wp_str << " --/-- ";
    } else {
        wp_str << std::setw(2) << (int)current_nav_status_.current_waypoint_index + 1
               << "/" << std::setw(2) << (int)current_nav_status_.total_waypoints;
    }

    // 飞行模式
    std::string mode = current_nav_status_.current_mode;
    if (mode.empty()) mode = "--";

    // 解锁状态
    std::string armed = current_nav_status_.is_armed ? "\033[32mARMED\033[0m" : "\033[90mDISARM\033[0m";

    // 偏差（如果有 metrics 数据）
    std::string dev_str;
    if (has_metrics_ && !std::isnan(current_metrics_.plan_deviation_total)) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << current_metrics_.plan_deviation_total;
        dev_str = oss.str() + "m";
    } else {
        dev_str = "  --  ";
    }

    // 主状态行
    std::cout << "\033[K"  // 清除行
              << time_str << " "
              << state_str << " "
              << "wp:" << wp_str.str() << " "
              << "pos:[" << cur_str << "] "
              << "tgt:[" << tgt_str << "] "
              << "dev:" << dev_str << " "
              << mode << " "
              << armed
              << "\n";

    // 分隔线（每10行打印一次表头提示）
    static int line_count = 0;
    line_count++;
    if (line_count % 20 == 0) {
        std::cout << "──────┼────────┼─────┼──────────────────────────┼──────────────────────────┼───────┼──────┼───────\n";
    }
}

void Logger::printSummary() {
    if (!has_nav_status_) {
        std::cout << "\033[K[" << formatTime() << "] \033[33mWaiting for navigator status...\033[0m\n";
        return;
    }
    printStatusLine();
}

void Logger::run() {
    ros::Rate rate(config_.print_rate);
    while (ros::ok()) {
        printSummary();
        ros::spinOnce();
        rate.sleep();
    }
}

} // namespace uav_navigator

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_logger");
    ros::NodeHandle nh;

    try {
        uav_navigator::Logger logger(nh);
        logger.run();
    } catch (const std::exception& e) {
        ROS_ERROR("[Logger] Exception: %s", e.what());
        return 1;
    }

    return 0;
}
