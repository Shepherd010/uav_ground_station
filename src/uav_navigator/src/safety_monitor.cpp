#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <uav_navigator/NavigatorStatus.h>

#include <geometry_msgs/PoseStamped.h>
#include <cmath>

namespace uav_navigator {

class SafetyMonitor {
public:
    SafetyMonitor(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~SafetyMonitor();
    void run();

private:
    // ROS接口
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber odom_sub_;
    ros::Subscriber mavros_state_sub_;
    ros::Subscriber navigator_status_sub_;
    ros::Subscriber setpoint_sub_;
    ros::Subscriber config_reload_sub_;

    ros::Publisher alert_pub_;
    ros::Publisher heartbeat_pub_;          // 心跳发布者，证明节点存活
    ros::ServiceServer emergency_stop_srv_;

    ros::Timer check_timer_;

    // 数据
    nav_msgs::Odometry current_odom_;
    mavros_msgs::State current_mavros_state_;
    bool has_odom_;
    bool has_mavros_state_;
    ros::Time last_navigator_status_time_;
    bool has_navigator_status_;
    uint8_t last_nav_state_;
    ros::Time last_setpoint_time_;
    ros::Time mode_mismatch_start_time_;
    ros::Time last_odom_time_;
    ros::Time last_mavros_state_time_;       // MAVROS 状态消息最后接收时间
    geometry_msgs::Point last_odom_position_;
    int setpoint_count_;

    // 告警去重：跟踪上次发布的告警类型和时间
    std::string last_alert_type_;
    ros::Time last_alert_time_;

    // 配置参数
    struct Config {
        std::string local_position_topic;
        std::string mavros_state_topic;
        std::string navigator_status_topic;
        std::string safety_alert_topic;
        std::string emergency_stop_service;
        std::string setpoint_topic;
        std::string config_reload_topic;
        std::string heartbeat_topic;        // 心跳话题

        double max_height_limit;
        double communication_timeout;
        double battery_threshold;
        double check_interval;

        double min_setpoint_rate_hz;
        double setpoint_timeout;
        double mode_mismatch_tolerance;
        double position_jump_distance;
        double position_jump_window;
        double alert_min_interval;          // 同类型告警最小间隔（秒）
    } config_;

    // 方法
    void loadConfig();
    void initROS();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    void navigatorStatusCallback(const uav_navigator::NavigatorStatus::ConstPtr& msg);
    void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void configReloadCallback(const std_msgs::String::ConstPtr& msg);
    void checkTimerCallback(const ros::TimerEvent& event);
    bool emergencyStopCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void publishAlert(const std::string& alert_type);
    bool isPositionValid(const geometry_msgs::Point& p);
};

SafetyMonitor::SafetyMonitor(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), has_odom_(false), has_mavros_state_(false), has_navigator_status_(false),
      last_nav_state_(0), setpoint_count_(0) {

    loadConfig();
    initROS();

    last_setpoint_time_ = ros::Time::now();
    mode_mismatch_start_time_ = ros::Time(0);
    last_odom_time_ = ros::Time(0);
    last_mavros_state_time_ = ros::Time(0);
    last_alert_time_ = ros::Time(0);

    ROS_INFO("[SafetyMonitor] Initialization complete. Heartbeat on: %s", config_.heartbeat_topic.c_str());
}

SafetyMonitor::~SafetyMonitor() {
    ROS_INFO("[SafetyMonitor] Destructor");
}

void SafetyMonitor::loadConfig() {
    ROS_INFO("[SafetyMonitor] Loading configuration parameters...");

    // 使用全局命名空间读取参数，确保与 panel 的 loadConfigFromFile 一致
    ros::NodeHandle global_nh;

    global_nh.param<std::string>("topics/local_position_odom", config_.local_position_topic, "mavros/local_position/odom");
    global_nh.param<std::string>("topics/mavros_state", config_.mavros_state_topic, "mavros/state");
    global_nh.param<std::string>("topics/navigator_status", config_.navigator_status_topic, "uav/navigator/status");
    global_nh.param<std::string>("topics/safety_alert", config_.safety_alert_topic, "uav/safety/alert");
    global_nh.param<std::string>("topics/emergency_stop", config_.emergency_stop_service, "uav/safety/emergency_stop");
    global_nh.param<std::string>("topics/setpoint_position", config_.setpoint_topic, "mavros/setpoint_position/local");
    global_nh.param<std::string>("topics/config_reload_topic", config_.config_reload_topic, "uav/config/reload");
    global_nh.param<std::string>("safety/heartbeat_topic", config_.heartbeat_topic, "uav/safety/heartbeat");

    global_nh.param<double>("safety/max_height_limit", config_.max_height_limit, 10.0);
    global_nh.param<double>("safety/communication_timeout", config_.communication_timeout, 5.0);
    global_nh.param<double>("safety/battery_threshold", config_.battery_threshold, 20.0);
    global_nh.param<double>("safety/check_interval", config_.check_interval, 1.0);
    global_nh.param<double>("safety/alert_min_interval", config_.alert_min_interval, 1.0);

    global_nh.param<double>("offboard_safety/min_setpoint_rate_hz", config_.min_setpoint_rate_hz, 10.0);
    global_nh.param<double>("offboard_safety/setpoint_timeout_emergency", config_.setpoint_timeout, 1.0);
    global_nh.param<double>("offboard_safety/mode_mismatch_tolerance", config_.mode_mismatch_tolerance, 2.0);
    global_nh.param<double>("position_safety/max_jump_distance", config_.position_jump_distance, 2.0);
    global_nh.param<double>("position_safety/jump_window", config_.position_jump_window, 0.1);

    // 验证关键参数
    if (config_.check_interval <= 0.0) {
        ROS_ERROR("[SafetyMonitor] check_interval=%.1f is invalid, using default 1.0 s", config_.check_interval);
        config_.check_interval = 1.0;
    }
    if (config_.max_height_limit <= 0.0) {
        ROS_ERROR("[SafetyMonitor] max_height_limit=%.1f is invalid, using default 10.0 m", config_.max_height_limit);
        config_.max_height_limit = 10.0;
    }
    if (config_.communication_timeout <= 0.0) {
        ROS_WARN("[SafetyMonitor] communication_timeout=%.1f is invalid, using default 5.0 s", config_.communication_timeout);
        config_.communication_timeout = 5.0;
    }

    ROS_INFO("[SafetyMonitor] Configuration:");
    ROS_INFO("  - max height: %.2f m", config_.max_height_limit);
    ROS_INFO("  - communication timeout: %.2f s", config_.communication_timeout);
    ROS_INFO("  - check interval: %.1f s", config_.check_interval);
    ROS_INFO("  - alert min interval: %.1f s", config_.alert_min_interval);
}

void SafetyMonitor::initROS() {
    odom_sub_ = nh_.subscribe(config_.local_position_topic, 10, &SafetyMonitor::odomCallback, this);
    mavros_state_sub_ = nh_.subscribe(config_.mavros_state_topic, 10, &SafetyMonitor::mavrosStateCallback, this);
    navigator_status_sub_ = nh_.subscribe(config_.navigator_status_topic, 10, &SafetyMonitor::navigatorStatusCallback, this);
    setpoint_sub_ = nh_.subscribe(config_.setpoint_topic, 10, &SafetyMonitor::setpointCallback, this);
    config_reload_sub_ = nh_.subscribe(config_.config_reload_topic, 1, &SafetyMonitor::configReloadCallback, this);

    alert_pub_ = nh_.advertise<std_msgs::String>(config_.safety_alert_topic, 10);
    heartbeat_pub_ = nh_.advertise<std_msgs::String>(config_.heartbeat_topic, 10);
    emergency_stop_srv_ = nh_.advertiseService(config_.emergency_stop_service, &SafetyMonitor::emergencyStopCallback, this);

    check_timer_ = nh_.createTimer(ros::Duration(config_.check_interval), &SafetyMonitor::checkTimerCallback, this);

    ROS_INFO("[SafetyMonitor] ROS interface initialization complete");
}

bool SafetyMonitor::isPositionValid(const geometry_msgs::Point& p) {
    return !std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)
        && !std::isinf(p.x) && !std::isinf(p.y) && !std::isinf(p.z);
}

void SafetyMonitor::setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    last_setpoint_time_ = ros::Time::now();
    setpoint_count_++;
}

void SafetyMonitor::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // NaN/Inf 校验：无效位置数据跳过，使用上一个有效值
    if (!isPositionValid(msg->pose.pose.position)) {
        ROS_WARN_THROTTLE(2.0, "[SafetyMonitor] Invalid position data (NaN/Inf), skipping");
        return;
    }

    current_odom_ = *msg;
    if (!has_odom_) {
        has_odom_ = true;
        last_odom_position_ = msg->pose.pose.position;
        last_odom_time_ = msg->header.stamp;
        return;  // 第一条有效消息不进行跳变检测
    }

    // 位置跳变检测：使用宽松的时间窗口，适应低于10Hz的odom频率
    double time_delta = (msg->header.stamp - last_odom_time_).toSec();
    double effective_window = std::max(config_.position_jump_window, time_delta * 1.5);
    if (time_delta > 0.0 && time_delta < effective_window) {
        double dx = msg->pose.pose.position.x - last_odom_position_.x;
        double dy = msg->pose.pose.position.y - last_odom_position_.y;
        double dz = msg->pose.pose.position.z - last_odom_position_.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist > config_.position_jump_distance) {
            ROS_ERROR("[SafetyMonitor] Position jump detected: %.2f m in %.3f s, triggering emergency",
                      dist, time_delta);
            publishAlert("POSITION_JUMP");
            // 不更新 last_odom_position_，保留跳变前的参考位置
            last_odom_time_ = msg->header.stamp;
            return;
        }
    }
    last_odom_position_ = msg->pose.pose.position;
    last_odom_time_ = msg->header.stamp;
}

void SafetyMonitor::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_mavros_state_ = *msg;
    has_mavros_state_ = true;  // 持续更新，表示正在接收 MAVROS 状态
    last_mavros_state_time_ = ros::Time::now();  // 记录最后收到 MAVROS 状态的时间
}

void SafetyMonitor::navigatorStatusCallback(const uav_navigator::NavigatorStatus::ConstPtr& msg) {
    last_navigator_status_time_ = ros::Time::now();
    has_navigator_status_ = true;  // 持续设 true，不放回 false，确保超时检测持续触发
    last_nav_state_ = msg->state;
}

void SafetyMonitor::configReloadCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[SafetyMonitor] Reloading configuration from rosparam (source: %s)", msg->data.c_str());
    loadConfig();
    // 限制定时器最小周期，防止重载后定时器高频运行
    double safe_interval = std::max(config_.check_interval, 0.1);
    check_timer_.setPeriod(ros::Duration(safe_interval), true);
    ROS_INFO("[SafetyMonitor] Configuration reloaded. max_height=%.2f m, check_interval=%.1f s",
             config_.max_height_limit, config_.check_interval);
}

void SafetyMonitor::checkTimerCallback(const ros::TimerEvent& event) {
    // 心跳：每次检查周期都发布，证明 SafetyMonitor 仍在运行
    {
        std_msgs::String heartbeat;
        heartbeat.data = "ALIVE";
        heartbeat_pub_.publish(heartbeat);
    }

    try {
        // 1. 高度超限检查（有 odom 时立即检查，不依赖其他条件）
        if (has_odom_) {
            if (isPositionValid(current_odom_.pose.pose.position)) {
                if (current_odom_.pose.pose.position.z > config_.max_height_limit) {
                    ROS_ERROR("[SafetyMonitor] Height exceeded: %.2f m > %.2f m",
                              current_odom_.pose.pose.position.z, config_.max_height_limit);
                    publishAlert("HEIGHT_EXCEEDED");
                }
            } else {
                ROS_WARN_THROTTLE(5.0, "[SafetyMonitor] Position data contains NaN/Inf, skipping height check");
            }
        } else {
            ROS_WARN_THROTTLE(10.0, "[SafetyMonitor] No position data received yet, height check skipped");
        }

        // 2. 通信超时检查（navigator节点是否存活）
        // 注：不再将 has_navigator_status_ 设回 false，确保超时检测持续触发
        if (has_navigator_status_) {
            double elapsed = (ros::Time::now() - last_navigator_status_time_).toSec();
            if (elapsed > config_.communication_timeout) {
                ROS_ERROR("[SafetyMonitor] Navigator communication timeout: no status for %.1f seconds", elapsed);
                publishAlert("NAVIGATOR_TIMEOUT");
            }
        }

        // 3. setpoint 流检查（仅在飞行状态）
        // 使用 NavigatorStatus 消息常量
        bool is_flying = (last_nav_state_ == uav_navigator::NavigatorStatus::STATE_TAKEOFF
                       || last_nav_state_ == uav_navigator::NavigatorStatus::STATE_NAVIGATING
                       || last_nav_state_ == uav_navigator::NavigatorStatus::STATE_HOVERING
                       || last_nav_state_ == uav_navigator::NavigatorStatus::STATE_RETURNING);
        if (is_flying) {
            double setpoint_elapsed = (ros::Time::now() - last_setpoint_time_).toSec();
            if (setpoint_elapsed > config_.setpoint_timeout) {
                ROS_ERROR("[SafetyMonitor] Setpoint stream timeout: %.2f s, triggering emergency", setpoint_elapsed);
                publishAlert("SETPOINT_TIMEOUT");
            }
        }

        // 4. 模式一致性检查
        if (is_flying && has_mavros_state_ && current_mavros_state_.connected) {
            if (current_mavros_state_.mode != "OFFBOARD") {
                if (mode_mismatch_start_time_.isZero()) {
                    mode_mismatch_start_time_ = ros::Time::now();
                    ROS_WARN("[SafetyMonitor] Mode mismatch: navigator flying but mode is %s",
                             current_mavros_state_.mode.c_str());
                } else if ((ros::Time::now() - mode_mismatch_start_time_).toSec() > config_.mode_mismatch_tolerance) {
                    ROS_ERROR("[SafetyMonitor] Mode mismatch persisted for %.1f s, triggering emergency",
                              config_.mode_mismatch_tolerance);
                    publishAlert("MODE_MISMATCH");
                    mode_mismatch_start_time_ = ros::Time(0);
                }
            } else {
                mode_mismatch_start_time_ = ros::Time(0);
            }
        }

        // 5. MAVROS 连接断开检查
        if (has_mavros_state_ && !current_mavros_state_.connected) {
            ROS_ERROR("[SafetyMonitor] MAVROS connection lost!");
            publishAlert("MAVROS_DISCONNECTED");
        }

        // 5b. MAVROS 状态消息超时检查（MAVROS 进程崩溃后消息停止到达）
        if (has_mavros_state_ && !last_mavros_state_time_.isZero()) {
            double mavros_elapsed = (ros::Time::now() - last_mavros_state_time_).toSec();
            if (mavros_elapsed > config_.communication_timeout) {
                ROS_ERROR("[SafetyMonitor] MAVROS state timeout: no state update for %.1f s", mavros_elapsed);
                publishAlert("MAVROS_TIMEOUT");
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR("[SafetyMonitor] Exception in check timer callback: %s", e.what());
        // 即使异常也尝试发布心跳和告警
        try {
            std_msgs::String heartbeat;
            heartbeat.data = "ERROR";
            heartbeat_pub_.publish(heartbeat);
            std_msgs::String alert;
            alert.data = "SAFETY_MONITOR_ERROR";
            alert_pub_.publish(alert);
        } catch (...) {}
    }
}

bool SafetyMonitor::emergencyStopCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    ROS_ERROR("[SafetyMonitor] Emergency stop request received!");
    publishAlert("EMERGENCY_STOP");
    // 验证告警是否真正发布
    if (alert_pub_.getNumSubscribers() == 0) {
        ROS_ERROR("[SafetyMonitor] No subscribers for safety alert topic! Emergency stop may be ineffective.");
        res.success = false;
        res.message = "Emergency stop triggered but no alert subscribers detected";
    } else {
        res.success = true;
        res.message = "Emergency stop triggered";
    }
    return true;
}

void SafetyMonitor::publishAlert(const std::string& alert_type) {
    // 告警去重：同类型告警在 alert_min_interval 秒内不重复发布
    ros::Time now = ros::Time::now();
    if (alert_type == last_alert_type_ && (now - last_alert_time_).toSec() < config_.alert_min_interval) {
        return;
    }

    // 检查订阅者
    if (alert_pub_.getNumSubscribers() == 0) {
        ROS_WARN_THROTTLE(5.0, "[SafetyMonitor] Alert '%s' published but no subscribers on %s",
                          alert_type.c_str(), config_.safety_alert_topic.c_str());
    }

    std_msgs::String msg;
    msg.data = alert_type;
    alert_pub_.publish(msg);
    ROS_ERROR("[SafetyMonitor] Publishing safety alert: %s", alert_type.c_str());

    last_alert_type_ = alert_type;
    last_alert_time_ = now;
}

void SafetyMonitor::run() {
    ROS_INFO("[SafetyMonitor] Entering main loop");
    ros::spin();
}

} // namespace uav_navigator

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_safety_monitor");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        uav_navigator::SafetyMonitor monitor(nh, pnh);
        monitor.run();
    } catch (const std::exception& e) {
        ROS_ERROR("[SafetyMonitor] Exception: %s", e.what());
        return 1;
    }

    return 0;
}
