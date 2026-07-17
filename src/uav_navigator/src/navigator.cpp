#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>
#include <string>
#include <mutex>

#include <limits>

#include "uav_navigator/NavigatorStatus.h"
#include "uav_navigator/NavigatorCommand.h"
#include "uav_navigator/ExperimentMetrics.h"

namespace uav_navigator {

// 状态机状态枚举
enum class State {
    IDLE = 0,
    PRE_FLIGHT = 1,
    ARMING = 2,
    TAKEOFF = 3,
    NAVIGATING = 4,
    HOVERING = 5,
    LANDING = 6,
    LANDED = 7,
    EMERGENCY = 8,
    RETURNING = 9
};

// 状态机状态名（用于日志输出）
const char* stateToString(State state) {
    switch (state) {
        case State::IDLE: return "IDLE";
        case State::PRE_FLIGHT: return "PRE_FLIGHT";
        case State::ARMING: return "ARMING";
        case State::TAKEOFF: return "TAKEOFF";
        case State::NAVIGATING: return "NAVIGATING";
        case State::HOVERING: return "HOVERING";
        case State::LANDING: return "LANDING";
        case State::LANDED: return "LANDED";
        case State::EMERGENCY: return "EMERGENCY";
        case State::RETURNING: return "RETURNING";
        default: return "UNKNOWN";
    }
}

class Navigator {
public:
    Navigator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~Navigator();
    void run();

private:
    // ========== ROS 接口 ==========
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // 订阅者
    ros::Subscriber state_sub_;
    ros::Subscriber local_pos_sub_;
    ros::Subscriber waypoints_sub_;
    ros::Subscriber safety_alert_sub_;
    ros::Subscriber config_reload_sub_;

    // 发布者
    ros::Publisher setpoint_pub_;
    ros::Publisher status_pub_;
    ros::Publisher planned_path_pub_;
    ros::Publisher real_path_pub_;
    ros::Publisher metrics_pub_;

    // 服务
    ros::ServiceServer command_srv_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    // 定时器
    ros::Timer state_machine_timer_;
    ros::Timer setpoint_timer_;

    // ========== MAVROS 状态 ==========
    mavros_msgs::State mavros_state_;
    nav_msgs::Odometry current_odom_;
    bool has_odom_;

    // ========== 航点数据 ==========
    geometry_msgs::PoseArray waypoints_;
    size_t current_waypoint_idx_;
    bool has_waypoints_;

    // ========== 线程安全 ==========
    mutable std::mutex state_mutex_;

    // ========== 状态机 ==========
    State nav_state_;
    State previous_nav_state_;
    ros::Time state_enter_time_;
    ros::Time mission_start_time_;       // 任务开始时间（进入 PRE_FLIGHT 时记录）
    ros::Time last_mode_request_time_;
    ros::Time last_arm_request_time_;
    ros::Time hover_start_time_;
    ros::Time last_status_pub_time_;

    // 安全标志
    bool emergency_triggered_;
    std::string emergency_reason_;
    ros::Time emergency_time_;  // 记录进入紧急状态的时间

    // 轨迹记录开关
    bool is_recording_path_;

    // 飞行统计
    int arm_request_count_;
    int mode_request_count_;
    int preflight_fail_count_;

    // OFFBOARD 预发布计数
    int pre_pub_count_;
    bool offboard_pre_pub_complete_;
    ros::Time last_real_path_sample_time_;

    // ========== 当前 setpoint ==========
    geometry_msgs::PoseStamped current_setpoint_;
    bool has_home_position_;
    geometry_msgs::Pose home_position_;

    // ========== 轨迹与实验指标 ==========
    nav_msgs::Path planned_path_;
    nav_msgs::Path real_path_;
    ros::Time last_odom_time_;
    geometry_msgs::Point last_odom_position_;

    // ========== OFFBOARD 安全监控 ==========
    ros::Time last_setpoint_pub_time_;
    int setpoint_pub_count_;
    ros::Time setpoint_rate_check_start_;
    ros::Time mode_loss_time_;
    ros::Time last_emergency_mode_req_time_;  // 紧急状态下模式请求限流

    // ========== 配置参数 ==========
    struct Config {
        // 话题名
        std::string mavros_state_topic;
        std::string local_position_topic;
        std::string setpoint_topic;
        std::string arming_service;
        std::string set_mode_service;
        std::string waypoint_topic;
        std::string navigator_status_topic;
        std::string navigator_command_service;
        std::string safety_alert_topic;

        // 飞行参数
        double takeoff_height;
        double hover_duration;
        double setpoint_rate;
        int offboard_pre_pub_count;
        double landing_height_threshold;

        // 航点参数
        double reach_threshold_xy;
        double reach_threshold_z;
        double min_waypoint_spacing;

        // 模式切换参数
        double offboard_timeout;
        double takeoff_timeout;
        double landing_timeout;           // 降落超时（秒）
        double emergency_timeout;         // 紧急状态超时（秒）
        double mode_retry_interval;
        double arm_retry_interval;

        // 安全参数
        double max_height_limit;
        double communication_timeout;

        // OFFBOARD 安全参数
        double min_setpoint_rate_hz;
        double setpoint_timeout_warn;
        double setpoint_timeout_emergency;
        double mode_mismatch_tolerance;
        double position_jump_distance;
        double position_jump_window;

        // 轨迹/实验参数
        std::string planned_path_topic;
        std::string real_path_topic;
        std::string metrics_topic;
        std::string config_reload_topic;
        std::string default_frame_id;
        double real_path_sample_interval;
        int max_real_path_points;    // real_path 最大点数（FIFO），防止 RViz 卡顿
    } config_;

    // ========== 方法 ==========
    // 参数加载
    void loadConfig();

    // ROS 初始化
    void initROS();

    // 轨迹
    void buildPlannedPath();
    void appendRealPath();
    double computePlanDeviation();
    void publishMetrics();

    // 回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void localPosCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void safetyAlertCallback(const std_msgs::String::ConstPtr& msg);
    void configReloadCallback(const std_msgs::String::ConstPtr& msg);
    bool commandCallback(uav_navigator::NavigatorCommand::Request& req,
                         uav_navigator::NavigatorCommand::Response& res);

    // 定时器回调
    void stateMachineTimerCallback(const ros::TimerEvent& event);
    void setpointTimerCallback(const ros::TimerEvent& event);

    // 状态机处理
    void processStateMachine();
    void transitionState(State new_state);

    // 各状态处理
    void handleIdle();
    void handlePreFlight();
    void handleArming();
    void handleTakeoff();
    void handleNavigating();
    void handleHovering();
    void handleLanding();
    void handleLanded();
    void handleEmergency();
    void handleReturning();

    // 辅助函数
    bool requestArming(bool arm);
    bool requestMode(const std::string& mode);
    bool isWaypointReached();
    void publishStatus();
    void setSetpoint(const geometry_msgs::Pose& pose);
    void setSetpointXYZ(double x, double y, double z);
    void updateSetpointFromCurrentPosition();
    bool checkPreFlight();
    void enterEmergency(const std::string& reason);
};

// 构造函数
Navigator::Navigator(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), nav_state_(State::IDLE), previous_nav_state_(State::IDLE),
      has_odom_(false), has_waypoints_(false), current_waypoint_idx_(0),
      emergency_triggered_(false), has_home_position_(false),
      arm_request_count_(0), mode_request_count_(0), preflight_fail_count_(0),
      pre_pub_count_(0), offboard_pre_pub_complete_(false), setpoint_pub_count_(0),
      is_recording_path_(false) {

    loadConfig();
    initROS();

    // 初始化setpoint：默认悬停在起飞高度，避免地面或无效位置导致异常
    current_setpoint_.header.frame_id = config_.default_frame_id;
    current_setpoint_.pose.position.x = 0.0;
    current_setpoint_.pose.position.y = 0.0;
    current_setpoint_.pose.position.z = config_.takeoff_height;
    current_setpoint_.pose.orientation.x = 0.0;
    current_setpoint_.pose.orientation.y = 0.0;
    current_setpoint_.pose.orientation.z = 0.0;
    current_setpoint_.pose.orientation.w = 1.0;

    // 初始化轨迹
    planned_path_.header.frame_id = config_.default_frame_id;
    real_path_.header.frame_id = config_.default_frame_id;

    last_setpoint_pub_time_ = ros::Time::now();
    setpoint_rate_check_start_ = ros::Time::now();
    last_real_path_sample_time_ = ros::Time(0);
    last_odom_time_ = ros::Time(0);
    mode_loss_time_ = ros::Time(0);
    last_emergency_mode_req_time_ = ros::Time(0);

    ROS_INFO("[Navigator] Initialization complete, current state: %s", stateToString(nav_state_));
}

Navigator::~Navigator() {
    ROS_INFO("[Navigator] Destructor");
}

// 加载配置
void Navigator::loadConfig() {
    ROS_INFO("[Navigator] Loading configuration parameters...");

    // 使用全局命名空间读取参数，确保与 panel 的 loadConfigFromFile 一致
    ros::NodeHandle global_nh;

    // 话题配置
    global_nh.param<std::string>("topics/mavros_state", config_.mavros_state_topic, "mavros/state");
    global_nh.param<std::string>("topics/local_position_odom", config_.local_position_topic, "mavros/local_position/odom");
    global_nh.param<std::string>("topics/setpoint_position", config_.setpoint_topic, "mavros/setpoint_position/local");
    global_nh.param<std::string>("topics/arming_service", config_.arming_service, "mavros/cmd/arming");
    global_nh.param<std::string>("topics/set_mode_service", config_.set_mode_service, "mavros/set_mode");
    global_nh.param<std::string>("topics/waypoint_current", config_.waypoint_topic, "uav/waypoints/current");
    global_nh.param<std::string>("topics/navigator_status", config_.navigator_status_topic, "uav/navigator/status");
    global_nh.param<std::string>("topics/navigator_command", config_.navigator_command_service, "uav/navigator/command");
    global_nh.param<std::string>("topics/safety_alert", config_.safety_alert_topic, "uav/safety/alert");

    // 飞行参数：优先从 flight_defaults 读取（与 waypoint_manager 保持一致），
    // 回退到 flight（兼容旧版 config.yaml）
    global_nh.param<double>("flight_defaults/takeoff_height", config_.takeoff_height, 1.0);
    global_nh.param<double>("flight_defaults/hover_duration", config_.hover_duration, 2.0);
    global_nh.param<double>("flight_defaults/setpoint_rate", config_.setpoint_rate, 20.0);
    global_nh.param<int>("flight_defaults/offboard_pre_pub_count", config_.offboard_pre_pub_count, 100);
    global_nh.param<double>("flight_defaults/landing_height_threshold", config_.landing_height_threshold, 0.15);
    global_nh.param<double>("flight_defaults/takeoff_timeout", config_.takeoff_timeout, 20.0);
    // 向后兼容：如果 flight_defaults 未设置，回退到 flight 节
    if (!global_nh.hasParam("flight_defaults/takeoff_height")) {
        global_nh.param<double>("flight/takeoff_height", config_.takeoff_height, 1.0);
        global_nh.param<double>("flight/hover_duration", config_.hover_duration, 2.0);
        global_nh.param<double>("flight/setpoint_rate", config_.setpoint_rate, 20.0);
        global_nh.param<int>("flight/offboard_pre_pub_count", config_.offboard_pre_pub_count, 100);
        global_nh.param<double>("flight/landing_height_threshold", config_.landing_height_threshold, 0.15);
        global_nh.param<double>("flight/takeoff_timeout", config_.takeoff_timeout, 20.0);
    }

    // 航点参数
    global_nh.param<double>("waypoint/reach_threshold_xy", config_.reach_threshold_xy, 0.15);
    global_nh.param<double>("waypoint/reach_threshold_z", config_.reach_threshold_z, 0.2);
    global_nh.param<double>("waypoint/min_waypoint_spacing", config_.min_waypoint_spacing, 0.3);

    // 模式切换参数
    global_nh.param<double>("mode/offboard_timeout", config_.offboard_timeout, 8.0);
    global_nh.param<double>("mode/landing_timeout", config_.landing_timeout, 60.0);
    global_nh.param<double>("mode/emergency_timeout", config_.emergency_timeout, 120.0);
    global_nh.param<double>("mode/mode_retry_interval", config_.mode_retry_interval, 5.0);
    global_nh.param<double>("mode/arm_retry_interval", config_.arm_retry_interval, 5.0);

    // 验证关键参数，防止除零和异常行为
    if (config_.setpoint_rate <= 0.0) {
        ROS_ERROR("[Navigator] setpoint_rate=%.1f is invalid, using default 20.0 Hz", config_.setpoint_rate);
        config_.setpoint_rate = 20.0;
    }
    if (config_.landing_timeout <= 0.0) {
        ROS_WARN("[Navigator] landing_timeout=%.1f is invalid, using default 60.0 s", config_.landing_timeout);
        config_.landing_timeout = 60.0;
    }
    if (config_.emergency_timeout <= 0.0) {
        ROS_WARN("[Navigator] emergency_timeout=%.1f is invalid, using default 120.0 s", config_.emergency_timeout);
        config_.emergency_timeout = 120.0;
    }

    // 安全参数
    global_nh.param<double>("safety/max_height_limit", config_.max_height_limit, 10.0);
    global_nh.param<double>("safety/communication_timeout", config_.communication_timeout, 5.0);

    // OFFBOARD 安全参数
    global_nh.param<double>("offboard_safety/min_setpoint_rate_hz", config_.min_setpoint_rate_hz, 10.0);
    global_nh.param<double>("offboard_safety/setpoint_timeout_warn", config_.setpoint_timeout_warn, 0.5);
    global_nh.param<double>("offboard_safety/setpoint_timeout_emergency", config_.setpoint_timeout_emergency, 1.0);
    global_nh.param<double>("offboard_safety/mode_mismatch_tolerance", config_.mode_mismatch_tolerance, 2.0);
    global_nh.param<double>("position_safety/max_jump_distance", config_.position_jump_distance, 2.0);
    global_nh.param<double>("position_safety/jump_window", config_.position_jump_window, 0.1);

    // 轨迹/实验参数
    global_nh.param<std::string>("topics/planned_path", config_.planned_path_topic, "uav/trajectory/planned");
    global_nh.param<std::string>("topics/real_path", config_.real_path_topic, "uav/trajectory/real");
    global_nh.param<std::string>("topics/metrics", config_.metrics_topic, "uav/experiment/metrics");
    global_nh.param<std::string>("topics/config_reload_topic", config_.config_reload_topic, "uav/config/reload");
    global_nh.param<double>("experiment/real_path_sample_interval", config_.real_path_sample_interval, 0.1);
    global_nh.param<int>("experiment/max_real_path_points", config_.max_real_path_points, 500);
    global_nh.param<std::string>("paths/default_frame_id", config_.default_frame_id, "map");

    ROS_INFO("[Navigator] Configuration loaded:");
    ROS_INFO("  - takeoff height: %.2f m", config_.takeoff_height);
    ROS_INFO("  - hover duration: %.2f s", config_.hover_duration);
    ROS_INFO("  - setpoint rate: %.1f Hz", config_.setpoint_rate);
    ROS_INFO("  - OFFBOARD pre-publish: %d", config_.offboard_pre_pub_count);
    ROS_INFO("  - reach threshold xy: %.2f m, z: %.2f m", config_.reach_threshold_xy, config_.reach_threshold_z);
    ROS_INFO("  - OFFBOARD timeout: %.1f s", config_.offboard_timeout);
    ROS_INFO("  - takeoff timeout: %.1f s", config_.takeoff_timeout);
    ROS_INFO("  - max height limit: %.2f m", config_.max_height_limit);
}

// 初始化ROS接口
void Navigator::initROS() {
    // 订阅者
    state_sub_ = nh_.subscribe(config_.mavros_state_topic, 10, &Navigator::stateCallback, this);
    local_pos_sub_ = nh_.subscribe(config_.local_position_topic, 10, &Navigator::localPosCallback, this);
    waypoints_sub_ = nh_.subscribe(config_.waypoint_topic, 1, &Navigator::waypointsCallback, this);
    safety_alert_sub_ = nh_.subscribe(config_.safety_alert_topic, 1, &Navigator::safetyAlertCallback, this);

    // 发布者
    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(config_.setpoint_topic, 10);
    status_pub_ = nh_.advertise<uav_navigator::NavigatorStatus>(config_.navigator_status_topic, 10);
    planned_path_pub_ = nh_.advertise<nav_msgs::Path>(config_.planned_path_topic, 1, true);
    real_path_pub_ = nh_.advertise<nav_msgs::Path>(config_.real_path_topic, 10);
    metrics_pub_ = nh_.advertise<uav_navigator::ExperimentMetrics>(config_.metrics_topic, 10);

    // 服务
    command_srv_ = nh_.advertiseService(config_.navigator_command_service, &Navigator::commandCallback, this);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(config_.arming_service);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(config_.set_mode_service);
    config_reload_sub_ = nh_.subscribe(config_.config_reload_topic, 1, &Navigator::configReloadCallback, this);

    // 状态机定时器（10Hz）
    state_machine_timer_ = nh_.createTimer(ros::Duration(0.1), &Navigator::stateMachineTimerCallback, this);

    // setpoint 定时器（从配置读取频率）
    double setpoint_period = 1.0 / config_.setpoint_rate;
    setpoint_timer_ = nh_.createTimer(ros::Duration(setpoint_period), &Navigator::setpointTimerCallback, this);

    ROS_INFO("[Navigator] ROS interface initialization complete");
    ROS_INFO("  - subscribe: %s, %s, %s", config_.mavros_state_topic.c_str(), config_.local_position_topic.c_str(), config_.waypoint_topic.c_str());
    ROS_INFO("  - publish: %s, %s", config_.setpoint_topic.c_str(), config_.navigator_status_topic.c_str());
}

// ========== 回调函数 ==========

void Navigator::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    mavros_state_ = *msg;
}

void Navigator::localPosCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_odom_ = *msg;
    if (!has_odom_) {
        has_odom_ = true;
        last_odom_position_ = msg->pose.pose.position;
        ROS_INFO_ONCE("[Navigator] First position data received");
    }

    // 位置跳变检测
    if ((msg->header.stamp - last_odom_time_).toSec() < config_.position_jump_window) {
        double dx = msg->pose.pose.position.x - last_odom_position_.x;
        double dy = msg->pose.pose.position.y - last_odom_position_.y;
        double dz = msg->pose.pose.position.z - last_odom_position_.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist > config_.position_jump_distance) {
            ROS_ERROR("[Navigator] Position jump detected: %.2f m in %.3f s, triggering emergency",
                      dist, (msg->header.stamp - last_odom_time_).toSec());
            enterEmergency("Position jump detected");
        }
    }
    last_odom_position_ = msg->pose.pose.position;
    last_odom_time_ = msg->header.stamp;

    // 高度超限检查
    if (nav_state_ != State::EMERGENCY && nav_state_ != State::LANDING && nav_state_ != State::LANDED) {
        if (current_odom_.pose.pose.position.z > config_.max_height_limit) {
            ROS_ERROR("[Navigator] Height exceeded: %.2f m > %.2f m, emergency protection triggered!",
                      current_odom_.pose.pose.position.z, config_.max_height_limit);
            enterEmergency("Height exceeded");
        }
    }
}

void Navigator::waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    // 飞行中收到新航点时不重置索引，避免无人机突然飞回第一个航点
    bool is_flying = (nav_state_ == State::TAKEOFF || nav_state_ == State::NAVIGATING ||
                      nav_state_ == State::HOVERING);
    waypoints_ = *msg;
    has_waypoints_ = true;
    if (!is_flying) {
        current_waypoint_idx_ = 0;
    } else {
        // 飞行中收到新航点：保持当前索引，但限制不超过新航点数量
        if (current_waypoint_idx_ >= waypoints_.poses.size()) {
            current_waypoint_idx_ = waypoints_.poses.size() > 0 ? waypoints_.poses.size() - 1 : 0;
        }
        ROS_WARN("[Navigator] Waypoints updated mid-flight, keeping current index %zu (new total: %zu)",
                 current_waypoint_idx_, waypoints_.poses.size());
        return;
    }
    ROS_INFO("[Navigator] Received %zu waypoints", waypoints_.poses.size());
}

void Navigator::safetyAlertCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_WARN("[Navigator] Received safety alert: %s", msg->data.c_str());

    if (msg->data == "EMERGENCY_STOP" || msg->data == "HEIGHT_EXCEEDED") {
        enterEmergency(msg->data);
    } else if (msg->data == "COMMUNICATION_TIMEOUT") {
        if (nav_state_ == State::TAKEOFF || nav_state_ == State::NAVIGATING || nav_state_ == State::HOVERING) {
            ROS_WARN("[Navigator] Communication timeout, triggering return to home");
            transitionState(State::RETURNING);
        }
    }
}

void Navigator::configReloadCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[Navigator] Reloading configuration from rosparam (source: %s)", msg->data.c_str());
    loadConfig();
    double setpoint_period = 1.0 / config_.setpoint_rate;
    setpoint_timer_.setPeriod(ros::Duration(setpoint_period), true);
    ROS_INFO("[Navigator] Configuration reloaded. setpoint_rate=%.1f Hz, takeoff_height=%.2f m, max_height=%.2f m",
             config_.setpoint_rate, config_.takeoff_height, config_.max_height_limit);
}

bool Navigator::commandCallback(uav_navigator::NavigatorCommand::Request& req,
                                uav_navigator::NavigatorCommand::Response& res) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ROS_INFO("[Navigator] Received command: %s", req.command.c_str());

    if (req.command == "START") {
        if (nav_state_ == State::IDLE || nav_state_ == State::LANDED) {
            if (!has_waypoints_) {
                res.success = false;
                res.message = "No waypoint data. Please load waypoints first.";
                return true;
            }
            transitionState(State::PRE_FLIGHT);
            res.success = true;
            res.message = "Navigation started";
        } else {
            res.success = false;
            res.message = "Current state does not support START command: " + std::string(stateToString(nav_state_));
        }
    } else if (req.command == "PAUSE") {
        if (nav_state_ == State::NAVIGATING) {
            // Pause: record current state, switch to HOVERING
            previous_nav_state_ = nav_state_;
            transitionState(State::HOVERING);
            res.success = true;
            res.message = "Navigation paused, hovering at current position";
        } else {
            res.success = false;
            res.message = "PAUSE command not supported in current state";
        }
    } else if (req.command == "CANCEL") {
        if (nav_state_ == State::NAVIGATING || nav_state_ == State::HOVERING || nav_state_ == State::TAKEOFF) {
            transitionState(State::LANDING);
            res.success = true;
            res.message = "Navigation cancelled, landing";
        } else {
            res.success = false;
            res.message = "CANCEL command not supported in current state";
        }
    } else if (req.command == "LAND") {
        if (nav_state_ != State::LANDING && nav_state_ != State::LANDED && nav_state_ != State::IDLE) {
            transitionState(State::LANDING);
            res.success = true;
            res.message = "Landing";
        } else {
            res.success = false;
            res.message = "Already landing or landed";
        }
    } else if (req.command == "RETURN_TO_HOME") {
        if (nav_state_ == State::NAVIGATING || nav_state_ == State::HOVERING) {
            transitionState(State::RETURNING);
            res.success = true;
            res.message = "Returning to home";
        } else {
            res.success = false;
            res.message = "RETURN_TO_HOME not supported in current state";
        }
    } else if (req.command == "EMERGENCY_STOP") {
        is_recording_path_ = false;
        enterEmergency("Manual emergency stop");
        res.success = true;
        res.message = "Emergency stop triggered";
    } else if (req.command == "RESET") {
        if (nav_state_ == State::EMERGENCY || nav_state_ == State::LANDED) {
            transitionState(State::IDLE);
            has_waypoints_ = false;
            current_waypoint_idx_ = 0;
            emergency_triggered_ = false;
            emergency_reason_.clear();
            res.success = true;
            res.message = "Reset complete, waiting for new command";
        } else {
            res.success = false;
            res.message = "RESET only available in EMERGENCY or LANDED state";
        }
    } else {
        res.success = false;
        res.message = "Unknown command: " + req.command;
    }

    return true;
}

// ========== 状态机处理 ==========

void Navigator::stateMachineTimerCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    processStateMachine();
    publishStatus();
    publishMetrics();
    if (is_recording_path_) {
        real_path_pub_.publish(real_path_);
    }
}

void Navigator::processStateMachine() {
    switch (nav_state_) {
        case State::IDLE:
            handleIdle();
            break;
        case State::PRE_FLIGHT:
            handlePreFlight();
            break;
        case State::ARMING:
            handleArming();
            break;
        case State::TAKEOFF:
            handleTakeoff();
            break;
        case State::NAVIGATING:
            handleNavigating();
            break;
        case State::HOVERING:
            handleHovering();
            break;
        case State::LANDING:
            handleLanding();
            break;
        case State::LANDED:
            handleLanded();
            break;
        case State::EMERGENCY:
            handleEmergency();
            break;
        case State::RETURNING:
            handleReturning();
            break;
        default:
            ROS_ERROR("[Navigator] Unknown state: %d", static_cast<int>(nav_state_));
            transitionState(State::EMERGENCY);
            break;
    }
}

void Navigator::transitionState(State new_state) {
    if (new_state == nav_state_) return;

    ROS_INFO("[Navigator] State transition: %s -> %s", stateToString(nav_state_), stateToString(new_state));
    previous_nav_state_ = nav_state_;
    nav_state_ = new_state;
    state_enter_time_ = ros::Time::now();

    // 状态进入时的初始化
    switch (new_state) {
        case State::IDLE:
            // 保持当前位置作为setpoint
            updateSetpointFromCurrentPosition();
            break;
        case State::PRE_FLIGHT:
            // 记录任务开始时间
            mission_start_time_ = ros::Time::now();
            // 预飞行检查
            break;
        case State::ARMING:
            last_arm_request_time_ = ros::Time::now();
            break;
        case State::TAKEOFF:
            last_mode_request_time_ = ros::Time::now();
            // 重置预发布计数
            pre_pub_count_ = config_.offboard_pre_pub_count;
            offboard_pre_pub_complete_ = false;
            // 设置起飞目标setpoint：始终使用起飞高度，避免has_odom_为false时保持默认z=0
            if (has_odom_) {
                setSetpointXYZ(current_odom_.pose.pose.position.x,
                               current_odom_.pose.pose.position.y,
                               config_.takeoff_height);
                // 记录 home position
                if (!has_home_position_) {
                    home_position_ = current_odom_.pose.pose;
                    has_home_position_ = true;
                    ROS_INFO("[Navigator] Recorded Home position: (%.2f, %.2f, %.2f)",
                             home_position_.position.x, home_position_.position.y, home_position_.position.z);
                }
            } else {
                setSetpointXYZ(0.0, 0.0, config_.takeoff_height);
                ROS_WARN("[Navigator] Takeoff without odom, using default setpoint (0, 0, %.2f)", config_.takeoff_height);
            }
            ROS_INFO("[Navigator] Takeoff setpoint set to: (%.2f, %.2f, %.2f)",
                     current_setpoint_.pose.position.x,
                     current_setpoint_.pose.position.y,
                     current_setpoint_.pose.position.z);
            break;
        case State::NAVIGATING:
            if (has_waypoints_ && current_waypoint_idx_ < waypoints_.poses.size()) {
                setSetpoint(waypoints_.poses[current_waypoint_idx_]);
                ROS_INFO("[Navigator] Starting navigation to waypoint %zu/%zu: (%.2f, %.2f, %.2f)",
                         current_waypoint_idx_ + 1, waypoints_.poses.size(),
                         waypoints_.poses[current_waypoint_idx_].position.x,
                         waypoints_.poses[current_waypoint_idx_].position.y,
                         waypoints_.poses[current_waypoint_idx_].position.z);
            } else {
                // 没有有效航点时，保持当前高度悬停，避免发布旧的目标点（如 z=0）
                ROS_WARN("[Navigator] NAVIGATING requested but no valid waypoint, hovering at current position");
                updateSetpointFromCurrentPosition();
            }
            is_recording_path_ = true;
            buildPlannedPath();
            appendRealPath();
            break;
        case State::HOVERING:
            hover_start_time_ = ros::Time::now();
            break;
        case State::LANDING:
            is_recording_path_ = false;
            // 设置降落目标
            if (has_odom_) {
                setSetpointXYZ(current_odom_.pose.pose.position.x,
                               current_odom_.pose.pose.position.y,
                               0.0);  // 目标高度为0
            }
            ROS_WARN("[Navigator] Starting landing");
            break;
        case State::LANDED:
            is_recording_path_ = false;
            ROS_INFO("[Navigator] Landed, waiting for RESET command");
            break;
        case State::EMERGENCY:
            is_recording_path_ = false;
            ROS_ERROR("[Navigator] Entering emergency state! Reason: %s", emergency_reason_.c_str());
            break;
        case State::RETURNING:
            if (has_home_position_) {
                setSetpoint(home_position_);
                ROS_INFO("[Navigator] Starting return to Home: (%.2f, %.2f, %.2f)",
                         home_position_.position.x, home_position_.position.y, home_position_.position.z);
            } else if (has_odom_) {
                setSetpoint(current_odom_.pose.pose);
                ROS_WARN("[Navigator] No Home position recorded, hovering at current position");
            }
            break;
        default:
            break;
    }
}

// ========== 各状态处理 ==========

void Navigator::handleIdle() {
    // IDLE 状态：保持当前位置setpoint，等待命令
    updateSetpointFromCurrentPosition();
}

void Navigator::handlePreFlight() {
    // 预飞行检查
    if (checkPreFlight()) {
        ROS_INFO("[Navigator] Pre-flight check passed");
        transitionState(State::ARMING);
    } else {
        // 检查超时
        if ((ros::Time::now() - state_enter_time_).toSec() > config_.offboard_timeout) {
            ROS_ERROR("[Navigator] Pre-flight check timeout, returning to IDLE");
            transitionState(State::IDLE);
        }
    }
}

void Navigator::handleArming() {
    // 请求解锁
    if (!mavros_state_.armed) {
        if ((ros::Time::now() - last_arm_request_time_).toSec() > config_.arm_retry_interval) {
            if (requestArming(true)) {
                ROS_INFO("[Navigator] Arming request sent");
            }
            last_arm_request_time_ = ros::Time::now();
        }
    } else {
        ROS_INFO("[Navigator] Armed, ready for takeoff");
        transitionState(State::TAKEOFF);
    }

    // 解锁超时
    if ((ros::Time::now() - state_enter_time_).toSec() > config_.offboard_timeout) {
        ROS_ERROR("[Navigator] Arming timeout, returning to IDLE");
        requestArming(false);  // 尝试上锁
        transitionState(State::IDLE);
    }
}

void Navigator::handleTakeoff() {
    // Phase 1: 预发布setpoint（PX4需要持续收到>2Hz setpoint流才能接受OFFBOARD模式）
    if (!offboard_pre_pub_complete_) {
        // 监控 setpoint 发布率
        ros::Duration elapsed = ros::Time::now() - setpoint_rate_check_start_;
        if (elapsed.toSec() >= 1.0) {
            double actual_rate = setpoint_pub_count_ / elapsed.toSec();
            if (actual_rate < config_.min_setpoint_rate_hz) {
                ROS_ERROR_THROTTLE(2.0, "[Navigator] Setpoint stream too low: %.1f Hz (min %.1f Hz), cannot enter OFFBOARD",
                                   actual_rate, config_.min_setpoint_rate_hz);
            }
            setpoint_pub_count_ = 0;
            setpoint_rate_check_start_ = ros::Time::now();
        }
        // 预发布超时保护：预发布最长时间不应超过 offboard_timeout
        if ((ros::Time::now() - state_enter_time_).toSec() > config_.offboard_timeout) {
            ROS_ERROR("[Navigator] Pre-publish phase timeout (%.1f s), setpoint stream may be too slow, returning to IDLE",
                      config_.offboard_timeout);
            transitionState(State::IDLE);
        }
        return;
    }

    // Phase 2: 预发布完成后，请求 OFFBOARD 模式
    if (mavros_state_.mode != "OFFBOARD") {
        if ((ros::Time::now() - last_mode_request_time_).toSec() > config_.mode_retry_interval) {
            if (requestMode("OFFBOARD")) {
                ROS_INFO("[Navigator] OFFBOARD mode request sent");
            }
            last_mode_request_time_ = ros::Time::now();
        }
    } else {
        // 已在 OFFBOARD 模式，检查是否到达起飞目标点（使用 isWaypointReached 统一判断）
        if (has_odom_) {
            if (isWaypointReached()) {
                ROS_INFO("[Navigator] Takeoff complete, current position: (%.2f, %.2f, %.2f)",
                         current_odom_.pose.pose.position.x,
                         current_odom_.pose.pose.position.y,
                         current_odom_.pose.pose.position.z);
                transitionState(State::NAVIGATING);
                return;
            } else {
                ROS_INFO_THROTTLE(1.0, "[Navigator] Takeoff in progress, current: (%.2f, %.2f, %.2f), target: (%.2f, %.2f, %.2f)",
                                  current_odom_.pose.pose.position.x,
                                  current_odom_.pose.pose.position.y,
                                  current_odom_.pose.pose.position.z,
                                  current_setpoint_.pose.position.x,
                                  current_setpoint_.pose.position.y,
                                  current_setpoint_.pose.position.z);
            }
        }
    }

    // OFFBOARD模式请求超时检查（从进入 TAKEOFF 状态开始计时，非每次重试刷新）
    if ((ros::Time::now() - state_enter_time_).toSec() > config_.takeoff_timeout) {
        ROS_ERROR("[Navigator] Takeoff timeout (%.1f s overall since entering TAKEOFF), starting landing",
                  config_.takeoff_timeout);
        transitionState(State::LANDING);
    }
}

void Navigator::handleNavigating() {
    // 模式异常检测：飞行中模式被切出 OFFBOARD
    if (mavros_state_.mode != "OFFBOARD") {
        if (mode_loss_time_.isZero()) {
            mode_loss_time_ = ros::Time::now();
            ROS_WARN("[Navigator] Mode lost while navigating: %s", mavros_state_.mode.c_str());
        } else if ((ros::Time::now() - mode_loss_time_).toSec() > config_.mode_mismatch_tolerance) {
            ROS_ERROR("[Navigator] Mode not restored to OFFBOARD for %.1f s, triggering emergency",
                      config_.mode_mismatch_tolerance);
            enterEmergency("Mode lost in flight");
            return;
        }
    } else {
        mode_loss_time_ = ros::Time(0);
    }

    if (!has_waypoints_) {
        ROS_ERROR("[Navigator] Lost waypoint data during navigation, starting landing");
        transitionState(State::LANDING);
        return;
    }

    if (current_waypoint_idx_ >= waypoints_.poses.size()) {
        ROS_INFO("[Navigator] All waypoints completed, starting landing");
        transitionState(State::LANDING);
        return;
    }

    // 注：setpoint 由 setpointTimerCallback 统一发布（20Hz），此处不做重复发布

    // 记录真实轨迹
    if ((ros::Time::now() - last_real_path_sample_time_).toSec() >= config_.real_path_sample_interval) {
        appendRealPath();
        last_real_path_sample_time_ = ros::Time::now();
    }

    // 检查是否到达当前航点
    if (isWaypointReached()) {
        ROS_INFO("[Navigator] Reached waypoint %zu/%zu", current_waypoint_idx_ + 1, waypoints_.poses.size());

        if (current_waypoint_idx_ + 1 < waypoints_.poses.size()) {
            // 还有下一个航点，先悬停
            transitionState(State::HOVERING);
        } else {
            // 最后一个航点，直接降落
            ROS_INFO("[Navigator] Last waypoint reached, starting landing");
            transitionState(State::LANDING);
        }
    }
}

void Navigator::handleHovering() {
    // 悬停：保持当前位置setpoint（由 setpointTimerCallback 统一发布）
    // 注：setpoint 不再在此处重复发布

    // 检查悬停时间
    if ((ros::Time::now() - hover_start_time_).toSec() >= config_.hover_duration) {
        // 悬停完成，切换到下一个航点
        current_waypoint_idx_++;
        if (current_waypoint_idx_ < waypoints_.poses.size()) {
            transitionState(State::NAVIGATING);
        } else {
            transitionState(State::LANDING);
        }
    }
}

void Navigator::handleLanding() {
    // 注：setpoint 由 setpointTimerCallback 统一发布

    // 请求 AUTO.LAND 模式（可选，PX4会自动处理）
    if (mavros_state_.mode != "AUTO.LAND" && mavros_state_.mode != "OFFBOARD") {
        if (!requestMode("AUTO.LAND")) {
            ROS_WARN_THROTTLE(2.0, "[Navigator] Failed to request AUTO.LAND mode during landing");
        }
    }

    // 检查是否降落完成
    if (has_odom_) {
        if (current_odom_.pose.pose.position.z < config_.landing_height_threshold) {
            ROS_INFO("[Navigator] Landing complete, current height: %.2f m", current_odom_.pose.pose.position.z);
            // 尝试上锁
            requestArming(false);
            transitionState(State::LANDED);
        }
    }

    // 降落超时检查（从配置读取，默认60秒）
    if ((ros::Time::now() - state_enter_time_).toSec() > config_.landing_timeout) {
        ROS_ERROR("[Navigator] Landing timeout (%.1f s), forcing LANDED state", config_.landing_timeout);
        transitionState(State::LANDED);
    }
}

void Navigator::handleLanded() {
    // 已降落状态，不再发布setpoint
    ROS_INFO_THROTTLE(10.0, "[Navigator] Landed, waiting for RESET command. Send RESET to start a new mission.");
}

void Navigator::handleEmergency() {
    // 紧急状态：更新setpoint到当前位置（防止漂移），由 setpointTimerCallback 统一发布
    if (has_odom_) {
        updateSetpointFromCurrentPosition();
    }

    // 限制模式请求频率（每秒最多一次）
    if ((ros::Time::now() - last_emergency_mode_req_time_).toSec() > 1.0) {
        if (!requestMode("AUTO.LAND")) {
            ROS_WARN_THROTTLE(5.0, "[Navigator] Failed to request AUTO.LAND in emergency");
        }
        last_emergency_mode_req_time_ = ros::Time::now();
    }

    // 尝试上锁（如果已降落）
    if (has_odom_ && current_odom_.pose.pose.position.z < config_.landing_height_threshold) {
        if (!mavros_state_.armed) {
            // 已经上锁，可以自动转为LANDED
            ROS_WARN("[Navigator] Landed and disarmed in emergency state, auto-transitioning to LANDED state");
            transitionState(State::LANDED);
        } else {
            requestArming(false);
        }
    }

    // 紧急状态超时检查（从配置读取，默认120秒）
    if ((ros::Time::now() - emergency_time_).toSec() > config_.emergency_timeout) {
        ROS_ERROR("[Navigator] Emergency state timeout (%.1f s), forcing LANDED state", config_.emergency_timeout);
        transitionState(State::LANDED);
    }
}

void Navigator::handleReturning() {
    // setpoint 由 setpointTimerCallback 统一发布（20Hz），此处不重复发布，仅做位置判断和超时检测
    // 检查是否到达home位置
    if (has_odom_ && has_home_position_) {
        double dx = fabs(current_odom_.pose.pose.position.x - home_position_.position.x);
        double dy = fabs(current_odom_.pose.pose.position.y - home_position_.position.y);
        double dz = fabs(current_odom_.pose.pose.position.z - home_position_.position.z);

        if (dx < config_.reach_threshold_xy && dy < config_.reach_threshold_xy && dz < config_.reach_threshold_z) {
            ROS_INFO("[Navigator] Return to home complete, starting landing");
            transitionState(State::LANDING);
            return;
        }
    }

    // 返航超时保护：如果超过起飞超时*3 仍未到达home，强制降落
    double return_timeout = config_.takeoff_timeout * 3.0;
    if ((ros::Time::now() - state_enter_time_).toSec() > return_timeout) {
        ROS_ERROR("[Navigator] Return to home timeout (%.1f s), forcing landing", return_timeout);
        transitionState(State::LANDING);
    }
}

// ========== setpoint 定时器 ==========

void Navigator::setpointTimerCallback(const ros::TimerEvent& event) {
    // 在锁内复制需要的数据，锁外发布（最小化锁持有时间，保证 20Hz setpoint 流不被阻塞）
    geometry_msgs::PoseStamped setpoint_to_publish;
    bool should_publish = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 在 IDLE 和 PRE_FLIGHT 状态也需要发布setpoint以保持心跳
        if (nav_state_ == State::IDLE || nav_state_ == State::PRE_FLIGHT || nav_state_ == State::ARMING) {
            if (has_odom_) {
                updateSetpointFromCurrentPosition();
            }
            setpoint_to_publish = current_setpoint_;
            should_publish = true;
        }

        // TAKEOFF 状态：持续发布setpoint，同时计数预发布（PX4需要>2Hz持续流才能接受OFFBOARD）
        if (nav_state_ == State::TAKEOFF) {
            if (!offboard_pre_pub_complete_ && pre_pub_count_ > 0) {
                pre_pub_count_--;
                if (pre_pub_count_ == 0) {
                    offboard_pre_pub_complete_ = true;
                    ROS_INFO("[Navigator] OFFBOARD pre-publish complete (%d setpoints at %.1fHz, ~%.1f seconds), now requesting OFFBOARD mode",
                             config_.offboard_pre_pub_count, config_.setpoint_rate,
                             static_cast<double>(config_.offboard_pre_pub_count) / config_.setpoint_rate);
                }
            }
            setpoint_to_publish = current_setpoint_;
            should_publish = true;
        }

        // 其他飞行状态由定时器统一发布setpoint
        if (nav_state_ == State::NAVIGATING || nav_state_ == State::HOVERING ||
            nav_state_ == State::LANDING || nav_state_ == State::EMERGENCY || nav_state_ == State::RETURNING) {
            setpoint_to_publish = current_setpoint_;
            should_publish = true;
        }

        // 调试日志（默认关闭，需配置 rosconsole 级别为 DEBUG 才输出）
        ROS_DEBUG_THROTTLE(5.0, "[Navigator] state=%s setpoint=(%.2f, %.2f, %.2f)",
                 stateToString(nav_state_),
                 current_setpoint_.pose.position.x,
                 current_setpoint_.pose.position.y,
                 current_setpoint_.pose.position.z);

        // 记录 setpoint 发布时间并计数
        last_setpoint_pub_time_ = ros::Time::now();
        setpoint_pub_count_++;

        // setpoint 流健康检查（仅在非 IDLE/PRE_FLIGHT 且未紧急时）
        if (nav_state_ != State::IDLE && nav_state_ != State::PRE_FLIGHT && nav_state_ != State::EMERGENCY && nav_state_ != State::LANDED) {
            ros::Duration elapsed = ros::Time::now() - setpoint_rate_check_start_;
            if (elapsed.toSec() >= 1.0) {
                double actual_rate = setpoint_pub_count_ / elapsed.toSec();
                if (actual_rate < config_.min_setpoint_rate_hz) {
                    ROS_ERROR_THROTTLE(2.0, "[Navigator] Setpoint stream unhealthy: %.1f Hz (min %.1f Hz)",
                                       actual_rate, config_.min_setpoint_rate_hz);
                }
                setpoint_pub_count_ = 0;
                setpoint_rate_check_start_ = ros::Time::now();
            }
        }
    }
    // 锁外发布，不阻塞其他回调
    if (should_publish) {
        setpoint_pub_.publish(setpoint_to_publish);
    }
}

// ========== 辅助函数 ==========

bool Navigator::requestArming(bool arm) {
    // 检查服务是否可用
    if (!arming_client_.exists()) {
        ROS_ERROR_THROTTLE(5.0, "[Navigator] Arming service unavailable: %s", config_.arming_service.c_str());
        return false;
    }

    arm_request_count_++;
    mavros_msgs::CommandBool srv;
    srv.request.value = arm;
    if (arming_client_.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("[Navigator] %s request successful (attempts: %d)", arm ? "Arm" : "Disarm", arm_request_count_);
            return true;
        } else {
            ROS_WARN("[Navigator] %s request denied (attempts: %d)", arm ? "Arm" : "Disarm", arm_request_count_);
            return false;
        }
    } else {
        ROS_ERROR("[Navigator] %s service call failed (attempts: %d)", arm ? "Arm" : "Disarm", arm_request_count_);
        return false;
    }
}

bool Navigator::requestMode(const std::string& mode) {
    // 检查服务是否可用
    if (!set_mode_client_.exists()) {
        ROS_ERROR_THROTTLE(5.0, "[Navigator] Mode switch service unavailable: %s", config_.set_mode_service.c_str());
        return false;
    }

    mode_request_count_++;
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode;
    if (set_mode_client_.call(srv)) {
        if (srv.response.mode_sent) {
            ROS_INFO("[Navigator] Mode switch request sent: %s (attempts: %d)", mode.c_str(), mode_request_count_);
            return true;
        } else {
            ROS_WARN("[Navigator] Mode switch request denied: %s (attempts: %d)", mode.c_str(), mode_request_count_);
            return false;
        }
    } else {
        ROS_ERROR("[Navigator] Mode switch service call failed: %s (attempts: %d)", mode.c_str(), mode_request_count_);
        return false;
    }
}

bool Navigator::isWaypointReached() {
    if (!has_odom_) return false;

    double dx = fabs(current_odom_.pose.pose.position.x - current_setpoint_.pose.position.x);
    double dy = fabs(current_odom_.pose.pose.position.y - current_setpoint_.pose.position.y);
    double dz = fabs(current_odom_.pose.pose.position.z - current_setpoint_.pose.position.z);

    return (dx < config_.reach_threshold_xy && dy < config_.reach_threshold_xy && dz < config_.reach_threshold_z);
}

void Navigator::publishStatus() {
    uav_navigator::NavigatorStatus msg;
    msg.state = static_cast<uint8_t>(nav_state_);

    // 无航点时，索引统一显示为 0，避免 logger 显示 1/0 的误导
    if (waypoints_.poses.empty()) {
        msg.current_waypoint_index = 0;
        msg.total_waypoints = 0;
    } else {
        msg.current_waypoint_index = static_cast<uint8_t>(
            std::min(current_waypoint_idx_, static_cast<size_t>(255)));
        msg.total_waypoints = static_cast<uint8_t>(
            std::min(waypoints_.poses.size(), static_cast<size_t>(255)));
    }

    if (has_odom_) {
        msg.current_x = current_odom_.pose.pose.position.x;
        msg.current_y = current_odom_.pose.pose.position.y;
        msg.current_z = current_odom_.pose.pose.position.z;
    } else {
        msg.current_x = msg.current_y = msg.current_z = 0.0f;
    }

    // 无航点且处于 IDLE/LANDED 等状态时，目标位置标记为无效（NaN）
    if (waypoints_.poses.empty() &&
        (nav_state_ == State::IDLE || nav_state_ == State::LANDED || nav_state_ == State::EMERGENCY)) {
        msg.target_x = std::numeric_limits<float>::quiet_NaN();
        msg.target_y = std::numeric_limits<float>::quiet_NaN();
        msg.target_z = std::numeric_limits<float>::quiet_NaN();
    } else {
        msg.target_x = current_setpoint_.pose.position.x;
        msg.target_y = current_setpoint_.pose.position.y;
        msg.target_z = current_setpoint_.pose.position.z;
    }

    msg.is_armed = mavros_state_.armed;
    msg.current_mode = mavros_state_.mode;
    msg.error_message = emergency_reason_;

    status_pub_.publish(msg);
}

void Navigator::setSetpoint(const geometry_msgs::Pose& pose) {
    current_setpoint_.header.stamp = ros::Time::now();
    current_setpoint_.header.frame_id = config_.default_frame_id;
    current_setpoint_.pose = pose;
}

void Navigator::setSetpointXYZ(double x, double y, double z) {
    current_setpoint_.header.stamp = ros::Time::now();
    current_setpoint_.header.frame_id = config_.default_frame_id;
    current_setpoint_.pose.position.x = x;
    current_setpoint_.pose.position.y = y;
    current_setpoint_.pose.position.z = z;
    current_setpoint_.pose.orientation.x = 0.0;
    current_setpoint_.pose.orientation.y = 0.0;
    current_setpoint_.pose.orientation.z = 0.0;
    current_setpoint_.pose.orientation.w = 1.0;
}

void Navigator::updateSetpointFromCurrentPosition() {
    current_setpoint_.header.stamp = ros::Time::now();
    current_setpoint_.header.frame_id = config_.default_frame_id;
    if (has_odom_) {
        current_setpoint_.pose = current_odom_.pose.pose;
    } else {
        // 没有位置数据时，保持起飞高度，避免发布 z=0 导致异常
        current_setpoint_.pose.position.x = 0.0;
        current_setpoint_.pose.position.y = 0.0;
        current_setpoint_.pose.position.z = config_.takeoff_height;
        current_setpoint_.pose.orientation.x = 0.0;
        current_setpoint_.pose.orientation.y = 0.0;
        current_setpoint_.pose.orientation.z = 0.0;
        current_setpoint_.pose.orientation.w = 1.0;
    }
}

bool Navigator::checkPreFlight() {
    preflight_fail_count_++;

    // 检查MAVROS连接
    if (!mavros_state_.connected) {
        ROS_WARN_THROTTLE(2.0, "[Navigator] Pre-flight check: waiting for MAVROS connection... (check count: %d)", preflight_fail_count_);
        return false;
    }

    // 检查是否有位置数据
    if (!has_odom_) {
        ROS_WARN_THROTTLE(2.0, "[Navigator] Pre-flight check: waiting for position data... (check count: %d)", preflight_fail_count_);
        return false;
    }

    // 检查位置数据是否有效（使用容差比较，避免浮点精度问题）
    if (std::abs(current_odom_.pose.pose.position.z) < 1e-6) {
        ROS_WARN_THROTTLE(2.0, "[Navigator] Pre-flight check: position height near zero, sensor may not be initialized...");
        return false;
    }

    // 检查是否有航点
    if (!has_waypoints_) {
        ROS_WARN_THROTTLE(2.0, "[Navigator] Pre-flight check: waiting for waypoint data... (check count: %d)", preflight_fail_count_);
        return false;
    }

    // 检查航点数量是否合理
    if (waypoints_.poses.empty()) {
        ROS_ERROR("[Navigator] Pre-flight check: waypoint list is empty");
        return false;
    }

    // 检查航点z值是否超过安全限制（警告但不阻止）
    for (size_t i = 0; i < waypoints_.poses.size(); ++i) {
        if (waypoints_.poses[i].position.z > config_.max_height_limit) {
            ROS_WARN("[Navigator] Waypoint %zu height (%.2f m) exceeds safety limit (%.2f m),"
                     "please check safety/max_height_limit setting in navigator_config.yaml",
                     i + 1, waypoints_.poses[i].position.z, config_.max_height_limit);
        }
    }

    // 检查当前高度是否已远超起飞高度（安全风险：若已在50m高处起飞，起飞后setpoint会突变到起飞高度导致骤降）
    double current_z = current_odom_.pose.pose.position.z;
    double height_margin = std::max(3.0, config_.takeoff_height * 2.0);
    if (current_z > config_.takeoff_height + height_margin) {
        ROS_ERROR("[Navigator] Current height (%.2f m) is %0.1f m above takeoff height (%.2f m). "
                  "This would cause a rapid descent! Please land manually before starting.",
                  current_z, current_z - config_.takeoff_height, config_.takeoff_height);
        return false;  // 阻止起飞
    } else if (current_z > config_.takeoff_height + 2.0) {
        ROS_WARN("[Navigator] Current height (%.2f m) exceeds takeoff height (%.2f m) + 2m,"
                 " please manually land near takeoff height before starting navigation",
                 current_z, config_.takeoff_height);
        // 警告但不阻止（2-3米的高度差是可接受的）
    }

    ROS_INFO("[Navigator] Pre-flight check passed: MAVROS connected, position valid, %zu waypoints", waypoints_.poses.size());
    preflight_fail_count_ = 0;  // 重置计数
    return true;
}

void Navigator::enterEmergency(const std::string& reason) {
    if (nav_state_ == State::EMERGENCY) return;

    emergency_triggered_ = true;
    emergency_reason_ = reason;
    emergency_time_ = ros::Time::now();
    ROS_ERROR("[Navigator] Emergency state triggered! Reason: %s", reason.c_str());
    transitionState(State::EMERGENCY);
}

void Navigator::run() {
    ROS_INFO("[Navigator] Entering main loop");
    // 主线程休眠等待 shutdown，实际工作由 AsyncSpinner 驱动
    ros::waitForShutdown();
}

} // namespace uav_navigator

// 轨迹与实验指标辅助函数实现
namespace uav_navigator {

void Navigator::buildPlannedPath() {
    planned_path_.poses.clear();
    planned_path_.header.stamp = ros::Time::now();
    planned_path_.header.frame_id = config_.default_frame_id;

    if (!has_waypoints_ || waypoints_.poses.empty()) return;

    for (const auto& pose : waypoints_.poses) {
        geometry_msgs::PoseStamped ps;
        ps.header = planned_path_.header;
        ps.pose = pose;
        planned_path_.poses.push_back(ps);
    }
    // 不再单独发布 planned_path，面板通过 uav/plan_maker/trajectory 已发布相同内容
    // 保留本地数据用于 computePlanDeviation() 偏差计算
}

void Navigator::appendRealPath() {
    if (!has_odom_) return;

    real_path_.header.stamp = ros::Time::now();
    real_path_.header.frame_id = config_.default_frame_id;

    geometry_msgs::PoseStamped ps;
    ps.header = real_path_.header;
    ps.pose = current_odom_.pose.pose;
    real_path_.poses.push_back(ps);

    // 限制最大点数，FIFO 淘汰旧数据，防止 RViz 长期运行后卡顿
    // 以 10Hz 采样率计，500 点 ≈ 50 秒可见历史，足够调试
    const size_t max_points = static_cast<size_t>(std::max(config_.max_real_path_points, 50));
    while (real_path_.poses.size() > max_points) {
        real_path_.poses.erase(real_path_.poses.begin());
    }
}

double Navigator::computePlanDeviation() {
    if (!has_odom_ || planned_path_.poses.empty()) {
        return -1.0;
    }

    // 找到 planned path 上最近的点
    double min_dist = std::numeric_limits<double>::max();
    size_t nearest_idx = 0;
    for (size_t i = 0; i < planned_path_.poses.size(); ++i) {
        double dx = current_odom_.pose.pose.position.x - planned_path_.poses[i].pose.position.x;
        double dy = current_odom_.pose.pose.position.y - planned_path_.poses[i].pose.position.y;
        double dz = current_odom_.pose.pose.position.z - planned_path_.poses[i].pose.position.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }

    // 计算到 nearest segment 的垂距（简化：用最近点近似）
    double dx = current_odom_.pose.pose.position.x - planned_path_.poses[nearest_idx].pose.position.x;
    double dy = current_odom_.pose.pose.position.y - planned_path_.poses[nearest_idx].pose.position.y;
    double dz = current_odom_.pose.pose.position.z - planned_path_.poses[nearest_idx].pose.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void Navigator::publishMetrics() {
    uav_navigator::ExperimentMetrics msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = config_.default_frame_id;

    msg.nav_state = static_cast<uint8_t>(nav_state_);
    msg.current_waypoint_index = 0;
    msg.total_waypoints = 0;
    if (!waypoints_.poses.empty()) {
        msg.current_waypoint_index = static_cast<uint8_t>(
            std::min(current_waypoint_idx_, static_cast<size_t>(255)));
        msg.total_waypoints = static_cast<uint8_t>(
            std::min(waypoints_.poses.size(), static_cast<size_t>(255)));
    }

    if (has_odom_) {
        msg.current_x = current_odom_.pose.pose.position.x;
        msg.current_y = current_odom_.pose.pose.position.y;
        msg.current_z = current_odom_.pose.pose.position.z;
    } else {
        msg.current_x = msg.current_y = msg.current_z = 0.0f;
    }

    msg.target_x = current_setpoint_.pose.position.x;
    msg.target_y = current_setpoint_.pose.position.y;
    msg.target_z = current_setpoint_.pose.position.z;

    if (waypoints_.poses.empty() &&
        (nav_state_ == State::IDLE || nav_state_ == State::LANDED || nav_state_ == State::EMERGENCY)) {
        msg.target_x = std::numeric_limits<float>::quiet_NaN();
        msg.target_y = std::numeric_limits<float>::quiet_NaN();
        msg.target_z = std::numeric_limits<float>::quiet_NaN();
    }

    msg.is_armed = mavros_state_.armed;
    msg.current_mode = mavros_state_.mode;

    double deviation = computePlanDeviation();
    if (deviation >= 0.0) {
        msg.plan_deviation_x = current_odom_.pose.pose.position.x - current_setpoint_.pose.position.x;
        msg.plan_deviation_y = current_odom_.pose.pose.position.y - current_setpoint_.pose.position.y;
        msg.plan_deviation_z = current_odom_.pose.pose.position.z - current_setpoint_.pose.position.z;
        msg.plan_deviation_total = deviation;
    } else {
        msg.plan_deviation_x = msg.plan_deviation_y = msg.plan_deviation_z = msg.plan_deviation_total = 0.0f;
    }

    // time_since_start: 从任务开始（进入 PRE_FLIGHT）到现在的总时间
    msg.time_since_start = mission_start_time_.toSec() > 0.0
        ? (ros::Time::now() - mission_start_time_).toSec() : 0.0;
    // time_in_current_state: 在当前状态中停留的时间
    msg.time_in_current_state = (ros::Time::now() - state_enter_time_).toSec();

    metrics_pub_.publish(msg);
}

} // namespace uav_navigator

// ========== main ==========
int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_navigator");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        uav_navigator::Navigator navigator(nh, pnh);
        // 使用多线程 spinner，避免定时器回调阻塞订阅回调（特别是 odom）
        ros::AsyncSpinner spinner(2);
        spinner.start();
        navigator.run();
        spinner.stop();
    } catch (const std::exception& e) {
        ROS_ERROR("[Navigator] Exception: %s", e.what());
        return 1;
    }

    return 0;
}
