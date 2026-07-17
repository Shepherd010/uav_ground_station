#include <ros/ros.h>
#include <rosbag/bag.h>
#include <ros/time.h>

#include <uav_navigator/ExperimentMetrics.h>
#include <uav_navigator/NavigatorStatus.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <cerrno>
#include <cstring>
#include <sys/stat.h>
#include <map>
#include <sys/stat.h>

namespace uav_navigator {

class ExperimentRecorder {
public:
    ExperimentRecorder(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~ExperimentRecorder();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Subscribers
    ros::Subscriber metrics_sub_;
    ros::Subscriber status_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber setpoint_sub_;
    ros::Subscriber mavros_state_sub_;
    ros::Subscriber safety_alert_sub_;
    ros::Subscriber planned_path_sub_;
    ros::Subscriber real_path_sub_;
    ros::Subscriber record_control_sub_;

    // Cached data
    uav_navigator::ExperimentMetrics latest_metrics_;
    uav_navigator::NavigatorStatus latest_status_;
    nav_msgs::Odometry latest_odom_;
    geometry_msgs::PoseStamped latest_setpoint_;
    mavros_msgs::State latest_mavros_state_;
    bool has_metrics_;
    bool has_status_;
    bool has_odom_;
    bool has_setpoint_;
    bool has_mavros_state_;

    // Recording state
    bool is_recording_;
    bool auto_record_;
    std::unique_ptr<rosbag::Bag> bag_;
    std::ofstream metrics_csv_;
    std::ofstream trajectory_real_csv_;
    std::ofstream summary_txt_;
    std::string experiment_dir_;
    std::string bag_path_;
    ros::Time record_start_time_;
    ros::Time mission_start_time_;

    // Metrics accumulation
    double max_deviation_;
    double cumulative_deviation_;
    uint64_t deviation_sample_count_;
    std::map<uint8_t, double> state_durations_;
    uint8_t last_nav_state_;
    ros::Time last_state_time_;

    // Config
    struct Config {
        std::string metrics_topic;
        std::string status_topic;
        std::string odom_topic;
        std::string setpoint_topic;
        std::string mavros_state_topic;
        std::string safety_alert_topic;
        std::string planned_path_topic;
        std::string real_path_topic;
        std::string record_control_topic;
        std::string output_dir;
        bool auto_record;
    } config_;

    void loadConfig();
    void ensureDirectory(const std::string& path);
    std::string getNextExperimentDir();
    void startRecording();
    void stopRecording();
    void writeMetadata();
    void writeSummary();
    void writeCsvHeader();

    void metricsCallback(const uav_navigator::ExperimentMetrics::ConstPtr& msg);
    void statusCallback(const uav_navigator::NavigatorStatus::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    void safetyAlertCallback(const std_msgs::String::ConstPtr& msg);
    void plannedPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void realPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void recordControlCallback(const std_msgs::Bool::ConstPtr& msg);

    std::string stateToString(uint8_t state);
    std::string getTimestamp();
};

ExperimentRecorder::ExperimentRecorder(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), is_recording_(false), auto_record_(false),
      has_metrics_(false), has_status_(false), has_odom_(false),
      has_setpoint_(false), has_mavros_state_(false),
      max_deviation_(0.0), cumulative_deviation_(0.0), deviation_sample_count_(0),
      last_nav_state_(0) {
    loadConfig();

    metrics_sub_ = nh_.subscribe(config_.metrics_topic, 10, &ExperimentRecorder::metricsCallback, this);
    status_sub_ = nh_.subscribe(config_.status_topic, 10, &ExperimentRecorder::statusCallback, this);
    odom_sub_ = nh_.subscribe(config_.odom_topic, 10, &ExperimentRecorder::odomCallback, this);
    setpoint_sub_ = nh_.subscribe(config_.setpoint_topic, 10, &ExperimentRecorder::setpointCallback, this);
    mavros_state_sub_ = nh_.subscribe(config_.mavros_state_topic, 10, &ExperimentRecorder::mavrosStateCallback, this);
    safety_alert_sub_ = nh_.subscribe(config_.safety_alert_topic, 10, &ExperimentRecorder::safetyAlertCallback, this);
    planned_path_sub_ = nh_.subscribe(config_.planned_path_topic, 1, &ExperimentRecorder::plannedPathCallback, this);
    real_path_sub_ = nh_.subscribe(config_.real_path_topic, 10, &ExperimentRecorder::realPathCallback, this);
    record_control_sub_ = nh_.subscribe(config_.record_control_topic, 1, &ExperimentRecorder::recordControlCallback, this);

    ROS_INFO("[ExperimentRecorder] Initialized, output dir: %s", config_.output_dir.c_str());
    if (config_.auto_record) {
        ROS_INFO("[ExperimentRecorder] Auto-record enabled: will start when navigator enters PRE_FLIGHT");
    }
}

ExperimentRecorder::~ExperimentRecorder() {
    if (is_recording_) {
        stopRecording();
    }
}

void ExperimentRecorder::loadConfig() {
    // 使用全局命名空间读取参数
    ros::NodeHandle global_nh;
    global_nh.param<std::string>("experiment/metrics_topic", config_.metrics_topic, "uav/experiment/metrics");
    global_nh.param<std::string>("experiment/status_topic", config_.status_topic, "uav/navigator/status");
    global_nh.param<std::string>("experiment/odom_topic", config_.odom_topic, "mavros/local_position/odom");
    global_nh.param<std::string>("experiment/setpoint_topic", config_.setpoint_topic, "mavros/setpoint_position/local");
    global_nh.param<std::string>("experiment/mavros_state_topic", config_.mavros_state_topic, "mavros/state");
    global_nh.param<std::string>("experiment/safety_alert_topic", config_.safety_alert_topic, "uav/safety/alert");
    global_nh.param<std::string>("experiment/planned_path_topic", config_.planned_path_topic, "uav/trajectory/planned");
    global_nh.param<std::string>("experiment/real_path_topic", config_.real_path_topic, "uav/trajectory/real");
    global_nh.param<std::string>("experiment/record_control_topic", config_.record_control_topic, "uav/experiment/record");
    global_nh.param<std::string>("experiment/output_dir", config_.output_dir, "/home/groundstation/experiments");
    global_nh.param<bool>("experiment/auto_record", config_.auto_record, true);
    auto_record_ = config_.auto_record;
}

void ExperimentRecorder::ensureDirectory(const std::string& path) {
    size_t pos = 0;
    while ((pos = path.find('/', pos + 1)) != std::string::npos) {
        std::string sub = path.substr(0, pos);
        if (mkdir(sub.c_str(), 0755) != 0 && errno != EEXIST) {
            ROS_ERROR("[ExperimentRecorder] Failed to create directory %s: %s", sub.c_str(), strerror(errno));
            throw std::runtime_error("Cannot create experiment output directory: " + sub);
        }
    }
    if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) {
        ROS_ERROR("[ExperimentRecorder] Failed to create directory %s: %s", path.c_str(), strerror(errno));
        throw std::runtime_error("Cannot create experiment output directory: " + path);
    }
}

std::string ExperimentRecorder::getTimestamp() {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

std::string ExperimentRecorder::stateToString(uint8_t state) {
    switch (state) {
        case uav_navigator::NavigatorStatus::STATE_IDLE:           return "IDLE";
        case uav_navigator::NavigatorStatus::STATE_PRE_FLIGHT:     return "PRE_FLIGHT";
        case uav_navigator::NavigatorStatus::STATE_ARMING:         return "ARMING";
        case uav_navigator::NavigatorStatus::STATE_TAKEOFF:        return "TAKEOFF";
        case uav_navigator::NavigatorStatus::STATE_NAVIGATING:     return "NAVIGATING";
        case uav_navigator::NavigatorStatus::STATE_HOVERING:       return "HOVERING";
        case uav_navigator::NavigatorStatus::STATE_LANDING:        return "LANDING";
        case uav_navigator::NavigatorStatus::STATE_LANDED:         return "LANDED";
        case uav_navigator::NavigatorStatus::STATE_EMERGENCY:      return "EMERGENCY";
        case uav_navigator::NavigatorStatus::STATE_RETURNING:      return "RETURNING";
        default: return "UNKNOWN";
    }
}

std::string ExperimentRecorder::getNextExperimentDir() {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream date_oss;
    date_oss << std::put_time(&tm, "%Y-%m-%d");
    std::string date_dir = config_.output_dir + "/" + date_oss.str();
    ensureDirectory(date_dir);

    int index = 1;
    const int MAX_INDEX = 9999;
    std::string exp_dir;
    while (index <= MAX_INDEX) {
        std::ostringstream oss;
        oss << date_dir << "/experiment_" << std::setfill('0') << std::setw(3) << index;
        exp_dir = oss.str();
        struct stat st;
        if (stat(exp_dir.c_str(), &st) != 0) {
            break;
        }
        index++;
    }
    if (index > MAX_INDEX) {
        ROS_ERROR("[ExperimentRecorder] Experiment directory index overflow (>%d)", MAX_INDEX);
        return "";
    }
    return exp_dir;
}

void ExperimentRecorder::startRecording() {
    if (is_recording_) return;

    experiment_dir_ = getNextExperimentDir();
    ensureDirectory(experiment_dir_);
    bag_path_ = experiment_dir_ + "/data.bag";

    try {
        bag_.reset(new rosbag::Bag);
        bag_->open(bag_path_, rosbag::bagmode::Write);
    } catch (const std::exception& e) {
        ROS_ERROR("[ExperimentRecorder] Failed to open bag: %s", e.what());
        return;
    }

    metrics_csv_.open(experiment_dir_ + "/metrics.csv");
    trajectory_real_csv_.open(experiment_dir_ + "/trajectory_real.csv");
    summary_txt_.open(experiment_dir_ + "/summary.txt");

    writeCsvHeader();
    summary_txt_ << "UAV Experiment Summary\n";
    summary_txt_ << "======================\n";
    summary_txt_ << "Start: " << getTimestamp() << "\n\n";

    record_start_time_ = ros::Time::now();
    last_state_time_ = record_start_time_;
    mission_start_time_ = ros::Time(0);
    max_deviation_ = 0.0;
    cumulative_deviation_ = 0.0;
    deviation_sample_count_ = 0;
    state_durations_.clear();

    is_recording_ = true;
    ROS_INFO("[ExperimentRecorder] Started recording to %s", experiment_dir_.c_str());
}

void ExperimentRecorder::stopRecording() {
    if (!is_recording_) return;

    writeSummary();
    writeMetadata();

    if (bag_) {
        bag_->close();
        bag_.reset();
    }
    if (metrics_csv_.is_open()) metrics_csv_.close();
    if (trajectory_real_csv_.is_open()) trajectory_real_csv_.close();
    if (summary_txt_.is_open()) summary_txt_.close();

    is_recording_ = false;
    ROS_INFO("[ExperimentRecorder] Stopped recording. Output: %s", experiment_dir_.c_str());
}

void ExperimentRecorder::writeCsvHeader() {
    if (metrics_csv_.is_open()) {
        metrics_csv_ << "timestamp,elapsed_time,nav_state,current_wp,total_wp,"
                     << "current_x,current_y,current_z,target_x,target_y,target_z,"
                     << "deviation_total,mode,armed\n";
    }
    if (trajectory_real_csv_.is_open()) {
        trajectory_real_csv_ << "timestamp,x,y,z,yaw,vel_x,vel_y,vel_z\n";
    }
}

void ExperimentRecorder::writeMetadata() {
    std::ofstream meta(experiment_dir_ + "/metadata.yaml");
    if (!meta.is_open()) return;

    meta << "experiment:\n";
    meta << "  start_time: \"" << getTimestamp() << "\"\n";
    meta << "  output_dir: \"" << experiment_dir_ << "\"\n";
    meta << "  bag_file: \"" << bag_path_ << "\"\n";
    meta << "  total_duration_sec: " << (ros::Time::now() - record_start_time_).toSec() << "\n";
    meta << "  max_deviation_m: " << max_deviation_ << "\n";
    meta << "  avg_deviation_m: " << (deviation_sample_count_ > 0 ? cumulative_deviation_ / deviation_sample_count_ : 0.0) << "\n";
    meta << "  recorded_topics:\n";
    meta << "    - " << config_.metrics_topic << "\n";
    meta << "    - " << config_.status_topic << "\n";
    meta << "    - " << config_.odom_topic << "\n";
    meta << "    - " << config_.setpoint_topic << "\n";
    meta << "    - " << config_.mavros_state_topic << "\n";
    meta << "    - " << config_.safety_alert_topic << "\n";
    meta << "    - " << config_.planned_path_topic << "\n";
    meta << "    - " << config_.real_path_topic << "\n";
    meta.close();
}

void ExperimentRecorder::writeSummary() {
    if (!summary_txt_.is_open()) return;

    summary_txt_ << "\nEnd: " << getTimestamp() << "\n";
    summary_txt_ << "Duration: " << (ros::Time::now() - record_start_time_).toSec() << " sec\n";
    summary_txt_ << "Max deviation: " << max_deviation_ << " m\n";
    summary_txt_ << "Average deviation: " << (deviation_sample_count_ > 0 ? cumulative_deviation_ / deviation_sample_count_ : 0.0) << " m\n";
    summary_txt_ << "Samples: " << deviation_sample_count_ << "\n";
    summary_txt_ << "\nState durations:\n";
    for (const auto& kv : state_durations_) {
        summary_txt_ << "  " << stateToString(kv.first) << ": " << kv.second << " sec\n";
    }
    summary_txt_.flush();
}

void ExperimentRecorder::metricsCallback(const uav_navigator::ExperimentMetrics::ConstPtr& msg) {
    latest_metrics_ = *msg;
    has_metrics_ = true;

    // Update deviation statistics
    if (msg->plan_deviation_total >= 0.0) {
        max_deviation_ = std::max(max_deviation_, static_cast<double>(msg->plan_deviation_total));
        cumulative_deviation_ += msg->plan_deviation_total;
        deviation_sample_count_++;
    }

    // State tracking
    if (msg->nav_state != last_nav_state_) {
        ros::Time now = ros::Time::now();
        if (last_state_time_.toSec() > 0) {
            state_durations_[last_nav_state_] += (now - last_state_time_).toSec();
        }
        last_nav_state_ = msg->nav_state;
        last_state_time_ = now;

        // Auto start/stop
        if (auto_record_) {
            // Start when mission begins (PRE_FLIGHT or later active states). PRE_FLIGHT can be
            // very short, so we also accept any state between PRE_FLIGHT and LANDING.
            if (msg->nav_state >= uav_navigator::NavigatorStatus::STATE_PRE_FLIGHT
                && msg->nav_state <= uav_navigator::NavigatorStatus::STATE_LANDING
                && !is_recording_) {
                mission_start_time_ = ros::Time::now();
                startRecording();
            } else if ((msg->nav_state == uav_navigator::NavigatorStatus::STATE_LANDED
                        || msg->nav_state == uav_navigator::NavigatorStatus::STATE_EMERGENCY)
                       && is_recording_) {
                stopRecording();
            }
        }
    }

    if (!is_recording_) return;

    ros::Time now = ros::Time::now();
    bag_->write(config_.metrics_topic, now, *msg);

    if (metrics_csv_.is_open()) {
        double elapsed = (now - record_start_time_).toSec();
        metrics_csv_ << std::fixed << std::setprecision(3)
                     << now.toSec() << "," << elapsed << ","
                     << stateToString(msg->nav_state) << ","
                     << static_cast<int>(msg->current_waypoint_index) << ","
                     << static_cast<int>(msg->total_waypoints) << ","
                     << msg->current_x << "," << msg->current_y << "," << msg->current_z << ","
                     << msg->target_x << "," << msg->target_y << "," << msg->target_z << ","
                     << msg->plan_deviation_total << ","
                     << msg->current_mode << "," << (msg->is_armed ? "1" : "0") << "\n";
    }
}

void ExperimentRecorder::statusCallback(const uav_navigator::NavigatorStatus::ConstPtr& msg) {
    latest_status_ = *msg;
    has_status_ = true;
    if (is_recording_ && bag_) {
        bag_->write(config_.status_topic, ros::Time::now(), *msg);
    }
}

void ExperimentRecorder::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_odom_ = *msg;
    has_odom_ = true;
    if (is_recording_ && bag_) {
        bag_->write(config_.odom_topic, ros::Time::now(), *msg);
    }
    if (is_recording_ && trajectory_real_csv_.is_open()) {
        trajectory_real_csv_ << std::fixed << std::setprecision(3)
                             << msg->header.stamp.toSec() << ","
                             << msg->pose.pose.position.x << ","
                             << msg->pose.pose.position.y << ","
                             << msg->pose.pose.position.z << ","
                             << "0.0,"  // yaw placeholder
                             << msg->twist.twist.linear.x << ","
                             << msg->twist.twist.linear.y << ","
                             << msg->twist.twist.linear.z << "\n";
    }
}

void ExperimentRecorder::setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    latest_setpoint_ = *msg;
    has_setpoint_ = true;
    if (is_recording_ && bag_) {
        bag_->write(config_.setpoint_topic, ros::Time::now(), *msg);
    }
}

void ExperimentRecorder::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
    latest_mavros_state_ = *msg;
    has_mavros_state_ = true;
    if (is_recording_ && bag_) {
        bag_->write(config_.mavros_state_topic, ros::Time::now(), *msg);
    }
}

void ExperimentRecorder::safetyAlertCallback(const std_msgs::String::ConstPtr& msg) {
    if (is_recording_ && bag_) {
        bag_->write(config_.safety_alert_topic, ros::Time::now(), *msg);
    }
    if (summary_txt_.is_open()) {
        summary_txt_ << "[SAFETY] " << getTimestamp() << ": " << msg->data << "\n";
        summary_txt_.flush();
    }
}

void ExperimentRecorder::plannedPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (is_recording_ && bag_) {
        bag_->write(config_.planned_path_topic, ros::Time::now(), *msg);
    }
}

void ExperimentRecorder::realPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (is_recording_ && bag_) {
        bag_->write(config_.real_path_topic, ros::Time::now(), *msg);
    }
}

void ExperimentRecorder::recordControlCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        startRecording();
    } else {
        stopRecording();
    }
}

}  // namespace uav_navigator

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_experiment_recorder");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        uav_navigator::ExperimentRecorder recorder(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("[ExperimentRecorder] Exception: %s", e.what());
        return 1;
    }

    return 0;
}
