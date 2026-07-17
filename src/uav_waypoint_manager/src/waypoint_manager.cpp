#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <ctime>

#include "uav_waypoint_manager/LoadWaypoints.h"
#include "uav_waypoint_manager/SaveWaypoints.h"
#include "uav_waypoint_manager/ClearWaypoints.h"

namespace uav_waypoint_manager {

class WaypointManager {
public:
    WaypointManager(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~WaypointManager();
    void run();

private:
    // ROS 接口
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber waypoints_sub_;
    ros::Subscriber waypoint_params_sub_;
    ros::Publisher waypoints_pub_;
    ros::Publisher waypoint_params_current_pub_;  // 重命名以避免与订阅话题相同

    ros::ServiceServer load_srv_;
    ros::ServiceServer save_srv_;
    ros::ServiceServer clear_srv_;

    // 航点数据
    geometry_msgs::PoseArray waypoints_;
    bool has_waypoints_;

    // 每航点扩展参数（与 waypoints_.poses 按索引对应）
    struct WaypointParams {
        double hover_time;
        double speed;
    };
    std::vector<WaypointParams> waypoint_params_;

    // 全局默认值（从配置加载，用于回退）
    double default_hover_time_;
    double default_speed_;

    // 配置参数
    struct Config {
        std::string waypoint_input_topic;
        std::string waypoint_current_topic;
        std::string waypoint_params_input_topic;    // 从 panel 接收的 per-waypoint 参数
        std::string waypoint_params_current_topic;  // 发布存储的 per-waypoint 参数
        std::string load_service;
        std::string save_service;
        std::string clear_service;
        std::string default_save_path;
        std::string default_load_path;
        double min_waypoint_spacing;
        double max_height;
        double min_height;
        double publish_rate;
    } config_;

    // 方法
    void loadConfig();
    void initROS();
    void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void waypointParamsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    bool loadWaypointsCallback(uav_waypoint_manager::LoadWaypoints::Request& req,
                               uav_waypoint_manager::LoadWaypoints::Response& res);
    bool saveWaypointsCallback(uav_waypoint_manager::SaveWaypoints::Request& req,
                               uav_waypoint_manager::SaveWaypoints::Response& res);
    bool clearWaypointsCallback(uav_waypoint_manager::ClearWaypoints::Request& req,
                                uav_waypoint_manager::ClearWaypoints::Response& res);

    // XML operations
    bool saveToXml(const std::string& file_path);
    bool loadFromXml(const std::string& file_path);

    // waypoint validation
    bool validateWaypoints(std::string& error_msg);
    bool checkDuplicateWaypoints(std::string& error_msg);
    bool checkWaypointSpacing(std::string& error_msg);
    bool checkWaypointHeight(std::string& error_msg);

    // helper functions
    std::string getCurrentTimestamp();
    double distance3D(const geometry_msgs::Point& a, const geometry_msgs::Point& b);
    void publishWaypointParams();
    bool isValidCoordinate(double val);
};

WaypointManager::WaypointManager(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), has_waypoints_(false) {

    loadConfig();
    initROS();

    ROS_INFO("[WaypointManager] Initialization complete");
}

WaypointManager::~WaypointManager() {
    ROS_INFO("[WaypointManager] Destructor");
}

void WaypointManager::loadConfig() {
    ROS_INFO("[WaypointManager] Loading configuration parameters...");

    // 使用全局命名空间读取参数，确保与 panel 的 loadConfigFromFile 一致
    ros::NodeHandle global_nh;

    global_nh.param<std::string>("topics/waypoint_input", config_.waypoint_input_topic, "uav/waypoints/input");
    global_nh.param<std::string>("topics/waypoint_current", config_.waypoint_current_topic, "uav/waypoints/current");
    global_nh.param<std::string>("topics/waypoint_params_input", config_.waypoint_params_input_topic, "uav/waypoints/params");
    global_nh.param<std::string>("topics/waypoint_params_current", config_.waypoint_params_current_topic, "uav/waypoints/params_loaded");
    global_nh.param<std::string>("services/load", config_.load_service, "uav/waypoint_manager/load_waypoints");
    global_nh.param<std::string>("services/save", config_.save_service, "uav/waypoint_manager/save_waypoints");
    global_nh.param<std::string>("services/clear", config_.clear_service, "uav/waypoint_manager/clear_waypoints");

    global_nh.param<std::string>("paths/default_save", config_.default_save_path, "/home/groundstation/waypoints.xml");
    global_nh.param<std::string>("paths/default_load", config_.default_load_path, "/home/groundstation/waypoints.xml");

    global_nh.param<double>("validation/min_waypoint_spacing", config_.min_waypoint_spacing, 0.3);
    global_nh.param<double>("validation/max_height", config_.max_height, 50.0);
    global_nh.param<double>("validation/min_height", config_.min_height, 0.5);
    global_nh.param<double>("publish_rate", config_.publish_rate, 1.0);

    // 从全局命名空间读取默认飞行参数（用于 per-waypoint 回退）
    global_nh.param<double>("flight_defaults/hover_duration", default_hover_time_, 5.0);
    global_nh.param<double>("flight_defaults/travel_speed", default_speed_, 2.0);

    ROS_INFO("[WaypointManager] Configuration:");
    ROS_INFO("  - default save path: %s", config_.default_save_path.c_str());
    ROS_INFO("  - min waypoint spacing: %.2f m", config_.min_waypoint_spacing);
    ROS_INFO("  - height range: %.2f - %.2f m", config_.min_height, config_.max_height);
    ROS_INFO("  - params input topic: %s", config_.waypoint_params_input_topic.c_str());
    ROS_INFO("  - params current topic: %s", config_.waypoint_params_current_topic.c_str());
}

void WaypointManager::initROS() {
    waypoints_sub_ = nh_.subscribe(config_.waypoint_input_topic, 1, &WaypointManager::waypointsCallback, this);
    // 修复自订阅回环：订阅 panel 的参数话题，发布到独立的 current 话题
    waypoint_params_sub_ = nh_.subscribe(config_.waypoint_params_input_topic, 1, &WaypointManager::waypointParamsCallback, this);
    waypoints_pub_ = nh_.advertise<geometry_msgs::PoseArray>(config_.waypoint_current_topic, 1, true);  // latch=true
    waypoint_params_current_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(config_.waypoint_params_current_topic, 1, true);

    load_srv_ = nh_.advertiseService(config_.load_service, &WaypointManager::loadWaypointsCallback, this);
    save_srv_ = nh_.advertiseService(config_.save_service, &WaypointManager::saveWaypointsCallback, this);
    clear_srv_ = nh_.advertiseService(config_.clear_service, &WaypointManager::clearWaypointsCallback, this);

    ROS_INFO("[WaypointManager] ROS interface initialization complete");
    ROS_INFO("  - subscribe (waypoints): %s", config_.waypoint_input_topic.c_str());
    ROS_INFO("  - subscribe (params): %s", config_.waypoint_params_input_topic.c_str());
    ROS_INFO("  - publish (waypoints): %s", config_.waypoint_current_topic.c_str());
    ROS_INFO("  - publish (params): %s", config_.waypoint_params_current_topic.c_str());
    ROS_INFO("  - services: %s, %s, %s", config_.load_service.c_str(), config_.save_service.c_str(), config_.clear_service.c_str());
}

bool WaypointManager::isValidCoordinate(double val) {
    return !std::isnan(val) && !std::isinf(val);
}

void WaypointManager::waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    // 验证输入坐标有效性（NaN/Inf 检查）
    for (const auto& pose : msg->poses) {
        if (!isValidCoordinate(pose.position.x) ||
            !isValidCoordinate(pose.position.y) ||
            !isValidCoordinate(pose.position.z)) {
            ROS_ERROR("[WaypointManager] Received waypoint with NaN/Inf coordinates, rejecting");
            return;
        }
    }

    waypoints_ = *msg;
    has_waypoints_ = true;

    // per-waypoint 参数由 waypointParamsCallback 从 params_input 话题接收
    // 如果尚未收到 params（waypoint_params_ 大小不匹配），先用默认值填充
    if (waypoint_params_.size() != waypoints_.poses.size()) {
        waypoint_params_.clear();
        for (size_t i = 0; i < waypoints_.poses.size(); ++i) {
            WaypointParams wp;
            wp.hover_time = default_hover_time_;
            wp.speed = default_speed_;
            waypoint_params_.push_back(wp);
        }
    }

    ROS_INFO("[WaypointManager] Received %zu waypoints", waypoints_.poses.size());

    // 验证航点
    std::string error_msg;
    if (!validateWaypoints(error_msg)) {
        ROS_WARN("[WaypointManager] Waypoint validation warning: %s", error_msg.c_str());
    }

    // 发布当前航点到 navigator
    waypoints_pub_.publish(waypoints_);
    // 发布当前 params 到 panel（通过独立的 current 话题，避免回环）
    publishWaypointParams();
}

void WaypointManager::waypointParamsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 2) return;

    waypoint_params_.clear();
    size_t count = msg->data.size() / 2;
    for (size_t i = 0; i < count; ++i) {
        WaypointParams wp;
        wp.hover_time = msg->data[i * 2];
        wp.speed = msg->data[i * 2 + 1];
        waypoint_params_.push_back(wp);
    }

    ROS_INFO("[WaypointManager] Received params for %zu waypoints from panel", count);
}

bool WaypointManager::loadWaypointsCallback(uav_waypoint_manager::LoadWaypoints::Request& req,
                                            uav_waypoint_manager::LoadWaypoints::Response& res) {
    std::string file_path = req.file_path.empty() ? config_.default_load_path : req.file_path;
    ROS_INFO("[WaypointManager] Received load waypoints request: %s", file_path.c_str());

    try {
        if (!loadFromXml(file_path)) {
            res.success = false;
            res.message = "Load failed: " + file_path + " (check file format and permissions)";
            res.waypoint_count = 0;
            return true;
        }

        // 验证
        std::string error_msg;
        if (!validateWaypoints(error_msg)) {
            ROS_WARN("[WaypointManager] Loaded waypoint validation warning: %s", error_msg.c_str());
        }

        // 发布到 navigator
        waypoints_pub_.publish(waypoints_);
        // 发布 params 到 panel（通过独立的 current 话题）
        publishWaypointParams();

        res.success = true;
        res.message = "Loaded " + std::to_string(waypoints_.poses.size()) + " waypoints successfully";
        res.waypoint_count = waypoints_.poses.size();
        res.waypoints = waypoints_;  // 直接在响应中返回航点数据

        ROS_INFO("[WaypointManager] Waypoints loaded successfully: %zu", waypoints_.poses.size());
    } catch (const std::exception& e) {
        ROS_ERROR("[WaypointManager] Load exception: %s", e.what());
        res.success = false;
        res.message = std::string("Load exception: ") + e.what();
        res.waypoint_count = 0;
    }
    return true;
}

bool WaypointManager::saveWaypointsCallback(uav_waypoint_manager::SaveWaypoints::Request& req,
                                            uav_waypoint_manager::SaveWaypoints::Response& res) {
    if (!has_waypoints_) {
        res.success = false;
        res.message = "No waypoint data to save";
        return true;
    }

    std::string file_path = req.file_path.empty() ? config_.default_save_path : req.file_path;
    ROS_INFO("[WaypointManager] Received save waypoints request: %s", file_path.c_str());

    try {
        if (!saveToXml(file_path)) {
            res.success = false;
            res.message = "Save failed: " + file_path + " (check directory permissions)";
            return true;
        }

        res.success = true;
        res.message = "Saved " + std::to_string(waypoints_.poses.size()) + " waypoints to " + file_path;
        ROS_INFO("[WaypointManager] Waypoints saved successfully: %s", file_path.c_str());
    } catch (const std::exception& e) {
        ROS_ERROR("[WaypointManager] Save exception: %s", e.what());
        res.success = false;
        res.message = std::string("Save exception: ") + e.what();
    }
    return true;
}

bool WaypointManager::clearWaypointsCallback(uav_waypoint_manager::ClearWaypoints::Request& req,
                                             uav_waypoint_manager::ClearWaypoints::Response& res) {
    waypoints_.poses.clear();
    waypoint_params_.clear();
    has_waypoints_ = false;

    geometry_msgs::PoseArray empty_msg;
    empty_msg.header.frame_id = "map";
    waypoints_pub_.publish(empty_msg);
    publishWaypointParams();

    res.success = true;
    res.message = "Waypoints cleared";

    ROS_INFO("[WaypointManager] Waypoints cleared");
    return true;
}

void WaypointManager::publishWaypointParams() {
    std_msgs::Float64MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = waypoint_params_.size();
    msg.layout.dim[0].stride = 2;
    msg.layout.dim[0].label = "waypoint_params";
    for (const auto& wp : waypoint_params_) {
        msg.data.push_back(wp.hover_time);
        msg.data.push_back(wp.speed);
    }
    // 发布到独立的 current 话题，避免自订阅回环
    waypoint_params_current_pub_.publish(msg);
}

// ========== XML operations ==========

bool WaypointManager::saveToXml(const std::string& file_path) {
    try {
        std::ofstream file(file_path);
        if (!file.is_open()) {
            ROS_ERROR("[WaypointManager] Cannot open file for writing: %s", file_path.c_str());
            return false;
        }

        file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        file << "<waypoints>\n";
        file << "  <metadata>\n";
        file << "    <created>" << getCurrentTimestamp() << "</created>\n";
        file << "    <frame_id>" << waypoints_.header.frame_id << "</frame_id>\n";
        file << "    <count>" << waypoints_.poses.size() << "</count>\n";
        file << "  </metadata>\n";

        for (size_t i = 0; i < waypoints_.poses.size(); ++i) {
            const auto& pose = waypoints_.poses[i];
            double yaw = tf::getYaw(pose.orientation);

            file << "  <waypoint id=\"" << (i + 1) << "\">\n";
            file << "    <x>" << pose.position.x << "</x>\n";
            file << "    <y>" << pose.position.y << "</y>\n";
            file << "    <z>" << pose.position.z << "</z>\n";
            file << "    <yaw>" << yaw << "</yaw>\n";
            // 每航点扩展参数
            if (i < waypoint_params_.size()) {
                file << "    <hover_time>" << waypoint_params_[i].hover_time << "</hover_time>\n";
                file << "    <speed>" << waypoint_params_[i].speed << "</speed>\n";
            } else {
                file << "    <hover_time>" << default_hover_time_ << "</hover_time>\n";
                file << "    <speed>" << default_speed_ << "</speed>\n";
            }
            file << "    <orientation>\n";
            file << "      <x>" << pose.orientation.x << "</x>\n";
            file << "      <y>" << pose.orientation.y << "</y>\n";
            file << "      <z>" << pose.orientation.z << "</z>\n";
            file << "      <w>" << pose.orientation.w << "</w>\n";
            file << "    </orientation>\n";
            file << "  </waypoint>\n";
        }

        file << "</waypoints>\n";
        file.close();

        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("[WaypointManager] XML save exception: %s", e.what());
        return false;
    }
}

bool WaypointManager::loadFromXml(const std::string& file_path) {
    // 保存当前数据以便失败时回滚
    geometry_msgs::PoseArray prev_waypoints = waypoints_;
    std::vector<WaypointParams> prev_params = waypoint_params_;
    bool prev_has_waypoints = has_waypoints_;

    try {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            ROS_ERROR("[WaypointManager] Cannot open file for reading: %s", file_path.c_str());
            return false;
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        file.close();

        std::string content = buffer.str();
        if (content.empty()) {
            ROS_ERROR("[WaypointManager] Empty file: %s", file_path.c_str());
            return false;
        }

        // 临时容器，成功后再替换
        geometry_msgs::PoseArray temp_waypoints;
        std::vector<WaypointParams> temp_params;
        temp_waypoints.header.frame_id = "map";

        size_t pos = 0;
        while ((pos = content.find("<waypoint id=\"", pos)) != std::string::npos) {
            pos = content.find(">", pos);
            if (pos == std::string::npos) break;
            pos++;

            geometry_msgs::Pose pose;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;
            bool has_valid_position = true;

            // 解析 x
            size_t x_start = content.find("<x>", pos);
            size_t x_end = content.find("</x>", pos);
            if (x_start != std::string::npos && x_end != std::string::npos) {
                try {
                    double val = std::stod(content.substr(x_start + 3, x_end - x_start - 3));
                    if (!isValidCoordinate(val)) {
                        ROS_ERROR("[WaypointManager] Invalid x coordinate (NaN/Inf), skipping waypoint");
                        has_valid_position = false;
                    }
                    pose.position.x = val;
                } catch (const std::exception& e) {
                    ROS_ERROR("[WaypointManager] Failed to parse x coordinate: %s", e.what());
                    has_valid_position = false;
                }
            } else {
                has_valid_position = false;
            }

            // 解析 y
            size_t y_start = content.find("<y>", pos);
            size_t y_end = content.find("</y>", pos);
            if (y_start != std::string::npos && y_end != std::string::npos) {
                try {
                    double val = std::stod(content.substr(y_start + 3, y_end - y_start - 3));
                    if (!isValidCoordinate(val)) {
                        ROS_ERROR("[WaypointManager] Invalid y coordinate (NaN/Inf), skipping waypoint");
                        has_valid_position = false;
                    }
                    pose.position.y = val;
                } catch (const std::exception& e) {
                    ROS_ERROR("[WaypointManager] Failed to parse y coordinate: %s", e.what());
                    has_valid_position = false;
                }
            } else {
                has_valid_position = false;
            }

            // 解析 z
            size_t z_start = content.find("<z>", pos);
            size_t z_end = content.find("</z>", pos);
            if (z_start != std::string::npos && z_end != std::string::npos) {
                try {
                    double val = std::stod(content.substr(z_start + 3, z_end - z_start - 3));
                    if (!isValidCoordinate(val)) {
                        ROS_ERROR("[WaypointManager] Invalid z coordinate (NaN/Inf), skipping waypoint");
                        has_valid_position = false;
                    }
                    pose.position.z = val;
                } catch (const std::exception& e) {
                    ROS_ERROR("[WaypointManager] Failed to parse z coordinate: %s", e.what());
                    has_valid_position = false;
                }
            } else {
                has_valid_position = false;
            }

            if (!has_valid_position) {
                pos = content.find("</waypoint>", pos);
                if (pos != std::string::npos) pos += 11;
                continue;  // 跳过此航点
            }

            // 解析 yaw（如果存在）
            size_t yaw_start = content.find("<yaw>", pos);
            size_t yaw_end = content.find("</yaw>", pos);
            if (yaw_start != std::string::npos && yaw_end != std::string::npos) {
                try {
                    double yaw = std::stod(content.substr(yaw_start + 5, yaw_end - yaw_start - 5));
                    tf::Quaternion quat;
                    quat.setRPY(0, 0, yaw);
                    pose.orientation.x = quat.x();
                    pose.orientation.y = quat.y();
                    pose.orientation.z = quat.z();
                    pose.orientation.w = quat.w();
                } catch (const std::exception& e) {
                    ROS_WARN("[WaypointManager] Failed to parse yaw, using identity: %s", e.what());
                }
            }

            // 检查 orientation 标签（如果存在）
            size_t ox_start = content.find("<orientation>", pos);
            if (ox_start != std::string::npos) {
                size_t ox_end = content.find("</orientation>", ox_start);
                if (ox_end != std::string::npos) {
                    auto parseOri = [&](const std::string& tag, double& out) {
                        size_t s = content.find("<" + tag + ">", ox_start);
                        size_t e = content.find("</" + tag + ">", ox_start);
                        if (s != std::string::npos && e != std::string::npos && s < ox_end) {
                            try { out = std::stod(content.substr(s + tag.size() + 2, e - s - tag.size() - 2)); }
                            catch (...) {}
                        }
                    };
                    parseOri("x", pose.orientation.x);
                    parseOri("y", pose.orientation.y);
                    parseOri("z", pose.orientation.z);
                    parseOri("w", pose.orientation.w);
                }
            }

            temp_waypoints.poses.push_back(pose);

            // 解析每航点扩展参数（hover_time, speed），不存在时用全局默认值
            WaypointParams wp;
            wp.hover_time = default_hover_time_;
            wp.speed = default_speed_;

            size_t ht_start = content.find("<hover_time>", pos);
            size_t ht_end = content.find("</hover_time>", pos);
            if (ht_start != std::string::npos && ht_end != std::string::npos) {
                try {
                    double val = std::stod(content.substr(ht_start + 12, ht_end - ht_start - 12));
                    if (isValidCoordinate(val)) wp.hover_time = val;
                } catch (...) {}
            }

            size_t sp_start = content.find("<speed>", pos);
            size_t sp_end = content.find("</speed>", pos);
            if (sp_start != std::string::npos && sp_end != std::string::npos) {
                try {
                    double val = std::stod(content.substr(sp_start + 7, sp_end - sp_start - 7));
                    if (isValidCoordinate(val)) wp.speed = val;
                } catch (...) {}
            }

            temp_params.push_back(wp);

            // 移动到下一个 waypoint
            pos = content.find("</waypoint>", pos);
            if (pos != std::string::npos) pos += 11;
        }

        // 成功解析所有航点后才替换全局状态
        waypoints_ = temp_waypoints;
        waypoint_params_ = temp_params;
        has_waypoints_ = !waypoints_.poses.empty();
        return has_waypoints_;

    } catch (const std::exception& e) {
        ROS_ERROR("[WaypointManager] XML load exception: %s", e.what());
        // 回滚到加载前的状态
        waypoints_ = prev_waypoints;
        waypoint_params_ = prev_params;
        has_waypoints_ = prev_has_waypoints;
        return false;
    }
}

// ========== waypoint validation ==========

bool WaypointManager::validateWaypoints(std::string& error_msg) {
    if (!has_waypoints_ || waypoints_.poses.empty()) {
        error_msg = "No waypoint data";
        return false;
    }

    // NaN/Inf 检查
    for (size_t i = 0; i < waypoints_.poses.size(); ++i) {
        const auto& p = waypoints_.poses[i].position;
        if (!isValidCoordinate(p.x) || !isValidCoordinate(p.y) || !isValidCoordinate(p.z)) {
            error_msg = "Waypoint " + std::to_string(i + 1) + " contains NaN/Inf coordinates";
            ROS_ERROR("[WaypointManager] %s", error_msg.c_str());
            return false;
        }
    }

    // 高度检查必须通过（>50m 的航点疑似配置错误，拒绝）
    if (!checkWaypointHeight(error_msg)) return false;
    // 间距检查仅警告
    checkWaypointSpacing(error_msg);
    // 重复航点仅警告
    checkDuplicateWaypoints(error_msg);

    return true;
}

bool WaypointManager::checkDuplicateWaypoints(std::string& error_msg) {
    for (size_t i = 0; i < waypoints_.poses.size(); ++i) {
        for (size_t j = i + 1; j < waypoints_.poses.size(); ++j) {
            if (distance3D(waypoints_.poses[i].position, waypoints_.poses[j].position) < 0.01) {
                error_msg = "Duplicate waypoints: " + std::to_string(i + 1) + " and " + std::to_string(j + 1);
                ROS_WARN("[WaypointManager] %s", error_msg.c_str());
                // 重复航点仅警告，不阻止（用户可能有意识地在同一位置设置航点做hover）
            }
        }
    }
    return true;
}

bool WaypointManager::checkWaypointSpacing(std::string& error_msg) {
    if (waypoints_.poses.size() < 2) return true;
    for (size_t i = 0; i + 1 < waypoints_.poses.size(); ++i) {
        double dist = distance3D(waypoints_.poses[i].position, waypoints_.poses[i + 1].position);
        if (dist < config_.min_waypoint_spacing) {
            error_msg = "Waypoint " + std::to_string(i + 1) + " and " + std::to_string(i + 2) +
                        " spacing too small: " + std::to_string(dist) + " m";
            ROS_WARN("[WaypointManager] %s", error_msg.c_str());
            // 间距过小仅警告
        }
    }
    return true;
}

bool WaypointManager::checkWaypointHeight(std::string& error_msg) {
    for (size_t i = 0; i < waypoints_.poses.size(); ++i) {
        double z = waypoints_.poses[i].position.z;
        if (!isValidCoordinate(z)) {
            error_msg = "Waypoint " + std::to_string(i + 1) + " height is NaN/Inf";
            ROS_ERROR("[WaypointManager] %s", error_msg.c_str());
            return false;
        }
        if (z < config_.min_height || z > config_.max_height) {
            error_msg = "Waypoint " + std::to_string(i + 1) + " height " + std::to_string(z) +
                        " m out of range [" + std::to_string(config_.min_height) + ", " +
                        std::to_string(config_.max_height) + "]";
            ROS_WARN("[WaypointManager] %s", error_msg.c_str());
            // 高度超出建议范围但不阻止（用户可能有特殊需求）
        }
    }
    return true;
}

// ========== helper functions ==========

std::string WaypointManager::getCurrentTimestamp() {
    time_t now = time(0);
    tm* ltm = localtime(&now);
    char buffer[80];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
             1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday,
             ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    return std::string(buffer);
}

double WaypointManager::distance3D(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void WaypointManager::run() {
    ROS_INFO("[WaypointManager] Entering main loop");
    ros::spin();
}

} // namespace uav_waypoint_manager

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_waypoint_manager");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        uav_waypoint_manager::WaypointManager manager(nh, pnh);
        manager.run();
    } catch (const std::exception& e) {
        ROS_ERROR("[WaypointManager] Exception: %s", e.what());
        return 1;
    }

    return 0;
}
