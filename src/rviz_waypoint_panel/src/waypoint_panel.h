#ifndef WAYPOINT_PANEL_H
#define WAYPOINT_PANEL_H

#include <string>
#include <vector>
#include <map>

#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/master.h>
#include <ros/package.h>

#include <rviz/panel.h>

#include <QPushButton>
#include <QTableWidget>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QHeaderView>
#include <QMessageBox>
#include <QTextEdit>
#include <QFrame>
#include <QProgressBar>
#include <QFileDialog>
#include <QScrollArea>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include "uav_navigator/NavigatorStatus.h"
#include "uav_navigator/NavigatorCommand.h"
#include "uav_waypoint_manager/SaveWaypoints.h"
#include "uav_waypoint_manager/LoadWaypoints.h"

namespace rviz_waypoint_panel {

class WaypointPanel : public rviz::Panel {
    Q_OBJECT
public:
    explicit WaypointPanel(QWidget *parent = 0);
    ~WaypointPanel();

    void loadConfig();

public Q_SLOTS:
    // ===== 配置加载 =====
    void loadConfigFromFile();
    void publishConfigLoaded(const QString &path, const QString &summary);
    void onConfigGroupToggled(bool checked);

    // ===== 航点操作 ======
    void receiveGoal(const geometry_msgs::PoseStamped::ConstPtr &pose);
    void updateMaxNumGoal();
    void initPoseTable();
    void updatePoseTable();
    void addWaypointToTable(const geometry_msgs::Pose &pose, double hover_time = 5.0, double speed = 2.0);
    void deleteSelectedWaypoint();
    void moveWaypointUp();
    void moveWaypointDown();
    void saveWaypoints();
    void loadWaypoints();
    void clearWaypoints();
    void publishWaypoints();
    void onTableChanged(int row, int column);

    // ===== 航点规划 (Plan Maker) =====
    void connectPlanTrajectory();
    void deleteSelectedPlanPoint();
    void clearPlanPoints();
    void savePlanWaypoints();    // 发布航点到 navigator（原名"保存"）

    // ===== 飞行控制 =====
    void startMission();        // ▶ 开始任务
    void hoverInPlace();        // ⏸ 悬停：PAUSE，保持当前位置
    void landNow();             // 🛬 降落：LAND，立即着陆
    void returnToHome();        // 🏠 返航：返回起飞点并着陆
    void resetNavigator();      // 🔄 重置：EMERGENCY/LANDED → IDLE
    void emergencyStop();       // 🛑 紧急停止

    // ===== 录制控制 =====
    void toggleRecording();     // ⏺/⏹ 切换录制状态

    // ===== 系统 =====
    void checkNodeStatus();
    void updateAllButtonStates();

    // ===== 状态接收 =====
    void receiveNavStatus(const uav_navigator::NavigatorStatus::ConstPtr &msg);
    void receiveMavrosState(const mavros_msgs::State::ConstPtr &msg);
    void receiveOdom(const nav_msgs::Odometry::ConstPtr &msg);
    void receiveWaypointParams(const std_msgs::Float64MultiArray::ConstPtr &msg);

    // ===== 定时器 =====
    void startSpin();

protected:
    // Marker
    void clearMarkers();

    // Plan Maker
    enum PlanMakerPhase { PLANNING, CONNECTED, SAVED, NAVIGATING };
    void addPlanMakerPoint(const geometry_msgs::PoseStamped &pose);
    void publishPlanMakerMarkers();
    void publishPlanTrajectory();
    void updatePlanMakerStatus();
    void updateWorkflowProgress();
    void setPlanMakerPhase(PlanMakerPhase phase);
    QString phaseToString(PlanMakerPhase phase);

    // 表格
    geometry_msgs::PoseArray getWaypointsFromPlan();
    void updateStatusDisplay(const uav_navigator::NavigatorStatus &status);

    // 日志
    void logInfo(const QString &msg);
    void logWarn(const QString &msg);
    void logError(const QString &msg);
    void truncateLog();  // 限制日志行数，防止内存膨胀

    // 辅助
    QString stateToString(uint8_t state);
    QString stateToColor(uint8_t state);

    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::Subscriber nav_status_sub_;
    ros::Subscriber mavros_state_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher waypoint_pub_;
    ros::Publisher plan_maker_points_pub_;
    ros::Publisher plan_maker_trajectory_pub_;
    ros::Publisher config_loaded_pub_;
    ros::Publisher config_reload_pub_;
    ros::Publisher waypoint_params_pub_;
    ros::Subscriber waypoint_params_sub_;
    ros::ServiceClient save_waypoints_client_;
    ros::ServiceClient load_waypoints_client_;
    ros::ServiceClient nav_command_client_;
    ros::Publisher record_control_pub_;     // 录制控制: uav/experiment/record

    // 配置
    struct Config {
        std::string goal_topic;
        std::string marker_topic;
        std::string waypoint_input_topic;
        std::string navigator_status_topic;
        std::string mavros_state_topic;
        std::string odom_topic;
        std::string save_service;
        std::string load_service;
        std::string nav_command_service;

        std::string plan_maker_points_topic;
        std::string plan_maker_trajectory_topic;
        double plan_maker_sphere_scale;
        double plan_maker_color_r;
        double plan_maker_color_g;
        double plan_maker_color_b;
        double plan_maker_color_a;

        double arrow_scale_x;
        double arrow_scale_y;
        double arrow_scale_z;
        double number_scale;
        double number_offset_z;
        int default_max_goals;
        int spin_timer_ms;
        std::string default_config_path;
        std::string config_loaded_topic;
        std::string config_reload_topic;
        std::string record_control_topic;          // 录制控制话题
        std::string waypoint_params_input_topic;   // 发布 per-waypoint params
        std::string waypoint_params_loaded_topic;  // 订阅 waypoint_manager 存储的 params
        std::string default_save_path;     // 默认航点保存路径
        std::string default_load_path;     // 默认航点加载路径
        std::string default_frame_id;      // 默认坐标系（通常为 "map"）
    } config_;

    // ===== 数据 =====
    int max_num_goal_;
    int current_waypoint_count_;
    uint8_t current_nav_state_;
    bool mavros_connected_;
    bool mavros_armed_;
    std::string mavros_mode_;
    double current_x_, current_y_, current_z_;
    uint8_t confirmed_waypoint_count_;

    // 导航进度追踪（从 NavigatorStatus 同步，用于 marker 颜色编码）
    uint8_t nav_current_waypoint_idx_;
    uint8_t nav_total_waypoints_;

    // Plan Maker data
    std::vector<geometry_msgs::PoseStamped> plan_maker_points_;
    PlanMakerPhase plan_maker_phase_;
    int plan_maker_selected_index_;

    // Per-waypoint parameters
    std::vector<double> waypoint_hover_times_;
    std::vector<double> waypoint_speeds_;
    double default_hover_time_;
    double default_speed_;

    // Navigator running flag
    bool navigator_running_;

    // 录制状态
    bool is_recording_;

    // auto-SAVED 确认计时（防止 navigator 无响应时永久卡在 CONNECTED）
    ros::Time confirm_request_time_;

    // Marker 刷新计数器（替代 static 局部变量，每 N 次 spin 刷新一次 marker）
    int refresh_counter_;

    // ===== 界面控件 =====
    QVBoxLayout *root_layout_;

    // ===== 航点规划状态显示 =====
    QLabel *plan_maker_status_label_;
    QProgressBar *workflow_progress_;    // 4 步工作流进度条
    QLabel *workflow_label_;             // 工作流步骤文字

    // ===== 配置显示区域 =====
    QGroupBox *config_group_;
    QTextEdit *config_display_;
    QPushButton *load_config_button_;

    // ===== 航点表格 =====
    QTableWidget *waypoint_table_;
    QLineEdit *max_num_goal_editor_;
    QPushButton *max_num_goal_button_;

    // ===== 状态显示 =====
    QLabel *status_led_;
    QLabel *status_text_;
    QLabel *wp_progress_label_;
    QProgressBar *wp_progress_bar_;
    QLabel *position_label_;
    QLabel *mavros_conn_label_;
    QLabel *mavros_armed_label_;
    QLabel *mavros_mode_label_;

    // ===== 飞行控制按钮 =====
    QPushButton *start_mission_button_;   // ▶ 开始任务
    QPushButton *record_button_;          // ⏺/⏹ 开始/停止录制
    QPushButton *hover_button_;           // ⏸ 悬停
    QPushButton *land_button_;            // 🛬 降落
    QPushButton *rth_button_;             // 🏠 返航
    QPushButton *reset_button_;           // 🔄 重置
    QPushButton *emergency_button_;       // 🛑 紧急停止

    // ===== 航点规划按钮 =====
    QPushButton *load_button_;            // 📂 加载文件
    QPushButton *save_file_button_;       // 💾 保存到文件
    QPushButton *connect_plan_button_;    // 🔗 连线
    QPushButton *publish_button_;         // 📤 发布航点（原 save_plan_button_）
    QPushButton *delete_plan_point_button_;  // 🗑 删除
    QPushButton *clear_plan_button_;      // 🧹 清空

    // ===== 航点列表操作按钮 =====
    QPushButton *delete_button_;
    QPushButton *move_up_button_;
    QPushButton *move_down_button_;
    QPushButton *clear_button_;

    // ===== 日志区域 =====
    QTextEdit *log_text_;

    // ===== 定时器 =====
    QTimer *spin_timer_;
    QTimer *status_check_timer_;
};

} // namespace rviz_waypoint_panel

#endif // WAYPOINT_PANEL_H
