#ifndef WAYPOINT_PANEL_H
#define WAYPOINT_PANEL_H

#include <string>
#include <vector>
#include <map>

#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/master.h>

#include <rviz/panel.h>

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>
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
#include <QComboBox>
#include <QInputDialog>
#include <QFileDialog>
#include <QScrollArea>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

#include "uav_navigator/NavigatorStatus.h"
#include "uav_navigator/NavigatorCommand.h"
#include "uav_waypoint_manager/SaveWaypoints.h"
#include "uav_waypoint_manager/LoadWaypoints.h"

namespace rviz_waypoint_panel {

// 节点状态结构
struct NodeStatus {
    std::string name;
    bool running;
    QLabel *label;
};

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
    void savePlanWaypoints();
    void publishPlanTask();

    // ===== 导航控制 =====
    void startNavigation();
    void emergencyStop();
    void pauseNavigation();
    void cancelNavigation();
    void returnToHome();
    void resetNavigator();

    // ===== 系统控制 =====
    void launchGroundStation();
    void killGroundStation();
    void oneKeyTakeoff();
    void oneKeyLand();
    void executeMission();
    void checkNodeStatus();
    void updateConnectionStatus();
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
    void markWaypoint(const geometry_msgs::PoseStamped &pose, int id);
    void clearMarkers();
    void republishMarkers();

    // Plan Maker
    enum PlanMakerPhase { PLANNING, CONNECTED, SAVED, NAVIGATING };
    void addPlanMakerPoint(const geometry_msgs::PoseStamped &pose);
    void publishPlanMakerMarkers();
    void publishPlanTrajectory();
    void updatePlanMakerStatus();
    void setPlanMakerPhase(PlanMakerPhase phase);
    QString phaseToString(PlanMakerPhase phase);

    // 表格
    geometry_msgs::PoseArray readWaypointsFromTable();
    void updateStatusDisplay(const uav_navigator::NavigatorStatus &status);

    // 日志
    void logInfo(const QString &msg);
    void logWarn(const QString &msg);
    void logError(const QString &msg);
    void truncateLog();  // 限制日志行数，防止内存膨胀

    // 辅助
    QString stateToString(uint8_t state);
    QString stateToColor(uint8_t state);
    void setButtonStyle(QPushButton *btn, const QString &color, bool enabled);

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
        double trajectory_width;

        double arrow_scale_x;
        double arrow_scale_y;
        double arrow_scale_z;
        double number_scale;
        double number_offset_z;
        double color_r;
        double color_g;
        double color_b;
        double color_a;
        int default_max_goals;
        int spin_timer_ms;
        std::string default_config_path;
        std::string config_loaded_topic;
        std::string config_reload_topic;
        std::string waypoint_current_topic;
        std::string default_save_path;     // 默认航点保存路径
        std::string default_load_path;     // 默认航点加载路径
    } config_;

    // ===== 数据 =====
    int max_num_goal_;
    int current_waypoint_count_;
    static int marker_id_counter_;
    uint8_t current_nav_state_;
    bool mavros_connected_;
    bool mavros_armed_;
    std::string mavros_mode_;
    double current_x_, current_y_, current_z_;
    uint8_t confirmed_waypoint_count_;

    // Plan Maker data
    std::vector<geometry_msgs::PoseStamped> plan_maker_points_;
    PlanMakerPhase plan_maker_phase_;
    int plan_maker_selected_index_;
    bool plan_maker_dirty_;

    // Per-waypoint parameters
    std::vector<double> waypoint_hover_times_;
    std::vector<double> waypoint_speeds_;
    double default_hover_time_;
    double default_speed_;

    // Navigator running flag
    bool navigator_running_;

    // ===== 界面控件 =====
    QVBoxLayout *root_layout_;

    // ===== 航点规划状态显示 =====
    QLabel *plan_maker_status_label_;

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

    // ===== 系统控制按钮 =====
    QPushButton *launch_gs_button_;
    QPushButton *kill_gs_button_;
    QPushButton *one_key_takeoff_button_;
    QPushButton *one_key_land_button_;
    QPushButton *execute_mission_button_;

    // ===== 导航控制按钮 =====
    QPushButton *start_nav_button_;
    QPushButton *pause_nav_button_;
    QPushButton *cancel_nav_button_;
    QPushButton *rth_button_;
    QPushButton *reset_button_;
    QPushButton *emergency_button_;

    // ===== 航点规划按钮 =====
    QPushButton *connect_plan_button_;
    QPushButton *delete_plan_point_button_;
    QPushButton *clear_plan_button_;
    QPushButton *save_plan_button_;
    QPushButton *publish_plan_task_button_;

    // ===== 航点操作按钮 =====
    QPushButton *delete_button_;
    QPushButton *move_up_button_;
    QPushButton *move_down_button_;
    QPushButton *save_button_;
    QPushButton *load_button_;
    QPushButton *clear_button_;
    QPushButton *publish_button_;

    // ===== 日志区域 =====
    QTextEdit *log_text_;

    // ===== 定时器 =====
    QTimer *spin_timer_;
    QTimer *status_check_timer_;

    // 节点状态
    std::map<std::string, NodeStatus> node_status_map_;
};

} // namespace rviz_waypoint_panel

#endif // WAYPOINT_PANEL_H
