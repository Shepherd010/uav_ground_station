#include <cstdio>
#include <fstream>
#include <sstream>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>
#include <QMessageBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QInputDialog>
#include <QTextEdit>
#include <QTextBlock>
#include <QTextCursor>
#include <QFrame>
#include <QProgressBar>
#include <QComboBox>
#include <QDateTime>
#include <QProcess>
#include <QScrollArea>
#include <QToolTip>
#include <yaml-cpp/yaml.h>

#include "waypoint_panel.h"
#include <tf/transform_datatypes.h>

namespace rviz_waypoint_panel {

int WaypointPanel::marker_id_counter_ = 0;

// ========== 构造函数 ==========
WaypointPanel::WaypointPanel(QWidget *parent)
    : rviz::Panel(parent), nh_(), max_num_goal_(10), current_waypoint_count_(0),
      current_nav_state_(0), mavros_connected_(false), mavros_armed_(false),
      current_x_(0), current_y_(0), current_z_(0), confirmed_waypoint_count_(0),
      nav_current_waypoint_idx_(0), nav_total_waypoints_(0),
      plan_maker_phase_(PLANNING), plan_maker_selected_index_(-1),
      plan_maker_dirty_(false), default_hover_time_(5.0), default_speed_(2.0),
      navigator_running_(false), refresh_counter_(0) {

    loadConfig();

    // ROS 接口
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        config_.goal_topic, 10, boost::bind(&WaypointPanel::receiveGoal, this, _1));
    nav_status_sub_ = nh_.subscribe<uav_navigator::NavigatorStatus>(
        config_.navigator_status_topic, 10, boost::bind(&WaypointPanel::receiveNavStatus, this, _1));
    mavros_state_sub_ = nh_.subscribe<mavros_msgs::State>(
        config_.mavros_state_topic, 10, boost::bind(&WaypointPanel::receiveMavrosState, this, _1));
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        config_.odom_topic, 10, boost::bind(&WaypointPanel::receiveOdom, this, _1));

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(config_.marker_topic, 10);
    waypoint_pub_ = nh_.advertise<geometry_msgs::PoseArray>(config_.waypoint_input_topic, 1, true);
    plan_maker_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(config_.plan_maker_points_topic, 10);
    plan_maker_trajectory_pub_ = nh_.advertise<nav_msgs::Path>(config_.plan_maker_trajectory_topic, 10);

    save_waypoints_client_ = nh_.serviceClient<uav_waypoint_manager::SaveWaypoints>(config_.save_service);
    load_waypoints_client_ = nh_.serviceClient<uav_waypoint_manager::LoadWaypoints>(config_.load_service);
    nav_command_client_ = nh_.serviceClient<uav_navigator::NavigatorCommand>(config_.nav_command_service);
    config_loaded_pub_ = nh_.advertise<std_msgs::String>(config_.config_loaded_topic, 1, true);
    config_reload_pub_ = nh_.advertise<std_msgs::String>(config_.config_reload_topic, 1, true);
    waypoint_params_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(config_.waypoint_params_input_topic, 1, true);
    // 从 waypoint_manager 的独立 current 话题接收存储的 params，避免自订阅回环
    waypoint_params_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>(
        config_.waypoint_params_loaded_topic, 1, boost::bind(&WaypointPanel::receiveWaypointParams, this, _1));

    // ========== 主布局 ==========
    root_layout_ = new QVBoxLayout;
    root_layout_->setSpacing(6);
    root_layout_->setContentsMargins(4, 4, 4, 4);

    // ===== 飞行状态（系统状态 + 导航状态合并）=====
    QGroupBox *flight_status_group = new QGroupBox("飞行状态");
    QVBoxLayout *flight_status_layout = new QVBoxLayout;
    flight_status_layout->setSpacing(4);
    flight_status_layout->setContentsMargins(6, 6, 6, 6);

    QHBoxLayout *status_row = new QHBoxLayout;
    status_row->setSpacing(6);
    status_led_ = new QLabel("●");
    status_led_->setStyleSheet("color: #2196F3; font-size: 16px;");
    status_row->addWidget(status_led_);
    status_text_ = new QLabel("IDLE");
    status_text_->setStyleSheet("font-weight: bold; color: #2196F3; font-size: 12px;");
    status_row->addWidget(status_text_);

    mavros_conn_label_ = new QLabel("MAVROS: 未连接");
    mavros_conn_label_->setStyleSheet("color: #F44336; font-weight: bold; padding: 1px 4px; border-radius: 3px; background: #FFEBEE; font-size: 11px;");
    status_row->addWidget(mavros_conn_label_);

    mavros_armed_label_ = new QLabel("未解锁");
    mavros_armed_label_->setStyleSheet("color: #757575; font-weight: bold; font-size: 11px;");
    status_row->addWidget(mavros_armed_label_);

    mavros_mode_label_ = new QLabel("模式: --");
    mavros_mode_label_->setStyleSheet("color: #757575; font-size: 11px;");
    status_row->addWidget(mavros_mode_label_);

    status_row->addStretch();
    wp_progress_label_ = new QLabel("航点: 0 / 0");
    wp_progress_label_->setStyleSheet("font-size: 11px;");
    status_row->addWidget(wp_progress_label_);
    flight_status_layout->addLayout(status_row);

    wp_progress_bar_ = new QProgressBar;
    wp_progress_bar_->setRange(0, 100);
    wp_progress_bar_->setValue(0);
    wp_progress_bar_->setTextVisible(true);
    wp_progress_bar_->setMaximumHeight(16);
    wp_progress_bar_->setStyleSheet("QProgressBar { font-size: 10px; }");
    flight_status_layout->addWidget(wp_progress_bar_);

    position_label_ = new QLabel("位置: --, --, -- | 目标: --, --, --");
    position_label_->setStyleSheet("color: #666; font-size: 11px;");
    flight_status_layout->addWidget(position_label_);

    flight_status_group->setLayout(flight_status_layout);
    root_layout_->addWidget(flight_status_group);

    // ===== 配置加载区域（默认折叠）=====
    config_group_ = new QGroupBox("配置参数");
    config_group_->setCheckable(true);
    config_group_->setChecked(false);
    QVBoxLayout *config_layout = new QVBoxLayout;
    config_layout->setSpacing(4);
    config_layout->setContentsMargins(6, 6, 6, 6);

    QHBoxLayout *config_button_layout = new QHBoxLayout;
    load_config_button_ = new QPushButton("📂 从文件加载");
    load_config_button_->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 4px; font-size: 11px;");
    config_button_layout->addWidget(load_config_button_);
    config_button_layout->addStretch();
    config_layout->addLayout(config_button_layout);

    config_display_ = new QTextEdit;
    config_display_->setReadOnly(true);
    config_display_->setMaximumHeight(120);
    config_display_->setStyleSheet("font-family: monospace; font-size: 10px; background-color: #f5f5f5;");
    config_display_->setPlainText("尚未加载配置文件。点击上方按钮选择 config.yaml。\n参数仅能通过配置文件修改，面板只读显示。");
    config_layout->addWidget(config_display_);

    config_group_->setLayout(config_layout);
    root_layout_->addWidget(config_group_);
    // 默认折叠配置内容
    config_display_->setVisible(false);
    load_config_button_->setVisible(false);
    connect(config_group_, SIGNAL(toggled(bool)), this, SLOT(onConfigGroupToggled(bool)));

    // ===== 任务控制（按生命周期分组）=====

    // ===== 航点规划 =====
    QGroupBox *plan_maker_group = new QGroupBox("航点规划");
    QVBoxLayout *plan_maker_layout = new QVBoxLayout;
    plan_maker_layout->setSpacing(4);
    plan_maker_layout->setContentsMargins(6, 6, 6, 6);

    QHBoxLayout *plan_button_layout = new QHBoxLayout;
    plan_button_layout->setSpacing(4);
    load_button_ = new QPushButton("📂 加载");
    load_button_->setToolTip("从 XML 文件加载航点数据");
    load_button_->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 4px; font-size: 11px;");
    connect_plan_button_ = new QPushButton("🔗 连接");
    connect_plan_button_->setToolTip("将 ≥2 个航点连接成轨迹线");
    connect_plan_button_->setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 4px; font-size: 11px;");
    save_plan_button_ = new QPushButton("💾 保存");
    save_plan_button_->setToolTip("发布航点到 waypoint_manager 并等待 navigator 确认");
    save_plan_button_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 4px; font-size: 11px;");
    save_plan_button_->setEnabled(false);
    delete_plan_point_button_ = new QPushButton("🗑 删除");
    delete_plan_point_button_->setStyleSheet("background-color: #F44336; color: white; padding: 4px; font-size: 11px;");
    clear_plan_button_ = new QPushButton("🧹 清除");
    clear_plan_button_->setStyleSheet("background-color: #757575; color: white; padding: 4px; font-size: 11px;");
    plan_button_layout->addWidget(load_button_);
    plan_button_layout->addWidget(connect_plan_button_);
    plan_button_layout->addWidget(save_plan_button_);
    plan_button_layout->addWidget(delete_plan_point_button_);
    plan_button_layout->addWidget(clear_plan_button_);
    plan_button_layout->addStretch();
    plan_maker_status_label_ = new QLabel("PLANNING | 0");
    plan_maker_status_label_->setStyleSheet("color: #2196F3; font-weight: bold; padding: 2px; font-size: 11px;");
    plan_button_layout->addWidget(plan_maker_status_label_);
    plan_maker_layout->addLayout(plan_button_layout);

    plan_maker_group->setLayout(plan_maker_layout);
    root_layout_->addWidget(plan_maker_group);

    // ===== 航点列表 =====
    QGroupBox *waypoint_group = new QGroupBox("航点列表");
    QVBoxLayout *waypoint_layout = new QVBoxLayout;
    waypoint_layout->setSpacing(4);
    waypoint_layout->setContentsMargins(6, 6, 6, 6);

    QHBoxLayout *max_goal_layout = new QHBoxLayout;
    max_goal_layout->addWidget(new QLabel("最大航点:"));
    max_num_goal_editor_ = new QLineEdit;
    max_num_goal_editor_->setText(QString::number(config_.default_max_goals));
    max_num_goal_editor_->setFixedWidth(50);
    max_goal_layout->addWidget(max_num_goal_editor_);
    max_num_goal_button_ = new QPushButton("确认");
    max_goal_layout->addWidget(max_num_goal_button_);
    max_goal_layout->addStretch();
    waypoint_layout->addLayout(max_goal_layout);

    waypoint_table_ = new QTableWidget;
    initPoseTable();
    waypoint_layout->addWidget(waypoint_table_);

    connect(waypoint_table_, SIGNAL(cellChanged(int, int)), this, SLOT(onTableChanged(int, int)));

    QHBoxLayout *wp_op_layout = new QHBoxLayout;
    wp_op_layout->setSpacing(4);
    delete_button_ = new QPushButton("🗑 删除选中");
    move_up_button_ = new QPushButton("▲ 上移");
    move_down_button_ = new QPushButton("▼ 下移");
    clear_button_ = new QPushButton("🧹 清除全部");
    wp_op_layout->addWidget(delete_button_);
    wp_op_layout->addWidget(move_up_button_);
    wp_op_layout->addWidget(move_down_button_);
    wp_op_layout->addWidget(clear_button_);
    wp_op_layout->addStretch();
    waypoint_layout->addLayout(wp_op_layout);

    waypoint_group->setLayout(waypoint_layout);
    root_layout_->addWidget(waypoint_group);

    // ===== 飞行控制 =====
    QGroupBox *flight_ctrl_group = new QGroupBox("飞行控制");
    QVBoxLayout *flight_ctrl_layout = new QVBoxLayout;
    flight_ctrl_layout->setSpacing(4);
    flight_ctrl_layout->setContentsMargins(6, 6, 6, 6);

    QHBoxLayout *flight_row1 = new QHBoxLayout;
    flight_row1->setSpacing(4);
    start_mission_button_ = new QPushButton("▶ 开始任务");
    start_mission_button_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 4px; font-size: 11px;");
    start_mission_button_->setToolTip("发布航点并启动导航：解锁→起飞→依次执行→降落。遥控器拨杆可随时接管。");
    hover_button_ = new QPushButton("⏸ 悬停");
    hover_button_->setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 4px; font-size: 11px;");
    hover_button_->setToolTip("暂停导航，保持当前位置悬停。可随时继续执行剩余航点。");
    land_button_ = new QPushButton("🛬 降落");
    land_button_->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 4px; font-size: 11px;");
    land_button_->setToolTip("立即结束任务并着陆。飞控切换到 AUTO.LAND 模式。");
    flight_row1->addWidget(start_mission_button_);
    flight_row1->addWidget(hover_button_);
    flight_row1->addWidget(land_button_);
    flight_ctrl_layout->addLayout(flight_row1);

    QHBoxLayout *flight_row2 = new QHBoxLayout;
    flight_row2->setSpacing(4);
    rth_button_ = new QPushButton("🏠 返航");
    rth_button_->setStyleSheet("background-color: #2196F3; color: white; padding: 4px; font-size: 11px;");
    rth_button_->setToolTip("返回起飞点（TAKEOFF 阶段记录的 Home 位置）并着陆。");
    reset_button_ = new QPushButton("🔄 重置");
    reset_button_->setStyleSheet("background-color: #607D8B; color: white; padding: 4px; font-size: 11px;");
    reset_button_->setToolTip("从 EMERGENCY 或 LANDED 状态恢复到 IDLE，准备下一次任务。");
    emergency_button_ = new QPushButton("🛑 紧急停止");
    emergency_button_->setStyleSheet("background-color: #F44336; color: white; font-weight: bold; padding: 4px; font-size: 11px;");
    emergency_button_->setToolTip("立即触发紧急停止（弹窗确认），飞控切换到 AUTO.LAND。RC 遥控器可随时接管。");
    flight_row2->addWidget(rth_button_);
    flight_row2->addWidget(reset_button_);
    flight_row2->addWidget(emergency_button_);
    flight_ctrl_layout->addLayout(flight_row2);

    flight_ctrl_group->setLayout(flight_ctrl_layout);
    root_layout_->addWidget(flight_ctrl_group);

    // ===== 操作日志 =====
    QGroupBox *log_group = new QGroupBox("操作日志");
    QVBoxLayout *log_layout = new QVBoxLayout;
    log_layout->setSpacing(4);
    log_layout->setContentsMargins(6, 6, 6, 6);
    log_text_ = new QTextEdit;
    log_text_->setReadOnly(true);
    log_text_->setMaximumHeight(80);
    log_text_->setStyleSheet("font-family: monospace; font-size: 10px; background-color: #f5f5f5;");
    log_layout->addWidget(log_text_);
    log_group->setLayout(log_layout);
    root_layout_->addWidget(log_group);

    // 设置主布局
    // 把 root_layout_ 放进 QScrollArea，防止面板过高时被截断
    QWidget *scroll_container = new QWidget(this);
    scroll_container->setLayout(root_layout_);
    scroll_container->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);

    QScrollArea *scroll_area = new QScrollArea(this);
    scroll_area->setWidget(scroll_container);
    scroll_area->setWidgetResizable(true);
    scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll_area->setFrameShape(QFrame::NoFrame);

    QVBoxLayout *outer_layout = new QVBoxLayout;
    outer_layout->setContentsMargins(0, 0, 0, 0);
    outer_layout->addWidget(scroll_area);
    setLayout(outer_layout);

    // 定时器
    spin_timer_ = new QTimer(this);
    spin_timer_->start(config_.spin_timer_ms);

    status_check_timer_ = new QTimer(this);
    status_check_timer_->start(1000);  // 每秒检查一次节点状态

    // 连接信号与槽
    connect(max_num_goal_button_, SIGNAL(clicked()), this, SLOT(updateMaxNumGoal()));
    connect(max_num_goal_button_, SIGNAL(clicked()), this, SLOT(updatePoseTable()));
    connect(delete_button_, SIGNAL(clicked()), this, SLOT(deleteSelectedWaypoint()));
    connect(move_up_button_, SIGNAL(clicked()), this, SLOT(moveWaypointUp()));
    connect(move_down_button_, SIGNAL(clicked()), this, SLOT(moveWaypointDown()));
    connect(clear_button_, SIGNAL(clicked()), this, SLOT(clearWaypoints()));

    connect(start_mission_button_, SIGNAL(clicked()), this, SLOT(startMission()));
    connect(hover_button_, SIGNAL(clicked()), this, SLOT(hoverInPlace()));
    connect(land_button_, SIGNAL(clicked()), this, SLOT(landNow()));
    connect(rth_button_, SIGNAL(clicked()), this, SLOT(returnToHome()));
    connect(reset_button_, SIGNAL(clicked()), this, SLOT(resetNavigator()));
    connect(emergency_button_, SIGNAL(clicked()), this, SLOT(emergencyStop()));

    connect(load_button_, SIGNAL(clicked()), this, SLOT(loadWaypoints()));
    connect(connect_plan_button_, SIGNAL(clicked()), this, SLOT(connectPlanTrajectory()));
    connect(delete_plan_point_button_, SIGNAL(clicked()), this, SLOT(deleteSelectedPlanPoint()));
    connect(clear_plan_button_, SIGNAL(clicked()), this, SLOT(clearPlanPoints()));
    connect(save_plan_button_, SIGNAL(clicked()), this, SLOT(savePlanWaypoints()));

    connect(load_config_button_, SIGNAL(clicked()), this, SLOT(loadConfigFromFile()));

    connect(spin_timer_, SIGNAL(timeout()), this, SLOT(startSpin()));
    connect(status_check_timer_, SIGNAL(timeout()), this, SLOT(checkNodeStatus()));

    logInfo("航点面板已就绪 | 工作流: 打点→连接→保存→开始任务");
    logInfo("提示：RC 遥控器拥有最高控制权，可随时切换模式接管无人机");
}

WaypointPanel::~WaypointPanel() {
    clearMarkers();
}

// ========== 配置加载 ==========
void WaypointPanel::loadConfig() {
    // 使用全局命名空间的参数句柄，因为 config.yaml 在 launch 中作为全局参数加载
    ros::NodeHandle pnh;
    pnh.param<std::string>("panel/goal_topic", config_.goal_topic, "move_base_simple/goal");
    pnh.param<std::string>("panel/marker_topic", config_.marker_topic, "visualization_marker");
    pnh.param<std::string>("panel/waypoint_input_topic", config_.waypoint_input_topic, "uav/waypoints/input");
    pnh.param<std::string>("panel/navigator_status_topic", config_.navigator_status_topic, "uav/navigator/status");
    pnh.param<std::string>("panel/mavros_state_topic", config_.mavros_state_topic, "mavros/state");
    pnh.param<std::string>("panel/odom_topic", config_.odom_topic, "mavros/local_position/odom");
    pnh.param<std::string>("panel/save_service", config_.save_service, "uav/waypoint_manager/save_waypoints");
    pnh.param<std::string>("panel/load_service", config_.load_service, "uav/waypoint_manager/load_waypoints");
    pnh.param<std::string>("panel/nav_command_service", config_.nav_command_service, "uav/navigator/command");

    pnh.param<std::string>("panel/plan_maker/points_topic", config_.plan_maker_points_topic, "uav/plan_maker/points");
    pnh.param<std::string>("panel/plan_maker/trajectory_topic", config_.plan_maker_trajectory_topic, "uav/plan_maker/trajectory");
    pnh.param<double>("panel/plan_maker/sphere_scale", config_.plan_maker_sphere_scale, 0.15);
    pnh.param<double>("panel/plan_maker/color_r", config_.plan_maker_color_r, 1.0);
    pnh.param<double>("panel/plan_maker/color_g", config_.plan_maker_color_g, 0.65);
    pnh.param<double>("panel/plan_maker/color_b", config_.plan_maker_color_b, 0.0);
    pnh.param<double>("panel/plan_maker/color_a", config_.plan_maker_color_a, 0.9);
    pnh.param<double>("panel/plan_maker/trajectory_width", config_.trajectory_width, 0.05);

    pnh.param<double>("panel/marker/arrow_scale_x", config_.arrow_scale_x, 0.6);
    pnh.param<double>("panel/marker/arrow_scale_y", config_.arrow_scale_y, 0.15);
    pnh.param<double>("panel/marker/arrow_scale_z", config_.arrow_scale_z, 0.15);
    pnh.param<double>("panel/marker/number_scale", config_.number_scale, 0.8);
    pnh.param<double>("panel/marker/number_offset_z", config_.number_offset_z, 0.6);
    pnh.param<double>("panel/marker/color_r", config_.color_r, 1.0);
    pnh.param<double>("panel/marker/color_g", config_.color_g, 0.84);
    pnh.param<double>("panel/marker/color_b", config_.color_b, 0.0);
    pnh.param<double>("panel/marker/color_a", config_.color_a, 1.0);
    pnh.param<int>("panel/table/default_max_goals", config_.default_max_goals, 10);
    pnh.param<int>("panel/spin_timer_ms", config_.spin_timer_ms, 100);
    pnh.param<std::string>("panel/default_config_path", config_.default_config_path, "/home/groundstation/catkin_ws/config.yaml");
    pnh.param<std::string>("topics/config_loaded_topic", config_.config_loaded_topic, "uav/config/loaded");
    pnh.param<std::string>("topics/config_reload_topic", config_.config_reload_topic, "uav/config/reload");
    pnh.param<std::string>("panel/waypoint_current_topic", config_.waypoint_current_topic, "uav/waypoints/current");
    pnh.param<std::string>("panel/waypoint_params_input_topic", config_.waypoint_params_input_topic, "uav/waypoints/params");
    pnh.param<std::string>("panel/waypoint_params_loaded_topic", config_.waypoint_params_loaded_topic, "uav/waypoints/params_loaded");

    // 文件路径
    pnh.param<std::string>("paths/default_save", config_.default_save_path, "/home/groundstation/waypoints.xml");
    pnh.param<std::string>("paths/default_load", config_.default_load_path, "/home/groundstation/waypoints.xml");
    pnh.param<std::string>("paths/default_frame_id", config_.default_frame_id, "map");

    // 从全局命名空间读取默认飞行参数
    ros::NodeHandle global_nh;
    global_nh.param<double>("flight_defaults/hover_duration", default_hover_time_, 5.0);
    global_nh.param<double>("flight_defaults/travel_speed", default_speed_, 2.0);

    max_num_goal_ = config_.default_max_goals;
}

// ========== 配置区域折叠 ==========
void WaypointPanel::onConfigGroupToggled(bool checked) {
    config_display_->setVisible(checked);
    load_config_button_->setVisible(checked);
}
void WaypointPanel::logInfo(const QString &msg) {
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    log_text_->append(QString("<span style='color:#4CAF50'>[%1] INFO</span> %2").arg(timestamp, msg));
    // 限制日志行数，防止长时间运行内存膨胀
    truncateLog();
}
void WaypointPanel::logWarn(const QString &msg) {
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    log_text_->append(QString("<span style='color:#FF9800'>[%1] WARN</span> %2").arg(timestamp, msg));
    truncateLog();
}
void WaypointPanel::logError(const QString &msg) {
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    log_text_->append(QString("<span style='color:#F44336'>[%1] ERROR</span> %2").arg(timestamp, msg));
    truncateLog();
}

void WaypointPanel::truncateLog() {
    const int MAX_LOG_LINES = 500;
    QTextDocument *doc = log_text_->document();
    while (doc->blockCount() > MAX_LOG_LINES) {
        QTextCursor cursor(doc->begin());
        cursor.select(QTextCursor::BlockUnderCursor);
        cursor.removeSelectedText();
        cursor.deleteChar();  // 删除换行符
    }
}

// ========== 状态转换辅助 ==========
QString WaypointPanel::stateToString(uint8_t state) {
    switch (state) {
        case 0: return "IDLE";
        case 1: return "PRE_FLIGHT";
        case 2: return "ARMING";
        case 3: return "TAKEOFF";
        case 4: return "NAVIGATING";
        case 5: return "HOVERING";
        case 6: return "LANDING";
        case 7: return "LANDED";
        case 8: return "EMERGENCY";
        case 9: return "RETURNING";
        default: return "UNKNOWN";
    }
}

QString WaypointPanel::stateToColor(uint8_t state) {
    switch (state) {
        case 0: return "#2196F3";  // IDLE - blue
        case 1: return "#FF9800";  // PRE_FLIGHT - orange
        case 2: return "#FF9800";  // ARMING - orange
        case 3: return "#4CAF50";  // TAKEOFF - green
        case 4: return "#4CAF50";  // NAVIGATING - green
        case 5: return "#9C27B0";  // HOVERING - purple
        case 6: return "#FF5722";  // LANDING - deep orange
        case 7: return "#2196F3";  // LANDED - blue
        case 8: return "#F44336";  // EMERGENCY - red
        case 9: return "#FF5722";  // RETURNING - deep orange
        default: return "#757575"; // UNKNOWN - grey
    }
}

void WaypointPanel::receiveGoal(const geometry_msgs::PoseStamped::ConstPtr &pose) {
    if (plan_maker_phase_ == NAVIGATING) {
        logWarn("任务执行中，请先重置或完成任务后再打点");
        return;
    }

    if (plan_maker_points_.size() >= static_cast<size_t>(max_num_goal_)) {
        logError(QString("规划点数量已达上限: %1").arg(max_num_goal_));
        QMessageBox::warning(this, "警告", QString("规划点数量已达上限 %1").arg(max_num_goal_));
        return;
    }
    logInfo(QString("接收到新规划点: (%1, %2, %3)")
            .arg(pose->pose.position.x, 0, 'f', 2)
            .arg(pose->pose.position.y, 0, 'f', 2)
            .arg(pose->pose.position.z, 0, 'f', 2));

    geometry_msgs::PoseStamped pose_stamped = *pose;
    pose_stamped.header.frame_id = config_.default_frame_id;
    addPlanMakerPoint(pose_stamped);

    // 同步添加到航点表格（方便用户编辑 z/yaw/hover/speed）
    addWaypointToTable(pose_stamped.pose, default_hover_time_, default_speed_);
    // 注：可视化由 addPlanMakerPoint() → publishPlanMakerMarkers() 统一管理
    // 不再单独调用 markWaypoint()，避免双重视觉标记（金色箭头 + 橙色球体）

    setPlanMakerPhase(PLANNING);
    updatePlanMakerStatus();
}

void WaypointPanel::addPlanMakerPoint(const geometry_msgs::PoseStamped &pose) {
    plan_maker_points_.push_back(pose);
    plan_maker_selected_index_ = static_cast<int>(plan_maker_points_.size()) - 1;
    publishPlanMakerMarkers();
}

void WaypointPanel::publishPlanMakerMarkers() {
    visualization_msgs::MarkerArray marker_array;

    // 先发送删除所有旧标记
    visualization_msgs::Marker delete_all;
    delete_all.header.frame_id = config_.default_frame_id;
    delete_all.header.stamp = ros::Time::now();
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    // marker 生命周期：面板定时器每 ~300ms 刷新一次，lifetime 设 1 秒
    // 面板崩溃后 marker 在 1 秒内自动消失，不留僵尸可视化
    ros::Duration marker_lifetime(1.0);

    // 判断是否处于导航执行中（用于颜色编码）
    bool is_navigating = (current_nav_state_ == uav_navigator::NavigatorStatus::STATE_TAKEOFF
                       || current_nav_state_ == uav_navigator::NavigatorStatus::STATE_NAVIGATING
                       || current_nav_state_ == uav_navigator::NavigatorStatus::STATE_HOVERING
                       || current_nav_state_ == uav_navigator::NavigatorStatus::STATE_RETURNING);

    for (size_t i = 0; i < plan_maker_points_.size(); ++i) {
        // 颜色编码：导航中按进度着色，规划中统一橙色
        float r, g, b, a;
        float sphere_scale = config_.plan_maker_sphere_scale;
        if (is_navigating && nav_total_waypoints_ > 0) {
            if (static_cast<uint8_t>(i) < nav_current_waypoint_idx_) {
                // 已到达：绿色
                r = 0.30f; g = 0.69f; b = 0.31f; a = 0.7f;  // #4CAF50
            } else if (static_cast<uint8_t>(i) == nav_current_waypoint_idx_) {
                // 当前目标：亮橙色 + 放大
                r = 1.00f; g = 0.60f; b = 0.00f; a = 1.0f;  // #FF9800
                sphere_scale *= 1.5f;
            } else {
                // 待飞行：蓝色
                r = 0.13f; g = 0.59f; b = 0.95f; a = 0.8f;  // #2196F3
            }
        } else {
            // 规划模式：橙色
            r = config_.plan_maker_color_r;
            g = config_.plan_maker_color_g;
            b = config_.plan_maker_color_b;
            a = config_.plan_maker_color_a;
        }

        // 选中高亮
        if (static_cast<int>(i) == plan_maker_selected_index_) {
            g = 1.0f;
            sphere_scale = std::max(sphere_scale, static_cast<float>(config_.plan_maker_sphere_scale * 1.3));
        }

        // === 位置球（精确标记航点位置）===
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = config_.default_frame_id;
        sphere.header.stamp = ros::Time::now();
        sphere.ns = "uav_plan_maker_points";
        sphere.id = static_cast<int>(i);
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.pose = plan_maker_points_[i].pose;
        sphere.scale.x = sphere_scale;
        sphere.scale.y = sphere_scale;
        sphere.scale.z = sphere_scale;
        sphere.color.r = r;
        sphere.color.g = g;
        sphere.color.b = b;
        sphere.color.a = a;
        sphere.lifetime = marker_lifetime;
        marker_array.markers.push_back(sphere);

        // === 方向箭头（显示 yaw 朝向）===
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = config_.default_frame_id;
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "uav_plan_maker_arrows";
        arrow.id = static_cast<int>(i);
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.pose = plan_maker_points_[i].pose;
        // 箭头尺寸： shaft=长, head=宽
        arrow.scale.x = config_.arrow_scale_x;   // 箭头总长
        arrow.scale.y = config_.arrow_scale_y;   // 箭头宽度
        arrow.scale.z = config_.arrow_scale_z;   // 箭头高度
        arrow.color.r = r;
        arrow.color.g = g;
        arrow.color.b = b;
        arrow.color.a = std::min(a + 0.15f, 1.0f);
        arrow.lifetime = marker_lifetime;
        marker_array.markers.push_back(arrow);

        // === 数字标签 ===
        visualization_msgs::Marker text;
        text.header.frame_id = config_.default_frame_id;
        text.header.stamp = ros::Time::now();
        text.ns = "uav_plan_maker_numbers";
        text.id = static_cast<int>(i);
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose = plan_maker_points_[i].pose;
        text.pose.position.z += config_.number_offset_z;
        text.scale.z = config_.number_scale;
        text.color.r = 1.0f;
        text.color.g = 1.0f;
        text.color.b = 1.0f;
        text.color.a = 1.0f;
        text.lifetime = marker_lifetime;
        text.text = std::to_string(i + 1);
        marker_array.markers.push_back(text);
    }

    plan_maker_points_pub_.publish(marker_array);
}

void WaypointPanel::publishPlanTrajectory() {
    nav_msgs::Path path;
    path.header.frame_id = config_.default_frame_id;
    path.header.stamp = ros::Time::now();

    for (const auto &pose : plan_maker_points_) {
        geometry_msgs::PoseStamped ps;
        ps.header = path.header;
        ps.pose = pose.pose;
        path.poses.push_back(ps);
    }

    plan_maker_trajectory_pub_.publish(path);
}

void WaypointPanel::connectPlanTrajectory() {
    if (plan_maker_points_.size() < 2) {
        logWarn("至少需要2个点才能连接成轨迹");
        QMessageBox::warning(this, "警告", "至少需要2个点才能连接成轨迹");
        return;
    }
    publishPlanTrajectory();
    setPlanMakerPhase(CONNECTED);
    updatePlanMakerStatus();
    logInfo(QString("已连接 %1 个规划点形成轨迹").arg(plan_maker_points_.size()));
}

void WaypointPanel::deleteSelectedPlanPoint() {
    if (plan_maker_points_.empty()) {
        logWarn("没有可删除的规划点");
        return;
    }

    int idx = plan_maker_selected_index_;
    if (idx < 0 || idx >= static_cast<int>(plan_maker_points_.size())) {
        // 默认删除最后一个
        idx = static_cast<int>(plan_maker_points_.size()) - 1;
    }

    // 不直接删除 plan_maker_points_ —— deleteSelectedWaypoint() 统一管理数据结构的删除
    // 避免 double-erase bug
    if (idx < current_waypoint_count_) {
        waypoint_table_->setCurrentCell(idx, 0);
        deleteSelectedWaypoint();
    } else {
        // 表格中没有对应的行，直接从数据结构中删除
        plan_maker_points_.erase(plan_maker_points_.begin() + idx);
        if (plan_maker_selected_index_ >= static_cast<int>(plan_maker_points_.size())) {
            plan_maker_selected_index_ = static_cast<int>(plan_maker_points_.size()) - 1;
        }
    }

    publishPlanMakerMarkers();
    if (plan_maker_phase_ == CONNECTED || plan_maker_phase_ == SAVED) {
        publishPlanTrajectory();
    }
    setPlanMakerPhase(PLANNING);
    updatePlanMakerStatus();
    logInfo(QString("已删除规划点 %1").arg(idx + 1));
}

void WaypointPanel::clearPlanPoints() {
    plan_maker_points_.clear();
    plan_maker_selected_index_ = -1;
    waypoint_hover_times_.clear();
    waypoint_speeds_.clear();
    clearMarkers();

    // 清空轨迹
    nav_msgs::Path empty_path;
    empty_path.header.frame_id = config_.default_frame_id;
    empty_path.header.stamp = ros::Time::now();
    plan_maker_trajectory_pub_.publish(empty_path);

    // 清空表格（6列）
    waypoint_table_->blockSignals(true);
    for (int i = 0; i < max_num_goal_; ++i) {
        for (int j = 0; j < 6; ++j) {
            waypoint_table_->setItem(i, j, nullptr);
        }
    }
    waypoint_table_->blockSignals(false);
    current_waypoint_count_ = 0;

    setPlanMakerPhase(PLANNING);
    updatePlanMakerStatus();
    logInfo("已清除所有规划点和航点");
}

void WaypointPanel::savePlanWaypoints() {
    if (plan_maker_points_.empty()) {
        logWarn("没有可保存的规划点");
        return;
    }

    geometry_msgs::PoseArray waypoints;
    waypoints.header.frame_id = config_.default_frame_id;
    waypoints.header.stamp = ros::Time::now();

    // 从 plan_maker_points_ 读取（唯一数据源），同时从表格读取最新编辑值
    waypoint_hover_times_.clear();
    waypoint_speeds_.clear();
    for (size_t i = 0; i < plan_maker_points_.size(); ++i) {
        geometry_msgs::Pose pose = plan_maker_points_[i].pose;
        double hover_time = default_hover_time_;
        double speed = default_speed_;

        if (static_cast<int>(i) < current_waypoint_count_) {
            QTableWidgetItem *xItem = waypoint_table_->item(static_cast<int>(i), 0);
            QTableWidgetItem *yItem = waypoint_table_->item(static_cast<int>(i), 1);
            QTableWidgetItem *zItem = waypoint_table_->item(static_cast<int>(i), 2);
            QTableWidgetItem *yawItem = waypoint_table_->item(static_cast<int>(i), 3);
            QTableWidgetItem *hoverItem = waypoint_table_->item(static_cast<int>(i), 4);
            QTableWidgetItem *speedItem = waypoint_table_->item(static_cast<int>(i), 5);
            if (xItem) pose.position.x = xItem->text().toDouble();
            if (yItem) pose.position.y = yItem->text().toDouble();
            if (zItem) pose.position.z = zItem->text().toDouble();
            if (yawItem) {
                double yaw_deg = yawItem->text().toDouble();
                double yaw_rad = yaw_deg * M_PI / 180.0;
                pose.orientation = tf::createQuaternionMsgFromYaw(yaw_rad);
            }
            if (hoverItem) hover_time = hoverItem->text().toDouble();
            if (speedItem) speed = speedItem->text().toDouble();
        }
        waypoints.poses.push_back(pose);
        waypoint_hover_times_.push_back(hover_time);
        waypoint_speeds_.push_back(speed);
    }

    confirmed_waypoint_count_ = static_cast<uint8_t>(waypoints.poses.size());
    waypoint_pub_.publish(waypoints);

    // 同步发布 per-waypoint 参数
    std_msgs::Float64MultiArray params_msg;
    params_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    params_msg.layout.dim[0].size = waypoint_hover_times_.size();
    params_msg.layout.dim[0].stride = 2;
    params_msg.layout.dim[0].label = "waypoint_params";
    for (size_t i = 0; i < waypoint_hover_times_.size(); ++i) {
        params_msg.data.push_back(waypoint_hover_times_[i]);
        params_msg.data.push_back(waypoint_speeds_[i]);
    }
    waypoint_params_pub_.publish(params_msg);

    logInfo(QString("已发布 %1 个航点到 waypoint_manager（含悬停/速度参数），等待 navigator 确认...")
            .arg(confirmed_waypoint_count_));
    // navigator 确认在 receiveNavStatus() 中自动处理，无需单独定时器轮询
}

void WaypointPanel::setPlanMakerPhase(PlanMakerPhase phase) {
    plan_maker_phase_ = phase;
    updateAllButtonStates();
}

void WaypointPanel::updatePlanMakerStatus() {
    plan_maker_status_label_->setText(QString("状态: %1 | 点数: %2")
        .arg(phaseToString(plan_maker_phase_))
        .arg(plan_maker_points_.size()));

    switch (plan_maker_phase_) {
        case PLANNING:
            plan_maker_status_label_->setStyleSheet("color: #2196F3; font-weight: bold; padding: 4px;");
            break;
        case CONNECTED:
            plan_maker_status_label_->setStyleSheet("color: #FF9800; font-weight: bold; padding: 4px;");
            break;
        case SAVED:
            plan_maker_status_label_->setStyleSheet("color: #4CAF50; font-weight: bold; padding: 4px;");
            break;
        case NAVIGATING:
            plan_maker_status_label_->setStyleSheet("color: #9C27B0; font-weight: bold; padding: 4px;");
            break;
    }
}

QString WaypointPanel::phaseToString(PlanMakerPhase phase) {
    switch (phase) {
        case PLANNING: return "PLANNING";
        case CONNECTED: return "CONNECTED";
        case SAVED: return "SAVED";
        case NAVIGATING: return "NAVIGATING";
        default: return "UNKNOWN";
    }
}

void WaypointPanel::loadConfigFromFile() {
    QString default_path = QString::fromStdString(config_.default_config_path);
    QString file_path = QFileDialog::getOpenFileName(this, "加载配置文件", default_path, "YAML Files (*.yaml *.yml)");
    if (file_path.isEmpty()) return;

    logInfo(QString("正在加载配置文件: %1").arg(file_path));

    // 使用异步 QProcess，不阻塞 UI 线程
    QProcess *proc = new QProcess(this);
    QStringList args;
    args << "load" << file_path;

    QObject::connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        this, [this, file_path, proc](int exitCode, QProcess::ExitStatus) {
            proc->deleteLater();

            if (exitCode != 0) {
                logError(QString("rosparam load 失败，退出码: %1").arg(exitCode));
                return;
            }

            // 读取 YAML 并在面板只读显示关键参数
            QString display_text;
            try {
                YAML::Node root = YAML::LoadFile(file_path.toStdString());
                display_text += QString("配置文件: %1\n").arg(file_path);
                display_text += "──────────────────────────────\n";

                auto append_section = [&](const QString &title, const YAML::Node &node) {
                    QString section = title + "\n";
                    if (node && node.IsMap()) {
                        for (const auto &kv : node) {
                            std::string key = kv.first.as<std::string>();
                            std::string value;
                            if (kv.second.IsScalar()) {
                                value = kv.second.as<std::string>();
                            } else {
                                std::stringstream ss;
                                ss << kv.second;
                                value = ss.str();
                            }
                            section += QString("  %1: %2\n").arg(QString::fromStdString(key), QString::fromStdString(value));
                        }
                    }
                    return section;
                };

                display_text += append_section("[flight]", root["flight"]);
                display_text += append_section("[waypoint]", root["waypoint"]);
                display_text += append_section("[safety]", root["safety"]);
                display_text += append_section("[offboard_safety]", root["offboard_safety"]);
                display_text += append_section("[position_safety]", root["position_safety"]);
                display_text += append_section("[experiment]", root["experiment"]);
                display_text += append_section("[panel]", root["panel"]);
            } catch (const std::exception &e) {
                logError(QString("解析 YAML 失败: %1").arg(e.what()));
                display_text = QString("配置文件已加载，但解析失败: %1").arg(e.what());
            }
            config_display_->setPlainText(display_text);

            // 面板自身重新加载配置（更新本地默认值，必须在 rosparam load 完成后）
            loadConfig();
            logInfo(QString("  悬停时间默认值: %1 s, 飞行速度默认值: %2 m/s")
                    .arg(default_hover_time_, 0, 'f', 1)
                    .arg(default_speed_, 0, 'f', 1));

            // 发布加载事件
            publishConfigLoaded(file_path, display_text);

            // 通知所有核心节点立即重新加载配置
            std_msgs::String reload_msg;
            reload_msg.data = file_path.toStdString();
            config_reload_pub_.publish(reload_msg);
            ROS_INFO("[WaypointPanel] Published config reload event to '%s'", config_.config_reload_topic.c_str());

            logInfo("✓ 配置文件已加载并立即生效（面板 + 所有核心节点已同步更新）");
        });

    // 超时保护：5 秒后强制终止
    QTimer::singleShot(5000, proc, [proc, this]() {
        if (proc->state() == QProcess::Running) {
            logError("rosparam load 超时");
            proc->kill();
        }
    });

    proc->start("rosparam", args);
}

void WaypointPanel::publishConfigLoaded(const QString &path, const QString &summary) {
    std_msgs::String msg;
    QString payload = QString("[CONFIG_LOADED] path=%1\n%2").arg(path).arg(summary);
    msg.data = payload.toStdString();
    ROS_INFO("[WaypointPanel] Publishing config loaded event to topic '%s'", config_.config_loaded_topic.c_str());
    config_loaded_pub_.publish(msg);
}

// ========== 航点表格操作 ==========
void WaypointPanel::initPoseTable() {
    current_waypoint_count_ = 0;
    waypoint_table_->clear();
    waypoint_table_->setRowCount(max_num_goal_);
    waypoint_table_->setColumnCount(6);
    waypoint_table_->setEditTriggers(QAbstractItemView::AllEditTriggers);
    waypoint_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    QStringList headers = { "x", "y", "z", "yaw(°)", "悬停(s)", "速度(m/s)" };
    waypoint_table_->setHorizontalHeaderLabels(headers);
    waypoint_table_->verticalHeader()->setVisible(true);
    waypoint_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    waypoint_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    for (int i = 0; i < max_num_goal_; ++i) {
        waypoint_table_->setVerticalHeaderItem(i, new QTableWidgetItem(QString::number(i + 1)));
    }
}

void WaypointPanel::updatePoseTable() {
    int old_count = waypoint_table_->rowCount();
    waypoint_table_->setRowCount(max_num_goal_);
    for (int i = old_count; i < max_num_goal_; ++i) {
        waypoint_table_->setVerticalHeaderItem(i, new QTableWidgetItem(QString::number(i + 1)));
    }
}

void WaypointPanel::updateMaxNumGoal() {
    QString text = max_num_goal_editor_->text();
    bool ok;
    int new_max = text.toInt(&ok);
    if (ok && new_max > 0 && new_max <= 100) {
        max_num_goal_ = new_max;
        logInfo(QString("最大航点数量更新为: %1").arg(max_num_goal_));
    } else {
        logWarn("无效的最大航点数量");
        max_num_goal_editor_->setText(QString::number(max_num_goal_));
    }
}

void WaypointPanel::addWaypointToTable(const geometry_msgs::Pose &pose, double hover_time, double speed) {
    if (current_waypoint_count_ >= max_num_goal_) return;
    int row = current_waypoint_count_;
    waypoint_table_->blockSignals(true);
    QTableWidgetItem *xItem = new QTableWidgetItem(QString::number(pose.position.x, 'f', 2));
    QTableWidgetItem *yItem = new QTableWidgetItem(QString::number(pose.position.y, 'f', 2));
    QTableWidgetItem *zItem = new QTableWidgetItem(QString::number(pose.position.z, 'f', 2));
    double yaw = tf::getYaw(pose.orientation) * 180.0 / M_PI;
    QTableWidgetItem *yawItem = new QTableWidgetItem(QString::number(yaw, 'f', 2));
    QTableWidgetItem *hoverItem = new QTableWidgetItem(QString::number(hover_time, 'f', 1));
    QTableWidgetItem *speedItem = new QTableWidgetItem(QString::number(speed, 'f', 1));
    waypoint_table_->setItem(row, 0, xItem);
    waypoint_table_->setItem(row, 1, yItem);
    waypoint_table_->setItem(row, 2, zItem);
    waypoint_table_->setItem(row, 3, yawItem);
    waypoint_table_->setItem(row, 4, hoverItem);
    waypoint_table_->setItem(row, 5, speedItem);
    waypoint_table_->blockSignals(false);
    current_waypoint_count_++;
}

void WaypointPanel::deleteSelectedWaypoint() {
    int row = waypoint_table_->currentRow();
    if (row < 0 || row >= current_waypoint_count_) {
        QMessageBox::warning(this, "警告", "请先选中一个航点");
        return;
    }
    // 同步删除 plan_maker_points_ 和 param vectors
    if (row < static_cast<int>(plan_maker_points_.size())) {
        plan_maker_points_.erase(plan_maker_points_.begin() + row);
        plan_maker_selected_index_ = std::min(plan_maker_selected_index_,
            static_cast<int>(plan_maker_points_.size()) - 1);
    }
    if (row < static_cast<int>(waypoint_hover_times_.size())) {
        waypoint_hover_times_.erase(waypoint_hover_times_.begin() + row);
    }
    if (row < static_cast<int>(waypoint_speeds_.size())) {
        waypoint_speeds_.erase(waypoint_speeds_.begin() + row);
    }

    // 更新表格（6列）
    waypoint_table_->blockSignals(true);
    for (int i = row; i < current_waypoint_count_ - 1; ++i) {
        for (int j = 0; j < 6; ++j) {
            QTableWidgetItem *item = waypoint_table_->item(i + 1, j);
            if (item) {
                waypoint_table_->setItem(i, j, new QTableWidgetItem(item->text()));
            } else {
                waypoint_table_->setItem(i, j, nullptr);
            }
        }
    }
    for (int j = 0; j < 6; ++j) {
        waypoint_table_->setItem(current_waypoint_count_ - 1, j, nullptr);
    }
    waypoint_table_->blockSignals(false);
    current_waypoint_count_--;

    // 重新发布可视化
    if (plan_maker_phase_ == CONNECTED || plan_maker_phase_ == SAVED) {
        publishPlanTrajectory();
    }
    publishPlanMakerMarkers();
    updatePlanMakerStatus();
    logInfo(QString("删除航点 %1, 剩余 %2 个").arg(row + 1).arg(current_waypoint_count_));
}

void WaypointPanel::moveWaypointUp() {
    int row = waypoint_table_->currentRow();
    if (row <= 0 || row >= current_waypoint_count_) return;

    // 同步 plan_maker_points_
    if (row < static_cast<int>(plan_maker_points_.size())) {
        std::swap(plan_maker_points_[row], plan_maker_points_[row - 1]);
        std::swap(waypoint_hover_times_[row], waypoint_hover_times_[row - 1]);
        std::swap(waypoint_speeds_[row], waypoint_speeds_[row - 1]);
    }

    waypoint_table_->blockSignals(true);
    for (int j = 0; j < 6; ++j) {
        QTableWidgetItem *item1 = waypoint_table_->item(row, j);
        QTableWidgetItem *item0 = waypoint_table_->item(row - 1, j);
        QString text1 = item1 ? item1->text() : "";
        QString text0 = item0 ? item0->text() : "";
        waypoint_table_->setItem(row, j, new QTableWidgetItem(text0));
        waypoint_table_->setItem(row - 1, j, new QTableWidgetItem(text1));
    }
    waypoint_table_->blockSignals(false);
    waypoint_table_->selectRow(row - 1);
    // 同步 plan_maker_selected_index_
    if (plan_maker_selected_index_ == row) {
        plan_maker_selected_index_ = row - 1;
    } else if (plan_maker_selected_index_ == row - 1) {
        plan_maker_selected_index_ = row;
    }
    publishPlanMakerMarkers();
    if (plan_maker_phase_ == CONNECTED || plan_maker_phase_ == SAVED) {
        publishPlanTrajectory();
    }
}

void WaypointPanel::moveWaypointDown() {
    int row = waypoint_table_->currentRow();
    if (row < 0 || row >= current_waypoint_count_ - 1) return;

    if (row + 1 < static_cast<int>(plan_maker_points_.size())) {
        std::swap(plan_maker_points_[row], plan_maker_points_[row + 1]);
        std::swap(waypoint_hover_times_[row], waypoint_hover_times_[row + 1]);
        std::swap(waypoint_speeds_[row], waypoint_speeds_[row + 1]);
    }

    waypoint_table_->blockSignals(true);
    for (int j = 0; j < 6; ++j) {
        QTableWidgetItem *item1 = waypoint_table_->item(row, j);
        QTableWidgetItem *item2 = waypoint_table_->item(row + 1, j);
        QString text1 = item1 ? item1->text() : "";
        QString text2 = item2 ? item2->text() : "";
        waypoint_table_->setItem(row, j, new QTableWidgetItem(text2));
        waypoint_table_->setItem(row + 1, j, new QTableWidgetItem(text1));
    }
    waypoint_table_->blockSignals(false);
    waypoint_table_->selectRow(row + 1);
    // 同步 plan_maker_selected_index_
    if (plan_maker_selected_index_ == row) {
        plan_maker_selected_index_ = row + 1;
    } else if (plan_maker_selected_index_ == row + 1) {
        plan_maker_selected_index_ = row;
    }
    publishPlanMakerMarkers();
    if (plan_maker_phase_ == CONNECTED || plan_maker_phase_ == SAVED) {
        publishPlanTrajectory();
    }
}

void WaypointPanel::clearWaypoints() {
    // 清除表格
    waypoint_table_->blockSignals(true);
    for (int i = 0; i < max_num_goal_; ++i) {
        for (int j = 0; j < 6; ++j) {
            waypoint_table_->setItem(i, j, nullptr);
        }
    }
    waypoint_table_->blockSignals(false);
    current_waypoint_count_ = 0;

    // 清除所有数据层（与 clearPlanPoints 保持一致）
    plan_maker_points_.clear();
    waypoint_hover_times_.clear();
    waypoint_speeds_.clear();
    plan_maker_selected_index_ = -1;

    clearMarkers();

    // 清空轨迹
    nav_msgs::Path empty_path;
    empty_path.header.frame_id = config_.default_frame_id;
    empty_path.header.stamp = ros::Time::now();
    plan_maker_trajectory_pub_.publish(empty_path);

    // 发布空 PoseArray 通知 waypoint_manager 和 navigator 清除航点
    geometry_msgs::PoseArray empty_waypoints;
    empty_waypoints.header.frame_id = config_.default_frame_id;
    empty_waypoints.header.stamp = ros::Time::now();
    waypoint_pub_.publish(empty_waypoints);

    // 发布空参数数组
    std_msgs::Float64MultiArray empty_params;
    waypoint_params_pub_.publish(empty_params);
    confirmed_waypoint_count_ = 0;

    setPlanMakerPhase(PLANNING);
    updatePlanMakerStatus();
    updateAllButtonStates();
    logInfo("所有航点已清除");
}

void WaypointPanel::saveWaypoints() {
    if (current_waypoint_count_ == 0) {
        QMessageBox::warning(this, "警告", "没有航点可保存");
        return;
    }
    publishWaypoints();
    QString default_path = QString::fromStdString(config_.default_save_path);
    QString filePath = QFileDialog::getSaveFileName(this, "保存航点", default_path, "XML Files (*.xml)");
    if (filePath.isEmpty()) return;
    uav_waypoint_manager::SaveWaypoints srv;
    srv.request.file_path = filePath.toStdString();
    if (save_waypoints_client_.call(srv)) {
        if (srv.response.success) {
            logInfo(QString::fromStdString(srv.response.message));
            QMessageBox::information(this, "成功", QString::fromStdString(srv.response.message));
        } else {
            logWarn(QString::fromStdString(srv.response.message));
        }
    } else {
        logError("保存服务调用失败");
    }
}

void WaypointPanel::loadWaypoints() {
    QString default_path = QString::fromStdString(config_.default_load_path);
    QString filePath = QFileDialog::getOpenFileName(this, "加载航点", default_path, "XML Files (*.xml)");
    if (filePath.isEmpty()) return;
    uav_waypoint_manager::LoadWaypoints srv;
    srv.request.file_path = filePath.toStdString();
    if (load_waypoints_client_.call(srv)) {
        if (srv.response.success) {
            logInfo(QString::fromStdString(srv.response.message));

            // 直接从服务响应获取航点数据（不再 waitForMessage）
            const auto& waypoints = srv.response.waypoints;

            // 1. 清空旧数据
            clearPlanPoints();

            // 2. 填充 plan_maker_points_（核心数据结构，唯一数据源）
            plan_maker_points_.clear();
            waypoint_hover_times_.clear();
            waypoint_speeds_.clear();
            for (const auto& pose : waypoints.poses) {
                geometry_msgs::PoseStamped ps;
                ps.header.frame_id = config_.default_frame_id;
                ps.header.stamp = ros::Time::now();
                ps.pose = pose;
                plan_maker_points_.push_back(ps);
                // 先用默认值，params topic 到达后会更新
                waypoint_hover_times_.push_back(default_hover_time_);
                waypoint_speeds_.push_back(default_speed_);
            }

            // 3. 填充表格（使用默认 hover_time/speed，params topic 到达后更新）
            for (size_t i = 0; i < plan_maker_points_.size(); ++i) {
                addWaypointToTable(plan_maker_points_[i].pose,
                                   waypoint_hover_times_[i],
                                   waypoint_speeds_[i]);
            }

            // 4. 发布可视化（橙色球体 + 编号）
            publishPlanMakerMarkers();

            // 5. 自动连接轨迹
            if (plan_maker_points_.size() >= 2) {
                publishPlanTrajectory();
                setPlanMakerPhase(CONNECTED);
                logInfo(QString("✓ 已加载 %1 个航点，可视化已更新。请确认路径后点击\"保存\"→\"发布\"")
                        .arg(plan_maker_points_.size()));
            } else if (plan_maker_points_.size() == 1) {
                setPlanMakerPhase(PLANNING);
                logInfo(QString("✓ 已加载 1 个航点，请继续添加航点"));
            }

            updatePlanMakerStatus();
            // per-waypoint 参数（hover_time/speed）由 receiveWaypointParams 回调异步更新，
            // 不再使用阻塞式 waitForMessage 冻结 UI
        } else {
            logWarn(QString::fromStdString(srv.response.message));
        }
    } else {
        logError("加载服务调用失败");
    }
}

void WaypointPanel::publishWaypoints() {
    geometry_msgs::PoseArray waypoints = readWaypointsFromTable();
    if (waypoints.poses.empty()) {
        QMessageBox::warning(this, "警告", "没有有效航点可发布");
        return;
    }
    waypoints.header.stamp = ros::Time::now();
    waypoints.header.frame_id = config_.default_frame_id;
    waypoint_pub_.publish(waypoints);

    // 同步发布 per-waypoint 参数
    std_msgs::Float64MultiArray params_msg;
    params_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    params_msg.layout.dim[0].size = waypoint_hover_times_.size();
    params_msg.layout.dim[0].stride = 2;
    params_msg.layout.dim[0].label = "waypoint_params";
    for (size_t i = 0; i < waypoint_hover_times_.size() && i < waypoint_speeds_.size(); ++i) {
        params_msg.data.push_back(waypoint_hover_times_[i]);
        params_msg.data.push_back(waypoint_speeds_[i]);
    }
    waypoint_params_pub_.publish(params_msg);

    logInfo(QString("已发布 %1 个航点（含悬停/速度参数）").arg(waypoints.poses.size()));
}

geometry_msgs::PoseArray WaypointPanel::readWaypointsFromTable() {
    // 改为从 plan_maker_points_ 读取（唯一数据源）
    geometry_msgs::PoseArray waypoints;
    waypoints.header.frame_id = config_.default_frame_id;
    waypoints.header.stamp = ros::Time::now();
    for (size_t i = 0; i < plan_maker_points_.size(); ++i) {
        waypoints.poses.push_back(plan_maker_points_[i].pose);
    }
    return waypoints;
}

void WaypointPanel::onTableChanged(int row, int column) {
    if (row < 0 || row >= static_cast<int>(plan_maker_points_.size())) return;
    if (row >= current_waypoint_count_) return;

    QTableWidgetItem* item = waypoint_table_->item(row, column);
    if (!item) return;

    bool ok = false;
    double val = item->text().toDouble(&ok);
    if (!ok) return;

    // 同步回 plan_maker_points_（唯一数据源）
    switch (column) {
        case 0: plan_maker_points_[row].pose.position.x = val; break;
        case 1: plan_maker_points_[row].pose.position.y = val; break;
        case 2: plan_maker_points_[row].pose.position.z = val; break;
        case 3: {
            double yaw_rad = val * M_PI / 180.0;
            tf::Quaternion q;
            q.setRPY(0, 0, yaw_rad);
            plan_maker_points_[row].pose.orientation.x = q.x();
            plan_maker_points_[row].pose.orientation.y = q.y();
            plan_maker_points_[row].pose.orientation.z = q.z();
            plan_maker_points_[row].pose.orientation.w = q.w();
            break;
        }
        case 4:
            if (static_cast<size_t>(row) < waypoint_hover_times_.size())
                waypoint_hover_times_[row] = val;
            break;
        case 5:
            if (static_cast<size_t>(row) < waypoint_speeds_.size())
                waypoint_speeds_[row] = val;
            break;
    }

    plan_maker_dirty_ = true;
    publishPlanMakerMarkers();
    if (plan_maker_phase_ == CONNECTED || plan_maker_phase_ == SAVED) {
        publishPlanTrajectory();
    }
}

// ========== Marker（LEGACY — 仅保留接口兼容性，新流程使用 publishPlanMakerMarkers()）==========
// 注意：以下两个函数不再被新代码路径调用，保留仅为向后兼容。
// 若意外调用 markWaypoint()，其 lifetime=0 会产生僵尸 marker。
void WaypointPanel::markWaypoint(const geometry_msgs::PoseStamped &pose, int id) {
    if (!ros::ok()) return;
    visualization_msgs::Marker arrow;
    arrow.header = pose.header;
    arrow.ns = "uav_waypoint_arrow";
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.pose = pose.pose;
    arrow.scale.x = config_.arrow_scale_x;
    arrow.scale.y = config_.arrow_scale_y;
    arrow.scale.z = config_.arrow_scale_z;
    arrow.color.r = config_.color_r;
    arrow.color.g = config_.color_g;
    arrow.color.b = config_.color_b;
    arrow.color.a = config_.color_a;
    arrow.id = id;
    arrow.lifetime = ros::Duration(1.0);  // 修复：防止僵尸 marker（旧代码为 0 = 永久）
    visualization_msgs::Marker number;
    number.header = pose.header;
    number.ns = "uav_waypoint_number";
    number.action = visualization_msgs::Marker::ADD;
    number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    number.pose = pose.pose;
    number.pose.position.z += config_.number_offset_z;
    number.scale.z = config_.number_scale;
    number.color.r = config_.color_r;
    number.color.g = config_.color_g;
    number.color.b = config_.color_b;
    number.color.a = config_.color_a;
    number.id = id;
    number.text = std::to_string(id);
    number.lifetime = ros::Duration(1.0);  // 修复：防止僵尸 marker
    marker_pub_.publish(arrow);
    marker_pub_.publish(number);
    marker_id_counter_ = std::max(marker_id_counter_, id);
}

void WaypointPanel::clearMarkers() {
    // 使用命名空间隔离的 DELETEALL，清理所有 marker 命名空间
    visualization_msgs::Marker marker_delete;
    marker_delete.header.frame_id = config_.default_frame_id;
    marker_delete.header.stamp = ros::Time::now();

    // 清理旧标记系统（金色箭头+数字）
    marker_delete.ns = "uav_waypoint_arrow";
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker_delete);

    marker_delete.ns = "uav_waypoint_number";
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker_delete);

    // 清理 plan_maker 标记系统（位置球+方向箭头+数字）
    marker_delete.ns = "uav_plan_maker_points";
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker_delete);

    marker_delete.ns = "uav_plan_maker_arrows";
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker_delete);

    marker_delete.ns = "uav_plan_maker_numbers";
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker_delete);

    marker_id_counter_ = 0;
}

void WaypointPanel::republishMarkers() {
    geometry_msgs::PoseArray waypoints = readWaypointsFromTable();
    for (size_t i = 0; i < waypoints.poses.size(); ++i) {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = config_.default_frame_id;
        ps.header.stamp = ros::Time::now();
        ps.pose = waypoints.poses[i];
        markWaypoint(ps, i + 1);
    }
}

// ========== 飞行控制 ==========
void WaypointPanel::startMission() {
    // 发布航点（确保 navigator 收到最新数据）并发送 START 命令
    publishWaypoints();
    if (!nav_command_client_.exists()) {
        logError("导航服务不可用，请确保 navigator 节点已启动");
        return;
    }
    uav_navigator::NavigatorCommand srv;
    srv.request.command = "START";
    if (nav_command_client_.call(srv)) {
        if (srv.response.success) {
            setPlanMakerPhase(NAVIGATING);
            updatePlanMakerStatus();
            logInfo("任务已开始 → 解锁 → 起飞 → 执行航点 → 降落");
            logInfo("提示：RC 遥控器拨杆可随时接管，优先级最高");
        } else {
            logWarn(QString("START 被拒绝: %1").arg(QString::fromStdString(srv.response.message)));
        }
    } else {
        logError("导航服务调用失败");
    }
}

void WaypointPanel::hoverInPlace() {
    if (!nav_command_client_.exists()) { logError("导航服务不可用"); return; }
    uav_navigator::NavigatorCommand srv;
    srv.request.command = "PAUSE";
    if (nav_command_client_.call(srv)) {
        logInfo("悬停：保持当前位置，可继续执行剩余航点");
    } else { logError("悬停命令失败"); }
}

void WaypointPanel::landNow() {
    if (!nav_command_client_.exists()) { logError("导航服务不可用"); return; }
    uav_navigator::NavigatorCommand srv;
    srv.request.command = "LAND";
    if (nav_command_client_.call(srv)) {
        logInfo("降落：飞控切换到 AUTO.LAND，立即着陆");
    } else { logError("降落命令失败"); }
}

void WaypointPanel::returnToHome() {
    if (!nav_command_client_.exists()) { logError("导航服务不可用"); return; }
    uav_navigator::NavigatorCommand srv;
    srv.request.command = "RETURN_TO_HOME";
    if (nav_command_client_.call(srv)) {
        logInfo("返航：返回起飞点（TAKEOFF 阶段记录的 Home 位置）并着陆");
    } else { logError("返航命令失败"); }
}

void WaypointPanel::resetNavigator() {
    if (!nav_command_client_.exists()) { logError("导航服务不可用"); return; }
    uav_navigator::NavigatorCommand srv;
    srv.request.command = "RESET";
    if (nav_command_client_.call(srv)) {
        logInfo("状态机已重置 → IDLE，可开始新任务");
    } else { logError("重置失败"); }
}

void WaypointPanel::emergencyStop() {
    int ret = QMessageBox::question(this, "确认紧急停止",
        "确定要触发紧急停止吗？\n\n飞控将切换到 AUTO.LAND 模式立即着陆。\nRC 遥控器拨杆可随时接管。",
        QMessageBox::Yes | QMessageBox::No);
    if (ret != QMessageBox::Yes) return;
    if (!nav_command_client_.exists()) { logError("导航服务不可用"); return; }
    uav_navigator::NavigatorCommand srv;
    srv.request.command = "EMERGENCY_STOP";
    if (nav_command_client_.call(srv)) {
        logError("⚠ 紧急停止已触发！飞控正在切换到 AUTO.LAND");
    } else { logError("紧急停止调用失败"); }
}

// ========== 系统 ==========
void WaypointPanel::checkNodeStatus() {
    ros::master::V_TopicInfo topics;
    navigator_running_ = false;
    if (ros::master::getTopics(topics)) {
        for (const auto& t : topics) {
            std::string expected_topic = "/" + config_.navigator_status_topic;
            if (t.name == expected_topic) navigator_running_ = true;
        }
    }
    updateAllButtonStates();
}

void WaypointPanel::updateAllButtonStates() {
    bool has_nav = navigator_running_;
    bool has_points = !plan_maker_points_.empty();
    bool can_connect = plan_maker_points_.size() >= 2;
    bool is_planning = (plan_maker_phase_ == PLANNING);
    bool is_connected = (plan_maker_phase_ == CONNECTED);
    bool is_saved = (plan_maker_phase_ == SAVED);
    bool is_navigating = (plan_maker_phase_ == NAVIGATING);

    // 航点规划按钮
    load_button_->setEnabled(has_nav && !is_navigating);
    connect_plan_button_->setEnabled(has_nav && has_points && can_connect && is_planning);
    save_plan_button_->setEnabled(has_nav && is_connected);
    delete_plan_point_button_->setEnabled(has_nav && has_points && !is_navigating);
    clear_plan_button_->setEnabled(has_nav && has_points && !is_navigating);

    // 飞行控制按钮
    start_mission_button_->setEnabled(has_nav && is_saved);
    hover_button_->setEnabled(has_nav && is_navigating);
    land_button_->setEnabled(has_nav && (is_navigating || is_connected || is_saved));
    rth_button_->setEnabled(has_nav && is_navigating);
    reset_button_->setEnabled(has_nav);
    emergency_button_->setEnabled(true);  // 始终可用

    // 航点列表操作按钮
    delete_button_->setEnabled(has_nav && has_points && !is_navigating);
    move_up_button_->setEnabled(has_nav && has_points && !is_navigating);
    move_down_button_->setEnabled(has_nav && has_points && !is_navigating);
    clear_button_->setEnabled(has_nav && has_points && !is_navigating);
}

// ========== MAVROS 状态接收 ==========
void WaypointPanel::receiveMavrosState(const mavros_msgs::State::ConstPtr &msg) {
    mavros_connected_ = msg->connected;
    mavros_armed_ = msg->armed;
    mavros_mode_ = msg->mode;
    if (mavros_connected_) {
        mavros_conn_label_->setText("MAVROS: 已连接");
        mavros_conn_label_->setStyleSheet("color: #4CAF50; font-weight: bold; padding: 2px 8px; border-radius: 4px; background: #E8F5E9;");
    } else {
        mavros_conn_label_->setText("MAVROS: 未连接");
        mavros_conn_label_->setStyleSheet("color: #F44336; font-weight: bold; padding: 2px 8px; border-radius: 4px; background: #FFEBEE;");
    }
    mavros_armed_label_->setText(mavros_armed_ ? "已解锁" : "未解锁");
    mavros_armed_label_->setStyleSheet(mavros_armed_
        ? "color: #4CAF50; font-weight: bold; padding: 2px 8px;"
        : "color: #757575; font-weight: bold; padding: 2px 8px;");
    mavros_mode_label_->setText(QString("模式: %1").arg(QString::fromStdString(mavros_mode_)));
}

void WaypointPanel::receiveOdom(const nav_msgs::Odometry::ConstPtr &msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;
}

void WaypointPanel::receiveWaypointParams(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    if (msg->data.size() < 2) return;

    size_t count = msg->data.size() / 2;
    waypoint_hover_times_.clear();
    waypoint_speeds_.clear();
    for (size_t i = 0; i < count; ++i) {
        waypoint_hover_times_.push_back(msg->data[i * 2]);
        waypoint_speeds_.push_back(msg->data[i * 2 + 1]);
    }

    // 更新表格中的 hover/speed 列（如果表格已填充）
    waypoint_table_->blockSignals(true);
    for (size_t i = 0; i < std::min(count, static_cast<size_t>(current_waypoint_count_)); ++i) {
        QTableWidgetItem *hoverItem = new QTableWidgetItem(
            QString::number(waypoint_hover_times_[i], 'f', 1));
        QTableWidgetItem *speedItem = new QTableWidgetItem(
            QString::number(waypoint_speeds_[i], 'f', 1));
        waypoint_table_->setItem(static_cast<int>(i), 4, hoverItem);
        waypoint_table_->setItem(static_cast<int>(i), 5, speedItem);
    }
    waypoint_table_->blockSignals(false);

    ROS_INFO("[WaypointPanel] Received %zu waypoint params", count);
}

// ========== 导航状态接收 ==========
void WaypointPanel::receiveNavStatus(const uav_navigator::NavigatorStatus::ConstPtr &msg) {
    current_nav_state_ = msg->state;
    updateStatusDisplay(*msg);
}

void WaypointPanel::updateStatusDisplay(const uav_navigator::NavigatorStatus &status) {
    QString state_str = stateToString(status.state);
    QString color = stateToColor(status.state);
    status_led_->setStyleSheet(QString("color: %1; font-size: 20px;").arg(color));
    status_text_->setText(state_str);
    status_text_->setStyleSheet(QString("font-weight: bold; color: %1; font-size: 14px;").arg(color));

    // 同步导航进度（用于 marker 颜色编码）
    nav_current_waypoint_idx_ = status.current_waypoint_index;
    nav_total_waypoints_ = status.total_waypoints;

    // 航点显示：无航点时显示 --/--
    if (status.total_waypoints == 0) {
        wp_progress_label_->setText("航点: --/--");
        wp_progress_bar_->setMaximum(1);
        wp_progress_bar_->setValue(0);
    } else {
        wp_progress_label_->setText(QString("航点: %1 / %2")
            .arg(status.current_waypoint_index + 1).arg(status.total_waypoints));
        wp_progress_bar_->setMaximum(status.total_waypoints);
        wp_progress_bar_->setValue(status.current_waypoint_index + 1);
    }

    // 目标位置：无效时显示 --
    QString target_text;
    if (std::isnan(status.target_x) || std::isnan(status.target_y) || std::isnan(status.target_z)) {
        target_text = "--, --, --";
    } else {
        target_text = QString("%1, %2, %3")
            .arg(status.target_x, 0, 'f', 2)
            .arg(status.target_y, 0, 'f', 2)
            .arg(status.target_z, 0, 'f', 2);
    }

    // 位置 + 错误信息
    QString pos_text = QString("位置: %1, %2, %3 | 目标: %4")
            .arg(status.current_x, 0, 'f', 2)
            .arg(status.current_y, 0, 'f', 2)
            .arg(status.current_z, 0, 'f', 2)
            .arg(target_text);
    if (!status.error_message.empty()) {
        pos_text += QString(" | ⚠ %1").arg(QString::fromStdString(status.error_message));
        position_label_->setStyleSheet("color: #F44336; font-weight: bold; font-size: 11px;");
    } else {
        position_label_->setStyleSheet("color: #666; font-size: 11px;");
    }
    position_label_->setText(pos_text);

    // 航点保存确认：如果正在等待确认且 total_waypoints 匹配
    if (confirmed_waypoint_count_ > 0 && plan_maker_phase_ != SAVED && plan_maker_phase_ != NAVIGATING) {
        if (status.total_waypoints == confirmed_waypoint_count_) {
            setPlanMakerPhase(SAVED);
            updatePlanMakerStatus();
            logInfo(QString("✓ navigator 已确认收到 %1 个航点").arg(confirmed_waypoint_count_));
            confirmed_waypoint_count_ = 0;
        }
    }
}

// ========== ROS自旋 ==========
void WaypointPanel::startSpin() {
    if (ros::ok()) {
        ros::spinOnce();
    }

    // 周期性刷新 plan_maker marker（因为 marker 有 lifetime，需持续重发）
    // 每 3 次 spin 刷新一次，减少不必要的发布
    refresh_counter_++;
    if (refresh_counter_ >= 3 && !plan_maker_points_.empty()) {
        publishPlanMakerMarkers();
        refresh_counter_ = 0;
    }
}

} // namespace rviz_waypoint_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_waypoint_panel::WaypointPanel, rviz::Panel)
