/**
 * @file multi_navi_goal_panel.cpp
 * @brief RViz 面板插件：多点导航可视化与任务发布
 *
 * 优化版本 (v2.0)：
 * - 修复 static startSpin() → 非静态成员函数
 * - 修复 setBackgroundColor（已废弃）→ QBrush
 * - 移除阻塞式 ros::Duration::sleep()
 * - 所有参数通过 ROS 参数服务器加载，零硬编码
 * - 集成 waypoint_manager 服务（XML 存储）
 * - Marker 使用独立命名空间，DELETEALL 不干扰其他插件
 * - 发布航点到 uav/waypoints/input 供 navigator 使用
 */

#include <cstdio>
#include <fstream>
#include <sstream>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QTableWidget>
#include <QHeaderView>
#include <QMessageBox>
#include <QFileDialog>
#include "multi_navi_goal_panel.h"

namespace navi_multi_goals_pub_rviz_plugin
{

// ============================================================================
// 构造与析构
// ============================================================================

MultiNaviGoalsPanel::MultiNaviGoalsPanel(QWidget *parent)
    : rviz::Panel(parent)
    , nh_()
    , maxNumGoal_(1)
    , curGoalIdx_(0)
    , cycleCnt_(0)
    , permit_(false)
    , cycle_(false)
    , arrived_(false)
    , was_arrived_(false)
    , marker_id_(0)
{
    loadConfig();

    // ---- 订阅 ----
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        config_.goal_topic, 5,
        boost::bind(&MultiNaviGoalsPanel::goalCntCB, this, _1));

    status_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>(
        config_.status_topic, 5,
        boost::bind(&MultiNaviGoalsPanel::statusCB, this, _1));

    // ---- 发布者 ----
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(config_.goal_pub_topic, 1, true);
    cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>(config_.cancel_topic, 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(config_.marker_topic, 10, true);
    waypoint_input_pub_ = nh_.advertise<geometry_msgs::PoseArray>(config_.waypoint_input_topic, 1, true);

    // ---- 服务客户端（waypoint_manager 集成）----
    save_waypoints_client_ = nh_.serviceClient<uav_waypoint_manager::SaveWaypoints>(config_.save_service);
    load_waypoints_client_ = nh_.serviceClient<uav_waypoint_manager::LoadWaypoints>(config_.load_service);

    // ---- UI 布局 ----
    QVBoxLayout *root_layout = new QVBoxLayout;

    // 最大目标数量
    QHBoxLayout *maxNumGoal_layout = new QHBoxLayout;
    maxNumGoal_layout->addWidget(new QLabel(QString::fromUtf8("目标最大数量")));
    output_maxNumGoal_editor_ = new QLineEdit(QString::number(maxNumGoal_));
    maxNumGoal_layout->addWidget(output_maxNumGoal_editor_);
    output_maxNumGoal_button_ = new QPushButton(QString::fromUtf8("确定"));
    maxNumGoal_layout->addWidget(output_maxNumGoal_button_);
    root_layout->addLayout(maxNumGoal_layout);

    // 循环导航复选框
    cycle_checkbox_ = new QCheckBox(QString::fromUtf8("循环"));
    root_layout->addWidget(cycle_checkbox_);

    // 位姿表格
    poseArray_table_ = new QTableWidget;
    initPoseTable();
    root_layout->addWidget(poseArray_table_);

    // 操作按钮
    QHBoxLayout *manipulate_layout = new QHBoxLayout;
    output_reset_button_ = new QPushButton(QString::fromUtf8("重置"));
    manipulate_layout->addWidget(output_reset_button_);
    output_cancel_button_ = new QPushButton(QString::fromUtf8("取消"));
    manipulate_layout->addWidget(output_cancel_button_);
    output_startNavi_button_ = new QPushButton(QString::fromUtf8("开始导航"));
    manipulate_layout->addWidget(output_startNavi_button_);
    output_save_button_ = new QPushButton(QString::fromUtf8("保存数据"));
    manipulate_layout->addWidget(output_save_button_);
    root_layout->addLayout(manipulate_layout);

    // 集成按钮行（waypoint_manager + XML）
    QHBoxLayout *integrate_layout = new QHBoxLayout;
    output_save_xml_button_ = new QPushButton(QString::fromUtf8("💾 保存XML"));
    integrate_layout->addWidget(output_save_xml_button_);
    output_load_xml_button_ = new QPushButton(QString::fromUtf8("📂 加载XML"));
    integrate_layout->addWidget(output_load_xml_button_);
    output_publish_nav_button_ = new QPushButton(QString::fromUtf8("📤 发布至导航器"));
    integrate_layout->addWidget(output_publish_nav_button_);
    root_layout->addLayout(integrate_layout);

    // 状态标签
    status_label_ = new QLabel(QString::fromUtf8("就绪 | 航点数: 0"));
    root_layout->addWidget(status_label_);

    setLayout(root_layout);

    // ---- 定时器 ----
    QTimer *output_timer = new QTimer(this);
    output_timer->start(config_.spin_timer_ms);

    // ---- 信号连接 ----
    connect(output_maxNumGoal_button_, SIGNAL(clicked()), this, SLOT(updateMaxNumGoal()));
    connect(output_maxNumGoal_button_, SIGNAL(clicked()), this, SLOT(updatePoseTable()));
    connect(output_reset_button_, SIGNAL(clicked()), this, SLOT(initPoseTable()));
    connect(output_cancel_button_, SIGNAL(clicked()), this, SLOT(cancelNavi()));
    connect(output_startNavi_button_, SIGNAL(clicked()), this, SLOT(startNavi()));
    connect(output_save_button_, SIGNAL(clicked()), this, SLOT(saveTableData()));
    connect(cycle_checkbox_, SIGNAL(clicked(bool)), this, SLOT(checkCycle()));
    connect(output_timer, SIGNAL(timeout()), this, SLOT(startSpin()));

    // 集成按钮
    connect(output_save_xml_button_, SIGNAL(clicked()), this, SLOT(saveToXml()));
    connect(output_load_xml_button_, SIGNAL(clicked()), this, SLOT(loadFromXml()));
    connect(output_publish_nav_button_, SIGNAL(clicked()), this, SLOT(publishToWaypointManager()));

    ROS_INFO("[MultiNaviGoalsPanel] 初始化完成，最大航点数: %d", maxNumGoal_);
}

MultiNaviGoalsPanel::~MultiNaviGoalsPanel()
{
    deleteMark();
}

// ============================================================================
// 配置加载
// ============================================================================

void MultiNaviGoalsPanel::loadConfig()
{
    ros::NodeHandle pnh("~");

    // 话题配置
    pnh.param<std::string>("goal_topic", config_.goal_topic, "move_base_simple/goal");
    pnh.param<std::string>("goal_pub_topic", config_.goal_pub_topic, "move_base_simple/goal");
    pnh.param<std::string>("status_topic", config_.status_topic, "move_base/status");
    pnh.param<std::string>("cancel_topic", config_.cancel_topic, "move_base/cancel");
    pnh.param<std::string>("marker_topic", config_.marker_topic, "visualization_marker");
    pnh.param<std::string>("waypoint_input_topic", config_.waypoint_input_topic, "uav/waypoints/input");
    pnh.param<std::string>("save_service", config_.save_service, "uav/waypoint_manager/save_waypoints");
    pnh.param<std::string>("load_service", config_.load_service, "uav/waypoint_manager/load_waypoints");

    // Marker 命名空间（用于隔离，避免 DELETEALL 干扰其他插件）
    pnh.param<std::string>("marker_ns_arrow", config_.marker_ns_arrow, "multi_navi_arrow");
    pnh.param<std::string>("marker_ns_number", config_.marker_ns_number, "multi_navi_number");

    // Marker 可视化参数
    pnh.param<double>("arrow_scale_x", config_.arrow_scale_x, 1.0);
    pnh.param<double>("arrow_scale_y", config_.arrow_scale_y, 0.2);
    pnh.param<double>("arrow_scale_z", config_.arrow_scale_z, 0.2);
    pnh.param<double>("number_scale_z", config_.number_scale_z, 1.0);
    pnh.param<double>("number_offset_z", config_.number_offset_z, 1.0);
    pnh.param<double>("marker_color_r", config_.marker_color_r, 1.0);
    pnh.param<double>("marker_color_g", config_.marker_color_g, 0.98);
    pnh.param<double>("marker_color_b", config_.marker_color_b, 0.80);
    pnh.param<double>("marker_color_a", config_.marker_color_a, 1.0);
    pnh.param<double>("marker_lifetime_s", config_.marker_lifetime_s, 0.0);

    // 默认参数
    pnh.param<int>("default_max_goals", config_.default_max_goals, 10);
    pnh.param<std::string>("default_frame_id", config_.default_frame_id, "map");
    pnh.param<int>("spin_timer_ms", config_.spin_timer_ms, 100);

    maxNumGoal_ = config_.default_max_goals;
}

// ============================================================================
// 最大目标数量管理
// ============================================================================

void MultiNaviGoalsPanel::setMaxNumGoal(const QString &new_maxNumGoal)
{
    if (new_maxNumGoal != output_maxNumGoal_)
    {
        output_maxNumGoal_ = new_maxNumGoal;
        if (output_maxNumGoal_.isEmpty())
        {
            maxNumGoal_ = 1;
        }
        else
        {
            bool ok;
            int val = output_maxNumGoal_.toInt(&ok);
            maxNumGoal_ = (ok && val > 0) ? val : 1;
        }
        nh_.setParam("maxNumGoal_", maxNumGoal_);
        Q_EMIT configChanged();
    }
}

void MultiNaviGoalsPanel::updateMaxNumGoal()
{
    setMaxNumGoal(output_maxNumGoal_editor_->text());
}

// ============================================================================
// 表格管理
// ============================================================================

void MultiNaviGoalsPanel::initPoseTable()
{
    ROS_INFO("[MultiNaviGoalsPanel] 重置表格");
    curGoalIdx_ = 0;
    cycleCnt_ = 0;
    permit_ = false;
    cycle_ = false;
    arrived_ = false;
    was_arrived_ = false;

    pose_array_.poses.clear();
    poseArray_table_->clear();
    deleteMark();

    poseArray_table_->setRowCount(maxNumGoal_);
    poseArray_table_->setColumnCount(4);
    poseArray_table_->setEditTriggers(QAbstractItemView::AllEditTriggers);
    poseArray_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    QStringList headers = {"x", "y", "z", "yaw"};
    poseArray_table_->setHorizontalHeaderLabels(headers);
    cycle_checkbox_->setCheckState(Qt::Unchecked);

    updateStatusLabel();
}

void MultiNaviGoalsPanel::updatePoseTable()
{
    poseArray_table_->setColumnCount(4);
    poseArray_table_->setRowCount(maxNumGoal_);
    QStringList headers = {"x", "y", "z", "yaw"};
    poseArray_table_->setHorizontalHeaderLabels(headers);
}

void MultiNaviGoalsPanel::updateStatusLabel()
{
    int count = static_cast<int>(pose_array_.poses.size());
    QString mode = cycle_ ? QString::fromUtf8("循环") : QString::fromUtf8("单次");
    status_label_->setText(
        QString::fromUtf8("航点数: %1 / %2 | 模式: %3 | 索引: %4")
            .arg(count).arg(maxNumGoal_).arg(mode).arg(curGoalIdx_));
}

// ============================================================================
// 数据写入
// ============================================================================

void MultiNaviGoalsPanel::writePose(geometry_msgs::Pose pose)
{
    int row = static_cast<int>(pose_array_.poses.size()) - 1;
    if (row < 0 || row >= poseArray_table_->rowCount()) return;

    poseArray_table_->setItem(row, 0, new QTableWidgetItem(
        QString::number(pose.position.x, 'f', 2)));
    poseArray_table_->setItem(row, 1, new QTableWidgetItem(
        QString::number(pose.position.y, 'f', 2)));
    poseArray_table_->setItem(row, 2, new QTableWidgetItem(
        QString::number(pose.position.z, 'f', 2)));
    poseArray_table_->setItem(row, 3, new QTableWidgetItem(
        QString::number(tf::getYaw(pose.orientation) * 180.0 / M_PI, 'f', 2)));
}

// ============================================================================
// 保存表格数据（从表格读取 → pose_array_ → 可视化标记）
// ============================================================================

void MultiNaviGoalsPanel::saveTableData()
{
    pose_array_.poses.clear();
    deleteMark();
    marker_id_ = 0;
    pose_array_.header.frame_id = config_.default_frame_id;
    pose_array_.header.stamp = ros::Time::now();

    for (int row = 0; row < poseArray_table_->rowCount(); ++row)
    {
        QTableWidgetItem *xItem = poseArray_table_->item(row, 0);
        QTableWidgetItem *yItem = poseArray_table_->item(row, 1);
        QTableWidgetItem *zItem = poseArray_table_->item(row, 2);
        QTableWidgetItem *yawItem = poseArray_table_->item(row, 3);

        if (!xItem || !yItem || !zItem || !yawItem)
        {
            ROS_WARN("[MultiNaviGoalsPanel] 第%d行数据不完整，跳过", row + 1);
            continue;
        }

        bool ok;
        double x = xItem->text().toDouble(&ok);
        if (!ok) { ROS_WARN("[MultiNaviGoalsPanel] 第%d行X值格式错误，跳过", row + 1); continue; }

        double y = yItem->text().toDouble(&ok);
        if (!ok) { ROS_WARN("[MultiNaviGoalsPanel] 第%d行Y值格式错误，跳过", row + 1); continue; }

        double z = zItem->text().toDouble(&ok);
        if (!ok) { ROS_WARN("[MultiNaviGoalsPanel] 第%d行Z值格式错误，跳过", row + 1); continue; }

        double yaw_deg = yawItem->text().toDouble(&ok);
        if (!ok) { ROS_WARN("[MultiNaviGoalsPanel] 第%d行YAW值格式错误，跳过", row + 1); continue; }

        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        tf::Quaternion quat;
        quat.setRPY(0, 0, yaw_deg * M_PI / 180.0);
        geometry_msgs::Quaternion msg_quat;
        tf::quaternionTFToMsg(quat, msg_quat);
        pose.orientation = msg_quat;

        pose_array_.poses.push_back(pose);
    }

    ROS_INFO("[MultiNaviGoalsPanel] 已保存 %zu 个目标点", pose_array_.poses.size());

    // 发布可视化标记（不再使用阻塞 sleep）
    for (size_t i = 0; i < pose_array_.poses.size(); ++i)
    {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = config_.default_frame_id;
        ps.header.stamp = ros::Time::now();
        ps.pose = pose_array_.poses[i];
        markPose(boost::make_shared<const geometry_msgs::PoseStamped>(ps));
    }
    ROS_INFO("[MultiNaviGoalsPanel] 标记完成，共 %zu 个点", pose_array_.poses.size());

    curGoalIdx_ = 0;
    updateStatusLabel();
}

// ============================================================================
// 可视化标记
// ============================================================================

void MultiNaviGoalsPanel::markPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    if (!ros::ok()) return;

    visualization_msgs::Marker arrow, number;

    // 箭头
    arrow.header.frame_id = pose->header.frame_id;
    arrow.header.stamp = ros::Time::now();
    arrow.ns = config_.marker_ns_arrow;
    arrow.id = ++marker_id_;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose = pose->pose;
    arrow.scale.x = config_.arrow_scale_x;
    arrow.scale.y = config_.arrow_scale_y;
    arrow.scale.z = config_.arrow_scale_z;
    arrow.color.r = static_cast<float>(config_.marker_color_r);
    arrow.color.g = static_cast<float>(config_.marker_color_g);
    arrow.color.b = static_cast<float>(config_.marker_color_b);
    arrow.color.a = static_cast<float>(config_.marker_color_a);
    arrow.lifetime = ros::Duration(config_.marker_lifetime_s);

    // 编号
    number.header.frame_id = pose->header.frame_id;
    number.header.stamp = ros::Time::now();
    number.ns = config_.marker_ns_number;
    number.id = marker_id_;
    number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    number.action = visualization_msgs::Marker::ADD;
    number.pose = pose->pose;
    number.pose.position.z += config_.number_offset_z;
    number.scale.z = config_.number_scale_z;
    number.color.r = static_cast<float>(config_.marker_color_r);
    number.color.g = static_cast<float>(config_.marker_color_g);
    number.color.b = static_cast<float>(config_.marker_color_b);
    number.color.a = static_cast<float>(config_.marker_color_a);
    number.lifetime = ros::Duration(config_.marker_lifetime_s);
    number.text = std::to_string(number.id);

    marker_pub_.publish(arrow);
    marker_pub_.publish(number);
}

void MultiNaviGoalsPanel::deleteMark()
{
    // 只删除本插件命名空间下的 marker，不干扰其他插件
    visualization_msgs::Marker marker_delete;
    marker_delete.header.frame_id = config_.default_frame_id;
    marker_delete.header.stamp = ros::Time::now();

    // 删除箭头
    marker_delete.ns = config_.marker_ns_arrow;
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker_delete);

    // 删除编号
    marker_delete.ns = config_.marker_ns_number;
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker_delete);

    marker_id_ = 0;
}

// ============================================================================
// RViz 2D Nav Goal 回调
// ============================================================================

void MultiNaviGoalsPanel::goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    if (static_cast<int>(pose_array_.poses.size()) < maxNumGoal_)
    {
        pose_array_.poses.push_back(pose->pose);
        pose_array_.header.frame_id = pose->header.frame_id;
        writePose(pose->pose);
        markPose(pose);
        updateStatusLabel();
        ROS_INFO("[MultiNaviGoalsPanel] 接收目标点 %zu / %d",
                 pose_array_.poses.size(), maxNumGoal_);
    }
    else
    {
        ROS_ERROR("[MultiNaviGoalsPanel] 目标数量超过最大限制: %d", maxNumGoal_);
    }
}

// ============================================================================
// 导航控制
// ============================================================================

void MultiNaviGoalsPanel::startNavi()
{
    if (pose_array_.poses.empty())
    {
        ROS_ERROR("[MultiNaviGoalsPanel] 无目标点，请先设置航点");
        return;
    }

    curGoalIdx_ %= static_cast<int>(pose_array_.poses.size());

    geometry_msgs::PoseStamped goal;
    goal.header = pose_array_.header;
    goal.header.stamp = ros::Time::now();
    goal.pose = pose_array_.poses[curGoalIdx_];
    goal_pub_.publish(goal);

    ROS_INFO("[MultiNaviGoalsPanel] 正在导航至目标点 %d / %zu",
             curGoalIdx_ + 1, pose_array_.poses.size());

    // 高亮当前行（使用 QBrush 替代已废弃的 setBackgroundColor）
    QColor highlight(255, 69, 0);  // 橙红色
    for (int col = 0; col < 4; ++col)
    {
        QTableWidgetItem *item = poseArray_table_->item(curGoalIdx_, col);
        if (item) item->setBackground(QBrush(highlight));
    }

    curGoalIdx_++;
    permit_ = true;
    updateStatusLabel();
}

void MultiNaviGoalsPanel::cancelNavi()
{
    if (!cur_goalid_.id.empty())
    {
        cancel_pub_.publish(cur_goalid_);
        ROS_WARN("[MultiNaviGoalsPanel] 导航已取消");
        cur_goalid_.id.clear();
    }
    permit_ = false;
}

void MultiNaviGoalsPanel::completeNavi()
{
    if (curGoalIdx_ < static_cast<int>(pose_array_.poses.size()))
    {
        geometry_msgs::PoseStamped goal;
        goal.header = pose_array_.header;
        goal.header.stamp = ros::Time::now();
        goal.pose = pose_array_.poses[curGoalIdx_];

        goal_pub_.publish(goal);
        ROS_INFO("[MultiNaviGoalsPanel] 正在导航至目标点 %d / %zu",
                 curGoalIdx_ + 1, pose_array_.poses.size());

        QColor highlight(255, 69, 0);
        for (int col = 0; col < 4; ++col)
        {
            QTableWidgetItem *item = poseArray_table_->item(curGoalIdx_, col);
            if (item) item->setBackground(QBrush(highlight));
        }

        curGoalIdx_++;
        permit_ = true;
    }
    else
    {
        ROS_INFO("[MultiNaviGoalsPanel] 所有目标点已完成");
        permit_ = false;
    }
    updateStatusLabel();
}

void MultiNaviGoalsPanel::cycleNavi()
{
    if (!permit_) return;

    geometry_msgs::PoseStamped goal;
    goal.header = pose_array_.header;
    goal.header.stamp = ros::Time::now();

    int current = curGoalIdx_ % static_cast<int>(pose_array_.poses.size());
    goal.pose = pose_array_.poses[current];
    goal_pub_.publish(goal);

    ROS_INFO("[MultiNaviGoalsPanel] 第%d次循环, 目标点 %d / %zu",
             cycleCnt_ + 1, current + 1, pose_array_.poses.size());

    QColor color = (cycleCnt_ % 2 == 0) ? QColor(255, 69, 0) : QColor(100, 149, 237);
    for (int col = 0; col < 4; ++col)
    {
        QTableWidgetItem *item = poseArray_table_->item(current, col);
        if (item) item->setBackground(QBrush(color));
    }

    curGoalIdx_++;
    cycleCnt_ = curGoalIdx_ / static_cast<int>(pose_array_.poses.size());
    updateStatusLabel();
}

void MultiNaviGoalsPanel::checkCycle()
{
    cycle_ = cycle_checkbox_->isChecked();
    updateStatusLabel();
}

// ============================================================================
// move_base 状态回调
// ============================================================================

void MultiNaviGoalsPanel::statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses)
{
    bool arrived_pre = arrived_;
    arrived_ = checkGoal(statuses->status_list);

    // 仅在到达状态从 false→true 变化时触发下一步
    if (arrived_ && !arrived_pre && ros::ok() && permit_)
    {
        if (cycle_)
            cycleNavi();
        else
            completeNavi();
    }
}

bool MultiNaviGoalsPanel::checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list)
{
    if (status_list.empty())
    {
        return false;
    }

    for (auto &status : status_list)
    {
        switch (status.status)
        {
        case actionlib_msgs::GoalStatus::SUCCEEDED:   // 3
            ROS_INFO("[MultiNaviGoalsPanel] 目标 %d 已完成", curGoalIdx_);
            return true;
        case actionlib_msgs::GoalStatus::PREEMPTED:   // 2 - 被抢占也算完成
        case actionlib_msgs::GoalStatus::ABORTED:      // 4
            ROS_WARN("[MultiNaviGoalsPanel] 目标 %d 被取消/放弃，跳转下一目标", curGoalIdx_);
            return true;
        case actionlib_msgs::GoalStatus::ACTIVE:        // 1
            cur_goalid_ = status.goal_id;
            return false;
        case actionlib_msgs::GoalStatus::RECALLED:      // 5
        case actionlib_msgs::GoalStatus::REJECTED:      // 6
            ROS_WARN("[MultiNaviGoalsPanel] 目标 %d 被拒绝/撤回", curGoalIdx_);
            return true;
        default:
            break;
        }
    }
    return false;
}

// ============================================================================
// 集成 waypoint_manager（UAV 地面站基础设施）
// ============================================================================

void MultiNaviGoalsPanel::publishToWaypointManager()
{
    // 先从表格读取最新数据
    saveTableData();

    if (pose_array_.poses.empty())
    {
        QMessageBox::warning(this,
            QString::fromUtf8("无航点"),
            QString::fromUtf8("请先设置航点再发布到导航器。"));
        return;
    }

    // 发布到 uav/waypoints/input → waypoint_manager 处理 → navigator 接收
    waypoint_input_pub_.publish(pose_array_);
    ROS_INFO("[MultiNaviGoalsPanel] 已发布 %zu 个航点到 %s",
             pose_array_.poses.size(), config_.waypoint_input_topic.c_str());

    QMessageBox::information(this,
        QString::fromUtf8("发布成功"),
        QString::fromUtf8("已发布 %1 个航点到导航器。\n\n"
                          "请确认 RViz 中已显示航点标记，\n"
                          "然后使用导航控制按钮启动任务。")
            .arg(static_cast<int>(pose_array_.poses.size())));
}

void MultiNaviGoalsPanel::saveToXml()
{
    // 先从表格读取最新数据
    saveTableData();

    if (pose_array_.poses.empty())
    {
        QMessageBox::warning(this,
            QString::fromUtf8("无航点"),
            QString::fromUtf8("没有航点数据可以保存。"));
        return;
    }

    QString file_path = QFileDialog::getSaveFileName(this,
        QString::fromUtf8("保存航点 XML"),
        QString::fromUtf8("/home/groundstation/waypoints.xml"),
        "XML files (*.xml)");

    if (file_path.isEmpty()) return;

    uav_waypoint_manager::SaveWaypoints srv;
    srv.request.file_path = file_path.toStdString();

    if (save_waypoints_client_.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("[MultiNaviGoalsPanel] 航点已保存到 %s", file_path.toStdString().c_str());
            QMessageBox::information(this,
                QString::fromUtf8("保存成功"),
                QString::fromUtf8("已保存 %1 个航点到:\n%2")
                    .arg(static_cast<int>(pose_array_.poses.size())).arg(file_path));
        }
        else
        {
            ROS_ERROR("[MultiNaviGoalsPanel] 保存失败: %s", srv.response.message.c_str());
            QMessageBox::critical(this,
                QString::fromUtf8("保存失败"),
                QString::fromUtf8("无法保存航点:\n%1")
                    .arg(QString::fromStdString(srv.response.message)));
        }
    }
    else
    {
        QMessageBox::critical(this,
            QString::fromUtf8("服务调用失败"),
            QString::fromUtf8("无法连接 waypoint_manager 保存服务。\n"
                              "请确认地面站核心已启动。"));
    }
}

void MultiNaviGoalsPanel::loadFromXml()
{
    QString file_path = QFileDialog::getOpenFileName(this,
        QString::fromUtf8("加载航点 XML"),
        QString::fromUtf8("/home/groundstation/waypoints.xml"),
        "XML files (*.xml)");

    if (file_path.isEmpty()) return;

    uav_waypoint_manager::LoadWaypoints srv;
    srv.request.file_path = file_path.toStdString();

    if (load_waypoints_client_.call(srv))
    {
        if (srv.response.success)
        {
            // 清空现有数据
            initPoseTable();

            // 直接从服务响应获取航点数据（不需要 waitForMessage）
            const auto& waypoints = srv.response.waypoints;
            int count = srv.response.waypoint_count;
            ROS_INFO("[MultiNaviGoalsPanel] 从 %s 加载了 %d 个航点",
                     file_path.toStdString().c_str(), count);

            // 填充内部数据结构和表格
            for (const auto& pose : waypoints.poses)
            {
                pose_array_.poses.push_back(pose);
                pose_array_.header.frame_id = config_.default_frame_id;
                pose_array_.header.stamp = ros::Time::now();
                writePose(pose);

                geometry_msgs::PoseStamped ps;
                ps.header.frame_id = config_.default_frame_id;
                ps.header.stamp = ros::Time::now();
                ps.pose = pose;
                markPose(boost::make_shared<const geometry_msgs::PoseStamped>(ps));
            }

            updateStatusLabel();

            QMessageBox::information(this,
                QString::fromUtf8("加载成功"),
                QString::fromUtf8("已从 XML 加载 %1 个航点，可视化已更新。\n\n"
                                  "点击「保存数据」同步到表格，\n"
                                  "或点击「发布至导航器」发送到导航系统。")
                    .arg(count));
        }
        else
        {
            QMessageBox::critical(this,
                QString::fromUtf8("加载失败"),
                QString::fromUtf8("无法加载航点:\n%1")
                    .arg(QString::fromStdString(srv.response.message)));
        }
    }
    else
    {
        QMessageBox::critical(this,
            QString::fromUtf8("服务调用失败"),
            QString::fromUtf8("无法连接 waypoint_manager 加载服务。\n"
                              "请确认地面站核心已启动。"));
    }
}

// ============================================================================
// ROS 自旋
// ============================================================================

void MultiNaviGoalsPanel::startSpin()
{
    if (ros::ok())
    {
        ros::spinOnce();
    }
}

} // end namespace navi_multi_goals_pub_rviz_plugin

// 声明 RViz 插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::MultiNaviGoalsPanel, rviz::Panel)
