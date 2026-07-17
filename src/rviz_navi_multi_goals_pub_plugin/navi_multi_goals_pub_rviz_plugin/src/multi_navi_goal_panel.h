#ifndef MULTI_NAVI_GOAL_PANEL_H
#define MULTI_NAVI_GOAL_PANEL_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <rviz/panel.h>

#include <QPushButton>
#include <QTableWidget>
#include <QCheckBox>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QHeaderView>
#include <QMessageBox>
#include <QTimer>
#include <QFileDialog>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_datatypes.h>

// 集成 waypoint_manager 服务的头文件
#include <uav_waypoint_manager/SaveWaypoints.h>
#include <uav_waypoint_manager/LoadWaypoints.h>

namespace navi_multi_goals_pub_rviz_plugin {

class MultiNaviGoalsPanel : public rviz::Panel {
Q_OBJECT
public:
    explicit MultiNaviGoalsPanel(QWidget *parent = 0);
    ~MultiNaviGoalsPanel();

    /// 从 ROS 参数服务器加载配置
    void loadConfig();

public Q_SLOTS:
    /// 设置最大目标数量
    void setMaxNumGoal(const QString &new_maxNumGoal);
    /// 将位姿写入表格
    void writePose(geometry_msgs::Pose pose);
    /// 接收并标记目标位姿（通过 RViz 2D Nav Goal 工具）
    void markPose(const geometry_msgs::PoseStamped::ConstPtr &pose);
    /// 清除地图上的标记
    void deleteMark();

protected Q_SLOTS:
    /// 更新最大目标数量并刷新表格
    void updateMaxNumGoal();
    /// 初始化位姿表格
    void initPoseTable();
    /// 更新位姿表格内容
    void updatePoseTable();
    /// 开始导航（顺序发送目标点）
    void startNavi();
    /// 保存表格数据到内存并发布可视化
    void saveTableData();
    /// 取消导航任务
    void cancelNavi();
    /// 切换循环导航模式
    void checkCycle();

    // ===== 集成 UAV 地面站基础设施 =====
    /// 将航点发布到 uav/waypoints/input（与 waypoint_manager 集成）
    void publishToWaypointManager();
    /// 通过 waypoint_manager 服务保存到 XML
    void saveToXml();
    /// 通过 waypoint_manager 服务从 XML 加载
    void loadFromXml();

    /// 接收目标点（RViz 2D Nav Goal 回调）
    void goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose);
    /// 接收导航状态（move_base status 回调）
    void statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses);
    /// 周期性自旋 ROS 消息队列
    void startSpin();
    /// 完成当前目标后继续导航后续目标
    void completeNavi();
    /// 循环导航模式
    void cycleNavi();
    /// 检查是否达到目标位姿
    bool checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list);
    /// 更新状态标签显示
    void updateStatusLabel();

protected:
    // ===== 配置 =====
    struct Config {
        // 话题
        std::string goal_topic;               // 订阅 RViz 2D Nav Goal
        std::string goal_pub_topic;           // 发布导航目标
        std::string status_topic;             // 订阅导航状态
        std::string cancel_topic;             // 发布取消指令
        std::string marker_topic;             // 发布可视化标记
        // waypoint_manager 集成
        std::string waypoint_input_topic;     // 发布航点到 waypoint_manager
        std::string save_service;             // waypoint_manager save 服务
        std::string load_service;             // waypoint_manager load 服务
        // 可视化参数
        std::string marker_ns_arrow;          // 箭头 marker 命名空间
        std::string marker_ns_number;         // 编号 marker 命名空间
        double arrow_scale_x;
        double arrow_scale_y;
        double arrow_scale_z;
        double number_scale_z;
        double number_offset_z;
        double marker_color_r;
        double marker_color_g;
        double marker_color_b;
        double marker_color_a;
        double marker_lifetime_s;            // marker 生命周期（秒），0=永久
        // 默认值
        int default_max_goals;
        std::string default_frame_id;
        int spin_timer_ms;
    } config_;

    // ===== 界面控件 =====
    QLineEdit *output_maxNumGoal_editor_;
    QPushButton *output_maxNumGoal_button_;
    QPushButton *output_reset_button_;
    QPushButton *output_startNavi_button_;
    QPushButton *output_cancel_button_;
    QPushButton *output_save_button_;
    QPushButton *output_save_xml_button_;       // 保存到 XML
    QPushButton *output_load_xml_button_;       // 从 XML 加载
    QPushButton *output_publish_nav_button_;    // 发布到 navigator
    QTableWidget *poseArray_table_;
    QCheckBox *cycle_checkbox_;
    QLabel *status_label_;                      // 状态显示

    QString output_maxNumGoal_;

    // ===== ROS 通信 =====
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    ros::Publisher cancel_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher waypoint_input_pub_;          // 集成 waypoint_manager
    ros::Subscriber goal_sub_;
    ros::Subscriber status_sub_;
    ros::ServiceClient save_waypoints_client_;
    ros::ServiceClient load_waypoints_client_;

    // ===== 状态变量 =====
    int maxNumGoal_;
    int curGoalIdx_;
    int cycleCnt_;
    bool permit_;
    bool cycle_;
    bool arrived_;
    int marker_id_;                              // 非静态，每个实例独立计数
    geometry_msgs::PoseArray pose_array_;
    actionlib_msgs::GoalID cur_goalid_;

    // 用于检测到达状态变化
    bool was_arrived_;
};

} // end namespace navi_multi_goals_pub_rviz_plugin
#endif // MULTI_NAVI_GOAL_PANEL_H
