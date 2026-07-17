# 实时实验指标
# 由 navigator 发布，experiment_recorder 订阅并记录
std_msgs/Header header

# 导航状态
uint8 nav_state
uint8 current_waypoint_index
uint8 total_waypoints

# 位置数据
float32 current_x
float32 current_y
float32 current_z
float32 target_x
float32 target_y
float32 target_z

# MAVROS 状态
bool is_armed
string current_mode

# 规划轨迹与实际轨迹的偏差
float32 plan_deviation_x
float32 plan_deviation_y
float32 plan_deviation_z
float32 plan_deviation_total

# 时间
float32 time_since_start
float32 time_in_current_state
