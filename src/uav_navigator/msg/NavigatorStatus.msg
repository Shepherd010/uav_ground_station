uint8 STATE_IDLE=0
uint8 STATE_PRE_FLIGHT=1
uint8 STATE_ARMING=2
uint8 STATE_TAKEOFF=3
uint8 STATE_NAVIGATING=4
uint8 STATE_HOVERING=5
uint8 STATE_LANDING=6
uint8 STATE_LANDED=7
uint8 STATE_EMERGENCY=8
uint8 STATE_RETURNING=9

uint8 state

uint8 current_waypoint_index
uint8 total_waypoints

float32 current_x
float32 current_y
float32 current_z

float32 target_x
float32 target_y
float32 target_z

bool is_armed
string current_mode
string error_message
