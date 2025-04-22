std_msgs/Header header

uint8 CONTROL_MODE_DIRECT = 0
uint8 CONTROL_MODE_SHARED = 1
uint8 CONTROL_MODE_WAYPOINT = 2
uint8 CONTROL_MODE_TRAJECTORY_GUIDANCE = 3          
uint8 CONTROL_MODE_PERCEPTION_MODIFICATION = 4
uint8 CONTROL_MODE_SAFECORRIDOR = 5
uint8 CONTROL_MODE_NONE = 99

uint8 CONNECT_BUTTON = 0
uint8 DISCONNECT_BUTTON = 1
uint8 START_BUTTON = 2
uint8 STOP_BUTTON = 3
uint8 NONE_BUTTON = 3


string vehicle_ip_address
string operator_ip_address

uint8 vehicle_control_mode

uint8 clicked_button