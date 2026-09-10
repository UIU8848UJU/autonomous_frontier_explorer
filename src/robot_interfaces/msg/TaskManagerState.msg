# TaskManager 状态，向其他模块广播

uint8 IDLE=0
uint8 STARTING_MAPPING=1
uint8 MAPPING=2
uint8 WAITING_MAP_SAVE=3
uint8 MAPPING_DONE=4
uint8 FAILED=5

builtin_interfaces/Time stamp
uint8 state
string state_text
bool map_ready
bool exploration_running
bool navigation_running
string last_error
string last_exploration_state
