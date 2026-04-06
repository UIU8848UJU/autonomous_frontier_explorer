# Task manager state broadcast to other modules

uint8 IDLE=0
uint8 STARTING_EXPLORATION=1
uint8 EXPLORING=2
uint8 MAPPING_DONE=3
uint8 STARTING_NAVIGATION=4
uint8 NAVIGATING=5
uint8 PAUSED=6
uint8 FAILED=7

builtin_interfaces/Time stamp
uint8 state
string state_text
bool map_ready
bool exploration_running
bool navigation_running
string last_error
string last_exploration_state
