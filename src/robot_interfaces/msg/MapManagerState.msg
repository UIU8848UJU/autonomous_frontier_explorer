# Map manager state shared by map_manager and task_manager

uint8 IDLE=0
uint8 MAP_RECEIVED=1
uint8 COMPLETION_DETECTED=2
uint8 SAVING=3
uint8 SAVED=4
uint8 SAVE_FAILED=5

builtin_interfaces/Time stamp
uint8 state
string state_text
bool map_ready
bool save_requested
bool save_succeeded
string map_url
string detail
float64 unknown_ratio
uint32 width
uint32 height
float64 resolution
