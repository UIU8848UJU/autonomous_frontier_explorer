# map_lifecycle 与 task_manager 共享的地图生命周期状态

uint8 EMPTY=0
uint8 ACTIVE=1
uint8 READY=2
uint8 SAVING=3
uint8 SAVED=4
uint8 FAILED=5

builtin_interfaces/Time stamp
uint8 state
string state_text
bool valid
float64 unknown_ratio
# 该字段仅作为进度指标，不用于判定探索是否完成。
float64 explored_ratio
float64 free_ratio
float64 occupied_ratio
uint32 width
uint32 height
float64 resolution
string saved_map_url
string detail
