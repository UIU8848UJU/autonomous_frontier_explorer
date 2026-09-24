# 单个 frontier 候选目标的统一 ROS 传输模型。
# 候选在策略层产生，后续可达性检查结果由 BT/导航适配层回填。
geometry_msgs/PoseStamped goal
int32 goal_row
int32 goal_col
int32 cluster_centroid_row
int32 cluster_centroid_col
uint32 source_cluster_index
uint32 cluster_size
uint32 retry_count
float32 score
float32 distance_m
float32 clearance_m
float32 unknown_ratio
# 候选位姿可见的未知地图面积，单位 m²。
float32 information_gain
bool information_gain_valid
bool used_fallback
bool goal_inset_applied
bool reachability_checked
bool reachable
float32 path_length_m
string reachability_reason
bool feasible
bool footprint_valid
float32 footprint_cost
string feasibility_detail
