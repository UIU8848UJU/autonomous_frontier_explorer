#pragma once

namespace frontier_explorer
{

// Classic scorer 的内部权重配置。ROS 参数层不再直接暴露这些字段，
// 后续 ML scorer 可以替换 IFrontierCandidateScorer 而不受 YAML 权重约束。
struct FrontierScoringWeights
{
    double weight_distance{1.0};
    double weight_cluster_size{1.0};
    double weight_clearance{0.0};
    double weight_revisit_penalty{1.0};
    double weight_retry_penalty{1.0};
    double weight_unknown_risk_penalty{1.0};
    double weight_information_gain{0.4};
    double unknown_risk_threshold{0.4};

    bool enable_clearance_score{false};
    bool enable_revisit_penalty{false};
    bool enable_unknown_risk_penalty{true};
    bool enable_information_gain_score{true};
};

}  // namespace frontier_explorer
