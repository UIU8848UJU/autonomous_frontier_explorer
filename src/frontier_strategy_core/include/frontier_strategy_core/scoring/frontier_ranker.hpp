#pragma once

#include <optional>
#include <vector>

#include "frontier_strategy_core/scoring/frontier_scorer.hpp"

namespace frontier_strategy
{

/// 候选排序器接口；规则排序、学习排序都通过这个接口接入选择流程。
class IFrontierRanker
{
public:
    virtual ~IFrontierRanker() = default;

    virtual std::vector<ScoredFrontierCandidate> rank(
        const std::vector<FrontierCandidate> & candidates,
        const std::optional<GridCell> & last_goal) const = 0;
};

/// 当前默认的规则排序器，保持现有分项评分和确定性排序行为。
class RuleBasedFrontierRanker final : public IFrontierRanker
{
public:
    explicit RuleBasedFrontierRanker(
        FrontierScoringWeights weights = FrontierScoringWeights{},
        int max_retry_count = 2);

    std::vector<ScoredFrontierCandidate> rank(
        const std::vector<FrontierCandidate> & candidates,
        const std::optional<GridCell> & last_goal) const override;

private:
    FrontierScorer scorer_;
};

}  // 命名空间 frontier_strategy
