#include "frontier_strategy_core/detector/frontier_detector.hpp"
#include "frontier_strategy_core/information_gain/information_gain_estimator.hpp"
#include "frontier_strategy_core/scoring/frontier_scorer.hpp"
#include "frontier_strategy_core/selector/filters/frontier_pruner.hpp"
#include "frontier_strategy_core/selector/frontier_selection_policy.hpp"
#include "grid_map_core/types/grid_map.hpp"

int main()
{
  frontier_strategy::FrontierDetector detector(0);
  frontier_strategy::FrontierScoringWeights weights;
  frontier_strategy::FrontierScorer scorer(weights, 3);
  frontier_strategy::FrontierPruner pruner(0.5, 2, 3, 1U, 2, 2, 0.4);
  frontier_strategy::FrontierSelectionPolicy policy;
  frontier_strategy::InformationGainEstimator information_gain_estimator(3.0);
  grid_map_core::GridMap map;
  const auto estimate = information_gain_estimator.estimate(
    map, frontier_strategy::GridCell{});

  (void)detector;
  (void)scorer;
  (void)pruner;
  (void)policy;
  (void)estimate;
  return map.isReady() ? 1 : 0;
}
