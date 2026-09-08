#include "frontier_explorer_core/detector/frontier_detector.hpp"
#include "frontier_explorer_core/scoring/frontier_scorer.hpp"
#include "frontier_explorer_core/selector/filters/frontier_pruner.hpp"
#include "frontier_explorer_core/selector/frontier_selection_policy.hpp"
#include "frontier_explorer_core/types/grid_map.hpp"

int main()
{
  frontier_explorer::FrontierDetector detector(0);
  frontier_explorer::FrontierScoringWeights weights;
  frontier_explorer::FrontierScorer scorer(weights, 3);
  frontier_explorer::FrontierPruner pruner(0.5, 2, 3, 1U, 2, 2, 0.4);
  frontier_explorer::FrontierSelectionPolicy policy;
  frontier_explorer::GridMap map;

  (void)detector;
  (void)scorer;
  (void)pruner;
  (void)policy;
  return map.isReady() ? 1 : 0;
}
