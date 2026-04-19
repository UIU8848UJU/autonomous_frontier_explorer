#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "types.h"

namespace frontier_explorer
{

struct FrontierSelectionCandidate
{
    GridCell goal{};
    GridCell cluster_centroid{};
    std::size_t cluster_size{0U};
    double distance_m{0.0};
};

class IFrontierSelectionStrategy
{
public:
    virtual ~IFrontierSelectionStrategy() = default;

    virtual std::optional<GridCell> select_goal(
        const std::vector<FrontierSelectionCandidate> & candidates) const = 0;

    virtual const char * name() const = 0;
};

class NearestFrontierSelectionStrategy final : public IFrontierSelectionStrategy
{
public:
    std::optional<GridCell> select_goal(
        const std::vector<FrontierSelectionCandidate> & candidates) const override;

    const char * name() const override;
};

class LargestClusterFrontierSelectionStrategy final : public IFrontierSelectionStrategy
{
public:
    std::optional<GridCell> select_goal(
        const std::vector<FrontierSelectionCandidate> & candidates) const override;

    const char * name() const override;
};

std::unique_ptr<IFrontierSelectionStrategy> create_frontier_selection_strategy(
    const std::string & strategy_name);

}  // namespace frontier_explorer
