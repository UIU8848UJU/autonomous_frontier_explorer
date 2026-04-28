#include "frontier_detector.hpp"

#include <cmath>
#include <queue>
#include <unordered_set>

namespace frontier_explorer
{

FrontierDetector::FrontierDetector(
    int obstacle_search_radius_cells,
    const rclcpp::Logger & logger)
: logger_(rclcpp::Logger(logger).get_child("detector")),
  obstacle_search_radius_cells_(obstacle_search_radius_cells)
{

}

bool FrontierDetector::is_frontier_cell_safe( const CostmapAdapter & costmap,
    const GridCell & cell) const
{
    for (int dr = -obstacle_search_radius_cells_; 
            dr <= obstacle_search_radius_cells_; ++dr) {

        for (int dc = -obstacle_search_radius_cells_; 
                dc <= obstacle_search_radius_cells_; ++dc) {
                
            const int row = cell.row + dr;
            const int col = cell.col + dc;
            if (!costmap.inBounds(col, row) ||
                costmap.isObstacle(static_cast<unsigned int>(col), static_cast<unsigned int>(row)))
            {
                return false;
            }
        }
    }
  return true;
}

std::vector<GridCell> FrontierDetector::detect_frontier_cells(
    const CostmapAdapter & costmap) const
{
    std::vector<GridCell> frontier_cells;

    if (!costmap.isReady()) {
        RCLCPP_WARN(logger_, "Cannot detect frontiers because costmap is not ready.");
        return frontier_cells;
    }

    const int rows = static_cast<int>(costmap.getSizeInCellsY());
    const int cols = static_cast<int>(costmap.getSizeInCellsX());

    RCLCPP_DEBUG(
        logger_,
        "Detecting frontier cells on costmap: width=%d, height=%d, resolution=%.3f",
        cols,
        rows,
        costmap.getResolution());

    for (int r = 1; r < rows - 1; ++r) {

        for (int c = 1; c < cols - 1; ++c) {
            const auto mx = static_cast<unsigned int>(c);
            const auto my = static_cast<unsigned int>(r);
            if (!costmap.isFree(mx, my)) {
                continue;
            }

            if (!costmap.hasUnknownNeighbor(mx, my)) {
                continue;
            }

            GridCell cell{r, c};
            if (is_frontier_cell_safe(costmap, cell)) {
                frontier_cells.push_back(cell);
            }
        }
    }

    if (frontier_cells.empty()) {
        RCLCPP_WARN(logger_, "No frontier cells found.");
    }

    return frontier_cells;
}

std::vector<GridCell> FrontierDetector::detect_frontier_cells(
    const nav_msgs::msg::OccupancyGrid & map) const
{
    CostmapAdapter costmap(logger_);
    if (!costmap.updateFromOccupancyGrid(map)) {
        return {};
    }
    return detect_frontier_cells(costmap);
}

std::vector<FrontierCluster> FrontierDetector::cluster_frontiers(
    const CostmapAdapter &,
    const std::vector<GridCell> & frontier_cells) const
{
    std::vector<FrontierCluster> clusters;

    std::unordered_set<GridCell, GridCellHash> frontier_set(
        frontier_cells.begin(), frontier_cells.end());
    std::unordered_set<GridCell, GridCellHash> visited;

    for (const auto & start : frontier_cells) {
        if (visited.count(start) > 0) {
            continue;
        }

        FrontierCluster cluster;
        std::queue<GridCell> q;
        q.push(start);
        visited.insert(start);

        while (!q.empty()) {
            const auto current = q.front();
            q.pop();
            cluster.cells.push_back(current);

            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {

                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    GridCell next{current.row + dr, current.col + dc};

                    if (frontier_set.count(next) > 0 && visited.count(next) == 0) {
                        visited.insert(next);
                        q.push(next);
                    }
                }
            }
        }

        double sum_row = 0.0;
        double sum_col = 0.0;
        for (const auto & cell : cluster.cells) {
            sum_row += static_cast<double>(cell.row);
            sum_col += static_cast<double>(cell.col);
        }

        cluster.centroid = GridCell{
            static_cast<int>(std::round(sum_row / cluster.cells.size())),
            static_cast<int>(std::round(sum_col / cluster.cells.size()))
        };

        // centroid 只作为 cluster id 和优先候选点；如果它不可用，
        // FrontierPruner 会在 cluster.cells 里寻找 fallback goal。
        clusters.push_back(cluster);
    }

  return clusters;
}

std::vector<FrontierCluster> FrontierDetector::cluster_frontiers(
    const nav_msgs::msg::OccupancyGrid &,
    const std::vector<GridCell> & frontier_cells) const
{
    CostmapAdapter costmap(logger_);
    return cluster_frontiers(costmap, frontier_cells);
}

}  // namespace frontier_explorer
