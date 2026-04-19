#include "frontier_detector.hpp"

#include <cmath>
#include <queue>
#include <unordered_set>

namespace frontier_explorer
{

FrontierDetector::FrontierDetector( int obstacle_search_radius_cells,
    int min_frontier_cluster_size)
: obstacle_search_radius_cells_(obstacle_search_radius_cells),
  min_frontier_cluster_size_(min_frontier_cluster_size)
{

}

bool FrontierDetector::is_frontier_cell_safe( const nav_msgs::msg::OccupancyGrid & map,
    const GridCell & cell) const
{
    for (int dr = -obstacle_search_radius_cells_; 
            dr <= obstacle_search_radius_cells_; ++dr) {

        for (int dc = -obstacle_search_radius_cells_; 
                dc <= obstacle_search_radius_cells_; ++dc) {
                
            if (is_cell_obstacle(map, cell.row + dr, cell.col + dc)) {
                return false;
            }
        }
    }
  return true;
}

std::vector<GridCell> FrontierDetector::detect_frontier_cells(
    const nav_msgs::msg::OccupancyGrid & map) const
{
    std::vector<GridCell> frontier_cells;

    const int rows = static_cast<int>(map.info.height);
    const int cols = static_cast<int>(map.info.width);

    for (int r = 1; r < rows - 1; ++r) {

        for (int c = 1; c < cols - 1; ++c) {
            if (!is_cell_free(map, r, c)) {
                continue;
            }

            bool has_unknown_neighbor = false;

            for (int dr = -1; dr <= 1 && !has_unknown_neighbor; ++dr) {

                for (int dc = -1; dc <= 1; ++dc) {
                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    if (is_cell_unknown(map, r + dr, c + dc)) {
                        has_unknown_neighbor = true;
                        break;
                    }
                }
            }

            if (!has_unknown_neighbor) {
                continue;
            }

            GridCell cell{r, c};
            if (is_frontier_cell_safe(map, cell)) {
                frontier_cells.push_back(cell);
            }
        }
    }

    return frontier_cells;
}

std::vector<FrontierCluster> FrontierDetector::cluster_frontiers( 
    const nav_msgs::msg::OccupancyGrid & map,
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

        // 这里是为了除去一些离散点的噪声
        if (static_cast<int>(cluster.cells.size()) < min_frontier_cluster_size_) {
            continue;
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
        // 出去前确保这个中心点是可探测的，这里暂时没有判断是否可达
        if (is_frontier_cell_safe(map, cluster.centroid)) {
            clusters.push_back(cluster);
        }
    }

  return clusters;
}

}  // namespace frontier_explorer