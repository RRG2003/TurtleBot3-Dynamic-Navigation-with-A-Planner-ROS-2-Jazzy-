#include <memory>
#include <vector>
#include <queue>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_ros/buffer.h"

namespace tb3_astar_planner
{

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    name_ = name;
    RCLCPP_INFO(node_->get_logger(), "AStarPlanner plugin loaded");
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override
  {
    nav_msgs::msg::Path path;
    path.header.stamp = node_->get_clock()->now();
    path.header.frame_id = costmap_ros_->getGlobalFrameID();

    if (!costmap_) {
      RCLCPP_ERROR(node_->get_logger(), "Costmap is null");
      return path;
    }

    // Convert world coordinates to map indices
    unsigned int start_mx, start_my, goal_mx, goal_my;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;

    if (!costmap_->worldToMap(start_x, start_y, start_mx, start_my) ||
        !costmap_->worldToMap(goal_x, goal_y, goal_mx, goal_my)) {
      RCLCPP_WARN(node_->get_logger(), "Start or goal out of costmap bounds — returning empty path");
      return path;
    }

    const unsigned int size_x = costmap_->getSizeInCellsX();
    const unsigned int size_y = costmap_->getSizeInCellsY();

    auto idx = [&](int mx, int my){ return static_cast<unsigned int>(mx) + static_cast<unsigned int>(my) * size_x; };

    // A* structures
    const float INF = std::numeric_limits<float>::infinity();

    std::vector<float> g_score(size_x * size_y, INF);
    std::vector<float> f_score(size_x * size_y, INF);
    std::vector<int> came_from(size_x * size_y, -1);
    std::vector<char> closed(size_x * size_y, 0);

    // Heuristic (Euclidean)
    auto heuristic = [&](int mx, int my) {
      float dx = static_cast<float>(mx) - static_cast<float>(goal_mx);
      float dy = static_cast<float>(my) - static_cast<float>(goal_my);
      return std::sqrt(dx*dx + dy*dy);
    };

    // Check occupancy threshold: treat high cost as obstacle
    auto is_obstacle = [&](int mx, int my) {
      unsigned char cost = costmap_->getCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my));
      // cost>=253 is usually lethal in Nav2 costmaps; tune if required
      return cost >= nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::NO_INFORMATION;
    };

    // Priority queue for open set (min-heap) ordered by f_score
    struct NodeEntry { unsigned int index; float f; };
    struct Comp { bool operator()(NodeEntry const& a, NodeEntry const& b) const { return a.f > b.f; } };
    std::priority_queue<NodeEntry, std::vector<NodeEntry>, Comp> openq;

    unsigned int start_idx = idx(static_cast<int>(start_mx), static_cast<int>(start_my));
    unsigned int goal_idx = idx(static_cast<int>(goal_mx), static_cast<int>(goal_my));

    // If goal cell is obstacle, attempt to find nearby free cell (within small radius)
    if (is_obstacle(goal_mx, goal_my)) {
      RCLCPP_WARN(node_->get_logger(), "Goal cell is occupied — searching nearby free cell");
      bool found = false;
      const int search_rad = 3;
      for (int r = 1; r <= search_rad && !found; ++r) {
        for (int dx = -r; dx <= r && !found; ++dx) {
          for (int dy = -r; dy <= r && !found; ++dy) {
            int mx = static_cast<int>(goal_mx) + dx;
            int my = static_cast<int>(goal_my) + dy;
            if (mx < 0 || my < 0 || mx >= static_cast<int>(size_x) || my >= static_cast<int>(size_y)) continue;
            if (!is_obstacle(mx, my)) {
              goal_mx = mx; goal_my = my;
              goal_idx = idx(mx, my);
              found = true;
            }
          }
        }
      }
      if (!found) {
        RCLCPP_ERROR(node_->get_logger(), "No nearby free goal found — aborting plan");
        return path;
      }
    }

    g_score[start_idx] = 0.0f;
    f_score[start_idx] = heuristic(static_cast<int>(start_mx), static_cast<int>(start_my));

    openq.push({start_idx, f_score[start_idx]});

    // 8-connected neighbors (dx,dy)
    const int nbors[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    const float diag_cost = std::sqrt(2.0f);

    bool success = false;
    unsigned int expansions = 0;
    const unsigned int max_expansions = size_x * size_y; // safety guard

    while (!openq.empty()) {
      if (cancel_checker && cancel_checker()) {
        RCLCPP_WARN(node_->get_logger(), "Plan cancelled");
        return path;
      }

      NodeEntry cur = openq.top(); openq.pop();
      unsigned int cur_idx = cur.index;
      if (closed[cur_idx]) continue;
      closed[cur_idx] = 1;

      int cur_mx = static_cast<int>(cur_idx % size_x);
      int cur_my = static_cast<int>(cur_idx / size_x);

      ++expansions;
      if (expansions > max_expansions) break;

      if (cur_idx == goal_idx) {
        success = true;
        break;
      }

      for (auto &n : nbors) {
        int nx = cur_mx + n[0];
        int ny = cur_my + n[1];
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y)) continue;
        if (is_obstacle(nx, ny)) continue;

        unsigned int n_index = idx(nx, ny);

        // movement cost: base 1.0, diagonal diag_cost; add costmap cost penalty (0..1)
        float move_cost = (std::abs(n[0]) + std::abs(n[1]) == 2) ? diag_cost : 1.0f;
        unsigned char cell_cost = costmap_->getCost(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
        float cost_penalty = static_cast<float>(cell_cost) / 255.0f; // normalize
        float tentative_g = g_score[cur_idx] + move_cost + cost_penalty * 5.0f; // scale penalty

        if (tentative_g < g_score[n_index]) {
          g_score[n_index] = tentative_g;
          f_score[n_index] = tentative_g + heuristic(nx, ny);
          came_from[n_index] = static_cast<int>(cur_idx);
          openq.push({n_index, f_score[n_index]});
        }
      }
    } // end search loop

    if (!success) {
      RCLCPP_WARN(node_->get_logger(), "A* did not find a path");
      return path;
    }

    // Reconstruct path from goal_idx -> start_idx
    std::vector<unsigned int> revpath;
    unsigned int cur = goal_idx;
    while (cur != start_idx && cur != static_cast<unsigned int>(-1)) {
      revpath.push_back(cur);
      int parent = came_from[cur];
      if (parent < 0) break;
      cur = static_cast<unsigned int>(parent);
    }
    revpath.push_back(start_idx);

    // Convert to poses in world coords (reverse)
    for (auto it = revpath.rbegin(); it != revpath.rend(); ++it) {
      unsigned int index = *it;
      int mx = static_cast<int>(index % size_x);
      int my = static_cast<int>(index / size_x);
      double wx, wy;
      costmap_->mapToWorld(mx, my, wx, wy);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = path.header.frame_id;
      pose.header.stamp = node_->get_clock()->now();
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    RCLCPP_INFO(node_->get_logger(), "A* plan found with %zu poses", path.poses.size());
    return path;
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
};

}  // namespace tb3_astar_planner

PLUGINLIB_EXPORT_CLASS(tb3_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)