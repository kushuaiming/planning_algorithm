#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <unordered_set>
#include <vector>

#include "map.h"

namespace {
static constexpr int32_t kMapDimensionX = 50;
static constexpr int32_t kMapDimensionY = 50;
static Map map(kMapDimensionX, kMapDimensionY);
}  // namespace

bool IsCoordinateValid(int32_t x, int32_t y) {
  return x >= 0 && x < kMapDimensionX && y >= 0 && y < kMapDimensionY;
}

// Return the set of nodes which are adjacent to n.
void Star(Node* n, std::vector<Node*>& adjacent_nodes) {
  int32_t x = n->point().x, y = n->point().y;
  if (IsCoordinateValid(x, y - 1)) {
    Node* upper_node = map.mutable_node(x, y - 1);
    *(upper_node->mutable_path_length_cost()) = n->path_length_cost() + 1;
    adjacent_nodes.push_back(upper_node);
  }
  if (IsCoordinateValid(x, y + 1)) {
    Node* down_node = map.mutable_node(x, y + 1);
    *(down_node->mutable_path_length_cost()) = n->path_length_cost() + 1;
    adjacent_nodes.push_back(down_node);
  }
  if (IsCoordinateValid(x - 1, y)) {
    Node* left_node = map.mutable_node(x - 1, y);
    *(left_node->mutable_path_length_cost()) = n->path_length_cost() + 1;
    adjacent_nodes.push_back(left_node);
  }
  if (IsCoordinateValid(x + 1, y)) {
    Node* right_node = map.mutable_node(x + 1, y);
    *(right_node->mutable_path_length_cost()) = n->path_length_cost() + 1;
    adjacent_nodes.push_back(right_node);
  }
  if (IsCoordinateValid(x - 1, y - 1)) {
    Node* upper_left_node = map.mutable_node(x - 1, y - 1);
    // 1.4 is the square of 2.
    *(upper_left_node->mutable_path_length_cost()) = n->path_length_cost() + 1.4;
    adjacent_nodes.push_back(upper_left_node);
  }
  if (IsCoordinateValid(x + 1, y - 1)) {
    Node* upper_right_node = map.mutable_node(x + 1, y - 1);
    *(upper_right_node->mutable_path_length_cost()) = n->path_length_cost() + 1.4;
    adjacent_nodes.push_back(upper_right_node);
  }
  if (IsCoordinateValid(x - 1, y + 1)) {
    Node* down_left_node = map.mutable_node(x - 1, y + 1);
    *(down_left_node->mutable_path_length_cost()) = n->path_length_cost() + 1.4;
    adjacent_nodes.push_back(down_left_node);
  }
  if (IsCoordinateValid(x + 1, y + 1)) {
    Node* down_right_node = map.mutable_node(x + 1, y + 1);
    *(down_right_node->mutable_path_length_cost()) = n->path_length_cost() + 1.4;
    adjacent_nodes.push_back(down_right_node);
  }
  return;
}

void CalculateHeuristicCost(Node* goal_node) {
  for (int i = 0; i < kMapDimensionX; ++i) {
    for (int j = 0; j < kMapDimensionY; ++j) {
      double heuristic_cost =
          std::hypot(i - goal_node->point().x, j - goal_node->point().y);
      *(map.mutable_node(i, j)->mutable_heuristic_cost()) = heuristic_cost;
    }
  }
}

void AddObstacles() {
  Obstacle obstacle;
  obstacle.upper_left_x_ = 2;
  obstacle.upper_left_y_ = 4;
  obstacle.dimension_x_ = 8;
  obstacle.dimension_y_ = 2;
  map.AddObstacle(obstacle);

  obstacle.upper_left_x_ = 20;
  obstacle.upper_left_y_ = 30;
  obstacle.dimension_x_ = 4;
  obstacle.dimension_y_ = 4;
  map.AddObstacle(obstacle);

  map.DrawObstacles();
}

bool AStarAlgorithm() {
  Node* start_node = map.mutable_node(10, 40);
  Node* goal_node = map.mutable_node(40, 10);

  CalculateHeuristicCost(goal_node);

  AddObstacles();

  std::unordered_set<Node*> C;  // processed nodes
  auto compare = [](Node* lhs, Node* rhs) {
    return lhs->total_cost() < rhs->total_cost();
  };
  std::priority_queue<Node*, std::vector<Node*>, decltype(compare)> O(compare);
  O.push(start_node);
  C.insert(start_node);
  while (!O.empty()) {
    // Pick nbest from O such that f(nbest) <= f(n).
    Node* n_best = O.top();
    // Remove n_best from O and add to C(which mutable_node has been visited).
    O.pop();
    C.insert(n_best);
    if (n_best == goal_node) break;
    std::vector<Node*> adjacent_nodes;
    Star(n_best, adjacent_nodes);
    for (Node* adjacent_node : adjacent_nodes) {
      if (C.find(adjacent_node) != C.end() || adjacent_node->is_obstacle())
        continue;
      O.push(adjacent_node);
      adjacent_node->set_pre_node(n_best);
      map.DrawNode(*adjacent_node);
      cv::imshow("a_star", map.background());
      cv::waitKey(5);
    }
  }

  return true;
}

int main(int argc, char* argv[]) {
  AStarAlgorithm();
  cv::imshow("a_star", map.background());
  cv::waitKey();
  return -1;
}
