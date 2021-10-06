#ifndef PATHPLANNING_COMMON_MAP_H_
#define PATHPLANNING_COMMON_MAP_H_

#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace {
static constexpr int32_t kMapGridSize = 10;
}  // namespace

struct Point {
  Point(int32_t x, int32_t y) : x(x), y(y) {}
  int32_t x;
  int32_t y;
};

class Node {
 public:
  Node(Point point)
      : point_(point),
        path_length_cost_(0.0),
        heuristic_cost_(0.0),
        total_cost_(0.0),
        is_obstacle_(false){};

  const Point& point() { return point_; }
  const Node* pre_node() const { return pre_node_; }
  void set_pre_node(Node* pre_node) { pre_node_ = pre_node; }

  double* mutable_path_length_cost() { return &path_length_cost_; }
  double path_length_cost() { return path_length_cost_; }
  double* mutable_heuristic_cost() { return &heuristic_cost_; };
  double heuristic_cost() { return heuristic_cost_; };
  double total_cost() {
    total_cost_ = path_length_cost_ + heuristic_cost_;
    return total_cost_;
  }

  void set_is_obstacle() { is_obstacle_ = true; };
  bool is_obstacle() { return is_obstacle_; };

 private:
  Point point_;
  const Node* pre_node_ = nullptr;

  double path_length_cost_;  // g(n)
  double heuristic_cost_;    // h(n)
  double total_cost_;        // f(n) = g(n) + h(n)

  bool is_obstacle_;
};

struct Obstacle {
  int32_t upper_left_x_;
  int32_t upper_left_y_;
  int32_t dimension_x_;
  int32_t dimension_y_;
};

class Map {
 public:
  Map(int32_t dimension_x = 0, int32_t dimension_y = 0)
      : dimension_x_(dimension_x), dimension_y_(dimension_y) {
    Init();
  };

  void Init();
  void AddObstacle(const Obstacle& obstacle);
  void DrawObstacles();
  void DrawNode(Node node,
                   cv::Scalar color = cv::Scalar(0, 0, 0), int thickness = -1);

  Node* mutable_node(int32_t i, int32_t j) {
    return &nodes_[i][j];
  }
  const cv::Mat& background() { return background_; }

 private:
  std::vector<std::vector<Node>> nodes_;
  int32_t dimension_x_;
  int32_t dimension_y_;
  cv::Mat background_;
};

#endif  // PATHPLANNING_COMMON_MAP_H_
