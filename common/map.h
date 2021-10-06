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
        total_cost_(0.0),
        heuristic_cost_(0.0),
        is_obstacle_(false),
        is_visited_(false){};

  const Point& point() { return point_; }
  void set_is_obstacle() { is_obstacle_ = true; };
  bool is_obstacle() { return is_obstacle_; };
  void set_pre_node(std::shared_ptr<Node> pre_node) { pre_node_ = pre_node; };
  void set_is_visited() { is_visited_ = true; };
  bool is_visited() const { return is_visited_; };
  void set_heuristic_cost(double heuristic_cost) {
    heuristic_cost_ = heuristic_cost;
  };
  double heuristic_cost() { return heuristic_cost_; };

 private:
  Point point_;
  std::shared_ptr<Node> pre_node_ = nullptr;
  double total_cost_;  // f(n)
  double heuristic_cost_;
  bool is_obstacle_;
  bool is_visited_;
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
  void DrawNode(int32_t x, int32_t y);

  const cv::Mat& background() { return background_; }

 private:
  std::vector<std::vector<std::shared_ptr<Node>>> nodes_;
  int32_t dimension_x_;
  int32_t dimension_y_;
  cv::Mat background_;
};

#endif  // PATHPLANNING_COMMON_MAP_H_
