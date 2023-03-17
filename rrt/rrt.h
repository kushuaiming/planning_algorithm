#ifndef RRT_H_
#define RRT_H_

#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>

struct Point {
  Point(double x, double y) : x(x), y(y) {}
  double x = 0.0;
  double y = 0.0;
};

class Node {
 public:
  Node(double x, double y) : point_(x, y), parent_(nullptr), cost_(0.0) {}
  const Point& point() const { return point_; }
  void set_parent(Node* parent) { parent_ = parent; }
  Node* parent() { return parent_; }

 private:
  Point point_;
  std::vector<double> path_x_;
  std::vector<double> path_y_;
  Node* parent_ = nullptr;
  double cost_ = 0.0;
};

class RRT {
 public:
  RRT(Node* start_node, Node* goal_node,
      const std::vector<std::vector<double>>& obstacle_list,
      double step_size = 1.0, int goal_sample_rate = 5)
      : start_node_(start_node),
        goal_node_(goal_node),
        obstacle_list_(obstacle_list),
        step_size_(step_size),
        goal_sample_rate_(goal_sample_rate),
        goal_gen_(goal_rd_()),
        goal_dis_(std::uniform_int_distribution<int>(0, 100)),
        area_gen_(area_rd_()),
        area_dis_(std::uniform_real_distribution<double>(0, 15)) {}
  Node* GetNearestNode(const std::vector<double>& random_position);
  bool CollisionCheck(Node*);
  std::vector<Node*> Planning();

 private:
  Node* start_node_;
  Node* goal_node_;
  std::vector<std::vector<double>> obstacle_list_;
  std::vector<Node*> node_list_;
  double step_size_;

  int goal_sample_rate_;

  std::random_device goal_rd_;
  std::mt19937 goal_gen_;
  std::uniform_int_distribution<int> goal_dis_;

  std::random_device area_rd_;
  std::mt19937 area_gen_;
  std::uniform_real_distribution<double> area_dis_;
};

#endif
