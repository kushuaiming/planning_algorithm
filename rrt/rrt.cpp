#include "rrt.h"

Node* RRT::GetNearestNode(const std::vector<double>& random_position) {
  int min_id = -1;
  double min_distance = std::numeric_limits<double>::max();
  for (int i = 0; i < node_list_.size(); i++) {
    double square_distance =
        std::pow(node_list_[i]->point().x - random_position[0], 2) +
        std::pow(node_list_[i]->point().y - random_position[1], 2);
    if (square_distance < min_distance) {
      min_distance = square_distance;
      min_id = i;
    }
  }

  return node_list_[min_id];
}

bool RRT::CollisionCheck(Node* newNode) {
  for (auto item : obstacle_list_) {
    if (std::sqrt(std::pow((item[0] - newNode->point().x), 2) +
                  std::pow((item[1] - newNode->point().y), 2)) <= item[2])
      return false;
  }
  return true;
}

std::vector<Node*> RRT::Planning() {
  cv::namedWindow("RRT");

  int count = 0;

  constexpr int kImageSize = 15;
  constexpr int kImageResolution = 50;
  cv::Mat background(kImageSize * kImageResolution,
                     kImageSize * kImageResolution, CV_8UC3,
                     cv::Scalar(255, 255, 255));

  circle(background,
         cv::Point(start_node_->point().x * kImageResolution,
                   start_node_->point().y * kImageResolution),
         20, cv::Scalar(0, 0, 255), -1);
  circle(background,
         cv::Point(goal_node_->point().x * kImageResolution,
                   goal_node_->point().y * kImageResolution),
         20, cv::Scalar(255, 0, 0), -1);

  for (auto item : obstacle_list_) {
    circle(background,
           cv::Point(item[0] * kImageResolution, item[1] * kImageResolution),
           item[2] * kImageResolution, cv::Scalar(0, 0, 0), -1);
  }

  node_list_.push_back(start_node_);
  while (1) {
    std::vector<double> random_position;
    if (goal_dis_(goal_gen_) > goal_sample_rate_) {
      double randX = area_dis_(goal_gen_);
      double randY = area_dis_(goal_gen_);
      random_position.push_back(randX);
      random_position.push_back(randY);
    } else {
      random_position.push_back(goal_node_->point().x);
      random_position.push_back(goal_node_->point().y);
    }

    Node* nearestNode = GetNearestNode(random_position);
    double theta = atan2(random_position[1] - nearestNode->point().y,
                         random_position[0] - nearestNode->point().x);
    Node* newNode = new Node(nearestNode->point().x + step_size_ * cos(theta),
                             nearestNode->point().y + step_size_ * sin(theta));
    newNode->set_parent(nearestNode);

    if (!CollisionCheck(newNode)) continue;
    node_list_.push_back(newNode);

    line(background,
         cv::Point(static_cast<int>(newNode->point().x * kImageResolution),
                   static_cast<int>(newNode->point().y * kImageResolution)),
         cv::Point(static_cast<int>(nearestNode->point().x * kImageResolution),
                   static_cast<int>(nearestNode->point().y * kImageResolution)),
         cv::Scalar(0, 255, 0), 10);

    count++;
    imshow("RRT", background);
    cv::waitKey(5);

    if (sqrt(pow(newNode->point().x - goal_node_->point().x, 2) +
             pow(newNode->point().y - goal_node_->point().y, 2)) <=
        step_size_) {
      std::cout << "The path has been found!" << std::endl;
      break;
    }
  }

  std::vector<Node*> path;
  path.push_back(goal_node_);
  Node* tmp_node = node_list_.back();
  while (tmp_node->parent() != nullptr) {
    line(
        background,
        cv::Point(static_cast<int>(tmp_node->point().x * kImageResolution),
                  static_cast<int>(tmp_node->point().y * kImageResolution)),
        cv::Point(
            static_cast<int>(tmp_node->parent()->point().x * kImageResolution),
            static_cast<int>(tmp_node->parent()->point().y * kImageResolution)),
        cv::Scalar(255, 0, 255), 10);
    path.push_back(tmp_node);
    tmp_node = tmp_node->parent();
  }

  imshow("RRT", background);
  cv::waitKey(0);
  path.push_back(start_node_);
  return path;
}
