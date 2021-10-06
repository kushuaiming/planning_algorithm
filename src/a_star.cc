#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>

#include "map.h"

namespace {
static constexpr int32_t kMapDimensionX = 50;
static constexpr int32_t kMapDimensionY = 50;
}  // namespace

// bool verifyNode(Node rhs) {
//   if (rhs.coordinateX() < 0 || rhs.coordinateX() >= map_size ||
//       rhs.coordinateY() < 0 || rhs.coordinateY() >= map_size) {
//     Node start_node(40, 10);
//     Node goal_node(10, 40);
//     auto cmp = [](Node* left, Node* right) { return left->gN() > right->gN();
//     }; priority_queue<Node*, vector<Node*>, decltype(cmp)> O(cmp);

//     Node* tmp = roadMap[startNode.coordinateX()][startNode.coordinateY()];
//     O.push(tmp);

//     // Algorithm 24 A* Algorithm
//     while (!O.empty()) {
//       // Pick nbest from O such that f(nbest) <= f(n).
//       Node* nBest = O.top();
//       // Remove nbest from O and add to C(isVisited).
//       O.pop();
//       nBest->setIsVisited();
//       // if nbest == qgoal, EXIT.
//       if (*nBest == goalNode) break;

//       std::vector<Node> motion = {Node(1, 0, 1),
//                                   Node(0, 1, 1),
//                                   Node(-1, 0, 1),
//                                   Node(0, -1, 1),
//                                   Node(-1, -1, std::sqrt(2)),
//                                   Node(-1, 1, std::sqrt(2)),
//                                   Node(1, -1, std::sqrt(2)),
//                                   Node(1, 1, std::sqrt(2))};

//       for (int i = 0; i < 8; i++) {
//         Node tmpNode = motion[i];
//         tmpNode += *nBest;
//         Node* tmpNodePointer =
//             roadMap[tmpNode.coordinateX()][tmpNode.coordinateY()];
//         *tmpNodePointer = tmpNode;
//         if (verifyNode(*tmpNodePointer) && !tmpNodePointer->returnIsVisited()
//         &&
//             !tmpNodePointer->isObstacle()) {
//           O.push(tmpNodePointer);
//           tmpNodePointer->setIsVisited();
//           tmpNodePointer->setBackpoint(nBest);
//           tmpNodePointer->drawNode(background, imgGridSize, Scalar(0, 255,
//           0),
//                                    0);
//           imshow("aStar", background);
//           waitKey(5);
//         }
//       }
//     }

//     tmp = roadMap[goalNode.coordinateX()][goalNode.coordinateY()];
//     while (!(*tmp == startNode)) {
//       tmp->drawNode(background, imgGridSize, Scalar(255, 0, 0));
//       tmp = tmp->returnBackpoint();
//       imshow("aStar", background);
//       waitKey(5);
//     }
//   }
// }

// bool AStarAlgorithm() {
//   auto compare = [](Node lhs, Node rhs) { return lhs.fx() < rhs.fx() };
//   std::priority_queue<Node, std::vector<Node>, compare> O;

// }

// void CalculateHeuristicCost() {
//   for (int i = 0; i < kMapDimensionX; ++i) {
//     for (int j = 0; j < kMapDimensionY; ++j) {
//       double heuristic_cost = static_cast<double>(
//           sqrt(pow(i - goal_node_.x(), 2) + pow(j - goal_node_.y(), 2)));
//       nodes_[i][j].set_heuristic_cost(heuristic_cost);
//     }
//   }
// }

int main(int argc, char* argv[]) {
  // Point start_point, goal_point;
  // start_point.x = 40;
  // start_point.y = 10;
  // goal_point.x = 10;
  // goal_point.y = 40;
  // std::shared_ptr<Node> start_node = std::make_shared<Node>(start_point);
  // std::shared_ptr<Node> goal_node = std::make_shared<Node>(goal_point);

  Map map(kMapDimensionX, kMapDimensionY);
  map.Init();

  Obstacle obstacle;
  obstacle.upper_left_x_ = 2;
  obstacle.upper_left_y_ = 4;
  obstacle.dimension_x_ = 8;
  obstacle.dimension_y_ = 2;
  map.AddObstacle(obstacle);
  map.DrawObstacles();

  cv::imshow("a_star", map.background());
  cv::waitKey();
  // aStar(startNode, goalNode, roadMap, background);
  return -1;
}
