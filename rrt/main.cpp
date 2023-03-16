#include <iostream>
#include <vector>

#include "rrt.h"
using namespace std;

int main(int argc, char* argv[]) {
  // 障碍物,前两个数表示圆心坐标,最后一个数表示半径
  vector<vector<double>> obstacle_list{{7, 5, 1},  {5, 6, 2}, {5, 8, 2},
                                     {5, 10, 2}, {9, 5, 2}, {11, 5, 2}};

  Node* start_node = new Node(2.0, 2.0);
  Node* goal_node = new Node(14.0, 9.0);

  RRT rrt(start_node, goal_node, obstacle_list, 0.5, 5);
  rrt.planning();
  return 0;
}
