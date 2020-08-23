#include <iostream>
#include <vector>
#include "rrt.h"
using namespace std;

int main(int argc, char* argv[]) {
    // 障碍物,前两个数表示圆心坐标,最后一个数表示半径
    vector<vector<float>> obstacleList{
        {7, 5, 1},
        {5, 6, 2},
        {5, 8, 2},
        {5, 10, 2},
        {9, 5, 2},
        {11, 5, 2}
    };

    // 起始点和目标点
    node* startNode = new node(2.0, 2.0);
    node* goalNode = new node(14.0, 9.0);

    RRT rrt(startNode, goalNode, obstacleList, 0.5, 5);
    rrt.planning();
    return 0;
}
