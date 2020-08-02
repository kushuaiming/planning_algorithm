#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "aStar.h"
using namespace std;
using namespace cv;

const int mapSize = 50;   // 地图大小
const int imgGridSize = 5;// 显示的图像栅格大小

// 设置障碍物
void defineObstacle(vector<vector<node*>>& roadmap) {
    // 先框住四周
    for (int i = 0; i < mapSize; ++i) {
        roadmap[0][i]->setObstacle();
        roadmap[i][0]->setObstacle();
        roadmap[i][mapSize - 1]->setObstacle();
        roadmap[mapSize - 1][i]->setObstacle();
    }

    // 再定义一个条形快
    for (int i = 1; i < mapSize / 2; ++i) {
        roadmap[mapSize * 2 / 3][i]->setObstacle();
        roadmap[mapSize * 2 / 3 - 1][i]->setObstacle();
        roadmap[mapSize * 1 / 3][mapSize - i]->setObstacle();
        roadmap[mapSize * 1 / 3 - 1][mapSize - i]->setObstacle();
    }
}

// 画出地图
void drawMap(Mat& background, node startNode, node goalNode, vector<vector<node*>> roadmap) {
    // 画出障碍物
    for (int i = 0; i < mapSize; ++i) {
        for (int j = 0; j < mapSize; ++j) {
            if (roadmap[i][j]->isObstacle())
                roadmap[i][j]->drawNode(background, imgGridSize, Scalar(0, 0, 0));
        }
    }

    // 画出起始点和终点
    startNode.drawNode(background, imgGridSize, Scalar(255, 0, 0));//起点为蓝色
    goalNode.drawNode(background, imgGridSize, Scalar(0, 0, 255)); //目标位置为红色
}

// 计算启发式距离
double calHeristic(node* n1, node n2) {
    return (double)sqrt(pow(n1->coordinateX() - n2.coordinateX(), 2) + pow(n1->coordinateY() - n2.coordinateY(), 2));
}

// 判断节点是否出界
bool verifyNode(node rhs) {
    if (rhs.coordinateX() < 0 || rhs.coordinateX() >= mapSize
        || rhs.coordinateY() < 0 || rhs.coordinateY() >= mapSize) {
        return false;
    }
    return true;
}

void aStar(const node& startNode, const node& goalNode, vector<vector<node*>>& roadmap, Mat& background) {
    // 使用Lambda表达式定义一个优先队列
    auto cmp = [](node* left, node* right) { return left->gN() > right->gN(); };
    priority_queue<node*, vector<node*>, decltype(cmp)> O(cmp);

    node* tmp = roadmap[startNode.coordinateX()][startNode.coordinateY()];
    O.push(tmp);

    // Algorithm 24 A* Algorithm
    while (!O.empty()) {
        // Pick nbest from O such that f(nbest) <= f(n).
        node* nBest = O.top();
        // Remove nbest from O and add to C(isVisited).
        O.pop();
        nBest->setIsVisited();
        // if nbest == qgoal, EXIT.
        if (*nBest == goalNode)
            break;

        // 八个方向都可以走
        std::vector<node> motion = { node(1,   0,  1),
                node(0,   1,  1),
                node(-1,   0,  1),
                node(0,   -1,  1),
                node(-1,   -1,  std::sqrt(2)),
                node(-1,   1,  std::sqrt(2)),
                node(1,   -1,  std::sqrt(2)),
                node(1,    1,  std::sqrt(2)) };

        for (int i = 0; i < 8; i++) {
            node tmpNode = motion[i];
            tmpNode += *nBest;
            node* tmpNodePointer = roadmap[tmpNode.coordinateX()][tmpNode.coordinateY()];
            *tmpNodePointer = tmpNode;
            if (verifyNode(*tmpNodePointer) && !tmpNodePointer->returnIsVisited() && !tmpNodePointer->isObstacle()) {
                O.push(tmpNodePointer);
                tmpNodePointer->setIsVisited();
                tmpNodePointer->setBackpoint(nBest);
                tmpNodePointer->drawNode(background, imgGridSize, Scalar(0, 255, 0), 0);
                imshow("aStar", background);
                waitKey(5);
            }
        }
    }

    // 画出最终的路径
    tmp = roadmap[goalNode.coordinateX()][goalNode.coordinateY()];
    while (!(*tmp == startNode)) {
        tmp->drawNode(background, imgGridSize, Scalar(255, 0, 0));
        tmp = tmp->returnBackpoint();
        imshow("aStar", background);
        waitKey(5);
    }
}

int main(int argc, char* argv[]) {
    
    node startNode(40, 10);// 起始点
    node goalNode(10, 40); // 目标点

    int gridSize = 1, robotSize = 1;// 栅格尺寸和机器人尺寸

    vector<vector<node*>> roadmap;// 地图,这里存储的是指针(方便后续程序的攥写)

    // 初始化roadmap
    for (int i = 0; i < mapSize; ++i) {
        vector<node*> tmp;
        for (int j = 0; j < mapSize; ++j) {
            node* tmpNode = new node(i, j);
            tmpNode->setHeuristic(calHeristic(tmpNode, goalNode));
            tmp.push_back(tmpNode);
        }
        roadmap.push_back(tmp);
    }

    // 添加障碍物
    defineObstacle(roadmap);

    // 打开一个窗口
    namedWindow("aStar", WINDOW_AUTOSIZE);
    
    Mat background(mapSize * imgGridSize,
                   mapSize * imgGridSize,
                   CV_8UC3,//8位无符号char型,3通道
                   cv::Scalar(255, 255, 255));

    // 画出地图
    drawMap(background, startNode, goalNode, roadmap);
    imshow("aStar", background);
    waitKey(5);

    aStar(startNode, goalNode, roadmap, background);
    waitKey();
    return 0;
}