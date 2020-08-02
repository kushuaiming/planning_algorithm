#ifndef _ASTAR_H
#define _ASTAR_H

#include <iostream>
#include <vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace cv;

// 用来存储节点
class node {
public:
    node(int _x = 0, int _y = 0, double _sumCost = 0, double _heuristic = 0);
    ~node();
    void setObstacle();
    bool isObstacle();
    void drawNode(Mat& background, int imgGridSize, const Scalar& color, int shift = -1);
    bool operator==(const node& rhs);
    node& operator+=(node rhs);
    node& operator=(node rhs);
    int coordinateX() const;
    int coordinateY() const;
    void setBackpoint(node* rhs);
    node* returnBackpoint();
    void setHeuristic(double _heuristic);
    double returnHeuristic();
    double gN();
    bool returnIsVisited();
    void setIsVisited();
private:
    int x, y;        // 坐标
    double sumCost;  // f(n)
    double heuristic;// 启发值
    bool obstacle;   // 是否是障碍物
    node* backpoint; // 前驱节点
    bool isVisited;  // 判断是否访问过
};

#endif
