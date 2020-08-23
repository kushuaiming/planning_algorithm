#ifndef _RRT_H
#define _RRT_H

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class node {
private:
    float x, y;                // 节点坐标
    vector<float> pathX, pathY;// 路径
    node* parent;              // 父节点
    float cost;
public:
    node(float _x, float _y);
    float getX();
    float getY();
    void setParent(node*);
    node* getParent();
};

class RRT {
private:
    node* startNode, * goalNode;        // 起始节点和目标节点
    vector<vector<float>> obstacleList; // 障碍物
    vector<node*> nodeList;             // 
    float stepSize;                     // 步长

    int goal_sample_rate;

    random_device goal_rd;
    mt19937 goal_gen;
    uniform_int_distribution<int> goal_dis;

    random_device area_rd;
    mt19937 area_gen;
    uniform_real_distribution<float> area_dis;
public:
    RRT(node*, node*, const vector<vector<float>>&, float , int);
    node* getNearestNode(const vector<float>&);
    bool collisionCheck(node*);
    vector<node*> planning();
};

#endif
