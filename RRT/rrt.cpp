#include "rrt.h"

node::node(float _x, float _y) : x(_x), y(_y), parent(nullptr), cost(0) {}

float node::getX() { return x; }
float node::getY() { return y; }

void node::setParent(node* _parent) { parent = _parent; }
node* node::getParent() { return parent; }

RRT::RRT(node* _startNode, node* _goalNode, const vector<vector<float>>& _obstacleList,
         float _stepSize = 1.0, int _goal_sample_rate = 5)
    : startNode(_startNode), goalNode(_goalNode),
      obstacleList(_obstacleList),
      stepSize(_stepSize), goal_sample_rate(_goal_sample_rate),
      goal_gen(goal_rd()), goal_dis(uniform_int_distribution<int>(0, 100)),
      area_gen(area_rd()), area_dis(uniform_real_distribution<float>(0, 15))
      {}

node* RRT::getNearestNode(const vector<float>& randomPosition) {
    int minID = -1;
    float minDistance = numeric_limits<float>::max(); // 编译器允许的float类型的最大值

    // 找到和随机位置距离最小的节点
    for (int i = 0; i < nodeList.size(); i++) {
        // 在这里距离不需要开根号
        float distance = pow(nodeList[i]->getX() - randomPosition[0], 2) + pow(nodeList[i]->getY() - randomPosition[1], 2);
        if (distance < minDistance) {
            minDistance = distance;
            minID = i;
        }
    }

    return nodeList[minID];
}

bool RRT::collisionCheck(node* newNode) {
    for (auto item : obstacleList)
        if (sqrt(pow((item[0] - newNode->getX()), 2) + std::pow((item[1] - newNode->getY()), 2)) <= item[2])
            return false;
    return true;
}

vector<node*> RRT::planning() {
    // 建立一个窗口,WINDOW_NORMAL表示可以调整窗口大小
    namedWindow("RRT", WINDOW_NORMAL);

    int count = 0;

    // 建立背景
    const int imageSize = 15;
    const int imageResolution = 50;
    Mat background(imageSize * imageResolution, imageSize * imageResolution,
        CV_8UC3, cv::Scalar(255, 255, 255)); // CV_[位数][是否带符号]C[通道数]

    // 画出起始位置和目标位置
    circle(background, Point(startNode->getX() * imageResolution, startNode->getY() * imageResolution), 20, Scalar(0, 0, 255), -1);
    circle(background, Point(goalNode->getX() * imageResolution, goalNode->getY() * imageResolution), 20, Scalar(255, 0, 0), -1);
    // 画出障碍物
    for (auto item : obstacleList)
        circle(background, Point(item[0] * imageResolution, item[1] * imageResolution), item[2] * imageResolution, Scalar(0, 0, 0), -1);

    // RRT
    nodeList.push_back(startNode);
    while (1) {
        // 生成一个随机位置
        vector<float> randomPosition;
        if (goal_dis(goal_gen) > goal_sample_rate) { // 这里可以优化成直接用节点来表示
            float randX = area_dis(goal_gen);
            float randY = area_dis(goal_gen);
            randomPosition.push_back(randX);
            randomPosition.push_back(randY);
        }
        else { // 找到了目标,将目标位置保存
            randomPosition.push_back(goalNode->getX());
            randomPosition.push_back(goalNode->getY());
        }

        // 找到和新生成随机节点距离最近的节点
        node* nearestNode = getNearestNode(randomPosition);
        // 利用反正切计算角度,然后利用角度和步长计算新坐标
        float theta = atan2(randomPosition[1] - nearestNode->getY(), randomPosition[0] - nearestNode->getX());
        node* newNode = new node(nearestNode->getX() + stepSize * cos(theta), nearestNode->getY() + stepSize * sin(theta));
        newNode->setParent(nearestNode);

        if (!collisionCheck(newNode)) continue;
        nodeList.push_back(newNode);

        // 画出路径
        line(background,
            Point(static_cast<int>(newNode->getX() * imageResolution), static_cast<int>(newNode->getY() * imageResolution)),
            Point(static_cast<int>(nearestNode->getX() * imageResolution), static_cast<int>(nearestNode->getY() * imageResolution)),
            Scalar(0, 255, 0),10);

        count++;
        imshow("RRT", background);
        waitKey(5);

        if (sqrt(pow(newNode->getX() - goalNode->getX(), 2) + pow(newNode->getY() - goalNode->getY(), 2)) <= stepSize) {
            cout << "The path has been found!" << endl;
            break;
        }
    }

    // 画出最终得到的路径
    vector<node*> path;
    path.push_back(goalNode);
    node* tmpNode = nodeList.back();
    while (tmpNode->getParent() != nullptr) {
        line(background,
            Point(static_cast<int>(tmpNode->getX() * imageResolution), static_cast<int>(tmpNode->getY() * imageResolution)),
            Point(static_cast<int>(tmpNode->getParent()->getX() * imageResolution), static_cast<int>(tmpNode->getParent()->getY() * imageResolution)),
            Scalar(255, 0, 255), 10);
        path.push_back(tmpNode);
        tmpNode = tmpNode->getParent();
    }

    // 展示背景
    imshow("RRT", background);
    waitKey(0);
    path.push_back(startNode);
    return path;
}
