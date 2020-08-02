#include "aStar.h"

node::node(int _x, int _y, double _sumCost, double _heuristic)
    : x(_x),
     y(_y),
     heuristic(_heuristic),
     sumCost(_sumCost),
     obstacle(false),
     backpoint(nullptr),
     isVisited(false)
{}

node::~node() {
    
}

void node::setObstacle() {
    obstacle = true;
}

bool node::isObstacle() {
    return obstacle;
}

void node::drawNode(Mat& background, int imgGridSize, const Scalar& color, int shift) {
    rectangle(background,
              Point(x * imgGridSize, y * imgGridSize),
              Point((x + 1) * imgGridSize, (y + 1) * imgGridSize),
              color, shift);
}

node& node::operator+=(node rhs) {
    x += rhs.x;
    y += rhs.y;
    sumCost += rhs.sumCost;
    return *this;
}

node& node::operator=(node rhs) {
    x = rhs.x;
    y = rhs.y;
    sumCost = rhs.sumCost;
    return *this;
}

bool node::operator==(const node& rhs) {
    return x == rhs.x && y == rhs.y;
}

int node::coordinateX() const {
    return x;
}

int node::coordinateY() const{
    return y;
}

void node::setBackpoint(node* rhs) {
    backpoint = rhs;
}

node* node::returnBackpoint() {
    return backpoint;
}

void node::setHeuristic(double _heuristic) {
    heuristic = _heuristic;
}

double node::returnHeuristic() {
    return heuristic;
}

double node::gN() {
    return heuristic + sumCost;
}

bool node::returnIsVisited() {
    return isVisited;
}

void node::setIsVisited() {
    isVisited = true;
}