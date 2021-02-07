#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <stack>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
class ASTAR {
public:
    ASTAR();
    ~ASTAR();
    stack<int> findPath(int, int, int, Eigen::Vector3d[], vector<pair<int, double>>[]);
    double heuristic(Eigen::Vector3d a, Eigen::Vector3d b);
    stack<int> reconstructPath(vector<int> came_from, int current, int);
private:
};
#endif