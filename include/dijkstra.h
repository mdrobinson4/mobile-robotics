#ifndef DIJKSTRA_H
#define DIJKSTRA_H

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
class Dijkstra {
public:
    stack<int> findPath(int, int, int, vector<pair<int, double>> b[]);
private:
};

#endif