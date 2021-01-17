#ifndef DFS_H
#define DFS_H

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
class DFS {
public:
    DFS();
    ~DFS();
    void util(int, vector<bool>&, vector<int>&);
    stack<int> findPath(int, int, int, vector<pair<int, double>>[]);
    stack<int> reconstructPath(vector<int> came_from, int current, int);
private:
    int init;
    int goal;
    vector<pair<int, double>> *graph;
};
#endif