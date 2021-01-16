#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <utility>
#include <stack>

#include<opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "map.h"
#include "dijkstra.h"
#include "astar.h"
#include "bfs.h"
#include "dfs.h"

using namespace std;

typedef priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> MIN_QUEUE;

class RRT {
private:
    class Dijkstra *dijkstra;
    class ASTAR *astar;
    class BFS *bfs;
    class DFS *dfs;
    int n; // number of nodes to put in roadmap
    int k; // number of closest neighbors to examine for each configuration
    MAP *map;
    int width;
    int height;
    int init_idx;
    int goal_idx;
    int node_cnt;
    cv::Mat input_image;
    vector<Eigen::Vector3d> configs;
    vector<vector<pair<int, double>>> graph;
public:
    RRT(string, int, int);
    RRT(cv::Mat, int, int);
    ~RRT();
    void search();
    void clearCoords();
    void drawMap();
    void constructRoadmap(Eigen::Vector3d, Eigen::Vector3d);
private:
    int addNode(Eigen::Vector3d);
    void removeNode(int);
    void addEdge(int u_idx, int v_idx, double weight);
    void connectNodes(Eigen::Vector3d q_init, Eigen::Vector3d q_goal);
    MIN_QUEUE nearestNeighbors(Eigen::Vector3d q);
    bool checkDuplicateEdge(int u, int v);
    int checkDuplicateEdge(Eigen::Vector3d);
    bool bresenham(int x1, int y1, int x2, int y2);
};