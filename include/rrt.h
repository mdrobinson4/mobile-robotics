#ifndef RRT_H
#define RRT_H

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
    MAP *map;
    int init_node;
    int goal_node;
    int init_id;
    int goal_id;
    std::vector<Eigen::Vector3d> configs;
    vector<pair<int, double>> *graph;
public:
    RRT(cv::Mat, double, int, int);
    RRT(string, int, int);
    ~RRT();
    void drawMap();
    void clearCoords();
    RRT(cv::Mat input_image);
    void buildRRT(Eigen::Vector3d, Eigen::Vector3d, int, int);
    bool addPath(std::pair<int, Eigen::Vector3d>, std::pair<int, Eigen::Vector3d>);
    bool addNode(Eigen::Vector3d, int);
    void addEdge(int, int, double);
    void nearestNeighbor(Eigen::Vector3d, std::pair<int, Eigen::Vector3d>&, int&);
    Eigen::Vector3d randConfig();
    double computeDistance(Eigen::Vector3d, Eigen::Vector3d);
    Eigen::Vector3d newConfig(Eigen::Vector3d, Eigen::Vector3d, int);
    void search();
    int isDuplicateNode(Eigen::Vector3d);
/*
    Eigen::Vector3d randConfig();
    Eigen::Vector3d newConfig(Eigen::Vector3d, Eigen::Vector3d);
    int nearestNeighbor(Eigen::Vector3d);
    double computeDistance(Eigen::Vector3d, Eigen::Vector3d);
    void addNode(Eigen::Vector3d);
    void addEdge(int, int, double);
    int checkDuplicateEdge(Eigen::Vector3d);
*/
};

#endif