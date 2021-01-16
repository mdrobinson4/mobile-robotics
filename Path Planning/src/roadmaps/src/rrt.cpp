#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "rrt.h"

using namespace std;

RRT::RRT(cv::Mat input_image, int n_, int k_) {
    n = n_; // number of nodes to initialize
    k = k_; // number of neighbors
    
    width = input_image.cols - 1;
    height = input_image.rows - 1;
    map = new MAP(input_image);

    configs.reserve(k);
    graph.reserve(k);

    init_node = -1;
    init_goal = -1;
    node_cnt = 0;
}

void RRT::buildRRT(Eigen::Vector3d init, Eigen::Vector3d goal) {
    Eigen::Vector3d q_rand, q_new, q_near;
    init_node = 0;
    configs[init_node] = init;
    node_cnt = 1;
    
    while (node_cnt < k) {
        q_rand = randConfig();
        if (q_rand(0) != - 1 && q_rand(1) != -1 && q_rand(2) != -1) {
            extend(q_rand);
        }
    }
}

void RRT::extend(Eigen::Vector3d q_rand) {
    q_near = nearestNeighbor(q_rand);
    q_new = newConfig(q_near, q_rand);

    if (!map->isCollision(q_near, q_new)) {
        addNode(q_new);
        addEdge(q_near, q_new);
        node_cnt += 1;
    }
}

Eigen::Vector3d RRT::randConfig() {
    double x, y, theta = 0;
    Eigen::Vector3d q;

    x = rand() % width;
    y = rand() % height;
    theta = -1;

    q = Eigen::Vector(x, y, theta);
    if (map->isCollision(q) == false) {
        return q;
    } 
    else {
        return Eigen::Vector3d(-1, -1, -1);
    }
}

int RRT::nearestNeighbor(Eigen::Vector3d q) {
    double dist = 0.0;
    MIN_QUEUE min_queue;
    for (int i = 0; i < n; i++) {
        if (q != configs[i]) {
            dist = computeDistance(q, configs[i]);
            min_queue.push(make_pair(dist, i));
        }
    }
    return min_queue.top().second();
}

double RRT::computeDistance(Eigen::Vector3d q0 Eigen::Vector3d q1) {
    double dx, dy, dist = 0.0;
    dx = abs(q0(0) - q1(0));
    dy = abs(q0(1) - q1(1));
    dist = sqrt(pow(dx, 2) + pow(dy, 2));
    return dist;
}

Eigen::Vector3d newConfig(Eigen::Vector3d q_near, Eigen::Vector3d q_rand) {

}

void RRT::search() {
    //stack<int> path = dijkstra->findPath(init_idx, goal_idx, vector_cnt, graph);
    //stack<int> path = astar->findPath(init_idx, goal_idx, vector_cnt, V, graph);
    //stack<int> path = bfs->findPath(init_idx, goal_idx, vector_cnt, graph);
    stack<int> path = dfs->findPath(init_idx, goal_idx, vector_cnt, graph);
    if (path.empty())
        return;

    Eigen::Vector3d pnt1 = V[path.top()];
    path.pop();
    
    while (!path.empty()) {
        Eigen::Vector3d pnt2 = V[path.top()];
        map->updateMap(pnt1, pnt2);
        path.pop();
        pnt1 = pnt2;
    } 
    cv::waitKey(0);
}

void RRT::addEdge(int u_idx, int v_idx, double weight) {
    graph[u_idx].push_back(make_pair(v_idx, weight));
    graph[v_idx].push_back(make_pair(u_idx, weight));
}

int RRT::checkDuplicateEdge(Eigen::Vector3d pos) {
    for (int i = 0; i < node_cnt; i++) {
        if (pos == (V[i])) {
            return i;
        }
    }
    return -1;
}

bool RRT::checkDuplicateEdge(int u, int v) {
    const int X_v(u);
    const int X_u(v);

    if (graph[u].size() == 0 || graph[v].size() == 0) {
        return false;
    }

    auto it_u = std::find_if(graph[u].begin(), 
                        graph[u].end(), 
                        [&X_v](const pair<int, double>& p)
                        {return p.first == X_v; });

    auto it_v = std::find_if(graph[v].begin(), 
                        graph[v].end(), 
                        [&X_u](const pair<int, double>& p)
                        {return p.first == X_u; });

    if (it_u == graph[u].end() && it_v == graph[v].end()) {
        return false; // duplicate entry
    }
    else {
        return true; // new entry
    }
}

void RRT::drawMap() {
    for (int u_idx = 0; u_idx < vector_cnt; u_idx++) {
        for (auto p : graph[u_idx]) {
            int v_idx = p.first;
            Eigen::Vector3d u = V[u_idx];
            Eigen::Vector3d v = V[v_idx];
            //map->updateMap(u, v);
        }
    }
    if (init_idx >= 0) {
        map->updateMap(V[init_idx]);
        map->updateMap(V[goal_idx]);
    }
}

void RRT::clearCoords() {
    map->clear();
}

RRT::~RRT() {
    delete V;
    delete graph;
    return;
}