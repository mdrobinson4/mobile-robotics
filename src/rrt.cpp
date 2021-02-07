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

RRT::RRT(cv::Mat input_image, double step_size_, int n_, int k_) {
    n = n_; // number of nodes to initialize
    k = k_; // number of neighbors
    step_size = step_size_;
    
    width = input_image.cols - 1;
    height = input_image.rows - 1;
    map = new MAP(input_image);

    configs = new Eigen::Vector3d[n];
    graph = new vector<pair<int, double>>[n];

    found_path = false;

    init_node = -1;
    goal_node = -1;
    node_cnt = 0;
}

void RRT::buildRRT(Eigen::Vector3d init, Eigen::Vector3d goal) {
    Eigen::Vector3d q_rand, q_new, q_near;
    init_node = 0;
    goal_node = 1;
    configs[node_cnt] = init;
    node_cnt += 1;
    configs[node_cnt] = goal;
    node_cnt += 1;
    
    while (node_cnt < k && found_path == false) {
        q_rand = randConfig();
        if (!map->isCollision(q_rand)) {
            extend(q_rand);
        }
        cout << node_cnt << endl;
    }
    cout << found_path;
    return;
}

bool RRT::extend(Eigen::Vector3d q_rand) {
    int u, v = 0;
    double dist = 0;
    u = nearestNeighbor(q_rand);
    Eigen::Vector3d q_near = configs[u];
    Eigen::Vector3d q_new = newConfig(q_near, q_rand);

    if (!map->isCollision(q_near, q_new)) {
        addNode(q_new);
        dist = computeDistance(q_new, q_near);
        addEdge(u, node_cnt, dist);
        if (u == goal_node) {
            found_path = true;
        }
        return true;
    }
    return false;
}

Eigen::Vector3d RRT::randConfig() {
    double x, y, theta = 0;
    Eigen::Vector3d q;
    x = rand() % width;
    y = rand() % height;
    theta = -1;

    q = Eigen::Vector3d(x, y, theta);
    return q;
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
    return min_queue.top().second;
}

double RRT::computeDistance(Eigen::Vector3d q0, Eigen::Vector3d q1) {
    double dx, dy, dist = 0.0;
    dx = abs(q0(0) - q1(0));
    dy = abs(q0(1) - q1(1));
    dist = sqrt(pow(dx, 2) + pow(dy, 2));
    return dist;
}

Eigen::Vector3d RRT::newConfig(Eigen::Vector3d q_near, Eigen::Vector3d q_rand) {
    Eigen::Vector3d q_new;
    double x, y, dx, dy, theta = 0;
    
    dx = q_rand(0)  - q_near(0);
    dy = q_rand(1) - q_near(1);
    theta = atan(dx / dy);
    x = round(q_near(0) + step_size * cos(theta));
    y = round(q_near(1) + step_size * sin(theta));
    q_new = Eigen::Vector3d(x, y, theta);

    return q_new;
} 

void RRT::search() {
    stack<int> path = astar->findPath(init_node, goal_node, node_cnt, configs, graph);
    if (path.empty())
        return;

    Eigen::Vector3d pnt1 = configs[path.top()];
    path.pop();
    
    while (!path.empty()) {
        Eigen::Vector3d pnt2 = configs[path.top()];
        map->updateMap(pnt1, pnt2);
        path.pop();
        pnt1 = pnt2;
    }
    cv::waitKey(0);
}

void RRT::addNode(Eigen::Vector3d q_new) {
    configs[node_cnt] = q_new;
    node_cnt += 1;
    return;
}

void RRT::addEdge(int u, int v, double dist) {
    graph[u].push_back(make_pair(v, dist));
    graph[v].push_back(make_pair(u, dist));
}

int RRT::checkDuplicateEdge(Eigen::Vector3d pos) {
    for (int i = 0; i < node_cnt; i++) {
        if (pos == (configs[i])) {
            return i;
        }
    }
    return -1;
}

void RRT::drawMap() {
    for (int i = 0; i < node_cnt; i++) {
        for (auto p : graph[i]) {
            int j = p.first;
            Eigen::Vector3d u = configs[i];
            Eigen::Vector3d v = configs[j];
            map->updateMap(u, v);
        }
    }
 
    map->updateMap(configs[init_node]);
    map->updateMap(configs[goal_node]);
}

void RRT::clearCoords() {
    map->clear();
}

RRT::~RRT() {
    delete configs;
    delete graph;
    return;
}