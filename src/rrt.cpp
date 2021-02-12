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

RRT::RRT(cv::Mat input_image) {
    map = new MAP(input_image);
    init_id = 0;
    goal_id = 0;
}

void RRT::buildRRT(Eigen::Vector3d q_init, Eigen::Vector3d q_goal, int step_size, int node_capacity) {
    bool ret, found_path = false;
    graph = new std::vector<std::pair<int, double>>[node_capacity];

    // initialize graph with configuration
    ret = addNode(q_init, 0);
    if (ret == false) {
        std::cout << "initial node not added" << std::endl;
        return;
    }
    init_id = configs.size() - 1;

    int q_near_id, k, cnt = 0;
    Eigen::Vector3d q_rand, q_new;
    while (k < node_capacity && found_path == false) {
        std::pair<int, Eigen::Vector3d> q_near;
        if (cnt % 10 == 0) {
            // std::cout << "trying to connect to goal" << std::endl;
            nearestNeighbor(q_goal, q_near, q_near_id);
            ret = addPath(q_near, std::make_pair(configs.size(), q_goal));
            if (ret) {
                std::cout << "found early" << std::endl;
                drawMap();
                break;
            }
        }
        q_rand = randConfig();
        if (!map->isCollision(q_rand)) {
            nearestNeighbor(q_rand, q_near, q_near_id);
            q_new = newConfig(q_near.second, q_rand, step_size);
            std::cout << "1" << std::endl;
            ret = addPath(q_near, std::make_pair(configs.size(), q_new));
            std::cout << "0" << std::endl;
            if (ret) {
                std::cout << "before update map" << std::endl;
                map->updateMap(q_rand);
                std::cout << "before draw map" << std::endl;
                drawMap();
                std::cout << "after draw map" << std::endl;
                if (q_new(0) == q_goal(0) && q_new(1) == q_goal(1)) {
                    found_path = true;
                }
                k++;
            }
        }
        cnt++;
    }
    goal_id = configs.size() - 1;
}

bool RRT::addPath(std::pair<int, Eigen::Vector3d> q_near, std::pair<int, Eigen::Vector3d> q_new) {
    if (map->isCollision(q_near.second) || map->isCollision(q_new.second, q_near.second))
        return false;
    
    int idx = isDuplicateNode(q_new.second);
    double weight = computeDistance(q_new.second, q_near.second);
    if (idx > -1) {
        q_new.first = idx;
    }
    else {
        addNode(q_new.second, q_new.first);
    }
    addEdge(q_new.first, q_near.first, weight);
    return true;
}

bool RRT::addNode(Eigen::Vector3d q_new, int idx) {
    if (map->isCollision(q_new)) {
        return false;
    }

    configs.push_back(q_new);
    return true;
}

void RRT::addEdge(int u, int v, double weight) {
    graph[u].push_back(make_pair(v, weight));
    graph[v].push_back(make_pair(u, weight));
}

void RRT::nearestNeighbor(Eigen::Vector3d q_rand, std::pair<int, Eigen::Vector3d> &q_near, int &q_near_id) {
    double dist;
    MIN_QUEUE min_queue;
    int id;

    for (int i = 0; i < configs.size(); i++) {
        if (q_rand != configs[i]) {
            dist = computeDistance(q_rand, configs[i]);
            min_queue.push(make_pair(dist, i));
        }
    }
    
    id = min_queue.top().second;
    q_near = std::make_pair(id, configs[id]);
    return;
}

Eigen::Vector3d RRT::randConfig() {
    double x, y, theta = 0;
    Eigen::Vector3d q;
    x = rand() % map->getWidth();
    y = rand() % map->getHeight();
    theta = -1;
    q = Eigen::Vector3d(x, y, theta);
    return q;
}

double RRT::computeDistance(Eigen::Vector3d q0, Eigen::Vector3d q1) {
    double dx, dy, dist = 0.0;
    dx = abs(q0(0) - q1(0));
    dy = abs(q0(1) - q1(1));
    dist = sqrt(pow(dx, 2) + pow(dy, 2));
    return dist;
}

Eigen::Vector3d RRT::newConfig(Eigen::Vector3d q_near, Eigen::Vector3d q_rand, int step_size) {
    Eigen::Vector3d q_new;
    double x, y, dx, dy, theta = 0;
    /*
    dx = (q_rand(0)  - q_near(0));
    dy = (q_rand(1) - q_near(1));
    //dx = q_near(0)  - q_rand(0);
    //dy = q_near(1) - q_rand(1);
    std::cout << dx << " " << dy << std::endl;
    theta = atan(dx / dy);
    x = round((q_near(0) + step_size) * cos(theta));
    y = round((q_near(1) + step_size) * sin(theta));
    q_new = Eigen::Vector3d(x, y, theta);
    */
    x = round((q_rand(0) + q_near(0)) / step_size);
    y = round((q_rand(1) + q_near(1)) / step_size);
    //q_new = Eigen::Vector3d(q_near(0) + x, q_near(1) + y, theta);
    q_new = q_rand;
    return q_new;
} 

void RRT::search() {
    stack<int> path = astar->findPath(init_id, goal_id, configs.size(), configs, graph);
    if (path.empty())
        return;

    Eigen::Vector3d pnt1 = configs[path.top()];
    path.pop();
    
    while (!path.empty()) {
        Eigen::Vector3d pnt2 = configs[path.top()];
        path.pop();
        pnt1 = pnt2;
    }
    cv::waitKey(0);
}

void RRT::drawMap() {
    // loop through each of the nodes in the graph
    for (int i = 0; i < configs.size(); i++) {
        // loop through all of the edges
        for (auto edge : graph[i]) {
            int j = edge.first;
            Eigen::Vector3d u = configs[i];
            Eigen::Vector3d v = configs[j];
            map->updateMap(u, v);
        }
    }
}

void RRT::clearCoords() {
    map->clear();
}

RRT::~RRT() {
    delete graph;
    return;
}

int RRT::isDuplicateNode(Eigen::Vector3d pos) {
    for (int i = 0; i < configs.size(); i++) {
        if (pos == (configs[i])) {
            return i;
        }
    }
    return -1;
}

/*

void RRT::buildRRT(Eigen::Vector3d init, Eigen::Vector3d goal) {
    Eigen::Vector3d q_rand, q_new, q_near;
    init_node = 0;
    goal_node = 1;
    configs[node_cnt] = init;
    configs[node_cnt] = goal;
    node_cnt += 2;
    
    while (node_cnt < k && found_path == false) {
        q_rand = randConfig();
        if (!map->isCollision(q_rand)) {
            extend(q_rand);
        }
        cout << node_cnt << endl;
    }
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

int RRT::isDuplicateNode(Eigen::Vector3d pos) {
    for (int i = 0; i < node_cnt; i++) {
        if (pos == (configs[i])) {
            return i;
        }
    }
    return -1;
}
*/