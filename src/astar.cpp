#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <inttypes.h>
#include <limits>

#include "astar.h"

using namespace std;

typedef priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> MIN_QUEUE;

ASTAR::ASTAR() {
    //visited_nodes.reserve(1);
    return;
}

stack<int> ASTAR::findPath(int init, int goal, int cnt, std::vector<Eigen::Vector3d> V, vector<pair<int, double>> graph[]) {
    MIN_QUEUE open_set;
    vector<bool> closed_set(cnt, false);
    vector<int> came_from(cnt);
    vector<double> g_score(cnt, numeric_limits<double>::max());
    vector<double> f_score(cnt, numeric_limits<double>::max());

    g_score[init] = 0;
    f_score[init] = g_score[init] + heuristic(V[init], V[goal]);
    open_set.push(make_pair(0, init));

    while (!open_set.empty()) {
        int current = open_set.top().second;
        open_set.pop();
        if (current == goal) {
            return reconstructPath(came_from, current, init);
        }
        for (std::vector<pair<int, double>>::iterator it = graph[current].begin() ; it != graph[current].end(); ++it) {
            pair<int, double> neighbor = *it;
            int neighbor_id = neighbor.first;
            double new_score = g_score[current] + neighbor.second;
            if (new_score < g_score[neighbor_id]) {
                came_from[neighbor_id] = current;
                g_score[neighbor_id] = new_score;
                f_score[neighbor_id] = g_score[neighbor_id] + heuristic(V[neighbor_id], V[goal]);
                if (closed_set[neighbor_id] == false) {
                    open_set.push(make_pair(f_score[neighbor_id], neighbor_id));
                    closed_set[neighbor_id] = true;
                }
            }
        }
    }
    return std::stack<int>();
}

double ASTAR::heuristic(Eigen::Vector3d a, Eigen::Vector3d b) {
    double dx, dy, dist = 0.0;
    dx = abs(a(0) - b(0));
    dy = abs(a(1) - b(1));
    dist = sqrt(pow(dx, 2) + pow(dy, 2));
    return dist;
}

stack<int> ASTAR::reconstructPath(vector<int> came_from, int current, int init) {
    stack<int> path;
    path.push(current);

    while (current != init) {
        current = came_from[current];
        path.push(current);
    }
    return path;
}