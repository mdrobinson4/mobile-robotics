#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <inttypes.h>
#include <limits>

#include "bfs.h"

using namespace std;

BFS::BFS() {
    //visited_nodes.reserve(1);
    return;
}

stack<int> BFS::findPath(int init, int goal, int cnt, vector<pair<int, double>> graph[]) {
    queue<int> open_set;
    vector<bool> closed_set(cnt, false);
    vector<int> came_from(cnt);

    open_set.push(init);
    closed_set[init] = true;

    while (!open_set.empty()) {
        int current = open_set.front();
        open_set.pop();

        if (current == goal) {
            return reconstructPath(came_from, current, init);
        }

        for (std::vector<pair<int, double>>::iterator it = graph[current].begin() ; it != graph[current].end(); ++it) {
            pair<int, double> neighbor = *it;
            int neighbor_id = neighbor.first;

            if (closed_set[neighbor_id] == false) {
                open_set.push(neighbor_id);
                came_from[neighbor_id] = current;
                closed_set[neighbor_id] = true;
            }
        }
    }
}


stack<int> BFS::reconstructPath(vector<int> came_from, int current, int init) {
    stack<int> path;
    path.push(current);

    while (current != init) {
        current = came_from[current];
        path.push(current);
    }
    return path;
}