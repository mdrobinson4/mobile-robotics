#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <inttypes.h>
#include <limits>

#include "dfs.h"

using namespace std;

DFS::DFS() {
    //visited_nodes.reserve(1);
    return;
}

void DFS::util(int node, vector<bool> &visited, vector<int> &came_from) {
    if (node == goal) {
        cout << goal << endl;
        return;
    }
    //cout << node << endl; 

    for (vector<pair<int, double>>::iterator it = graph[node].begin(); it != graph[node].end(); ++it) {
        pair<int, double> neighbor = *it;
        int neighbor_id = neighbor.first;
        if (visited[neighbor_id] == false) {
            came_from[neighbor_id] = node;
            visited[neighbor_id] = true;
            util(neighbor_id, visited, came_from);
        }
    }
}

stack<int> DFS::findPath(int init_, int goal_, int cnt, vector<pair<int, double>> graph_[]) {
    graph = graph_;
    init = init_;
    goal = goal_;

    vector<bool> visited(cnt, false);
    vector<int> came_from(cnt);
    util(init, visited, came_from);
    return reconstructPath(came_from, goal, init);

}


stack<int> DFS::reconstructPath(vector<int> came_from, int current, int init) {
    stack<int> path;
    path.push(current);

    while (current != init) {
        current = came_from[current];
        path.push(current);
    }
    return path;
}