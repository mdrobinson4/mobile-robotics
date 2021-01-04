#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>

using namespace std;

struct Graph {
    std::unordered_map<char, std::vector<char>> edges;

    std::vector<char> neighbors(char id) {
        return edges[id];
    }
};

struct GridLocation {
    int x, y;
};

namespace std {
    template <> struct hash<GridLocation> {
        typedef GridLocation argument_type;
        typedef std::size_t result_type;
        std::size_t operator()(const GridLocation& id) const nexcept {
            return std::hash<int>()(id.x ^ (id.y << 4));
        }
    };
}

struct SquareGrid {
    static 
}

void bfs(Graph graph, char start) {
    std::queue<char> frontier;
    frontier.push(start);

    std::unordered_set<char> reached;
    reached.insert(start);

    while (!frontier.empty()) {
        char current = frontier.front();
        frontier.pop();

        std::cout << "Visiting " << current << '\n';
        for (char next : graph.neighbors(current)) {
            if (reached.find(next) == reached.end()) {
                frontier.push(next);
                reached.insert(next);
            }
        }
    }
}

int main() {
    Graph graph {{
        {'A', {'B'}},
        {'B', {'C'}},
        {'C', {'B', 'D', 'F'}},
        {'D', {'C', 'E'}},
        {'E', {'F'}},
        {'F', {}},
    }};
    std::cout << "Reachable from A: \n";
    bfs(graph, 'A');
    std::cout << "Reachable from E:\n";
    bfs(graph, 'E');
}