#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>

// 기본적으로 std::pair를 사전식으로 비교하기 위한 VectorComparator는 생략 가능
struct VectorComparator {
    bool operator()(const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) const {
        return lhs < rhs;
    }
};

class GraphPlanner {
public:
    std::vector<std::pair<int, int>> dijkstra(std::pair<int, int> start_node, std::pair<int, int> target,
                                              std::map<std::pair<int, int>, std::map<std::pair<int, int>, double>> edge_cost);

    // 그래프 엣지 정보를 저장할 변수
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> graph_edge;
    int graph_horizon = 5;
    int graph_vertical = 5;

private:
    std::map<std::pair<int, int>, double> node_cost;  // 각 노드의 최소 비용
};

std::vector<std::pair<int, int>> GraphPlanner::dijkstra(
        std::pair<int, int> start_node,
        std::pair<int, int> target,
        std::map<std::pair<int, int>, std::map<std::pair<int, int>, double>> edge_cost) {
    
    std::map<std::pair<int, int>, std::pair<int, int>, VectorComparator> previous;
    std::priority_queue<std::pair<double, std::pair<int, int>>, std::vector<std::pair<double, std::pair<int, int>>>, std::greater<std::pair<double, std::pair<int, int>>>> pq;
    
    // 노드 비용을 매우 큰 값으로 초기화
    for (int j = 0; j <= graph_horizon; j++) {
        for (int k = -graph_vertical; k <= graph_vertical; k++) {
            node_cost[{j, k}] = std::numeric_limits<double>::infinity();
        }
    }

    // 시작 노드 초기화
    node_cost[start_node] = 0;
    pq.push({0, start_node});

    while (!pq.empty()) {
        double current_distance = pq.top().first;
        std::pair<int, int> current_node = pq.top().second;
        pq.pop();

        if (current_distance > node_cost[current_node]) continue;

        // 인접 노드를 순회
        for (std::pair<int, int> next_node : graph_edge[current_node]) {
            double new_cost = current_distance + edge_cost[next_node][current_node];
            if (new_cost < node_cost[next_node]) {
                node_cost[next_node] = new_cost;
                previous[next_node] = current_node;
                pq.push({new_cost, next_node});
            }
        }
    }

    // 경로가 존재하지 않으면 빈 벡터 반환
    if (node_cost[target] == std::numeric_limits<double>::infinity()) {
        std::cout << "No path exists from node (" << start_node.first << ", " << start_node.second
                  << ") to node (" << target.first << ", " << target.second << ")" << std::endl;
        return {};
    }

    // 경로 역추적
    std::vector<std::pair<int, int>> path;
    for (std::pair<int, int> at = target; at != start_node; at = previous[at]) {
        path.push_back(at);
    }
    path.push_back(start_node);
    std::reverse(path.begin(), path.end());

    return path;
}

int main() {
    GraphPlanner planner;

    // 그래프 엣지 설정 (인접 노드 설정)
    planner.graph_edge[{-1, 0}] = {{0, 0}};
    planner.graph_edge[{0, 0}] = {{1, 0}, {0, 1}, {-1, 0}};
    planner.graph_edge[{1, 0}] = {{2, 0}, {1, 1}, {0, 0}};
    planner.graph_edge[{0, 1}] = {{0, 0}, {1, 1}};
    planner.graph_edge[{1, 1}] = {{1, 0}, {2, 1}, {0, 1}};
    planner.graph_edge[{2, 0}] = {{3, 0}, {1, 0}, {2, 1}};
    planner.graph_edge[{2, 1}] = {{2, 0}, {3, 1}};
    planner.graph_edge[{3, 0}] = {{4, 0}, {2, 0}};
    planner.graph_edge[{4, 0}] = {{3, 0}, {4, 1}};
    planner.graph_edge[{4, 1}] = {{4, 0}, {3, 1}};
    planner.graph_edge[{3, 1}] = {{3, 0}, {4, 1}};

    // 엣지 비용 설정 (각 경로의 비용 설정)
    std::map<std::pair<int, int>, std::map<std::pair<int, int>, double>> edge_cost;
    edge_cost[{-1, 0}][{0, 0}] = 1;
    edge_cost[{0, 0}][{-1, 0}] = 1;
    edge_cost[{0, 0}][{1, 0}] = 2;   // 비용 2
    edge_cost[{0, 0}][{0, 1}] = 3;   // 비용 3
    edge_cost[{1, 0}][{0, 0}] = 2;   // 비용 2
    edge_cost[{1, 0}][{2, 0}] = 1;   // 비용 1
    edge_cost[{1, 0}][{1, 1}] = 2;   // 비용 2
    edge_cost[{0, 1}][{0, 0}] = 3;   // 비용 3
    edge_cost[{0, 1}][{1, 1}] = 2;   // 비용 2
    edge_cost[{1, 1}][{1, 0}] = 2;
    edge_cost[{1, 1}][{2, 1}] = 3;
    edge_cost[{2, 0}][{1, 0}] = 1;
    edge_cost[{2, 0}][{3, 0}] = 4;
    edge_cost[{2, 1}][{2, 0}] = 1;
    edge_cost[{3, 0}][{4, 0}] = 1;
    edge_cost[{3, 0}][{2, 0}] = 4;
    edge_cost[{4, 0}][{3, 0}] = 1;

    // 다익스트라 알고리즘 실행
    std::pair<int, int> start_node = {-1, 0};
    std::pair<int, int> target_node = {4, 0};

    std::vector<std::pair<int, int>> path = planner.dijkstra(start_node, target_node, edge_cost);

    // 경로 출력
    if (!path.empty()) {
        std::cout << "Path found: ";
        for (const auto& node : path) {
            std::cout << "(" << node.first << ", " << node.second << ") ";
        }
        std::cout << std::endl;
    }

    return 0;
}
