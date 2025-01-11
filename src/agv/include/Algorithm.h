#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <cmath>
#include <algorithm>
#include <limits>

struct Node {
    int x, y;
    double f_score;
    Node(int x, int y, double f) : x(x), y(y), f_score(f) {}
    bool operator<(const Node& other) const {
        return f_score > other.f_score; // Min-heap için ters sıralama
    }
};

class AStar {
public:
    static double heuristic(const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return std::sqrt(std::pow(a.first - b.first, 2) + std::pow(a.second - b.second, 2));
    }

    static std::vector<std::pair<int, int>> astar(
        const std::vector<std::vector<int>>& grid,
        const std::pair<int, int>& start,
        const std::pair<int, int>& goal
    ) {
        static const std::vector<std::pair<int, int>> neighbors = {
            {0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        };

        std::set<std::pair<int, int>> closed_set;
        std::map<std::pair<int, int>, std::pair<int, int>> came_from;
        std::map<std::pair<int, int>, double> g_score;
        std::map<std::pair<int, int>, double> f_score;

        // Başlangıç g_score'u 0, diğerleri sonsuz olarak başlatılır.
        g_score[start] = 0.0;
        f_score[start] = heuristic(start, goal);
        for (int i = 0; i < grid.size(); ++i) {
            for (int j = 0; j < grid[0].size(); ++j) {
                if (std::make_pair(i,j) != start) {
                    g_score[{i, j}] = std::numeric_limits<double>::infinity();
                    f_score[{i, j}] = std::numeric_limits<double>::infinity();
                }
            }
        }


        std::priority_queue<Node> open_set;
        open_set.emplace(start.first, start.second, f_score[start]);

        while (!open_set.empty()) {
            Node current = open_set.top();
            open_set.pop();
            std::pair<int, int> current_pos = {current.x, current.y};

            if (current_pos == goal) {
                std::vector<std::pair<int, int>> path;
                while (came_from.find(current_pos) != came_from.end()) {
                    path.push_back(current_pos);
                    current_pos = came_from[current_pos];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }

            closed_set.insert(current_pos);

            for (const auto& [dx, dy] : neighbors) {
                std::pair<int, int> neighbor = {current_pos.first + dx, current_pos.second + dy};
                double tentative_g_score = g_score[current_pos] + heuristic(current_pos, neighbor);

                if (neighbor.first < 0 || neighbor.first >= grid.size() || neighbor.second < 0 || neighbor.second >= grid[0].size()) {
                    continue; // Sınırları kontrol et
                }

                if (grid[neighbor.first][neighbor.second] == 1) {
                    continue; // Engelli hücreler
                }

                // G_score'u kontrol etmeden önce g_score içinde var mı kontrolü eklenmeli.
                if (closed_set.find(neighbor) != closed_set.end() && (g_score.find(neighbor) != g_score.end() && tentative_g_score >= g_score[neighbor])) {
                    continue;
                }

                if (tentative_g_score < g_score[neighbor] || g_score.find(neighbor) == g_score.end()) {
                    came_from[neighbor] = current_pos;
                    g_score[neighbor] = tentative_g_score;
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal);
                    open_set.emplace(neighbor.first, neighbor.second, f_score[neighbor]);
                }
            }
        }

        // Hedefe yol bulunamadıysa en yakın düğümü geri döndür
        std::pair<int, int> closest_node;
        double closest_distance = std::numeric_limits<double>::infinity();
        for (const auto& node : closed_set) {
            double dist = heuristic(node, goal);
            if (dist < closest_distance) {
                closest_node = node;
                closest_distance = dist;
            }
        }

        if (closest_distance < std::numeric_limits<double>::infinity()) {
            std::vector<std::pair<int, int>> path;
            std::pair<int, int> temp = closest_node;
            while (came_from.find(temp) != came_from.end()) {
                path.push_back(temp);
                temp = came_from[temp];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        return {}; // Yol bulunamadı
    }
};