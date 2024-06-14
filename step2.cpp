#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <limits>

using namespace std;
using namespace cv;

struct Edge {
    Point u, v;
    float weight;
    Edge(Point u, Point v, float weight) : u(u), v(v), weight(weight) {}
};

struct Graph {
    unordered_map<Point, vector<Edge>, hash<Point>> adjList;
};

float distance(const Point &a, const Point &b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

Graph buildGraph(const vector<pair<Point, Point>> &pointLineModel) {
    Graph graph;
    for (const auto &line : pointLineModel) {
        float dist = distance(line.first, line.second);
        graph.adjList[line.first].emplace_back(line.first, line.second, dist);
        graph.adjList[line.second].emplace_back(line.second, line.first, dist);
    }
    return graph;
}

vector<Graph> primMST(const Graph &graph) {
    unordered_set<Point, hash<Point>> visited;
    priority_queue<Edge, vector<Edge>, greater<>> pq;
    vector<Graph> trees;

    for (const auto &entry : graph.adjList) {
        if (visited.count(entry.first)) continue;

        Graph tree;
        visited.insert(entry.first);
        for (const auto &edge : entry.second) {
            pq.push(edge);
        }

        while (!pq.empty()) {
            Edge e = pq.top();
            pq.pop();
            if (visited.count(e.v)) continue;

            visited.insert(e.v);
            tree.adjList[e.u].push_back(e);
            tree.adjList[e.v].emplace_back(e.v, e.u, e.weight);

            for (const auto &nextEdge : graph.adjList[e.v]) {
                if (!visited.count(nextEdge.v)) {
                    pq.push(nextEdge);
                }
            }
        }
        trees.push_back(tree);
    }
    return trees;
}

pair<vector<Point>, int> dfsFindLongestPath(const Graph &tree, const Point &start, unordered_map<Point, bool, hash<Point>> &visited) {
    stack<pair<Point, int>> s;
    unordered_map<Point, int, hash<Point>> depths;
    s.push({start, 0});
    depths[start] = 0;

    while (!s.empty()) {
        auto [u, depth] = s.top();
        s.pop();
        visited[u] = true;

        for (const auto &edge : tree.adjList.at(u)) {
            if (!visited[edge.v]) {
                depths[edge.v] = depth + 1;
                s.push({edge.v, depth + 1});
            }
        }
    }

    auto maxIt = max_element(depths.begin(), depths.end(), [](const auto &a, const auto &b) { return a.second < b.second; });
    return {vector<Point>{maxIt->first}, maxIt->second};
}

pair<vector<Point>, int> findLongestRootPath(const Graph &tree) {
    Point start = tree.adjList.begin()->first;
    unordered_map<Point, bool, hash<Point>> visited;
    auto [farthest, _] = dfsFindLongestPath(tree, start, visited);

    visited.clear();
    auto [path, length] = dfsFindLongestPath(tree, farthest[0], visited);

    return {path, length};
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lychee_branch_structure");
    ros::NodeHandle nh;

    vector<pair<Point, Point>> pointLineModel = {
        {Point(0, 0), Point(1, 2)}, {Point(1, 2), Point(2, 3)}, {Point(2, 3), Point(3, 4)}
        // Populate this vector with your actual point-line model data
    };

    Graph graph = buildGraph(pointLineModel);
    vector<Graph> mstTrees = primMST(graph);

    vector<vector<Point>> longestPaths;
    for (const auto &tree : mstTrees) {
        auto [path, length] = findLongestRootPath(tree);
        longestPaths.push_back(path);
    }

    unordered_map<Point, int, hash<Point>> nodeFrequency;
    for (const auto &path : longestPaths) {
        for (const auto &node : path) {
            nodeFrequency[node]++;
        }
    }

    int minDegree = numeric_limits<int>::max();
    Point startNode;
    for (const auto &entry : nodeFrequency) {
        int degree = graph.adjList[entry.first].size();
        if (degree < minDegree) {
            minDegree = degree;
            startNode = entry.first;
        }
    }

    ROS_INFO("Starting point of the main branch is at (%d, %d)", startNode.x, startNode.y);

    ros::spin();
    return 0;
}
