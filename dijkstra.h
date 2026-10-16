#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <algorithm>

using namespace std;

const int INF = 1e9; // Represents infinity for distances

// Function to run Dijkstra and print the shortest path for a single leg of the trip
inline void findShortestPath(int start, int target, const vector<vector<pair<int, int>>>& roadNetwork, const vector<string>& locationNames) {
    int n = roadNetwork.size();
    vector<int> dist(n, INF);
    vector<int> parent(n, -1);
    
    // Priority queue stores {distance, node}
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        int d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > dist[u]) continue;
        if (u == target) break; // Stop early if we reached our target customer

        for (auto& edge : roadNetwork[u]) {
            int v = edge.first;
            int weight = edge.second;

            // Relaxation step
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    // Check if customer is reachable
    if (dist[target] == INF) {
        cout << "   [X] No path found to " << locationNames[target] << "\n";
        return;
    }

    // Reconstruct the path backwards from target to start
    vector<int> path;
    int curr = target;
    while (curr != -1) {
        path.push_back(curr);
        curr = parent[curr];
    }
    reverse(path.begin(), path.end());

    // Print results for this leg
    cout << "   [OK] Destination Reached: " << locationNames[target] << "\n";
    cout << "   Distance for this leg: " << dist[target] << " km\n";
    cout << "   Route: ";
    for (size_t i = 0; i < path.size(); ++i) {
        cout << locationNames[path[i]];
        if (i + 1 < path.size()) cout << " -> ";
    }
    cout << "\n\n";
}

#endif
