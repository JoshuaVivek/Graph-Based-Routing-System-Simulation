#include "approx.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

using namespace std;

const double INF = 1e18;
const double PI_RAD = 0.01745329251; // PI / 180 precomputed

// Priority Queue Structure: <f_score, node_index>
typedef pair<double, int> PQElement;

ApproxRouter::ApproxRouter(Graph* g) {
    this->graph = g;
    this->num_nodes = g->all_Nodes.size();
    
    // Pre-allocate memory once to avoid reallocation during batch queries
    this->g_score.resize(num_nodes, INF);
    this->visited_token.resize(num_nodes, 0);
    this->current_token = 0;
}

double ApproxRouter::fast_euclidean_dist(int u_idx, int v_idx) {
    const Node& n1 = graph->all_Nodes[u_idx];
    const Node& n2 = graph->all_Nodes[v_idx];

    // Approximation: 1 degree lat ~ 111km (111000m)
    double dlat = (n1.lat - n2.lat) * 111000.0;
    double dlon = (n1.lon - n2.lon) * 111000.0 * cos(n1.lat * PI_RAD);
    
    return sqrt(dlat*dlat + dlon*dlon);
}

json ApproxRouter::solve_batch(const json& query_object) {
    // 1. Parse Input Keys strictly as per PDF/JSON Format
    int q_id = query_object["id"];
    double budget_ms = query_object["time_budget_ms"];
    double error_pct = query_object["acceptable_error_pct"];

    // 2. Prepare Output Object
    json output;
    output["id"] = q_id;
    output["distances"] = json::array();

    auto batch_start = chrono::high_resolution_clock::now();

    // Calculate Epsilon (Greedy Factor) based on allowed error
    // If error is 5%, epsilon ~ 1.25. If 10%, epsilon ~ 1.5.
    double epsilon = 1.0 + (error_pct / 20.0); 
    if (epsilon < 1.1) epsilon = 1.1; 
    if (epsilon > 2.5) epsilon = 2.5;

    // 3. Iterate over the "queries" array
    for (const auto& q : query_object["queries"]) {
        int src = q["source"];
        int tgt = q["target"];

        // Check time budget
        auto now = chrono::high_resolution_clock::now();
        double elapsed = chrono::duration<double, milli>(now - batch_start).count();
        if (elapsed >= budget_ms) break;

        // Validate IDs exist in graph
        if (graph->node_id_Nodes_index.find(src) == graph->node_id_Nodes_index.end() || 
            graph->node_id_Nodes_index.find(tgt) == graph->node_id_Nodes_index.end()) {
            continue;
        }

        int u_idx = graph->node_id_Nodes_index[src];
        int v_idx = graph->node_id_Nodes_index[tgt];

        double dist = find_weighted_astar(u_idx, v_idx, epsilon, batch_start, budget_ms);

        // 4. Format Output Object
        if (dist != -1.0) {
            output["distances"].push_back({
                {"source", src},
                {"target", tgt},
                {"approx_shortest_distance", dist}
            });
        }
    }
    return output;
}

double ApproxRouter::find_weighted_astar(int start_idx, int end_idx, double epsilon, 
                                         chrono::high_resolution_clock::time_point batch_start, 
                                         double budget_ms) {
    
    // Increment token to simulate "clearing" the visited array in O(1)
    current_token++;
    
    // Min-Priority Queue
    priority_queue<PQElement, vector<PQElement>, greater<PQElement>> pq;

    g_score[start_idx] = 0.0;
    visited_token[start_idx] = current_token;
    
    double h = fast_euclidean_dist(start_idx, end_idx);
    pq.push({0.0 + (epsilon * h), start_idx});

    int iterations = 0;

    while (!pq.empty()) {
        // Periodically check time budget inside the algorithm
        if (++iterations % 1000 == 0) {
             auto now = chrono::high_resolution_clock::now();
             if (chrono::duration<double, milli>(now - batch_start).count() > budget_ms) return -1.0;
        }

        PQElement top = pq.top();
        pq.pop();

        double f_current = top.first;
        int u = top.second;

        if (u == end_idx) return g_score[end_idx];

        // Lazy Pruning (Skip stale entries)
        double h_u = fast_euclidean_dist(u, end_idx);
        double g_at_push = f_current - (epsilon * h_u);
        if (g_score[u] < g_at_push - 0.001) continue;

        for (auto& edge_pair : graph->adj_list[u]) {
            int v = edge_pair.first;
            int edge_idx = edge_pair.second;
            const Edge& e = graph->all_Edges[edge_idx];

            if (e.is_removed) continue;

            double tentative_g = g_score[u] + e.length;

            if (visited_token[v] != current_token || tentative_g < g_score[v]) {
                g_score[v] = tentative_g;
                visited_token[v] = current_token;
                
                double h_v = fast_euclidean_dist(v, end_idx);
                pq.push({tentative_g + (epsilon * h_v), v});
            }
        }
    }

    return -1.0;
}