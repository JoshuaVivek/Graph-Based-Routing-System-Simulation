#ifndef APPROX_H
#define APPROX_H

#include "graph.h"
#include <vector>
#include <chrono>
#include "../json.h" // Ensure this path points to your json.hpp

using namespace std;
using json = nlohmann::json;

class ApproxRouter {
private:
    Graph* graph;
    
    // Memory Optimization variables
    vector<double> g_score;
    vector<int> visited_token;
    int current_token;
    int num_nodes;

    double fast_euclidean_dist(int u_idx, int v_idx);
    
    // Core Solver Function
    double find_weighted_astar(int source_idx, int target_idx, double epsilon, 
                               chrono::high_resolution_clock::time_point start, 
                               double budget_ms);

public:
    ApproxRouter(Graph* g);

    // Validates the input JSON structure and produces the output JSON structure
    json solve_batch(const json& query_object);
};

#endif