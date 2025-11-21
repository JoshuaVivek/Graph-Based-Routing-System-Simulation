#include "approx.h"
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <iostream>

using namespace std;

// --- ContractionHierarchy Implementation ---

ContractionHierarchy::ContractionHierarchy(const Graph& g) : graph(g) {
    int n = graph.all_Nodes.size();
    node_rank.resize(n);
    is_contracted.resize(n, false);
    up_edges.resize(n);
    down_edges.resize(n);
}

// =========================================================================
// PHASE A: PRE-PROCESSING PLACEHOLDERS (COMPLEX CH IMPLEMENTATION)
// =========================================================================

// 1. DUMMY NODE RANKING (CUSTOM HEURISTIC REQUIRED)
void ContractionHierarchy::assign_node_rank() {
    // 🚨 IMPLEMENTATION REQUIRED: Replace this with a robust heuristic (e.g., Edge Difference)
    // For now, simple arbitrary ranking:
    int n = graph.all_Nodes.size();
    vector<NodeImportance> node_scores(n);
    for (size_t i = 0; i < n; ++i) {
        node_scores[i].node_index = i;
        node_scores[i].importance_score = graph.adj_list[i].size(); 
    }
    sort(node_scores.begin(), node_scores.end(), [](const NodeImportance& a, const NodeImportance& b) {
        return a.importance_score < b.importance_score;
    });
    for (size_t i = 0; i < n; ++i) {
        node_rank[node_scores[i].node_index] = i; 
    }
}

// 4. RESTRICTED NEIGHBORHOOD SEARCH (COMPLEX LOGIC REQUIRED)
double ContractionHierarchy::find_shortest_path_in_neighborhood(int u_index, int w_index, double max_dist) {
    // 🚨 IMPLEMENTATION REQUIRED: Specialized Dijkstra's to check if shortcut is necessary.
    return max_dist + 1.0; 
}

// 3. CONTRACT A SINGLE NODE (COMPLEX LOGIC REQUIRED)
void ContractionHierarchy::contract_node(int v_index) {
    is_contracted[v_index] = true;
    // 🚨 IMPLEMENTATION REQUIRED: Iterate neighbors, call find_shortest_path_in_neighborhood, and insert Shortcuts.
}

// 2. MAIN SHORTCUT BUILDER
void ContractionHierarchy::build_shortcuts() {
    vector<int> contraction_order(graph.all_Nodes.size());
    for(size_t i=0; i < graph.all_Nodes.size(); ++i) {
        contraction_order[node_rank[i]] = i;
    }
    for (int node_index : contraction_order) {
        contract_node(node_index);
    }
}

// --- Pre-processing Main Routine ---
void ContractionHierarchy::preprocess() {
    assign_node_rank();
    build_shortcuts();
}

// =========================================================================
// PHASE B: QUERYING AND APPROXIMATION LOGIC
// =========================================================================

// 5. EUCLIDEAN DISTANCE (FOR APPROXIMATION)
double ContractionHierarchy::calculate_euclidean_distance(int u_index, int v_index) {
    const Node& u_node = graph.all_Nodes[u_index];
    const Node& v_node = graph.all_Nodes[v_index];
    
    double dx = u_node.lon - v_node.lon;
    double dy = u_node.lat - v_node.lat;
    
    return sqrt(dx * dx + dy * dy) * 111000.0; // Approx meters
}

// CORE APPROXIMATE PATH FINDER
double ContractionHierarchy::find_approximate_distance(int source_id, int target_id, int max_visits) {
    int src_ix, tgt_ix;
    try {
        src_ix = graph.node_id_Nodes_index.at(source_id);
        tgt_ix = graph.node_id_Nodes_index.at(target_id);
    } catch (const out_of_range& e) {
        return numeric_limits<double>::infinity(); 
    }
    if (src_ix == tgt_ix) return 0.0;

    int n = graph.all_Nodes.size();
    vector<double> up_dist(n, numeric_limits<double>::infinity());
    vector<double> down_dist(n, numeric_limits<double>::infinity());

    priority_queue<P, vector<P>, greater<P>> upq, downq;
    unordered_set<int> seen_up, seen_down;
    
    upq.emplace(0.0, src_ix); up_dist[src_ix] = 0.0;
    downq.emplace(0.0, tgt_ix); down_dist[tgt_ix] = 0.0;

    double best_dist = numeric_limits<double>::infinity();
    int visits = 0;
    int last_up_node = src_ix; 
    int last_down_node = tgt_ix;

    while (!upq.empty() || !downq.empty()) {
        
        // 🚨 CRITICAL: APPROXIMATION EARLY TERMINATION (SPEED CONTROL) 🚨
        if (visits > max_visits) {
            // Geometric estimation for the remaining gap
            double est_dist = up_dist[last_up_node] + down_dist[last_down_node] + 
                              calculate_euclidean_distance(last_up_node, last_down_node);
            return min(best_dist, est_dist);
        }
        visits++;

        // --- CH Search Steps (Uses up_edges/down_edges) ---
        // (Full search logic for forward and backward expansions is implemented here)
        
        if (!upq.empty()) {
            auto [du, u] = upq.top(); upq.pop();
            last_up_node = u; 
            if(du > best_dist || seen_up.count(u)) continue;
            seen_up.insert(u);
            if (seen_down.count(u)) best_dist = min(best_dist, du + down_dist[u]); 
            for (const auto& shortcut : up_edges[u]) {
                if (up_dist[shortcut.target_node_index] > du + shortcut.weight) {
                    up_dist[shortcut.target_node_index] = du + shortcut.weight;
                    upq.emplace(up_dist[shortcut.target_node_index], shortcut.target_node_index);
                }
            }
        }
        // ... (Backward search logic similar to above, using down_edges) ...
        if (!downq.empty()) {
            auto [dv, v] = downq.top(); downq.pop();
            last_down_node = v; 
            if(dv > best_dist || seen_down.count(v)) continue;
            seen_down.insert(v);
            if (seen_up.count(v)) best_dist = min(best_dist, dv + up_dist[v]);
            for (const auto& shortcut : down_edges[v]) {
                if (down_dist[shortcut.target_node_index] > dv + shortcut.weight) {
                    down_dist[shortcut.target_node_index] = dv + shortcut.weight;
                    downq.emplace(down_dist[shortcut.target_node_index], shortcut.target_node_index);
                }
            }
        }
    }
    return best_dist; 
}


// HEURISTIC MAPPING (ERROR ADAPTATION)
int ContractionHierarchy::determine_max_visits(double acceptable_error_pct) {
    // 🚨 TUNING REQUIRED: These values must be calibrated based on your graph data
    // to ensure accuracy (error_pct) is met while maximizing speed.

    if (acceptable_error_pct >= 15.0) {
        return 300; // High error tolerated -> fastest search
    } else if (acceptable_error_pct >= 10.0) {
        return 500; // Moderate error
    } else { // 5.0%
        return 800; // Low error tolerance -> slower, more accurate search
    }
}


// BATCH QUERY HANDLER (TIME MANAGEMENT)
json ContractionHierarchy::process_approx_batch(const json& query_input_json) {
    
    // 1. Extract Constraints
    string query_id = query_input_json.contains("id") ? query_input_json["id"].get<string>() : "";
    long long time_budget_ms = query_input_json.contains("time_budget_ms") ? query_input_json["time_budget_ms"].get<long long>() : 0;
    double error_pct = query_input_json.contains("acceptable_error_pct") ? query_input_json["acceptable_error_pct"].get<double>() : 15.0;

    // 2. Determine Dynamic Search Limit
    int max_visits = determine_max_visits(error_pct);
    
    json output_results;
    output_results["id"] = query_id;
    output_results["distances"] = json::array();

    auto overall_start_time = std::chrono::high_resolution_clock::now();
    double total_time_ms = 0.0;

    // 3. Process Queries
    for (const auto& query : query_input_json["queries"]) {
        int source = query["source"];
        int target = query["target"];
        
        // 🚨 CRITICAL: Time Budget Check 🚨
        if (total_time_ms > time_budget_ms && time_budget_ms > 0) {
            break; // Stop processing the batch immediately
        }

        double approx_dist = find_approximate_distance(source, target, max_visits); 
        
        // Measure and accumulate time
        auto current_time = std::chrono::high_resolution_clock::now();
        total_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - overall_start_time).count();

        // Append result to output JSON
        output_results["distances"].push_back({
            {"source", source},
            {"target", target},
            {"approx_shortest_distance", approx_dist}
        });
    }

    return output_results;
}