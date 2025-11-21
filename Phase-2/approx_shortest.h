#ifndef APPROX_H
#define APPROX_H

#include "graph.h"
#include <vector>
#include <queue>
#include <limits>
#include <map>
#include <algorithm>
#include <unordered_set>
#include <chrono>

using json = nlohmann::json;
using namespace std;

// Pair for Dijkstra's Priority Queue: (distance, node_index)
using P = std::pair<double, int>; 

// --- Core Data Structure for Contraction Hierarchies ---
struct Shortcut {
    int target_node_index; // Index in Graph::all_Nodes
    double weight;
};

// --- Helper Structure for Node Ranking during Pre-processing ---
struct NodeImportance {
    int node_index;
    double importance_score; 

    bool operator>(const NodeImportance& other) const {
        return importance_score > other.importance_score;
    }
};

class ContractionHierarchy {
public:
    ContractionHierarchy(const Graph& g);

    // Main routine: builds the hierarchy, creates ranks and shortcuts
    void preprocess(); 

    /**
     * @brief Processes the entire batch query, managing time budget and error adaptation.
     * @param query_input_json The full JSON object for the approx_shortest_path query.
     * @return JSON object containing the computed distances.
     */
    json process_approx_batch(const json& query_input_json);

private:
    const Graph& graph; 

    // --- Contraction Hierarchy Data ---
    vector<int> node_rank; 
    vector<bool> is_contracted;
    vector<vector<Shortcut>> up_edges; 
    vector<vector<Shortcut>> down_edges; 

    // --- Internal Logic ---
    
    // Core Approximate Path Finder: Accepts max_visits dynamically
    double find_approximate_distance(int source_id, int target_id, int max_visits);

    // Heuristic Mapping: Converts required error % to an actual search limit (visits)
    int determine_max_visits(double acceptable_error_pct);
    
    // Pre-processing Implementation
    void assign_node_rank(); 
    void build_shortcuts(); 
    void contract_node(int v_index);
    double find_shortest_path_in_neighborhood(int u_index, int w_index, double max_dist);

    // Calculates Euclidean (straight-line) distance for approximation estimation
    double calculate_euclidean_distance(int u_index, int v_index);
};

#endif // APPROX_H