#include <limits>
#include <queue>
#include <algorithm>
#include <map>
#include <unordered_set>
#include <cmath>
#include "kshortest_heuristic.h"

// Define infinity distance (distance larger than any possible path)
constexpr double INFINITY_DISTANCE = std::numeric_limits<double>::infinity();

// Constructor 
KShortestPathsHeuristic::KShortestPathsHeuristic(const Graph& g) : g(g) {}


// This function (Extended Dijkstra) finds k_base shortest paths (may contain cycles)
std::vector<HeuristicPath> KShortestPathsHeuristic::generate_candidates(int source, int target, int k_base) {
    std::vector<HeuristicPath> candidates;
    if (g.node_id_Nodes_index.count(source) == 0 || g.node_id_Nodes_index.count(target) == 0) {
        return candidates;
    }
    
    // PathState holds the distance and the full path list to reconstruct the route
    struct PathState {
        double length;
        std::vector<int> nodes; 

        // Min-heap based on length
        bool operator>(const PathState& other) const {
            return length > other.length;
        }
    };

    // Priority Queue that holds candidate PathStates, ordered by smallest length
    std::priority_queue<PathState, std::vector<PathState>, std::greater<PathState>> pq;
    int num_nodes = g.all_Nodes.size();

    // k_found_at_node[i] counts how many paths have been found to node index i
    std::vector<int> k_found_at_node(num_nodes, 0);

    // Start with the source node
    PathState start_state;
    start_state.length = 0.0;
    start_state.nodes = {source};
    pq.push(start_state);

    // Main search loop
    while (!pq.empty() && candidates.size() < (size_t)k_base) {
        PathState current_path = pq.top();
        pq.pop();
        int u_id = current_path.nodes.back();
        
        // Ensure the node ID exists in the graph (should always be true here)
        if (g.node_id_Nodes_index.count(u_id) == 0) {
            continue; 
        }
        int u_idx = g.node_id_Nodes_index.at(u_id);

        // Count this path as found for node u
        k_found_at_node[u_idx]++;
        
        // If we found the target
        if (u_id == target) {
            HeuristicPath candidate_path;
            candidate_path.length = current_path.length;
            candidate_path.nodes = current_path.nodes;
            candidates.push_back(candidate_path);
            continue; 
        }

        // Check all outgoing edges from node u
        for (const auto& edge_pair : g.adj_list[u_idx]) {
            int v_idx = edge_pair.first;
            int edge_idx = edge_pair.second;
            
            //if edge is removed, continue
            if (g.all_Edges[edge_idx].is_removed) {
                continue;
            }
            // Check if the neighbor node is blocked/removed (not usually in KSP, but good practice)
            // Assuming no temporary blocking is needed for candidate generation
            const Edge& edge = g.all_Edges[edge_idx];
            double edge_weight = edge.length; // Use distance metric

            // Optimization: If we have found too many paths to neighbor v, skip it (optional heuristic)
            if (k_found_at_node[v_idx] >= k_base) {
                continue;
            }
            
            // Create the new path state for neighbor v
            PathState next_path;
            next_path.length = current_path.length + edge_weight;
            next_path.nodes = current_path.nodes;
            next_path.nodes.push_back(g.all_Nodes[v_idx].id);

            // Add the new candidate path to the priority queue
            pq.push(next_path);
        }
    }
    return candidates;
}

// Computing overlap percentage between two paths (by edge set)
double KShortestPathsHeuristic::overlap_percent(const HeuristicPath& a, const HeuristicPath& b) const {
    // A path with only one node (source == target) has no edges.
    if (a.nodes.size() < 2 || b.nodes.size() < 2) {
        return 0.0;
    }
    
    // Identify all edges in path A and store them in a set for quick lookup
    // An edge can be uniquely identified by the pair of node IDs (u, v)
    // Since graph is directed, (u,v) != (v,u)
    std::unordered_set<long long> edges_a;
    double shared_length = 0.0;
    for (size_t i = 0; i < a.nodes.size() - 1; ++i) {
        int u_id = a.nodes[i];
        int v_id = a.nodes[i+1];
        
        // Use a unique long long key for the edge (u_id, v_id)
        long long edge_key = (long long)u_id * g.all_Nodes.size() + v_id;
        edges_a.insert(edge_key);
    }

    // Iterating through edges in path B and find shared edges
    for (size_t i = 0; i < b.nodes.size() - 1; ++i) {
        int u_id = b.nodes[i];
        int v_id = b.nodes[i+1];
        long long edge_key = (long long)u_id * g.all_Nodes.size() + v_id;
        if (edges_a.count(edge_key)) {
            int u_idx = g.node_id_Nodes_index.at(u_id);
            int v_idx = g.node_id_Nodes_index.at(v_id);
            for(const auto& edge_pair : g.adj_list[u_idx]) {
                if(edge_pair.first == v_idx) {
                    shared_length += g.all_Edges[edge_pair.second].length;
                    break;
                }
            }
        }
    }

    // Calculating overlap: (Shared Length / Length of Shorter Path) * 100
    double min_length = std::min(a.length, b.length);
    
    // Safety check for division by zero (if one path is source=target with length 0)
    if (min_length < 1e-6) {
        return 0.0;
    }
    return (shared_length / min_length) * 100.0;
}

// Score a set of k paths according to a penalty formula
double KShortestPathsHeuristic::total_penalty(const std::vector<HeuristicPath>& paths, double overlap_threshold) const {
    if (paths.empty()) return
        INFINITY_DISTANCE;

    // A large constant penalty factor for violating the overlap constraint
    const double OVERLAP_PENALTY_FACTOR = 1000.0;
    double total_length_cost = 0.0;
    double total_diversity_penalty = 0.0;
    for (const auto& path : paths) {
        total_length_cost += path.length;
    }

    // Sum of length + sum of penalties for exceeding the overlap threshold
    for (size_t i = 0; i < paths.size(); ++i) {
        for (size_t j = i + 1; j < paths.size(); ++j) {
            double overlap = overlap_percent(paths[i], paths[j]);
            
            // Apply a penalty if the overlap exceeds the threshold
            if (overlap > overlap_threshold) {
                // The penalty increases with how much the threshold is exceeded
                total_diversity_penalty += OVERLAP_PENALTY_FACTOR * (overlap - overlap_threshold);
            }
        }
    }
    return total_length_cost + total_diversity_penalty;
}

// Compute K diverse paths using a Greedy Strategy
std::vector<HeuristicPath> KShortestPathsHeuristic::compute(int source, int target, int k, double overlap_threshold) {
    if (k <= 0) return {};
    if (source == target) return {
        HeuristicPath{{source}, 0.0}
    };

    // Generate a large set of candidate paths (k_base is an arbitrary factor of k)
    // The heuristic's success depends on having good, diverse candidates.
    const int K_BASE = 5 * k;
    std::vector<HeuristicPath> candidates = generate_candidates(source, target, K_BASE);

    // Filter out unreachable targets
    if (candidates.empty()) {
        return {};
    }
    std::vector<HeuristicPath> result_paths;
    
    // Sort candidates by length (shortest first)
    std::sort(candidates.begin(), candidates.end(), [](const HeuristicPath& a, const HeuristicPath& b) {
        return a.length < b.length;
    });

    // Selecting the single shortest path first (P1)
    result_paths.push_back(candidates[0]);

    // Removing P1 from candidates
    candidates.erase(candidates.begin());
    
    // Iteratively select the best path until K are found
    while (result_paths.size() < (size_t)k && !candidates.empty()) {
        double min_incremental_penalty = INFINITY_DISTANCE;
        int best_candidate_index = -1;
        
        // Iterating over all remaining candidates
        for (size_t i = 0; i < candidates.size(); ++i) {
            const HeuristicPath& candidate = candidates[i];
            
            // Calculate the penalty for adding this candidate to the current set
            double incremental_penalty = 0.0;
            
            // Length cost is the candidate's own length
            incremental_penalty += candidate.length;
            
            // Diversity penalty: sum of penalties for max overlap with existing paths
            // We penalize based on the MAX overlap to favor true diversity.
            double max_overlap = 0.0;
            for (const auto& existing_path : result_paths) {
                max_overlap = std::max(max_overlap, overlap_percent(candidate, existing_path));
            }

            // Apply penalty if the max overlap violates the threshold
            const double OVERLAP_PENALTY_FACTOR = 1000.0; // Same large factor as in total_penalty
            if (max_overlap > overlap_threshold) {
                 incremental_penalty += OVERLAP_PENALTY_FACTOR * (max_overlap - overlap_threshold);
            }

            // Checking if this candidate is the best choice so far
            if (incremental_penalty < min_incremental_penalty) {
                min_incremental_penalty = incremental_penalty;
                best_candidate_index = i;
            }
        }
        
        // Adding the best path to the result set
        if (best_candidate_index != -1) {
            result_paths.push_back(candidates[best_candidate_index]);

            // Removing the selected path from the candidates list
            candidates.erase(candidates.begin() + best_candidate_index);
        } else {
            break; 
        }
    }

    //The paths are returned in the order they were selected (shortest/most diverse first).
    return result_paths;
}