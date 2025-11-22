#include <limits>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <map>
#include "kshortest.h"

// Defining a  infinite distance (distances that are large or unreachable)
constexpr double INFINITY_DISTANCE = std::numeric_limits<double>::infinity();

// Constructor Function
KShortestPathsExact::KShortestPathsExact(const Graph& g) : g(g) {}

// This function finds the simple shortest path respecting temporarily blocked nodes/edges.
Path KShortestPathsExact::shortest_path(int source_id, int target_id, const std::vector<bool>& blocked_node, const std::vector<bool>& blocked_edge_index) const {
    Path result;
    result.length = INFINITY_DISTANCE;
    
    // Check if source/target exist and get their internal array indices
    if (g.node_id_Nodes_index.count(source_id) == 0 || g.node_id_Nodes_index.count(target_id) == 0) {
        return result;
    }
    int source_idx = g.node_id_Nodes_index.at(source_id);
    int target_idx = g.node_id_Nodes_index.at(target_id);

    // If source or target itself is blocked, no path can start/end there.
    if (blocked_node[source_idx] || blocked_node[target_idx]) {
        return result;
    }
    int num_nodes = g.all_Nodes.size();

    // dist[i]: shortest distance from source to node index i found so far
    std::vector<double> dist(num_nodes, INFINITY_DISTANCE);

    // parent[i]: the index of the node just before node i in the shortest path
    std::vector<int> parent_idx(num_nodes, -1);
    
    // Priority Queue: holds (distance, node_index) pairs, sorting by smallest distance
    using DistIndexPair = std::pair<double, int>;
    std::priority_queue<DistIndexPair, std::vector<DistIndexPair>, std::greater<DistIndexPair>> pq;
    dist[source_idx] = 0.0;
    pq.push({0.0, source_idx});

    // Dijkstra's core loop
    while (!pq.empty()) {
        double current_dist = pq.top().first;
        int u_idx = pq.top().second;
        pq.pop();

        // If we found a better path to u_idx already, skip this stale entry.
        if (current_dist > dist[u_idx]) {
            continue;
        }

        // Found the path to the target, we can stop early.
        if (u_idx == target_idx) {
            break;
        }

        // Check all outgoing edges from node u
        for (const auto& edge_pair : g.adj_list[u_idx]) {
            int v_idx = edge_pair.first;
            int edge_idx = edge_pair.second;
            
            // Skip this neighbor node if it is blocked
            if (blocked_node[v_idx]) {
                continue;
            }

            // Skip this edge if it is blocked
            if (blocked_edge_index[edge_idx]) {
                continue;
            }

            //skip if edge is removed
            if (g.all_Edges[edge_idx].is_removed) {
                continue;
            }
            const Edge& edge = g.all_Edges[edge_idx];
            double edge_weight = edge.length; 

            // check if a shorter path to v is found through u
            if (dist[u_idx] + edge_weight < dist[v_idx]) {
                dist[v_idx] = dist[u_idx] + edge_weight;
                parent_idx[v_idx] = u_idx;
                pq.push({dist[v_idx], v_idx});
            }
        }
    }

    // Path Reconstruction
    if (dist[target_idx] == INFINITY_DISTANCE) {
        return result; // Target is unreachable
    }
    
    // Build the path by tracing back from target using the parent array
    std::vector<int> path_node_ids;
    int current_idx = target_idx;
    while (current_idx != -1) {
        path_node_ids.push_back(g.all_Nodes[current_idx].id);
        current_idx = parent_idx[current_idx];
    }
    
    // Reverse the path to go from source to target
    std::reverse(path_node_ids.begin(), path_node_ids.end());
    result.nodes = path_node_ids;
    result.length = dist[target_idx];
    return result;
}

// This Function uses Yen's algorithm to find the k shortest simple paths.
std::vector<Path> KShortestPathsExact::compute(int source, int target, int k) {
    // The list of k shortest paths found so far (the final result)
    std::vector<Path> A; 
    
    // A min-priority queue of candidate paths that could be the next shortest one.
    using PathCandidate = std::pair<double, Path>;
    std::priority_queue<PathCandidate, std::vector<PathCandidate>, std::greater<PathCandidate>> B;
    
    // Find the shortest path (A[0]) using standard Dijkstra.
    int num_nodes = g.all_Nodes.size();
    int num_edges = g.all_Edges.size();
    std::vector<bool> no_blocked_nodes(num_nodes, false);
    std::vector<bool> no_blocked_edges(num_edges, false);
    Path shortest = shortest_path(source, target, no_blocked_nodes, no_blocked_edges);
    if (shortest.length == INFINITY_DISTANCE) {
        return A; // No path exists.
    }
    A.push_back(shortest);

    // Main loop: Find the k-1 next shortest paths
    for (int i = 1; i < k; ++i) {
        // The path we found in the previous iteration (A[i-1])
        const Path& previous_path = A.back();
        
        // Loop over all nodes in the previous path, except the target
        for (size_t j = 0; j < previous_path.nodes.size() - 1; ++j) {
            int spur_node_id = previous_path.nodes[j];
            
            // The root path goes from the source to spur node
            // We need its length and list of node IDs.
            std::vector<int> root_path_nodes;
            double root_path_length = 0.0;
            
            // Accumulate root path nodes and length
            for (size_t node_index = 0; node_index <= j; ++node_index) {
                root_path_nodes.push_back(previous_path.nodes[node_index]);

                // Accumulate length up to the spur node (only count edges before spur node)
                if (node_index < j) {
                    int u_id = previous_path.nodes[node_index];
                    int v_id = previous_path.nodes[node_index+1];

                    // Find edge length between u_id and v_id
                    int u_idx = g.node_id_Nodes_index.at(u_id);
                    int v_idx = g.node_id_Nodes_index.at(v_id);
                    for(const auto& edge_pair : g.adj_list[u_idx]) {
                        if(edge_pair.first == v_idx) {
                            root_path_length += g.all_Edges[edge_pair.second].length;
                            break;
                        }
                    }
                }
            }
            
            // Block all nodes in the root path (except the spur node itself)
            std::vector<bool> blocked_nodes_temp(num_nodes, false);
            for (size_t node_index = 0; node_index < j; ++node_index) {
                int node_id = previous_path.nodes[node_index];
                blocked_nodes_temp[g.node_id_Nodes_index.at(node_id)] = true;
            }
            
            // Block the specific edge from the root path of A[i-1] leaving the spur node and 
            // block all edges leaving the spur node that are prefixes of any path in A that share the same root path
            std::vector<bool> blocked_edges_temp(num_edges, false);
            for (const auto& existing_path : A) {
                // Check if existing_path in A shares the same root prefix up to the spur node.
                if (existing_path.nodes.size() > j) {
                    bool same_root = true;
                    for (size_t node_index = 0; node_index <= j; ++node_index) {
                        if (existing_path.nodes[node_index] != previous_path.nodes[node_index]) {
                            same_root = false;
                            break;
                        }
                    }
                    
                    // If roots are the same, block the edge immediately following the spur node in the existing path.
                    if (same_root) {
                        int u_id = existing_path.nodes[j]; // spur node
                        int v_id = existing_path.nodes[j+1]; // next node
                        int u_idx = g.node_id_Nodes_index.at(u_id);
                        int v_idx = g.node_id_Nodes_index.at(v_id);
                        
                        // Find the index of the specific edge (u_idx -> v_idx) and block it.
                        for(const auto& edge_pair : g.adj_list[u_idx]) {
                            if(edge_pair.first == v_idx) {
                                blocked_edges_temp[edge_pair.second] = true;
                                break;
                            }
                        }
                    }
                }
            }
            
            // Find the shortest path from the spur node to the target in the temporarily modified graph.
            Path spur_path = shortest_path(spur_node_id, target, blocked_nodes_temp, blocked_edges_temp);
            if (spur_path.length != INFINITY_DISTANCE) {
                // Total path = Root path + Spur path
                Path candidate_path;
                candidate_path.nodes = root_path_nodes;
                
                // Add spur path nodes, skipping the first one (the spur node itself)
                for (size_t node_index = 1; node_index < spur_path.nodes.size(); ++node_index) {
                    candidate_path.nodes.push_back(spur_path.nodes[node_index]);
                }
                
                // Calculate total length
                candidate_path.length = root_path_length + spur_path.length;
                
                // Add the new candidate to the queue B
                B.push({candidate_path.length, candidate_path});
            }
        }

        // Find the best unique candidate from B and add it to A.
        while (!B.empty()) {
            Path best_candidate = B.top().second;
            B.pop();
            
            // Check if this path is already in the final set A (Yen's needs to filter duplicates)
            bool is_duplicate = false;
            for (const auto& existing_path : A) {
                if (existing_path.nodes == best_candidate.nodes) {
                    is_duplicate = true;
                    break;
                }
            }
            
            if (!is_duplicate) {
                A.push_back(best_candidate);
                break; // Found the next shortest path, move to the next iteration (i).
            }
        }
        
        // If B is empty and we couldn't find a new unique path, stop the algorithm.
        if (A.size() <= (size_t)i) {
            break; 
        }
    }

    return A;
}