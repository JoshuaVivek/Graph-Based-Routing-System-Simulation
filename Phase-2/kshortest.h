#ifndef KSHORTEST_H
#define KSHORTEST_H

#include "graph.h"
#include <vector>
#include <utility> // For std::pair, used often in graph algorithms

// --- Data Structures ---

// A structure to hold one of the shortest paths found.
struct Path {
    std::vector<int> nodes;  // List of node IDs from source to target.
    double length;           // The total distance (or cost) of this path.
};

// --- Main Class for K-Shortest Paths (Exact) ---

class KShortestPathsExact {
public:
    // Constructor: Takes a reference to the main Graph object.
    explicit KShortestPathsExact(const Graph& g);

    // Main function: Finds the K shortest simple paths (e.g., using Yen's algorithm).
    // The paths are returned sorted by distance (shortest first).
    // It finds 'k' paths from 'source' node ID to 'target' node ID.
    std::vector<Path> compute(int source, int target, int k);

private:
    // Holds a reference to the main graph data.
    const Graph& g;

    // Helper function to find the single shortest path using Dijkstra's algorithm.
    // It needs to handle temporary blocked nodes or edges for Yen's algorithm.
    Path shortest_path(int source, int target,
                       const std::vector<bool>& blocked_node,
                       const std::vector<bool>& blocked_edge) const;
};

#endif