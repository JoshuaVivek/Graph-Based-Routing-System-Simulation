#pragma once
#include "graph.h"
#include <vector>

struct Path {
    std::vector<int> nodes;  // node ids in order
    double length;           // total distance
};

class KShortestPathsExact {
public:
    explicit KShortestPathsExact(const Graph& g);

    // Compute k simple paths by distance, in sorted order (shortest first).
    // Assumes source != target and k >= 1.
    std::vector<Path> compute(int source, int target, int k);

private:
    const Graph& g;

    // Helper: run Dijkstra and also store parent to reconstruct a single path.
    Path shortest_path(int source, int target,
                       const std::vector<bool>& blocked_node,
                       const std::vector<bool>& blocked_edge) const;
};
