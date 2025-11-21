#pragma once

#include <vector>
#include "graph.h"

struct HeuristicPath {
    std::vector<int> nodes;
    double length;
};

class KShortestPathsHeuristic {
public:
    explicit KShortestPathsHeuristic(const Graph& g);

    // overlap_threshold is given in percentage
    std::vector<HeuristicPath> compute(int source, int target, int k, double overlap_threshold);

private:
    const Graph& g;

    // Build a few candidate paths around shortest path
    std::vector<HeuristicPath> generate_candidates(int source, int target, int k_base);

    // Compute overlap percentage between two paths
    double overlap_percent(const HeuristicPath& a, const HeuristicPath& b) const;

    // Keep track of total penalty for a set of paths
    double total_penalty(const std::vector<HeuristicPath>& paths, double overlap_threshold) const;
};
