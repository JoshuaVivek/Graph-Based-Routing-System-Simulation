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

    // overlap_threshold is given in percentage (20–80).
    std::vector<HeuristicPath> compute(int source,
                                       int target,
                                       int k,
                                       double overlap_threshold);

private:
    const Graph& g;

    // Build a few candidate paths around shortest path: e.g., perturb edges, penalties, etc.
    std::vector<HeuristicPath> generate_candidates(int source, int target, int k_base);

    // Compute overlap percentage between two paths (by edge set).
    double overlap_percent(const HeuristicPath& a,
                           const HeuristicPath& b) const;

    // Score a set of k paths according to penalty formula from PDF.
    double total_penalty(const std::vector<HeuristicPath>& paths,
                         double overlap_threshold) const;
};
