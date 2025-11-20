#pragma once
#include "graph.h"
#include <vector>

struct ApproxQuery {
    int source;
    int target;
};

struct ApproxResult {
    int source;
    int target;
    double approx_distance;
};

class ApproxShortestPaths {
public:
    explicit ApproxShortestPaths(const Graph& g);

    // time_budget_ms and acceptable_error_pct come from JSON.
    // Must obey time budget over the whole batch.
    std::vector<ApproxResult> solve_batch(
        const std::vector<ApproxQuery>& queries,
        long long time_budget_ms,
        double acceptable_error_pct
    );

private:
    const Graph& g;

    // Example strategies: landmark-based A*, multi-source BFS on spanner, sampled Dijkstra, etc.
    double approx_distance_single(int s, int t) const;
};
