#ifndef KSHORTEST_H
#define KSHORTEST_H

#include <vector>
#include <utility> 
#include "graph.h"

struct Path {
    std::vector<int> nodes;  // Vector of node IDs from source to target
    double length;           // The total distance of this path

    bool operator<(const Path& other) const {
        return length < other.length;
    }// In priority queue <double,path> , if double was equal, it needs to compare path, adding the comparator
};

class KShortestPathsExact {
public:
    // Constructor Function
    explicit KShortestPathsExact(const Graph& g);

    // Finds the K shortest simple paths using Yen's algorithm
    std::vector<Path> compute(int source, int target, int k);

private:
    const Graph& g;// Holds a reference to the main graph data structure

    // function to find the single shortest path using Dijkstra's algorithm
    Path shortest_path(int source, int target, const std::vector<bool>& blocked_node, const std::vector<bool>& blocked_edge) const;
};

#endif