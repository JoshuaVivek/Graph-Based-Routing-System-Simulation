#pragma once
#include <vector>
#include <string>

struct Edge {
    int id;
    int u, v;           // endpoints
    double length;      // meters
    double avg_time;    // seconds
    bool oneway;
    std::string road_type;
};

struct Node {
    int id;
    double lat, lon;
    std::vector<int> incident_edges; // indices into edges vector
};

class Graph {
public:
    bool load_from_json(const std::string& path);
    int node_count() const;
    int edge_count() const;

    const std::vector<Node>& nodes() const;
    const std::vector<Edge>& edges() const;

    // adjacency as (neighbor, edge_index)
    const std::vector<std::vector<std::pair<int,int>>>& adj() const;

    // plain Dijkstra distance-only (used in many Phase‑2 routines)
    std::vector<double> dijkstra_distances(int source) const;
};
