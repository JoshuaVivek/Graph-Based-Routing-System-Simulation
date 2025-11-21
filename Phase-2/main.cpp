#include <bits/stdc++.h>
#include "../json.h"
#include "graph.h"
#include "kshortest.h"
#include "kshortest_heuristic.h"
#include "approx_shortest.h"

using namespace std;
using json = nlohmann::json;

// read a JSON file
json read_json(const std::string &path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Cannot open file: " + path);
    }
    json j;
    in >> j;
    return j;
}

// convert vector<int> to JSON array (for paths)
json vec_to_json(const std::vector<int> &v) {
    json a = json::array();
    for (int x : v) a.push_back(x);
    return a;
}

int main(int argc, char **argv) {
    // Only two arguments now: graph and queries
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <graph.json> <queries.json>\n";
        return 1;
    }
    std::string graph_file   = argv[1];
    std::string queries_file = argv[2];

    try {
        // 1) Load graph built from OSM
        json g_json = read_json(graph_file);
        Graph G;
        G.load_from_json(g_json);

        // 2) Prepare Phase‑2 algorithm objects
        KShortestPathsExact     exact_solver(G);
        KShortestPathsHeuristic heur_solver(G);
        ApproxShortestPaths     approx_solver(G);

        // 3) Load Phase‑2 queries
        json q_json = read_json(queries_file);
        json meta   = q_json.value("meta", json::object());
        const json &events = q_json.at("events");
        json out;
        out["meta"]    = meta;
        out["results"] = json::array();

        // 4) Answer each query one by one
        for (const json &ev : events) {
            json ans;
            ans["id"] = ev.value("id", -1);
            std::string type = ev.value("type", "");
            auto start = std::chrono::high_resolution_clock::now();
            if (type == "k_shortest_paths") {
                int s = ev.at("source");
                int t = ev.at("target");
                int k = ev.at("k");
                auto paths = exact_solver.compute(s, t, k);
                json arr = json::array();
                for (const auto &p : paths) {
                    json one;
                    one["path"]   = vec_to_json(p.nodes);
                    one["length"] = p.length;
                    arr.push_back(one);
                }
                ans["paths"] = arr;
            }
            else if (type == "k_shortest_paths_heuristic") {
                int s = ev.at("source");
                int t = ev.at("target");
                int k = ev.at("k");
                double thr = ev.at("overlap_threshold");
                auto paths = heur_solver.compute(s, t, k, thr);
                json arr = json::array();
                for (const auto &p : paths) {
                    json one;
                    one["path"]   = vec_to_json(p.nodes);
                    one["length"] = p.length;
                    arr.push_back(one);
                }
                ans["paths"] = arr;
            }
            else if (type == "approx_shortest_path") {
                long long budget_ms =
                    ev.at("time_budget_ms").get<long long>();
                double eps = ev.at("acceptable_error_pct");
                std::vector<ApproxQuery> qs;
                for (const auto &qj : ev.at("queries")) {
                    ApproxQuery q;
                    q.source = qj.at("source");
                    q.target = qj.at("target");
                    qs.push_back(q);
                }
                auto res = approx_solver.solve_batch(qs, budget_ms, eps);
                json arr = json::array();
                for (const auto &r : res) {
                    json one;
                    one["source"]                   = r.source;
                    one["target"]                   = r.target;
                    one["approx_shortest_distance"] = r.approx_distance;
                    arr.push_back(one);
                }
                ans["distances"] = arr;
            }
            else {
                ans["error"] = "unknown_query_type";
            }
            auto end = std::chrono::high_resolution_clock::now();
            auto ms  = std::chrono::duration_cast<
                std::chrono::milliseconds>(end - start).count();
            ans["processing_time"] = ms;
            out["results"].push_back(ans);
        }

        // 5) Print JSON result to stdout (no output.json file here)
        std::cout << std::setw(2) << out << "\n";
        return 0;
    }
    catch (const std::exception &e) {
        std::cerr << "Error in phase2: " << e.what() << "\n";
        return 1;
    }
}
