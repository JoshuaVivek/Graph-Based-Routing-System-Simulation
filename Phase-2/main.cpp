#include "graph.h"
#include "kshortest.h"
#include "kshortest_heuristic.h"
#include "approx_shortest.h"
#include "json.h"
#include <fstream>
#include <chrono>

int main(int argc, char** argv) {
    // 1. Parse args: <graph.json> <queries.json> <output.json>
    // 2. Load graph via Graph::load_from_json
    // 3. Read queries.json into a JSON object
    // 4. For each event in "events":
    //      * start chrono timer
    //      * dispatch based on "type":
    //          - "k_shortest_paths" -> use KShortestPathsExact
    //          - "k_shortest_paths_heuristic" -> use KShortestPathsHeuristic
    //          - "approx_shortest_path" -> use ApproxShortestPaths
    //      * stop timer, attach "processing_time" (ms) field
    //      * push to output["results"]
    // 5. Write output JSON to <output.json>
}
