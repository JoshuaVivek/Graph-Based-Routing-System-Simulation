#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <iomanip>

#include "graph.h"
#include "kshortest.h"
#include "kshortest_heuristic.h"
#include "approx_shortest.h"
#include "../json.h" 

using namespace std;
using json = nlohmann::json;

// Reads a json file and returns the parsed json object
json read_json_file(const string &filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        throw runtime_error("Error: Could not open file " + filename);
    }
    
    json json_data;
    file >> json_data;
    return json_data;
}

// Converts a C++ vector of nodes to a json array
json path_to_json(const vector<int> &path_nodes) {
    json json_array = json::array();
    for (int node_id : path_nodes) {
        json_array.push_back(node_id);
    }
    return json_array;
}

int main(int argc, char **argv) {
    // We expect 3 arguments strictly: Graph File, Query File, Output File.
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << endl;
        return 1;
    }

    string graph_filename = argv[1];
    string query_filename = argv[2];
    string output_filename = argv[3];

    try {
        // Loading the Graph
        Graph G;
        cout << "Loading graph from: " << graph_filename << " ..." << endl;
        
        // Check if loading was successful
        if (!G.load_from_json(graph_filename)) {
            throw runtime_error("Failed to load graph from JSON.");
        }

        // Initializing our solvers
        // We create these once and reuse them for all queries to save setup time
        KShortestPathsExact exact_solver(G);
        KShortestPathsHeuristic heuristic_solver(G);
        ApproxShortestPaths approx_solver(G);

        // Reading all queries from the input file
        json query_data = read_json_file(query_filename);
        
        // Get the list of queries and meta info
        json meta_info = query_data.value("meta", json::object());
        const json &events = query_data.at("events");

        // Prepare the final output json structure
        json final_output;
        final_output["meta"] = meta_info;
        final_output["results"] = json::array();
        cout << "Processing " << events.size() << " queries..." << endl;

        // Processing each and every query
        for (const json &current_query : events) {
            json query_result;
            
            // Copy the ID so that we can track which result corresponds to which query
            query_result["id"] = current_query.value("id", -1);
            string type = current_query.value("type", "");

            // Starting the timer
            auto start_time = chrono::high_resolution_clock::now();

            // We will wrap individual queries in a try-catch block.
            // If one query fails, we will record the error
            // but we will keep the program running for the other queries.
            try {
                
                // K Shortest Paths (Exact)
                if (type == "k_shortest_paths") {
                    int src = current_query.at("source");
                    int tgt = current_query.at("target");
                    int k = current_query.at("k");

                    // Run the algorithm
                    vector<Path> results = exact_solver.compute(src, tgt, k);

                    // output formatting
                    json paths_array = json::array();
                    for (const auto &p : results) {
                        json path_obj;
                        path_obj["path"] = path_to_json(p.nodes);
                        path_obj["length"] = p.length;
                        paths_array.push_back(path_obj);
                    }
                    query_result["paths"] = paths_array;
                }

                // K Shortest Paths (Heuristic)
                else if (type == "k_shortest_paths_heuristic") {
                    int src = current_query.at("source");
                    int tgt = current_query.at("target");
                    int k = current_query.at("k");
                    double threshold = current_query.at("overlap_threshold");

                    // Run the algorithm
                    auto results = heuristic_solver.compute(src, tgt, k, threshold);

                    // Formatting the output
                    json paths_array = json::array();
                    for (const auto &p : results) {
                        json path_obj;
                        path_obj["path"] = path_to_json(p.nodes);
                        path_obj["length"] = p.length;
                        paths_array.push_back(path_obj);
                    }
                    query_result["paths"] = paths_array;
                }

                // Approximate Shortest Path (Batch)
                else if (type == "approx_shortest_path") {
                    long long time_budget = current_query.at("time_budget_ms").get<long long>();
                    double max_error = current_query.at("acceptable_error_pct");

                    // Collect all sub-queries for this batch
                    vector<ApproxQuery> batch_queries;
                    for (const auto &q : current_query.at("queries")) {
                        ApproxQuery aq;
                        aq.source = q.at("source");
                        aq.target = q.at("target");
                        batch_queries.push_back(aq);
                    }

                    // Running the batch solver
                    auto results = approx_solver.solve_batch(batch_queries, time_budget, max_error);

                    // Formatting output
                    json dist_array = json::array();
                    for (const auto &res : results) {
                        json dist_obj;
                        dist_obj["source"] = res.source;
                        dist_obj["target"] = res.target;
                        dist_obj["approx_shortest_distance"] = res.approx_distance;
                        dist_array.push_back(dist_obj);
                    }
                    query_result["distances"] = dist_array;
                }

                // When query type is unknown
                else {
                    query_result["error"] = "Unknown query type: " + type;
                }

            } 
            catch (const exception &e) {
                // Something went wrong with this specific query
                cerr << "Warning: Query ID " << query_result["id"] << " failed: " << e.what() << endl;
                query_result["error"] = string("Execution failed: ") + e.what();
            }

            // Stop the timer
            auto end_time = chrono::high_resolution_clock::now();
            auto duration_ms = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();

            // Save the timing info
            query_result["processing_time"] = duration_ms;
            
            // Add this result to the main list
            final_output["results"].push_back(query_result);
        }

        // Write Results to File
        ofstream output_file(output_filename);
        if (!output_file.is_open()) {
            throw runtime_error("Could not create output file: " + output_filename);
        }
        output_file << setw(2) << final_output << endl;
        output_file.close();        
        cout << "Done! Results written to " << output_filename << endl;
        return 0;

    } 
    catch (const exception &e) {
        // This catches global errors like file not found, JSON parse errors, etc.
        cerr << "Fatal Error: " << e.what() << endl;
        return 1;
    }
}