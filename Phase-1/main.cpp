#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>


#include "graph.h"
#include "shortestpathfinder.h"
#include "knn.h"
#include "../json.h"


using namespace std;
using json = nlohmann::json;


// take constraints from a query and fill our Constraints
static Constraints parse_constraints(const json& query) {
    Constraints constraints;


    auto constraints_it = query.find("constraints");
    if (constraints_it != query.end() && constraints_it->is_object()) {
        // forbidden nodes: list of node ids
        auto forbidden_nodes_it = constraints_it->find("forbidden_nodes");
        if (forbidden_nodes_it != constraints_it->end() && forbidden_nodes_it->is_array()) {
            for (const auto& x : *forbidden_nodes_it) constraints.forbidden_nodes.insert(x.get<int>());
        }


        // road types may come as forbidden_road_types or forbidden_roadtypes
        auto forbidden_road_types_it = constraints_it->find("forbidden_road_types");
        auto forbidden_roadtypes_it  = constraints_it->find("forbidden_roadtypes");
        if (forbidden_road_types_it != constraints_it->end() && forbidden_road_types_it->is_array()) {
            for (const auto& s : *forbidden_road_types_it) constraints.forbidden_roadtypes.insert(s.get<string>());
        } else if (forbidden_roadtypes_it != constraints_it->end() && forbidden_roadtypes_it->is_array()) {
            for (const auto& s : *forbidden_roadtypes_it) constraints.forbidden_roadtypes.insert(s.get<string>());
        }
    }


    return constraints;
}


int main(int argc, char* argv[]) {
    // ./app graph.json queries.json output.json
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>\n";
        return 1;
    }


    string graph_path   = argv[1];
    string queries_path = argv[2];
    string output_path  = argv[3];


    try {
        // 1) load graph once, build adjacency for fast lookups
        Graph graph;
        graph.load_from_json(graph_path);
        graph.make_adj_list();


        // 2) read input queries (meta + events)
        ifstream queries_in(queries_path);
        if (!queries_in.is_open()) {
            cerr << "Failed to open " << queries_path << "\n";
            return 1;
        }
        json queries_json; queries_in >> queries_json; queries_in.close();


        json meta_obj = queries_json.contains("meta") ? queries_json["meta"] : json::object();
        vector<json> results_list;


        // 3) walk through events and answer each
        auto events = queries_json.find("events");
        if (events != queries_json.end() && events->is_array()) {
            for (const auto& event : *events) {
                auto t_start = chrono::high_resolution_clock::now();
                json result;


                string type_str = event.value("type", string(""));


                if (type_str == "remove_edge") {
                    // remove one edge by id
                    int edge_id = event.value("edge_id", -1);
                    bool success = (edge_id != -1) ? graph.remove_edge(edge_id) : false;
                    result["id"] = event.value("id", -1);
                    result["done"] = success;


                } else if (type_str == "modify_edge") {
                    // simple placeholder wire to Graph::modify_edge if you added it
                    result["id"] = event.value("id", -1);
                    result["done"] = false;


                } else if (type_str == "shortest_path") {
                    // find path using distance (default) or time
                    int query_id = event.value("id", -1);
                    int source_id = event.value("source", -1);
                    int target_id = event.value("target", -1);


                    // accept either keys for mode/metric
                    string mode_str = event.value("mode", event.value("metric", string("distance")));


                    Constraints constraints = parse_constraints(event);


                    Shortestpath sp_result = (mode_str == "time")
                        ? shortestpath_by_time(graph, constraints, source_id, target_id)
                        : shortestpath_by_distance(graph, constraints, source_id, target_id);


                    result["id"] = query_id;
                    result["possible"] = sp_result.possible;
                    if (sp_result.possible) {
                        if (mode_str == "time") result["minimum_time"] = sp_result.min_dis_time;
                        else                    result["minimum_distance"] = sp_result.min_dis_time;
                        result["path"] = sp_result.Path; // list of node ids
                    }


                } else if (type_str == "knn") {
                    result["id"] = event.value("id", -1);
                    int k = event.value("k",1);
                    string poi = event.value("poi","");
                    string metric = event.value("metric", event.value("mode", string("euclidean")));
                    vector<int> nodes;
                    if(poi !=""){
                        auto query_point = event["query_point"];
                        double q_lat = query_point.value("lat",0.0);
                        double q_lon = query_point.value("lon",0.0);
                        if(metric == "shortest_path"){
                            nodes = knn_by_shortestpath(graph,k,q_lat,q_lon,poi);
                        }else {
                            nodes = knn_by_euclidean (graph,k,q_lat,q_lon,poi);
                        }
                    }
                    result["nodes"]= nodes;

                } else {
                    result["error"] = "Unknown query type";
                }


                // timing per query in ms
                auto t_end = chrono::high_resolution_clock::now();
                result["processing_time_ms"] =
                    chrono::duration_cast<chrono::milliseconds>(t_end - t_start).count();


                results_list.push_back(result);
            }
        }


        // 4) writing { meta, results } for the autograder / checker
        json output_json;
        output_json["meta"] = meta_obj;
        output_json["results"] = results_list;


        ofstream out_stream(output_path);
        if (!out_stream.is_open()) {
            cerr << "Failed to open " << output_path << " for writing\n";
            return 1;
        }
        out_stream << output_json.dump(2) << "\n";
        out_stream.close();


        return 0;
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}