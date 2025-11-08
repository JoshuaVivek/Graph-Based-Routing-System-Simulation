#ifndef GRAPH_H
#define GRAPH_H

#include<vector>
#include<string>
#include<unordered_map>
#include "../json.h"

using json = nlohmann::json;
using namespace std;

struct Node
{
    int id;
    double lat,lon;
    vector<string> pois;

    //Constructor
    Node(int i,double l1,double l2,vector<string> p){
        id=i;
        lat=l1;
        lon=l2;
        pois=p;
    }
    Node() :id(-1),lat(0),lon(0) {}
};

struct Edge
{
    int id;
    int u; //source node
    int v; //destination node
    double length;
    double average_time;
    vector<double> speed_profiles;
    bool oneway;
    string road_type;
    bool is_removed;

    Edge() :id(-1),u(-1),v(-1),length(0),average_time(0),oneway(false),is_removed(false) {} 
    //speed_profile autoset to{} : road_type to ""
    Edge(int i,int n1,int n2,double l, double at,vector<double> sp,bool o,string r,bool b){
        id=i;
        u=n1;
        v=n2;
        length=l;
        average_time=at;
        speed_profiles=sp;
        oneway=o;
        road_type=r;
        is_removed=b;
    }
};

class Graph
{
    public:
    vector<Node> all_Nodes;
    vector<Edge> all_Edges;
    unordered_map<int,int> node_id_Nodes_index; //Easy to get index from node id
    unordered_map<int,int> edge_id_Edges_index; //Easy to get index from edge id
    


    vector<vector<int>> adj_nodes;  //adjacent nodes 
    vector<vector<pair<int,int>>> adj_list; //this contains both neighbour index , and the edge index in all_Edge vector

    Graph();

    void load_from_json(string file_name);
    //This reads the JSON file,parses it , and fill the graph with nodes and edges

    void make_adj_list();
    //This makes the adjacency list for each node

    //Dynamic Updates

    bool remove_edge(int edge_id);
    bool modify_edge(int edge_id, json& patch);
    //One for removing edge ,and modifying as Dynamic Updates

    //KNN Queries
    vector<int> get_nodes_with_poi(string s);
    int get_nearest_node(double l1,double l2);
};

#endif