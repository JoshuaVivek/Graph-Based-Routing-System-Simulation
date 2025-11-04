#include "graph.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <algorithm>

Graph:: Graph() {}
//Constructor

void Graph::load_from_json(string file_name)
{
    ifstream file(file_name);
    if(!file.is_open()){
        cout<<"Error: File not found" << endl; 
    }
    json data;
    file >> data;

    //loading nodes from JSON file
    for(auto node:data["nodes"]){
        int id=node["id"];
        double lat=node["lat"];
        double lon=node["lon"];
        vector<string> pois;
        for(auto poiss:node["pois"]){
            pois.push_back(poiss);
        }
        Node n(id,lat,lon,pois);
        node_id_Nodes_index[id]=all_Nodes.size();
        all_Nodes.push_back(n);
    }

    //loading edges from JSON file
    for(auto edge:data["edges"]){
        int id=edge["id"];
        int u=edge["u"];
        int v=edge["v"];
        double length=edge["length"];
        double average_time=edge["average_time"];
        bool oneway=edge["oneway"];
        string road_type=edge["road_type"];
        vector<double>speed_profiles;
        for(auto sp:edge["speed_profiles"]){
            speed_profiles.push_back(sp);
        }
        Edge e(id,u,v,length,average_time,speed_profiles,oneway,road_type,false);
        edge_id_Edges_index[id]=all_Edges.size();
        all_Edges.push_back(e);
    }

    return;

}

void Graph::make_adj_list()
{
    adj_nodes.clear(); //clears previous data

    int size=all_Nodes.size();
    adj_nodes.resize(size);
    //initialising all empty vectors

    for(auto edge:all_Edges){
        if(edge.is_removed)continue;
        int id1=edge.u;
        int id2=edge.v;
        int n1=node_id_Nodes_index[id1];
        int n2=node_id_Nodes_index[id2];
        adj_nodes[n1].push_back(n2);
        if(!edge.oneway){
            adj_nodes[n2].push_back(n1);
        }
    }

    return;
}

bool Graph::remove_edge(int edge_id)
{
    if(edge_id_Edges_index.find(edge_id)==edge_id_Edges_index.end()){
        //edge id not found 
        return false;
    }

    int index=edge_id_Edges_index[edge_id];
    if(all_Edges[index].is_removed){
        return false;
        //edge was already removed , can't remove again
    }

    all_Edges[index].is_removed=true;
    //if exists and not already removed, return true and remove it

    make_adj_list();
    return true;
}


bool Graph::modify_edge(int edge_id,json& patch)
{
    if(edge_id_Edges_index.find(edge_id)==edge_id_Edges_index.end()){
        //edge id not found 
        return false;
    }

    int index=edge_id_Edges_index[edge_id];
    Edge edge=all_Edges[index];
    //Finding index by the edge id

    edge.is_removed=false;
    //If edge was removed and then udes for modify_edge, it was added back

    //Changing was was present in patch
    if(patch.contains("length")){
        edge.length = patch["length"];
    }
    if(patch.contains("oneway")){
        edge.oneway = patch["oneway"];
    }
    if(patch.contains("average_time")){
        edge.average_time=patch["average_time"];
    }
    if(patch.contains("speed_profiles")){
        edge.speed_profiles.clear();
        for(auto i:patch["speed_profiles"]){
            edge.speed_profiles.push_back(i);
        }
    }
    if(patch.contains("road_type")){
        edge.road_type=patch["road_type"];
    }

    make_adj_list();  //new edge may be added
    return true;
    
}