#ifndef SHORTESTPATHFINDER_H
#define SHORTESTPATHFINDER_H

#include "graph.h"
#include<vector>
#include<queue>
#include<unordered_set>

struct Shortestpath
{
    bool possible;
    double min_dis_time;
    vector<int> Path;

    Shortestpath() : possible(false),min_dis_time(0) {}
};

struct Constraints
{
    unordered_set<int> forbidden_nodes;
    unordered_set<string> forbidden_roadtypes; // unordered_set is useful for finding a certain element presence in the set by counting its occurances
};

Shortestpath shortestpath_by_distance(Graph& g,Constraints constraints,int source_id,int target_id);

Shortestpath shortestpath_by_time(Graph& g,Constraints constraints,int source_id,int target_id);

//Shortestpath shortestpath_speedpf(Graph& g,Constraints constraints,int source_id,int target_id);

#endif