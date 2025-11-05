#ifndef SHORTESTPATHFINDER_H
#define SHORTESTPATHFINDER_H

#include "graph.h"
#include<vector>
#include<queue>

struct Shortestpath
{
    bool possible;
    double min_time;
    double min_distance;
    vector<int> Path;

    Shortestpath() : possible(false), min_time(0), min_distance(0) {}
};

struct Constraints
{
    vector<int> forbidden_nodes;
    vector<string> forbidden_roadtypes;
};

Shortestpath shortestpath_by_distance(Graph g,Constraints constraints,int source_id,int target_id);

Shortestpath shortestpath_by_time(Graph g,Constraints constraints,int source_id,int target_id);

#endif