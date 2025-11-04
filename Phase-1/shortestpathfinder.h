#ifndef SHORTESTPATHFINDER_H
#define SHORTESTPATHFINDER_H

#include "graph.h"
#include<vector>
#include<queue>

struct Shortestpath
{
    bool possible;
    double min_time/min_dis;
    vector<int> Path;

    Shortestpath() : possible(false),min_time/min_dis(0) {}
};

struct Constraints
{
    vector<int> forbidden_nodes;
    vector<string> forbidden_roadtypes;
}

Shortestpath shortestpath_by_distance(Graph g,Constraints constraints,int source_id,int target_id);

Shortestpath shortestpath_by_time(Grapg g,Constraints constraints,int source_id,int target_id);

#endif