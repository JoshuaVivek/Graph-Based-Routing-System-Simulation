#ifndef KNN_H
#define KNN_H

#include"graph.h"
#include"shortestpathfinder.h"
#include<vector>

using namespace std;

vector<int> knn_by_euclidean (Graph& g, int k,double q_lat,double q_lon,string poi);
//To find the k nearest neighbors of a given type based on euclidean distance

vector<int> knn_by_shortestpath(Graph& g,int k,double q_lat,double q_lon,string poi);
//To find the k nearest neighbors of a given type based on shoretestpaths with respect to distance

#endif