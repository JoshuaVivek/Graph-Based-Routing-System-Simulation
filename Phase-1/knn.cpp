#include<algorithm>
#include<queue>
#include "knn.h"

double calc_euclidean_dist(double lat1,double lat2,double lon1,double lon2)
{
    double answer;
    double x = lat2-lat1;
    double y = lon1-lon2;
    double x_sq = x*x;
    double y_sq = y*y;
    answer = sqrt(x_sq + y_sq);
    return answer;
}

vector<int> knn_by_euclidean (Graph& g, int k,double q_lat,double q_lon,string poi)
{
    vector<int> result;
    vector<int> nodes_same_poi = g.get_nodes_with_poi(poi);
    /*for(int i=0;i<nodes_with_poi.size();i++){
        Node node = g.all_Nodes[i];
        for(auto: p : node.pois){
            if(p == poi){
                nodes_same_poi.push_back(i);
                break;
            }
        }
    }*/
    priority_queue<pair<double,int>> pq;
    for(int i=0;i<nodes_same_poi.size();i++){
        int idx = nodes_same_poi[i];
        Node node = g.all_Nodes[idx];
        double dis = calc_euclidean_dist(q_lat,node.lat,q_lon,node.lon);
        if(pq.size()<k) {pq.push({dis,idx});}
        else {
            if (dis < pq.top().first){
                pq.pop();
                pq.push({dis,idx});
            }
        } 
    }
    while(!pq.empty()){
        result.push_back(g.all_Nodes[pq.top().second].id);
        pq.pop();
    }
    return result;
}

vector<int> knn_by_shortestpath(Graph& g,int k,double q_lat,double q_lon,string poi)
{   
    vector<int> answer;
    vector<int> nodes_same_poi = g.get_nodes_with_poi(poi);
    int node_idx = g.get_nearest_node(q_lat,q_lon);
    Node node = g.all_Nodes[node_idx];
    priority_queue<pair<double,int>> pq;
    for(int i=0;i<nodes_same_poi.size();i++){
        int target_id=g.all_Nodes[nodes_same_poi[i]].id;
        Shortestpath result;
        Constraints c;
        result = shortestpath_by_distance(g,c,node.id,target_id);
        double distance = result.min_dis_time;
        if(pq.size()<k) {pq.push({distance,target_id});}
        else {
            if (distance < pq.top().first){
                pq.pop();
                pq.push({distance,target_id});
            }
        } 
    }
    while(!pq.empty()){
        answer.push_back(pq.top().second);
        pq.pop();
    }
    return answer;
}