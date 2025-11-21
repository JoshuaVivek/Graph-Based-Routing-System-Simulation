#include "shortestpathfinder.h"
#include<algorithm>
#include <queue>

Shortestpath shortestpath_by_distance(Graph& g, Constraints constraints,int source_id,int target_id)
{
    Shortestpath result;
    result.possible = false;
    
    // Step 1: Checking if source or target are in invalid nodes(in nodes they should not be present)
    /*for(int i = 0; i < constraints.forbidden_nodes.size(); i++)
    {
        if(constraints.forbidden_nodes[i] == source_id || constraints.forbidden_nodes[i] == target_id)
        {
            return result; // Path not possible
        }
    }*/
   if(constraints.forbidden_nodes.count(source_id)>0 || constraints.forbidden_nodes.count(target_id)>0){
    return result;
   }
    
    // Step 2: Finding indices of source and target nodes
    if(g.node_id_Nodes_index.find(source_id) == g.node_id_Nodes_index.end() ||g.node_id_Nodes_index.find(target_id) == g.node_id_Nodes_index.end()) {
    return result; // Node doesn't exist
    }

    int source_index = g.node_id_Nodes_index[source_id];
    int target_index = g.node_id_Nodes_index[target_id];


    // Step 3: Setup arrays for Dijkstra
    int n = g.all_Nodes.size();
    vector<double> dist(n, 1e18); // Distance to each node (start with infinity)
    vector<int> parent(n, -1); // Parent of each node in shortest path
    vector<bool> visited(n, false); // Track visited nodes
    dist[source_index] = 0.0;

    // Let us keep a Priority queue that stores {distance, node_index}
    // Smaller distance = higher priority
    priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> pq;
    pq.push({0, source_index});

// Step 4: Run Dijkstra algorithm
    while(!(pq.empty()))
    {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();
        
        
        if(visited[u]) continue; // Already processed this node
        visited[u] = true;
        
        // Step 5: Check all edges from current node u
        for(auto [v_index,i]:g.adj_list[u])
        {
            Edge edge = g.all_Edges[i];
            
            /*if(edge.is_removed) 
            {
                continue; // Skip removed edges
            }

            int u_id = g.all_Nodes[u].id;
            int v_index = -1;
            
            // Find which neighbor this edge leads to
            if(edge.u == u_id)
            {
                v_index = g.node_id_Nodes_index[edge.v];
            }
            else if(edge.v == u_id && !edge.oneway)
            {
                v_index = g.node_id_Nodes_index[edge.u]; // Bidirectional edge
            }
            else
            {
                continue; // Edge doesn't start from u
            }*/
            
            // Step 6: Check if neighbor is forbidden
            int v_id = g.all_Nodes[v_index].id;
            //bool is_forbidden = false;
            /*for(int j = 0; j < constraints.forbidden_nodes.size(); j++)
            {
                if(constraints.forbidden_nodes[j] == v_id)
                {
                    is_forbidden = true;
                    break;
                }
            }*/
           if(constraints.forbidden_nodes.count(v_id)>0){
                continue;
            }

            // Step 7: Check if road type is forbidden
            //bool road_forbidden = false;
            /*for(int j = 0; j < constraints.forbidden_roadtypes.size(); j++)
            {
                if(constraints.forbidden_roadtypes[j] == edge.road_type)
                {
                    road_forbidden = true;
                    break;
                }
            }*/
           if(constraints.forbidden_roadtypes.count(edge.road_type)>0){
                continue;
            }

            // Step 8: Relax the edge (update distance if shorter path found)
            double new_distance = dist[u] + edge.length;
            if(new_distance < dist[v_index])
            {
                dist[v_index] = new_distance;
                parent[v_index] = u;
                pq.push({new_distance, v_index});
            }
        }
    }
    
    // Step 9: Check if target is reachable
    if(dist[target_index] >= 1e18)
    {
        return result; // Target not reachable
    }
    
    // Step 10: Build the path from source to target
    vector<int> path;
    int current = target_index;
    while(current != -1)
    {
        path.push_back(g.all_Nodes[current].id);
        current = parent[current];
    }
    
    // Reverse path (we built it backwards)
    vector<int> final_path;
    for(int i = path.size() - 1; i >= 0; i--)
    {
        final_path.push_back(path[i]);
    }
    
    result.possible = true;
    result.min_dis_time = dist[target_index];
    result.Path = final_path;
    
    return result;    
}

Shortestpath shortestpath_by_time(Graph& g,Constraints constraints,int source_id,int target_id)
{
    Shortestpath result;
    result.possible = false;
   if(constraints.forbidden_nodes.count(source_id)>0 || constraints.forbidden_nodes.count(target_id)>0){
    return result;
   }

    // Step 2: Finding indices of source and target nodes
    if(g.node_id_Nodes_index.find(source_id) == g.node_id_Nodes_index.end() ||g.node_id_Nodes_index.find(target_id) == g.node_id_Nodes_index.end()) {
    return result; // Node doesn't exist
    }

    int source_index = g.node_id_Nodes_index[source_id];
    int target_index = g.node_id_Nodes_index[target_id];


    // Step 3: Setup arrays for Dijkstra
    int n = g.all_Nodes.size();
    vector<double> time_taken(n, 1e18); // Time to each node (start with infinity)
    vector<int> parent(n, -1); // Parent of each node in shortest path
    vector<bool> visited(n, false); // Track visited nodes
    time_taken[source_index] = 0.0;


    // Let us keep a Priority queue that stores {time, node_index}
    // Smaller time = higher priority
    priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> pq;
    pq.push({0, source_index});


// Step 4: Run Dijkstra algorithm
    while(!(pq.empty()))
    {
        double t = pq.top().first;
        int u = pq.top().second;
        pq.pop();
        
        if(visited[u])
        {
            continue; // Already processed this node
        }
        visited[u] = true;
        
        // Step 5: Checking all edges from current node u
        for(auto [v_index,i]: g.adj_list[u])
        {
            Edge edge = g.all_Edges[i];
            vector<double> sp = edge.speed_profiles;
            
            // Step 6: Check if neighbor is forbidden
            int v_id = g.all_Nodes[v_index].id;
            if(constraints.forbidden_nodes.count(v_id)>0){
                continue;
            }

            // Step 7: Check if road type is forbidden
            if(constraints.forbidden_roadtypes.count(edge.road_type)>0){
                continue;
            }

            // Step 8: Relax the edge (update time if shorter path found)
            double travel_time;
            if(sp.empty()){
                travel_time = edge.average_time;
            } else{
            //vector<double> times(96,0);
                double distance = edge.length;
                double current_time = time_taken[u]; // arrival time at current node
                double true_time = 0;

                while (distance > 0) {
                    int slot = int(current_time / 900) % 96;
                    double speed = sp[slot];

                    double time_in_slot = 900 - (int(current_time) % 900);
                    double dist_possible = speed * time_in_slot;

                    if (dist_possible >= distance) {
                    // Finishing in current slot
                        double time_needed = distance / speed;
                        true_time += time_needed;
                        current_time += time_needed;
                        break;
                    } else {
                    // Using whole slot, moving to the next
                    true_time += time_in_slot;
                    current_time += time_in_slot;
                    distance -= dist_possible;
                    }
                }
                travel_time = true_time;
        }


            double new_time = time_taken[u] + travel_time;
            if(new_time < time_taken[v_index])
            {
                time_taken[v_index] = new_time;
                parent[v_index] = u;
                pq.push({new_time, v_index});
            }
        }
    }
    
    // Step 9: Checking if target is reachable
    if(time_taken[target_index] >= 1e18)
    {
        return result; // Target not reachable
    }
    
    // Step 10: Build the path from source to target
    vector<int> path;
    int current = target_index;
    while(current != -1)
    {
        path.push_back(g.all_Nodes[current].id);
        current = parent[current];
    }
    
    // Reverse path (we built it backwards)
    vector<int> final_path;
    for(int i = path.size() - 1; i >= 0; i--)
    {
        final_path.push_back(path[i]);
    }
    
    result.possible = true;
    result.min_dis_time = time_taken[target_index];
    result.Path = final_path;
    
    return result;    
}



