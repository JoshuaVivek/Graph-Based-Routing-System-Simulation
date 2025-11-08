#include "Phase-1/graph.h"
#include "Phase-1/shortestpathfinder.h"
#include <iostream>
using namespace std;

int main() {
    Graph g;
    g.load_from_json("testcases/manual_3nodes.json");
    
    Constraints c;
    c.forbidden_nodes.insert(1);  // Block node 1
    
    cout << "=== TEST 4: Path with Forbidden Node ===" << endl;
    Shortestpath result = shortestpath_by_distance(g, c, 0, 2);
    
    if(result.possible) {
        cout << "✅ Path found (avoiding node 1)!" << endl;
        cout << "Path: ";
        for(int node : result.Path) {
            cout << node << " ";
        }
        cout << endl;
    } else {
        cout << "❌ No path found (blocked)" << endl;
    }
    
    return 0;
}
