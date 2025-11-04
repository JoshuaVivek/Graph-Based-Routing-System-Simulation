#include "Phase-1/graph.h"
#include <iostream>
using namespace std;

int main() {
    cout << "=== COMPREHENSIVE GRAPH TEST ===" << endl;
    
    Graph g;
    
    // Test 1: Load JSON
    cout << "\n[TEST 1] Loading graph..." << endl;
    try {
        g.load_from_json("testcases/manual_3nodes.json");
        cout << "✅ Graph loaded successfully!" << endl;
    } catch (const exception& e) {
        cout << "❌ Failed: " << e.what() << endl;
        return 1;
    }
    
    // Test 2: Verify adjacency list
    cout << "\n[TEST 2] Adjacency list:" << endl;
    for(int i = 0; i < g.adj_nodes.size(); i++) {
        cout << "Node " << i << " → ";
        for(int neighbor : g.adj_nodes[i]) {
            cout << neighbor << " ";
        }
        cout << endl;
    }
    
    // Test 3: Remove edge
    cout << "\n[TEST 3] Removing edge 0..." << endl;
    bool remove_result = g.remove_edge(0);
    cout << "Result: " << (remove_result ? "SUCCESS" : "FAILED") << endl;
    
    cout << "Adjacency list after removal:" << endl;
    for(int i = 0; i < g.adj_nodes.size(); i++) {
        cout << "Node " << i << " → ";
        for(int neighbor : g.adj_nodes[i]) {
            cout << neighbor << " ";
        }
        cout << endl;
    }
    
    // Test 4: Modify edge
    cout << "\n[TEST 4] Modifying edge 1..." << endl;
    json patch = {
        {"length", 250.0},
        {"road_type", "expressway"}
    };
    bool modify_result = g.modify_edge(1, patch);
    cout << "Result: " << (modify_result ? "SUCCESS" : "FAILED") << endl;
    
    // Test 5: Edge cases
    cout << "\n[TEST 5] Edge cases..." << endl;
    
    // Try to remove non-existent edge
    cout << "Remove edge 999: ";
    bool result1 = g.remove_edge(999);
    cout << (result1 ? "TRUE (unexpected)" : "FALSE (correct)") << endl;
    
    // Try to remove already removed edge
    cout << "Remove edge 0 again: ";
    bool result2 = g.remove_edge(0);
    cout << (result2 ? "TRUE (unexpected)" : "FALSE (correct)") << endl;
    
    // Try to modify with empty patch
    cout << "Modify edge 2 with empty patch: ";
    json empty_patch = {};
    bool result3 = g.modify_edge(2, empty_patch);
    cout << (result3 ? "TRUE (correct)" : "FALSE") << endl;
    
    cout << "\n=== ALL TESTS COMPLETE ===" << endl;
    return 0;
}
