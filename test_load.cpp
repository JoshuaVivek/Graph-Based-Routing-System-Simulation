#include "Phase-1/graph.h"
#include <iostream>
using namespace std;

int main() {
    Graph g;
    try {
        g.load_from_json("testcases/manual_3nodes.json");
        cout << "✅ load_from_json() works!" << endl;
    } catch (const exception& e) {
        cout << "❌ Error: " << e.what() << endl;
    }
    g.make_adj_list();
    return 0;
}
