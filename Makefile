# Compiler
CXX = g++

# Common Flags (Warnings, C++ Version)
CXXFLAGS = -std=c++17 -O3

.PHONY: all clean phase1 phase2 phase3

all: phase1 phase2 phase3

#phase1
phase1:
	$(CXX) $(CXXFLAGS) Phase-1/main.cpp Phase-1/graph.cpp Phase-1/shortestpathfinder.cpp Phase-1/knn.cpp -o phase1

#phase2
phase2:
	$(CXX) $(CXXFLAGS) Phase-2/main.cpp Phase-2/graph.cpp Phase-2/kshortest.cpp Phase-2/kshortest_heuristic.cpp Phase-2/approx.cpp -o phase2

#phase3
#phase3:
#	$(CXX) $(CXXFLAGS_PHASE2) Phase-3/*.cpp -o phase3

clean:
	rm -f phase1 phase2 phase3