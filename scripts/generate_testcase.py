import json
import math
import os
import random
import subprocess
import sys

def write_json(path, data):
    # ensure folder exists
    folder = os.path.dirname(path)
    if folder and not os.path.exists(folder):
        os.makedirs(folder)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

def make_speed_profile(slots=24):
    # slower in the middle of the day, faster near the edges
    vals = []
    for i in range(slots):
        t = abs(i - (slots / 2.0)) / (slots / 2.0)
        vals.append(round(0.6 + 0.4 * (1.0 - t), 3))
    return vals

def make_graph(n=10, seed=17, extra_prob=0.4):
    random.seed(seed)

    # build nodes on a small grid
    nodes = []
    for i in range(n):
        lat = 17.0 + int(i / 5) * 0.001 + random.uniform(-0.00015, 0.00015)
        lon = 78.0 + (i % 5) * 0.001 + random.uniform(-0.00015, 0.00015)
        pois = []
        if i % 4 == 0:
            pois.append("food")
        if i % 6 == 0:
            pois.append("atm")
        nodes.append({"id": i, "lat": lat, "lon": lon, "pois": pois})

    def dist(u, v):
        dx = nodes[u]["lat"] - nodes[v]["lat"]
        dy = nodes[u]["lon"] - nodes[v]["lon"]
        return math.sqrt(dx * dx + dy * dy) * 111000.0

    edges = []
    road_types = ["residential", "primary", "secondary", "service"]
    sp = make_speed_profile()
    eid = 0

    # backbone (chain) so graph is connected
    for u in range(n - 1):
        v = u + 1
        length = dist(u, v)
        avg_t = max(1.0, length / 8.0)
        edges.append({
            "id": eid, "u": u, "v": v,
            "length": round(length, 2),
            "average_time": round(avg_t, 2),
            "oneway": False,
            "road_type": random.choice(road_types),
            "speed_profiles": sp
        })
        eid += 1

    # some extra edges for alternative paths
    for u in range(n):
        for v in range(u + 2, n):
            if random.random() < extra_prob:
                length = dist(u, v)
                avg_t = max(1.0, length / 8.0)
                edges.append({
                    "id": eid, "u": u, "v": v,
                    "length": round(length, 2),
                    "average_time": round(avg_t, 2),
                    "oneway": random.random() < 0.2,
                    "road_type": random.choice(road_types),
                    "speed_profiles": sp
                })
                eid += 1

    return {"nodes": nodes, "edges": edges}

def make_queries():
    # four basic events: distance, time, remove_edge, distance after update
    events = []

    events.append({
        "type": "shortest_path",
        "id": 0, "source": 0, "target": 6,
        "mode": "distance",
        "constraints": {
            "forbidden_nodes": [],
            "forbidden_roadtypes": []
        }
    })

    events.append({
        "type": "shortest_path",
        "id": 1, "source": 2, "target": 9,
        "mode": "time",
        "constraints": {
            "forbidden_nodes": [],
            "forbidden_roadtypes": ["service"]
        }
    })

    events.append({
        "type": "remove_edge",
        "id": 2, "edge_id": 0
    })

    events.append({
        "type": "shortest_path",
        "id": 3, "source": 0, "target": 6,
        "mode": "distance",
        "constraints": {
            "forbidden_nodes": [3],
            "forbidden_roadtypes": []
        }
    })

    return {"meta": {"id": "simple_smoke"}, "events": events}

def run_exe(exe, graph_path, queries_path, output_path, timeout=30):
    # runs: <exe> graph.json queries.json output.json
    cmd = [exe, graph_path, queries_path, output_path]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
    return proc.returncode, proc.stdout, proc.stderr

def check_output_shape(output_path):
    # basic checks so we know program didn’t break the contract
    with open(output_path, "r", encoding="utf-8") as f:
        out = json.load(f)

    if "meta" not in out:
        raise AssertionError("output missing meta")
    if "results" not in out or not isinstance(out["results"], list):
        raise AssertionError("output missing results list")

    for r in out["results"]:
        if not isinstance(r, dict):
            raise AssertionError("result is not an object")
        if "id" not in r:
            raise AssertionError("result missing id")
        if "error" in r:
            continue
        if "possible" not in r:
            raise AssertionError("result missing possible")
        if r["possible"] and "path" in r:
            if ("minimum_distance" not in r) and ("minimum_time" not in r):
                raise AssertionError("missing cost field for path result")

def main():
    # Usage: python3 generate_testcases.py <path_to_exe> [outdir]
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print("Usage: python3 generate_testcases.py <path_to_exe> [outdir]")
        sys.exit(1)

    exe = sys.argv[1]
    outdir = sys.argv[2] if len(sys.argv) == 3 else "testcases_auto"
    if not os.path.exists(outdir):
        os.makedirs(outdir)

    graph = make_graph(n=10, seed=17, extra_prob=0.4)
    queries = make_queries()

    graph_path = os.path.join(outdir, "graph_small.json")
    queries_path = os.path.join(outdir, "queries_smoke.json")
    output_path = os.path.join(outdir, "output_smoke.json")

    write_json(graph_path, graph)
    write_json(queries_path, queries)

    # run once and show program stdout/stderr to help while debugging
    print("[run]", exe, graph_path, queries_path, output_path)
    code, out, err = run_exe(exe, graph_path, queries_path, output_path)
    if out.strip():
        print("[stdout]"); print(out)
    if err.strip():
        print("[stderr]"); print(err)
    if code != 0:
        print("[fail] program exited with code", code)
        sys.exit(code)

    # quick contract check, not a correctness check
    check_output_shape(output_path)
    print("[ok] wrote", output_path)

if _name_ == "_main_":
    main()