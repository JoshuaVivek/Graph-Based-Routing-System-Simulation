import json
import math
import os
import random
import sys
import textwrap
from urllib.parse import urlparse
import requests

# For Writing JSON data to file and creating directories as needed
def write_json(path, data):
    folder = os.path.dirname(path)
    if folder and not os.path.exists(folder):
        os.makedirs(folder, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

# Helping Function to compute distance between two lat/lon points
def sine_square_half(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

#For Generating a simple speed profile with higher speeds in the middle of the day
def make_speed_profile(slots=24):
    vals = []
    for i in range(slots):
        t = abs(i - (slots / 2.0)) / (slots / 2.0)
        vals.append(round(0.6 + 0.4 * (1.0 - t), 3))
    return vals

# IITB map view link of OpenStreetMap(Hard coded with lat, lon of IITB)
IITB_OSM_URL = "https://www.openstreetmap.org/#map=17/19.1320/72.9150"

# Public Overpass API endpoint
OVERPASS_URL = "https://overpass-api.de/api/interpreter"

# Parse the OSM URL to extract center lat/lon/zoom
def parse_osm_center(url):
    parsed = urlparse(url)
    frag = parsed.fragment
    lat = 19.1320
    lon = 72.9150
    zoom = 17

    if frag.startswith("map="):
        parts = frag.split("=")[1].split("/")
        if len(parts) >= 3:
            try:
                zoom = int(parts[0])
                lat = float(parts[1])
                lon = float(parts[2])
            except ValueError:
                # If parsing fails, just keep the default IITB center
                pass
    return lat, lon, zoom

# Convert center to a small bounding box around IITB
def center_to_bbox(lat, lon, zoom):
    dlat = 0.005
    dlon = 0.005 / max(0.1, math.cos(math.radians(lat)))
    min_lat = lat - dlat
    max_lat = lat + dlat
    min_lon = lon - dlon
    max_lon = lon + dlon
    return min_lat, min_lon, max_lat, max_lon

# Query Overpass for all highway-type ways in the bounding box
def fetch_osm_road_network(bbox):
    min_lat, min_lon, max_lat, max_lon = bbox
    query = textwrap.dedent(f"""[out:json][timeout:25];(way["highway"]({min_lat},{min_lon},{max_lat},{max_lon}););(._;>;);out body;""").strip()
    resp = requests.post(OVERPASS_URL, data={"data": query})
    resp.raise_for_status()
    return resp.json()

# Rough mapping from OSM highway tag to (our road_type, base speed in km/h)
ROAD_SPEED_TABLE = {
    "motorway": ("expressway", 70),
    "motorway_link": ("expressway", 60),
    "trunk": ("expressway", 60),
    "trunk_link": ("expressway", 50),
    "primary": ("primary", 50),
    "primary_link": ("primary", 40),
    "secondary": ("secondary", 40),
    "secondary_link": ("secondary", 35),
    "tertiary": ("tertiary", 35),
    "tertiary_link": ("tertiary", 30),
    "residential": ("local", 25),
    "service": ("local", 20),
    "unclassified": ("local", 20),
}


# Turn Overpass JSON into our graph.json format
def build_graph_from_overpass(osm_data):
    elements = osm_data.get("elements", [])
    node_elems = {}
    way_elems = []
    for i in elements:
        if i.get("type") == "node":
            node_elems[i["id"]] = i
        elif i.get("type") == "way":
            way_elems.append(i)
    graph_nodes = []
    osm_to_idx = {}
    
    # Map OSM node id -> compact 0..N-1 index in our nodes array
    def get_or_create_node(osm_id):
        if osm_id in osm_to_idx:
            return osm_to_idx[osm_id]
        n = node_elems.get(osm_id)
        if n is None:
            return None
        idx = len(graph_nodes)
        node_obj = {
            "id": idx,
            "lat": n.get("lat", 0.0),
            "lon": n.get("lon", 0.0),
            "pois": [],
        }
        graph_nodes.append(node_obj)
        osm_to_idx[osm_id] = idx
        return idx

    graph_edges = []
    next_eid = 0
    speed_prof = make_speed_profile(24)
    for w in way_elems:
        tags = w.get("tags", {})
        highway = tags.get("highway")
        if not highway:
            continue
        if highway not in ROAD_SPEED_TABLE:
            continue
        road_type, base_speed_kmh = ROAD_SPEED_TABLE[highway]
        oneway = tags.get("oneway") in ("yes", "true", "1")
        node_ids = w.get("nodes", [])
        if len(node_ids) < 2:
            continue
        for u_osm, v_osm in zip(node_ids[:-1], node_ids[1:]):
            u_idx = get_or_create_node(u_osm)
            v_idx = get_or_create_node(v_osm)
            if u_idx is None or v_idx is None:
                continue
            u = graph_nodes[u_idx]
            v = graph_nodes[v_idx]
            length_m = sine_square_half(u["lat"], u["lon"], v["lat"], v["lon"])
            if length_m <= 0:
                continue
            base_speed_ms = base_speed_kmh * 1000.0 / 3600.0
            factor = random.uniform(0.8, 1.2)
            eff_speed_ms = max(1.0, base_speed_ms * factor)
            avg_time = length_m / eff_speed_ms

            def add_edge(a_idx, b_idx):
                nonlocal next_eid
                edge = {
                    "id": next_eid,
                    "u": a_idx,
                    "v": b_idx,
                    "length": length_m,
                    "average_time": avg_time,
                    "speed_profiles": speed_prof,
                    "oneway": oneway,
                    "road_type": road_type,
                    "is_removed": False,
                }
                graph_edges.append(edge)
                next_eid += 1

            add_edge(u_idx, v_idx)
            if not oneway:
                add_edge(v_idx, u_idx)
    meta = {
        "id": "iitb_osm",
        "nodes": len(graph_nodes),
        "description": "IIT Bombay graph from OSM (roads only)",
    }
    return {
        "meta": meta,
        "nodes": graph_nodes,
        "edges": graph_edges,
    }


#Generate a small mix of shortest_path / KNN queries for Phase-1.
def make_queries(graph, num_queries=25, seed=42):
    random.seed(seed)
    nodes = graph["nodes"]
    edges = graph["edges"]
    events = []
    if not nodes or not edges:
        return {"meta": {"id": "empty"}, "events": []}
    node_ids = [n["id"] for n in nodes]
    road_types = list({e["road_type"] for e in edges})

    def rand_node():
        return random.choice(node_ids)

    qid = 0
    modes = ["distance", "time"]
    for _ in range(num_queries):
        r = random.random()
        if r < 0.4:
            s = rand_node()
            t = rand_node()
            while t == s:
                t = rand_node()
            constraints = {}
            if road_types and random.random() < 0.3:
                k = random.randint(1, min(2, len(road_types)))
                constraints["forbidden_road_types"] = random.sample(road_types, k)
            events.append({
                "type": "shortest_path",
                "id": qid,
                "source": s,
                "target": t,
                "mode": random.choice(modes),
                "constraints": constraints,
            })
        elif r < 0.7:
            base_node = nodes[rand_node()]
            events.append({
                "type": "knn_by_euclidean",
                "id": qid,
                "k": random.randint(1, 5),
                "q_lat": base_node["lat"],
                "q_lon": base_node["lon"],
                "poi": "Restaurant",
            })
        else:
            base_node = nodes[rand_node()]
            constraints = {}
            if road_types and random.random() < 0.3:
                kf = random.randint(1, min(2, len(road_types)))
                constraints["forbidden_road_types"] = random.sample(road_types, kf)
            events.append({
                "type": "knn_by_shortestpath",
                "id": qid,
                "k": random.randint(1, 5),
                "q_lat": base_node["lat"],
                "q_lon": base_node["lon"],
                "poi": "Restaurant",
                "constraints": constraints,
            })
        qid += 1
    return {
        "meta": {"id": "iitb_osm_queries"},
        "events": events,
    }


# we only produce graph.json and queries.json; we don't run phase1.
def main():
    if len(sys.argv) != 2:
        print("Usage: python3 generate_testcase.py <outdir>")
        sys.exit(1)
    outdir = sys.argv[1]

    # 1) Find IITB center and bounding box
    lat, lon, zoom = parse_osm_center(IITB_OSM_URL)
    bbox = center_to_bbox(lat, lon, zoom)

    # 2) Download OSM data
    print("Downloading IITB roads from Overpass...")
    osm_data = fetch_osm_road_network(bbox)

    # 3) Build graph.json
    print("Building graph.json ...")
    graph = build_graph_from_overpass(osm_data)
    graph_path = os.path.join(outdir, "graph.json")
    write_json(graph_path, graph)
    print(f"Graph: {len(graph['nodes'])} nodes, {len(graph['edges'])} edges")

    # 4) Build queries.json
    print("Building queries.json ...")
    queries = make_queries(graph, num_queries=25)
    queries_path = os.path.join(outdir, "queries.json")
    write_json(queries_path, queries)
    print(f"Queries: {len(queries['events'])} events")
    print("Done. Generated graph.json and queries.json only.")

if __name__ == "__main__":
    main()