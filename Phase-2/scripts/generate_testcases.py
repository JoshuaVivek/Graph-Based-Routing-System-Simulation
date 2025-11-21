import json
import math
import os
import random
import sys
import requests
from typing import List, Dict, Any

# ----------------------------------------------------------------------------------
# CONFIGURATION
# ----------------------------------------------------------------------------------

# URL for a central point (e.g., IIT Bombay) to define the area for Overpass API
IITB_OSM_URL = "https://www.openstreetmap.org/relation/198754"

# Global mappers to convert huge OSM IDs to compact 0...N-1 indices
ID_MAPPER: Dict[int, int] = {}
NEXT_NODE_ID = 0
NEXT_EDGE_ID = 0

# Points of Interest (POIs) - Used to keep graph structure consistent
POIS = ["restaurant", "library", "hostel", "lab", "gate"]


# ----------------------------------------------------------------------------------
# HELPER FUNCTIONS (for Graph Generation)
# ----------------------------------------------------------------------------------

def write_json(path: str, data: Dict[str, Any]):
    """Writes JSON data to file and creates directories as needed."""
    folder = os.path.dirname(path)
    if folder and not os.path.exists(folder):
        os.makedirs(folder, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

def sine_square_half(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Computes Haversine distance in meters between two lat/lon points."""
    R = 6371000.0  # Earth radius in meters
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def make_speed_profile(slots=24) -> List[float]:
    """Generates a simple 24-slot speed profile (km/h)."""
    vals = []
    for i in range(slots):
        speed = 20 + 20 * math.sin((i / slots) * 2 * math.pi + math.pi/2)
        if 8 <= i <= 18:
            speed += random.uniform(5, 15)
        vals.append(max(5.0, round(speed, 2)))
    return vals

def parse_osm_center(url: str) -> tuple[float, float, int]:
    """Returns hardcoded center for simplicity (IIT Bombay)."""
    return 19.1334, 72.9133, 16

def center_to_bbox(lat: float, lon: float, zoom: int) -> List[float]:
    """Converts a center point to a fixed-size bounding box."""
    delta = 0.01  # Approximately 1.1 km
    return [lon - delta, lat - delta, lon + delta, lat + delta] # [min_lon, min_lat, max_lon, max_lat]

def fetch_osm_road_network(bbox: List[float]) -> Dict[str, Any]:
    """Fetches road network data from Overpass API."""
    min_lon, min_lat, max_lon, max_lat = bbox
    query = f"""
    [out:json][timeout:25];
    (
      way["highway"~"^(motorway|trunk|primary|secondary|tertiary|residential|service)$"]({min_lat},{min_lon},{max_lat},{max_lon});
    );
    (._;>;);
    out body;
    """
    overpass_url = "https://overpass-api.de/api/interpreter"
    response = requests.post(overpass_url, data=query)
    response.raise_for_status()
    return response.json()

def get_node_id(osm_id: int) -> int:
    """Maps large OSM IDs to compact 0...N-1 IDs."""
    global NEXT_NODE_ID
    if osm_id not in ID_MAPPER:
        ID_MAPPER[osm_id] = NEXT_NODE_ID
        NEXT_NODE_ID += 1
    return ID_MAPPER[osm_id]

def get_edge_id() -> int:
    """Generates the next compact edge ID."""
    global NEXT_EDGE_ID
    NEXT_EDGE_ID += 1
    return NEXT_EDGE_ID - 1

def build_graph_from_overpass(osm_data: Dict[str, Any]) -> Dict[str, List[Dict[str, Any]]]:
    """Transforms raw Overpass data into the project's graph.json format."""
    global ID_MAPPER, NEXT_NODE_ID, NEXT_EDGE_ID
    ID_MAPPER.clear()
    NEXT_NODE_ID = 0
    NEXT_EDGE_ID = 0

    nodes: List[Dict[str, Any]] = []
    edges: List[Dict[str, Any]] = []
    osm_nodes: Dict[int, Dict[str, Any]] = {}
    
    for element in osm_data.get("elements", []):
        if element["type"] == "node":
            osm_nodes[element["id"]] = element
            node_id = get_node_id(element["id"])
            
            pois_list = []
            if random.random() < 0.1:
                num_pois = random.randint(1, 3)
                pois_list = random.sample(POIS, min(num_pois, len(POIS)))
                
            nodes.append({
                "id": node_id,
                "lat": element["lat"],
                "lon": element["lon"],
                "pois": pois_list
            })

    for element in osm_data.get("elements", []):
        if element["type"] == "way":
            tags = element.get("tags", {})
            road_type = tags.get("highway", "residential")
            oneway = tags.get("oneway") == "yes"
            speed_profiles = make_speed_profile()
            
            for i in range(len(element["nodes"]) - 1):
                u_osm_id = element["nodes"][i]
                v_osm_id = element["nodes"][i+1]
                
                if u_osm_id not in osm_nodes or v_osm_id not in osm_nodes:
                    continue
                
                u_node = osm_nodes[u_osm_id]
                v_node = osm_nodes[v_osm_id]
                
                u_id = get_node_id(u_osm_id)
                v_id = get_node_id(v_osm_id)
                
                length = sine_square_half(u_node["lat"], u_node["lon"], v_node["lat"], v_node["lon"])
                
                avg_speed_kph = 30.0
                avg_speed_mps = avg_speed_kph * 1000.0 / 3600.0
                average_time = length / avg_speed_mps
                
                # Add edge u -> v
                edge = {
                    "id": get_edge_id(),
                    "u": u_id,
                    "v": v_id,
                    "length": round(length, 2),
                    "average_time": round(average_time, 2),
                    "speed_profiles": speed_profiles,
                    "oneway": oneway,
                    "road_type": road_type
                }
                edges.append(edge)

                # Add reverse edge v -> u if not oneway
                if not oneway:
                    reverse_edge = {
                        "id": get_edge_id(),
                        "u": v_id,
                        "v": u_id,
                        "length": round(length, 2),
                        "average_time": round(average_time, 2),
                        "speed_profiles": speed_profiles,
                        "oneway": False,
                        "road_type": road_type
                    }
                    edges.append(reverse_edge)

    return {"nodes": nodes, "edges": edges}


# ----------------------------------------------------------------------------------
# PHASE 2 QUERY GENERATION FUNCTION
# ----------------------------------------------------------------------------------

def generate_phase2_queries(all_nodes: List[Dict[str, Any]], n_shortest: int = 10, n_diverse: int = 10, n_approx_batches: int = 5) -> Dict[str, Any]:
    """Generates only Phase 2 queries."""
    events: List[Dict[str, Any]] = []
    qid = 0
        
    # --- 1. Exact K-Shortest Path ---
    for _ in range(n_shortest):
        source_node = random.choice(all_nodes)
        target_node = random.choice(all_nodes)
        
        events.append({
            "id": qid,
            "type": "k_shortest_path",
            "source": source_node["id"],
            "target": target_node["id"],
            "k": random.randint(2, 5)
        })
        qid += 1
        
    # --- 2. Heuristic K-Shortest Diverse Path ---
    for _ in range(n_diverse):
        source_node = random.choice(all_nodes)
        target_node = random.choice(all_nodes)
        
        events.append({
            "id": qid,
            "type": "k_shortest_diverse_path",
            "source": source_node["id"],
            "target": target_node["id"],
            "k": random.randint(2, 5),
            "overlap_threshold": round(random.uniform(0.3, 0.7), 2)
        })
        qid += 1
        
    # --- 3. Approximate Shortest Path (Batch) ---
    for _ in range(n_approx_batches):
        approx_queries_list: List[Dict[str, int]] = []
        # Generate a batch of 10 queries per batch event
        for _ in range(10): 
            source_node = random.choice(all_nodes)
            target_node = random.choice(all_nodes)
            approx_queries_list.append({
                "source": source_node["id"],
                "target": target_node["id"]
            })
            
        events.append({
            "id": qid,
            "type": "approx_shortest_path",
            "time_budget_ms": random.randint(50, 200),
            "acceptable_error_pct": round(random.uniform(5.0, 15.0), 2),
            "queries": approx_queries_list
        })
        qid += 1


    return {
        "meta": {"id": "iitb_osm_queries_phase2_only"},
        "events": events,
    }

# ----------------------------------------------------------------------------------
# MAIN EXECUTION
# ----------------------------------------------------------------------------------

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 generate_testcase.py <outdir>")
        print("Example: python3 generate_testcase.py ../testcases")
        sys.exit(1)
    outdir = sys.argv[1]
    
    try:
        # 1) Find IITB center and bounding box
        lat, lon, zoom = parse_osm_center(IITB_OSM_URL)
        bbox = center_to_bbox(lat, lon, zoom)

        # 2) Download OSM data
        print(f"Downloading road network for bounding box: {bbox} from Overpass...")
        osm_data = fetch_osm_road_network(bbox)

        # 3) Build graph.json
        print("Building graph.json ...")
        graph = build_graph_from_overpass(osm_data)
        graph_path = os.path.join(outdir, "graph.json")
        write_json(graph_path, graph)
        print(f"Graph: {len(graph['nodes'])} nodes, {len(graph['edges'])} edges. Written to {graph_path}")
        
        # 4) Generate queries.json (Phase 2 only)
        print("Generating queries.json (Phase 2 only)...")
        queries = generate_phase2_queries(graph["nodes"])
        queries_path = os.path.join(outdir, "queries.json")
        write_json(queries_path, queries)
        print(f"Queries: {len(queries['events'])} events. Written to {queries_path}")
        
        print("\nGeneration complete. Run your program with the generated files.")

    except requests.exceptions.HTTPError as e:
        print(f"\nERROR: Failed to fetch data from Overpass API. Status: {e.response.status_code}")
        print("Please check your internet connection or the Overpass API status.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")


if __name__ == "__main__":
    # Ensure requests library is installed if you encounter errors
    # pip install requests
    main()