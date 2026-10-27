#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import networkx as nx
from global_constants import COMMON_FOLDER, WORK_FOLDER, DEFAULT_CRS


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Convert network data (.pro, .pnt) into GeoJSON format and extract largest strongly connected component."
    )
    parser.add_argument(
        "-v",
        "--exclude-vaporetto",
        action="store_true",
        help="Exclude edges with a name starting with 'linea_vaporetto' or 'Ponte_del_Redentore'",
    )
    return parser.parse_args()


def parse_pro(pro_path: Path):
    """
    Read a .pro file formatted as:
      cid nFcid nTcid length ... speed oneway name
    """
    edges = []
    with pro_path.open("r", encoding="utf-8", errors="ignore") as f:
        _header = f.readline()  # Ignore header if present
        for line in f:
            s = line.strip()
            if not s:
                continue
            toks = s.split()
            if len(toks) < 11:
                continue

            cid = int(toks[0])
            u = int(toks[1])
            v = int(toks[2])
            length = float(toks[3])
            road_type = toks[5] if len(toks) > 5 else ""
            speed_kmh = float(toks[8]) if len(toks) > 8 else 0.0
            oneway = bool(int(toks[9])) if len(toks) > 9 else False
            name = toks[10] if len(toks) > 10 else ""

            edges.append({
                "cid": cid,
                "u": u,
                "v": v,
                "length": length,
                "type": road_type,
                "speed_kmh": speed_kmh,
                "oneway": oneway,
                "name": name if name != "_" else ""
            })
    return edges


def parse_pnt(pnt_path: Path):
    """
    Read a .pnt file in custom format:
      cid <ignored> npoints
      lat lon
      lat lon
      ...
    Where lat/lon are in microdegrees, divided by 1_000_000.0.
    """
    raw_tokens = pnt_path.read_text(encoding="utf-8", errors="ignore").split()
    i = 0
    blocks = {}

    while i + 2 < len(raw_tokens):
        cid = int(raw_tokens[i])
        _ignored = int(raw_tokens[i + 1])
        npoints = int(raw_tokens[i + 2])
        i += 3

        coords = []
        for _ in range(npoints):
            if i + 1 >= len(raw_tokens):
                raise ValueError(f"Malformed .pnt file: {pnt_path}")
            lat = int(raw_tokens[i]) / 1_000_000.0
            lon = int(raw_tokens[i + 1]) / 1_000_000.0
            coords.append((lon, lat))
            i += 2

        blocks[cid] = coords

    return blocks


def feature_collection(name, features):
    return {
        "type": "FeatureCollection",
        "name": name,
        "crs": {
            "type": "name",
            "properties": {
                "name": "urn:ogc:def:crs:OGC:1.3:CRS84"
            }
        },
        "features": features,
    }


def make_edge_feature(u, v, coords, edge_info, feature_id):
    """Helper to construct a single edge Feature object."""
    speed_mps = (edge_info["speed_kmh"] * 1000.0 / 3600.0) if edge_info["speed_kmh"] > 0 else None
    travel_time = (edge_info["length"] / speed_mps) if speed_mps else None

    return {
        "type": "Feature",
        "properties": {
            "u": u,
            "v": v,
            "key": 0,
            "name": edge_info["name"],
            "oneway": edge_info["oneway"],
            "length": edge_info["length"],
            "width": None,
            "in_scc": True,
            "travel_time": travel_time,
            "maxspeed": edge_info["speed_kmh"],
            "nlanes": 1,
            "type": edge_info["type"],
            "id": feature_id,
            "source": u,
            "target": v,
            "est_width": None,
            "area": None,
        },
        "geometry": {
            "type": "LineString",
            "coordinates": [[lon, lat] for lon, lat in coords]
        }
    }


def main():
    args = parse_args()

    pro_path = COMMON_FOLDER / "venezia.pro"
    pnt_path = COMMON_FOLDER / "venezia.pnt"

    edges = parse_pro(pro_path)
    pnt_map = parse_pnt(pnt_path)

    node_coords = {}
    edge_features = []
    excluded_count = 0

    for edge in edges:
        # Exclude specified edges when flag is passed
        if args.exclude_vaporetto and (
            edge["name"].startswith("linea_vaporetto") or edge["name"].startswith("Ponte_del_Redentore")
        ):
            excluded_count += 1
            continue

        cid = edge["cid"]
        coords = pnt_map.get(cid)
        if coords is None:
            continue  # Skip if coordinate block is missing

        u = edge["u"]
        v = edge["v"]

        # Store node coordinates
        if u not in node_coords:
            node_coords[u] = coords[0]
        if v not in node_coords:
            node_coords[v] = coords[-1]

        # Add forward directed edge (u -> v)
        edge_features.append(make_edge_feature(u, v, coords, edge, len(edge_features)))

        # If street is two-way, add reverse directed edge (v -> u) with inverted coordinates
        if not edge["oneway"]:
            edge_features.append(make_edge_feature(v, u, coords[::-1], edge, len(edge_features)))

    if args.exclude_vaporetto:
        print(f"Excluded {excluded_count} edges matching 'linea_vaporetto*' or 'Ponte_del_Redentore*'.")

    # Extract Largest Strongly Connected Component (LSCC) using a Directed Graph
    G = nx.DiGraph()
    for feat in edge_features:
        G.add_edge(feat["properties"]["u"], feat["properties"]["v"])

    if G.number_of_nodes() > 0:
        # Extract the largest strongly connected component
        scc_nodes = max(nx.strongly_connected_components(G), key=len)
        print(f"Retaining Largest Strongly Connected Component (LSCC): {len(scc_nodes)} / {G.number_of_nodes()} nodes.")

        # Keep only nodes belonging to the LSCC
        node_coords = {osmid: coords for osmid, coords in node_coords.items() if osmid in scc_nodes}

        # Keep only edges whose endpoint nodes are both in the LSCC
        edge_features = [
            feat for feat in edge_features
            if feat["properties"]["u"] in scc_nodes and feat["properties"]["v"] in scc_nodes
        ]

        # Re-index contiguous IDs for remaining edges
        for new_id, feat in enumerate(edge_features):
            feat["properties"]["id"] = new_id

    # Build node GeoJSON features
    node_features = []
    for osmid, (lon, lat) in sorted(node_coords.items()):
        node_features.append({
            "type": "Feature",
            "properties": {
                "osmid": osmid,
                "x": lon,
                "y": lat,
                "in_scc": True,
                "type": "N/A"
            },
            "geometry": {
                "type": "Point",
                "coordinates": [lon, lat]
            }
        })

    out_edges = Path(WORK_FOLDER / "0_edges.geojson")
    out_nodes = Path(WORK_FOLDER / "0_nodes.geojson")

    WORK_FOLDER.mkdir(exist_ok=True)
    out_edges.write_text(json.dumps(feature_collection("0_edges", edge_features), ensure_ascii=False, indent=2))
    out_nodes.write_text(json.dumps(feature_collection("0_nodes", node_features), ensure_ascii=False, indent=2))

    print(f"Edges file saved to: {out_edges} ({len(edge_features)} edges)")
    print(f"Nodes file saved to: {out_nodes} ({len(node_features)} nodes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())