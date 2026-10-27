"""Run a pedestrian traffic simulation on the locally generated Venezia network.

This script uses the road network already produced by 0_build_venezia_geojson.py
in the project output folder. It does not download data from OpenStreetMap.
"""

import argparse
from datetime import datetime
from pathlib import Path

import geopandas as gpd
import numpy as np

from dsf import logging
from dsf.mobility import TrafficSimulator
from global_constants import WORK_FOLDER
from dsf.mobility import AgentInsertionMethod


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a DSF traffic simulation on the local Venezia GeoJSON network."
    )
    parser.add_argument(
        "--edges",
        type=Path,
        default=WORK_FOLDER / "0_edges.geojson",
        help="Path to the local edge GeoJSON file (default: WORK_FOLDER/0_edges.geojson)",
    )
    parser.add_argument(
        "--nodes",
        type=Path,
        default=WORK_FOLDER / "0_nodes.geojson",
        help="Path to the local node GeoJSON file (default: WORK_FOLDER/0_nodes.geojson)",
    )
    parser.add_argument(
        "--fraction",
        type=float,
        default=0.05,
        help="Fraction of streets to use as simulation origins and destinations.",
    )
    parser.add_argument(
        "--seed", type=int, default=69, help="Random seed for reproducibility"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    np.random.seed(args.seed)

    edges_path = args.edges.resolve()
    nodes_path = args.nodes.resolve()

    if not edges_path.exists():
        raise FileNotFoundError(
            f"Missing edges GeoJSON at {edges_path}. Run 0_build_venezia_geojson.py first."
        )

    gdf_edges = gpd.read_file(edges_path).set_geometry("geometry")
    street_ids = gdf_edges["id"].astype(int).tolist()
    gdf_nodes = gpd.read_file(nodes_path).set_geometry("geometry") if nodes_path.exists() else None

    if not street_ids:
        raise ValueError(f"No streets were found in the local network: {edges_path}")

    if args.fraction <= 0 or args.fraction >= 1:
        raise ValueError("--fraction must be strictly between 0 and 1.")

    # sample_size = max(1, int(len(street_ids) * args.fraction))
    '''origin_ids = np.random.choice(street_ids, size=sample_size, replace=False)
    destination_ids = np.random.choice(
        [street_id for street_id in street_ids if street_id not in origin_ids],
        size=min(sample_size, max(1, len(street_ids) - len(origin_ids))),
        replace=False,
    )'''
    # set piazzale roma as origin and piazza san marco as destination
    origin_ids = [int(street_id) for street_id in gdf_edges[gdf_edges["name"] == "Piazzale_Roma"]["id"].tolist()]
    destination_ids = [int(street_id) for street_id in gdf_edges[gdf_edges["name"] == "Piazza_San_Marco"]["id"].tolist()]


    origins = {int(node_id): 1 for node_id in origin_ids}
    destinations = [int(node_id) for node_id in destination_ids]

    logging.info(f"Using local road network from {edges_path}")
    logging.info(
        f"Loaded {len(gdf_nodes) if gdf_nodes is not None else len(street_ids)} nodes and {len(gdf_edges)} edges"
    )
    logging.info(
        f"Using {len(origins)} origins and {len(destinations)} destinations."
    )

    local_now = datetime.now().astimezone()
    epoch = int(
        datetime.combine(
            local_now.date(), datetime.min.time(), tzinfo=local_now.tzinfo
        ).timestamp()
    )

    simulator = TrafficSimulator()
    simulator.setPedestrianMode(True)
    simulator.connectDataBase("venezia_pedestrian.db")
    simulator.importRoadNetwork(edges_path.as_posix())
    simulator.setTimeFrame(epoch)
    simulator.saveData(300, True, True, True)
    simulator.updatePaths(300, False)

    simulator.dynamics().setSeed(args.seed)
    simulator.dynamics().setOrigins(origins)
    simulator.dynamics().setDestinations(destinations)
    simulator.setAgentInsertionMethod(AgentInsertionMethod.RANDOM_ODS)
    simulator.dynamics().killStagnantAgents(40.0)

    simulator.run(np.random.randint(0, 50, size=8640), 10)


if __name__ == "__main__":
    main()
