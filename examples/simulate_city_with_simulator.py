"""Run a 24-hour traffic simulation using OpenStreetMap city cartography.

This script downloads city data, builds network CSV assets, configures origins
and destinations, initializes the dynamics engine, and runs a full-day
simulation while periodically updating shortest paths.
"""

import argparse
from datetime import datetime
from pathlib import Path
import pickle

import dsf
from dsf.cartography import get_cartography, to_folium_map
from dsf import logging
from dsf.mobility import TrafficSimulator

import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--seed", type=int, default=69, help="Random seed for reproducibility"
    )
    parser.add_argument(
        "--city", type=str, required=True, help="City name for cartography"
    )
    parser.add_argument(
        "--country", type=str, default="Italy", help="Country name for cartography"
    )
    args = parser.parse_args()
    logging.info(f"Using dsf version: {dsf.__version__}")
    EPOCH = int(datetime.combine(datetime.today(), datetime.min.time()).timestamp())
    np.random.seed(args.seed)
    args.city = args.city.lower().strip().replace(" ", "_")
    args.country = args.country.lower().strip().replace(" ", "_")

    logging.info(
        f"Getting data from OpenStreetMap for {args.city.capitalize()}, {args.country.capitalize()}..."
    )
    # Get the cartography of the specified city
    # Check if the cartography files already exist, if not download them from OpenStreetMap
    if (
        not Path(f"{args.city}_{args.country}_edges.csv").is_file()
        or not Path(f"{args.city}_{args.country}_nodes.csv").is_file()
        or not Path(f"{args.city}_{args.country}.pickle").is_file()
    ):
        G, df_edges, df_nodes = get_cartography(
            {"city": args.city.capitalize(), "country": args.country.capitalize()},
            infer_speeds=True,
            scc=True,
        )

        df_edges.to_csv(f"{args.city}_{args.country}_edges.csv", sep=";", index=False)
        df_nodes.to_csv(f"{args.city}_{args.country}_nodes.csv", sep=";", index=False)
        with open(f"{args.city}_{args.country}.pickle", "wb") as f:
            pickle.dump(G, f, protocol=pickle.HIGHEST_PROTOCOL)
        del df_edges, df_nodes, G
    else:
        logging.info("Cartography files already exist, skipping download.")

    with open(f"{args.city}_{args.country}.pickle", "rb") as f:
        G = pickle.load(f)

    to_folium_map(G).save(f"{args.city}_{args.country}_map.html")

    nodes = G.nodes(data=False)
    # Extract 10% random node ids as origins and destinations for the traffic simulation
    origin_ids = np.random.choice(
        list(nodes), size=int(0.05 * len(nodes)), replace=False
    )
    destination_ids = np.random.choice(
        list(nodes), size=int(0.05 * len(nodes)), replace=False
    )

    origins = {node_id: 1 for node_id in origin_ids}
    destinations = {node_id: 1 for node_id in destination_ids}

    logging.info("Creating road network and dynamics model...")

    simulator = TrafficSimulator()
    simulator.importRoadNetwork(
        f"{args.city}_{args.country}_edges.csv", f"{args.city}_{args.country}_nodes.csv"
    )
    simulator.setTimeFrame(EPOCH)
    simulator.saveData(300, True, True, True)
    simulator.updatePaths(300, False)

    simulator.dynamics().setSeed(args.seed)
    simulator.dynamics().setOriginNodes(origins)
    simulator.dynamics().setDestinationNodes(destinations)
    simulator.dynamics().killStagnantAgents(40.0)

    simulator.run(np.random.randint(0, 50, size=8640), 10)
