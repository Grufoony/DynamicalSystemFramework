"""Run a 24-hour traffic simulation using OpenStreetMap city cartography.

This script downloads city data, builds network CSV assets, configures origins
and destinations, initializes the dynamics engine, and runs a full-day
simulation while periodically updating shortest paths.
"""

import argparse
from datetime import datetime
import logging

from dsf.cartography import get_cartography
from dsf.mobility import (
    RoadNetwork,
    Dynamics,
    AgentInsertionMethod,
    SpeedFunction,
)

from tqdm import trange
from numba import cfunc, float64
import numpy as np
import networkx as nx


@cfunc(float64(float64, float64), nopython=True, cache=True)
def custom_speed(max_speed, density):
    """Compute a density-aware speed multiplier for custom speed modeling."""
    if density < 0.35:
        return max_speed * (0.9 - 0.1 * density)
    return max_speed * (1.2 - 0.7 * density)


logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

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
    np.random.seed(args.seed)
    args.city = args.city.lower().strip().replace(" ", "_")
    args.country = args.country.lower().strip().replace(" ", "_")

    logging.info(
        f"Getting data from OpenStreetMap for {args.city.capitalize()}, {args.country.capitalize()}..."
    )
    # Get the cartography of the specified city
    G, df_edges, df_nodes = get_cartography(
        {"city": args.city.capitalize(), "country": args.country.capitalize()},
        infer_speeds=True,
    )

    df_edges.to_csv(f"{args.city}_{args.country}_edges.csv", sep=";", index=False)
    df_nodes.to_csv(f"{args.city}_{args.country}_nodes.csv", sep=";", index=False)

    # Keep only strong connected components
    G = G.subgraph(max(nx.strongly_connected_components(G), key=len)).copy()
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

    del df_edges, df_nodes, G

    logging.info("Creating road network and dynamics model...")

    # Create a road network from the cartography
    road_network = RoadNetwork()
    road_network.importEdges(f"{args.city}_{args.country}_edges.csv", ";")
    road_network.importNodeProperties(f"{args.city}_{args.country}_nodes.csv", ";")
    # Adjust network parameters
    road_network.adjustNodeCapacities()
    road_network.autoMapStreetLanes()
    road_network.autoAssignRoadPriorities()
    road_network.autoInitTrafficLights()
    road_network.describe()

    # Generaate a random vector of integer values for vehicle input
    # We want values to have a 10s entry for a whole day
    vehicle_input = np.random.randint(0, 50, size=8640)

    # Create a dynamics model for the road network
    dynamics = Dynamics(road_network, seed=args.seed)
    dynamics.setSpeedFunction(SpeedFunction.LINEAR, 0.8)
    # To use a custom speed function, you must pass the pointer to the compiled function using the address attribute
    # dynamics.setSpeedFunction(SpeedFunction.CUSTOM, custom_speed.address)
    # Get epoch time of today at midnight
    epoch_time = int(
        datetime.combine(datetime.today(), datetime.min.time()).timestamp()
    )

    dynamics.killStagnantAgents(40.0)
    dynamics.setInitTime(epoch_time)
    dynamics.connectDataBase(f"{args.city}_{args.country}.db")
    dynamics.saveData(300, True, True, True)

    dynamics.setOriginNodes(origins)
    dynamics.setDestinationNodes(destinations)

    # Simulate traffic for 24 hours with a time step of 10 seconds
    for time_step in trange(86400):
        # Update paths every 5 minutes (300 seconds)
        if time_step % 300 == 0:
            dynamics.updatePaths()
        # Add agents every 10 seconds
        if time_step % 10 == 0:
            dynamics.addAgents(
                vehicle_input[time_step // 10], AgentInsertionMethod.RANDOM_ODS
            )
        dynamics.evolve()

    dynamics.summary()
