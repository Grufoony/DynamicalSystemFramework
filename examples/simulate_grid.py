"""Run a 24-hour traffic simulation on a synthetic Manhattan-style grid.

This script generates grid cartography CSV files, builds a road network,
configures the dynamics engine, and simulates agent flow with a 1-second
integration step and 10-second agent insertion cadence.
"""

import argparse
from datetime import datetime
import logging

from dsf.cartography import create_manhattan_cartography
from dsf.mobility import (
    RoadNetwork,
    Dynamics,
    AgentInsertionMethod,
)

from tqdm import trange
from numba import cfunc, float64
import numpy as np


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
        "--dim", type=str, default="12x12", help="Dimensions of the grid (e.g., 10x10)"
    )
    parser.add_argument(
        "--amp", type=int, required=True, help="Amplitude of the vehicle input"
    )
    args = parser.parse_args()
    np.random.seed(args.seed)

    # Parse the grid dimensions
    try:
        rows, cols = map(int, args.dim.split("x"))
    except ValueError:
        raise ValueError(
            "Invalid grid dimensions. Please use the format 'rowsxcols' (e.g., 10x10)."
        )

    logging.info(f"Creating manhattan cartography for {rows}x{cols} grid...")
    # Get the cartography of the specified city
    df_edges, df_nodes = create_manhattan_cartography(rows, cols)

    df_nodes["type"] = (
        "traffic_signals"  # Set all nodes as traffic lights for simplicity
    )

    df_edges.to_csv(f"grid_{args.dim}_edges.csv", sep=";", index=False)
    df_nodes.to_csv(f"grid_{args.dim}_nodes.csv", sep=";", index=False)

    del df_edges, df_nodes

    logging.info("Creating road network and dynamics model...")

    # Create a road network from the cartography
    road_network = RoadNetwork()
    road_network.importEdges(f"grid_{args.dim}_edges.csv", ";")
    road_network.importNodeProperties(f"grid_{args.dim}_nodes.csv", ";")
    # Adjust network parameters
    road_network.adjustNodeCapacities()
    road_network.autoMapStreetLanes()
    road_network.autoAssignRoadPriorities()
    road_network.autoInitTrafficLights()
    road_network.describe()

    # Generate a random vector of integer values for vehicle input
    # We want values to have a 10s entry for a whole day
    vehicle_input = np.random.normal(args.amp, args.amp * 0.1, size=8640)
    vehicle_input = np.clip(vehicle_input, 0, None).astype(int)

    # Create a dynamics model for the road network
    dynamics = Dynamics(road_network, seed=args.seed)
    # To use a custom speed function, you must pass the pointer to the compiled function using the address attribute
    # dynamics.setSpeedFunction(SpeedFunction.CUSTOM, custom_speed.address)
    # Get epoch time of today at midnight
    epoch_time = int(
        datetime.combine(datetime.today(), datetime.min.time()).timestamp()
    )

    dynamics.setMeanTravelDistance(10e3)  # Set mean travel distance to 10 km
    dynamics.killStagnantAgents(40.0)
    dynamics.setInitTime(epoch_time)
    dynamics.connectDataBase(f"grid_{args.dim}.db")
    dynamics.saveData(300, True, True, True)

    # Simulate traffic for 24 hours with a time step of 1 seconds
    for time_step in trange(86400):
        # Update paths every 5 minutes (300 seconds)
        if time_step % 300 == 0:
            dynamics.updatePaths()
        # Add agents every 10 seconds
        if time_step % 10 == 0:
            dynamics.addAgents(
                vehicle_input[time_step // 10], AgentInsertionMethod.RANDOM
            )
        dynamics.evolve()

    dynamics.summary()
