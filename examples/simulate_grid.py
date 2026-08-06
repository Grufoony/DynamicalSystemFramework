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
    TrafficSimulator,
    AgentInsertionMethod,
)

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

    simulator = TrafficSimulator()
    simulator.importRoadNetwork(
        f"grid_{args.dim}_edges.csv", f"grid_{args.dim}_nodes.csv"
    )
    # Generate a random vector of integer values for vehicle input
    # We want values to have a 10s entry for a whole day
    vehicle_input = np.random.normal(args.amp, args.amp * 0.1, size=8640)
    vehicle_input = np.clip(vehicle_input, 0, None).astype(int)

    EPOCH = int(datetime.combine(datetime.today(), datetime.min.time()).timestamp())

    simulator.setTimeFrame(EPOCH)
    simulator.saveData(300, True, True, True)
    simulator.updatePaths(300, False)

    simulator.dynamics().setSeed(args.seed)
    simulator.dynamics().killStagnantAgents(40.0)
    simulator.dynamics().setMeanTravelDistance(
        10e3
    )  # Set mean travel distance to 10 km
    simulator.setAgentInsertionMethod(AgentInsertionMethod.RANDOM)

    simulator.run(vehicle_input, 10)
