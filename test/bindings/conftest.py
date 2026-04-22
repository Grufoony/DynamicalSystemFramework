from pathlib import Path

import pytest

from dsf import mobility

TEST_DATA_DIR = Path(__file__).resolve().parents[1] / "data"
MANHATTAN_EDGES = TEST_DATA_DIR / "manhattan_edges.csv"
MANHATTAN_NODES = TEST_DATA_DIR / "manhattan_nodes.csv"


@pytest.fixture(scope="session")
def manhattan_edges_path() -> Path:
    return MANHATTAN_EDGES


@pytest.fixture(scope="session")
def manhattan_nodes_path() -> Path:
    return MANHATTAN_NODES


@pytest.fixture()
def empty_road_network():
    return mobility.RoadNetwork()


@pytest.fixture()
def loaded_road_network(manhattan_edges_path: Path, manhattan_nodes_path: Path):
    network = mobility.RoadNetwork()
    network.importEdges(str(manhattan_edges_path), ";")
    network.importNodeProperties(str(manhattan_nodes_path), ";")
    return network


@pytest.fixture()
def dynamics(loaded_road_network):
    return mobility.Dynamics(loaded_road_network, seed=69)


@pytest.fixture()
def trajectory_rows():
    return {
        "uid": [1, 1, 1, 2, 2, 2],
        "timestamp": [1000, 1600, 2200, 1000, 1600, 2200],
        "lat": [44.4949, 44.4950, 44.4951, 45.4064, 45.4065, 45.4066],
        "lon": [11.3426, 11.3427, 11.3428, 11.8767, 11.8768, 11.8769],
    }


@pytest.fixture()
def trajectory_pandas_df(trajectory_rows):
    pandas = pytest.importorskip("pandas")
    return pandas.DataFrame(trajectory_rows)


@pytest.fixture()
def trajectory_polars_df(trajectory_rows):
    polars = pytest.importorskip("polars")
    return polars.DataFrame(trajectory_rows)
