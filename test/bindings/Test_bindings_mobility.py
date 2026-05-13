from pathlib import Path
import sqlite3

import pytest

from dsf import mobility


def _write_tiny_edges_csv(file_path: Path) -> None:
    file_path.write_text(
        "id;source;target;length;maxspeed;name;type;nlanes\n"
        "0;0;1;13.8888888889;50;edge_0;residential;1\n"
        "1;1;0;13.8888888889;50;edge_1;residential;1\n"
    )


def test_traffic_simulator_configuration():
    simulator = mobility.TrafficSimulator()

    assert simulator.dynamics() is None

    simulator.setName("traffic simulator test")
    simulator.setTimeFrame(10, 16)

    assert simulator.id() > 0
    assert simulator.initTime() == 10
    assert simulator.endTime() == 16
    assert simulator.name() == "traffic simulator test"
    assert simulator.safeName() == "traffic_simulator_test"


def test_traffic_simulator_import_counts_and_shortest_path(
    manhattan_edges_path, manhattan_nodes_path
):
    simulator = mobility.TrafficSimulator()
    simulator.importRoadNetwork(str(manhattan_edges_path), str(manhattan_nodes_path))

    dynamics = simulator.dynamics()

    assert dynamics is not None
    assert dynamics.graph().nNodes() == 120
    assert dynamics.graph().nEdges() == 436

    dynamics.graph().setEdgeWeight("length")
    path_map = dynamics.graph().shortestPath(0, 119)

    assert isinstance(path_map, mobility.PathCollection)
    assert len(path_map) > 0
    assert 0 in path_map
    assert 119 not in path_map
    assert isinstance(path_map[0], list)
    assert len(path_map[0]) > 0


def test_path_collection_dict_protocol_and_explode():
    path_collection = mobility.PathCollection()

    assert len(path_collection) == 0

    path_collection[0] = [1]
    path_collection[1] = [2]

    assert 0 in path_collection
    assert path_collection[0] == [1]
    assert set(path_collection.keys()) == {0, 1}
    assert path_collection.items() == {0: [1], 1: [2]}

    all_paths = path_collection.explode(0, 2)
    assert all_paths == [[0, 1, 2]]

    with pytest.raises(KeyError):
        _ = path_collection[99]


def test_traffic_simulator_smoke_run_with_linear_speed(tmp_path):
    edges_path = tmp_path / "tiny_edges.csv"
    db_path = tmp_path / "traffic_simulator.db"
    _write_tiny_edges_csv(edges_path)

    simulator = mobility.TrafficSimulator()
    simulator.setName("traffic simulator csv test")
    simulator.connectDataBase(str(db_path))
    simulator.importRoadNetwork(str(edges_path))

    dynamics = simulator.dynamics()

    assert dynamics is not None

    dynamics.setSpeedFunction(mobility.SpeedFunction.LINEAR, 0.8)
    dynamics.setODs([(0, 1, 1.0)])
    dynamics.updatePaths()

    simulator.setTimeFrame(0, 6)
    simulator.saveData(1, True, True, False, False)
    simulator.setAgentInsertionMethod(mobility.AgentInsertionMethod.ODS)
    simulator.run([1, 0, 0, 0, 0, 0])

    assert db_path.exists()

    with sqlite3.connect(db_path) as connection:
        cursor = connection.cursor()

        for table_name in (
            "simulation_info",
            "edges",
            "nodes",
            "road_data",
            "avg_stats",
        ):
            cursor.execute(
                "SELECT name FROM sqlite_master WHERE type='table' AND name=?",
                (table_name,),
            )
            assert cursor.fetchone() is not None

        cursor.execute("SELECT COUNT(*) FROM road_data")
        assert cursor.fetchone()[0] > 0

        cursor.execute("SELECT COUNT(*) FROM avg_stats")
        assert cursor.fetchone()[0] > 0
