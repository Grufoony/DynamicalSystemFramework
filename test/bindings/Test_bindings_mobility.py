from datetime import datetime, timezone

import numpy as np
import pytest

from dsf import mobility


def test_road_network_import_counts(loaded_road_network):
    assert loaded_road_network.nNodes() == 120
    assert loaded_road_network.nEdges() == 436


def test_shortest_path_returns_path_collection(loaded_road_network):
    path_map = loaded_road_network.shortestPath(0, 119, mobility.PathWeight.LENGTH)

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


def test_dynamics_set_init_time_accepts_epoch_and_datetime(dynamics):
    epoch = 1_700_000_000
    dynamics.setInitTime(epoch)
    assert dynamics.time() == epoch

    dt = datetime.fromtimestamp(epoch + 3600, tz=timezone.utc)
    dynamics.setInitTime(dt)
    assert dynamics.time() == int(dt.timestamp())


def test_dynamics_accepts_origin_destination_overloads(dynamics):
    node_array = np.array([0, 1, 2], dtype=np.uint64)

    dynamics.setDestinationNodes([0, 1, 2])
    dynamics.setDestinationNodes({0: 0.6, 1: 0.4})
    dynamics.setDestinationNodes(node_array)

    dynamics.setOriginNodes({0: 1.0, 1: 1.0, 2: 1.0})
    dynamics.setOriginNodes(node_array)


def test_dynamics_smoke_step_with_linear_speed(dynamics):
    dynamics.setWeightFunction(mobility.PathWeight.LENGTH)
    dynamics.setSpeedFunction(mobility.SpeedFunction.LINEAR, 0.8)
    dynamics.setDestinationNodes([0, 1, 2])
    dynamics.setOriginNodes({3: 1.0, 4: 1.0, 5: 1.0})

    dynamics.updatePaths()
    dynamics.addAgentsUniformly(1)

    assert dynamics.nAgents() == 1

    previous_step = dynamics.time_step()
    dynamics.evolve(False)
    assert dynamics.time_step() == previous_step + 1


def test_compute_betweenness_rejects_invalid_weight(loaded_road_network):
    with pytest.raises(Exception, match="Invalid weight function"):
        loaded_road_network.computeBetweennessCentralities("invalid")


def test_dynamics_custom_weight_callable_uses_street_attributes(tmp_path):
    edge_file = tmp_path / "custom_edges.csv"
    edge_file.write_text(
        "id;source;target;length;maxspeed;nlanes;name;type;custom_cost\n"
        "0;0;1;5;50;1;s01;residential;100\n"
        "1;1;2;5;50;1;s12;residential;100\n"
        "2;0;3;5;50;1;s03;residential;1\n"
        "3;3;2;5;50;1;s32;residential;1\n",
        encoding="utf-8",
    )

    network = mobility.RoadNetwork()
    network.importEdges(str(edge_file), ";")
    dynamics = mobility.Dynamics(network, seed=69)
    dynamics.setDestinationNodes([2])

    calls = {"count": 0}
    seen_custom_costs = set()

    def custom_weight(street):
        calls["count"] += 1
        attrs = street.attributes()
        if "custom_cost" in attrs:
            seen_custom_costs.add(float(attrs["custom_cost"]))
        return float(attrs.get("custom_cost", street.length()))

    dynamics.setWeightFunction(mobility.PathWeight.CUSTOM, 0.0, custom_weight)
    dynamics.updatePaths()

    assert calls["count"] > 0
    assert seen_custom_costs == {1.0, 100.0}


def test_dynamics_custom_weight_requires_callable(dynamics):
    with pytest.raises(Exception, match="PathWeight.CUSTOM requires a callable"):
        dynamics.setWeightFunction(mobility.PathWeight.CUSTOM)


def test_dynamics_builtin_weight_rejects_callable_arg(dynamics):
    with pytest.raises(Exception, match="Built-in PathWeight values do not accept"):
        dynamics.setWeightFunction(
            mobility.PathWeight.LENGTH, None, lambda street: street.length()
        )


def test_shortest_path_rejects_custom_weight(loaded_road_network):
    with pytest.raises(Exception, match="PathWeight.CUSTOM is not supported"):
        loaded_road_network.shortestPath(0, 119, mobility.PathWeight.CUSTOM)
