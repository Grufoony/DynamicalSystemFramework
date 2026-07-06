"""
Tests for cartography module.
"""

import pytest
import networkx as nx
import folium
from dsf.cartography import (
    get_cartography,
    graph_to_gdfs,
    graph_from_gdfs,
    create_manhattan_cartography,
    to_folium_map,
)


def test_consistency():
    """
    A simple consistency test to verify that converting from GeoDataFrames to graph and back
    yields the same GeoDataFrames.
    """
    G_CART, edges_cart, nodes_cart = get_cartography("Postua, Piedmont, Italy")

    edges, nodes = graph_to_gdfs(G_CART)

    assert edges_cart.equals(edges), (
        "Edges GeoDataFrames are not equal after conversion."
    )
    assert nodes_cart.equals(nodes), (
        "Nodes GeoDataFrames are not equal after conversion."
    )

    G = graph_from_gdfs(edges_cart, nodes_cart)

    # Check structure first
    assert nx.is_isomorphic(G_CART, G), "Graphs are not isomorphic after conversion."


class TestCreateManhattanCartography:
    """Tests for create_manhattan_cartography function."""

    def test_default_parameters(self):
        """Test creation with default parameters."""
        edges, nodes = create_manhattan_cartography()

        # Check that we get the expected number of nodes (10x10 = 100)
        assert len(nodes) == 100
        # Check that we get bidirectional edges: (10-1)*10 + 10*(10-1) = 90 + 90 = 180 edges, times 2 for bidirectional = 360
        assert len(edges) == 360

        # Check that nodes have required columns
        assert "id" in nodes.columns
        assert "type" in nodes.columns
        assert "geometry" in nodes.columns

        # Check that edges have required columns
        assert "id" in edges.columns
        assert "source" in edges.columns
        assert "target" in edges.columns
        assert "nlanes" in edges.columns
        assert "type" in edges.columns
        assert "name" in edges.columns
        assert "length" in edges.columns
        assert "geometry" in edges.columns
        assert "maxspeed" in edges.columns

    def test_custom_grid_size(self):
        """Test creation with custom grid size."""
        n_x, n_y = 5, 4
        edges, nodes = create_manhattan_cartography(n_x=n_x, n_y=n_y)

        # Check node count
        assert len(nodes) == n_x * n_y

        # Calculate expected edge count
        # Horizontal edges: n_y * (n_x - 1) * 2 (bidirectional)
        # Vertical edges: n_x * (n_y - 1) * 2 (bidirectional)
        expected_edges = 2 * (n_y * (n_x - 1) + n_x * (n_y - 1))
        assert len(edges) == expected_edges

    def test_node_ids_are_unique(self):
        """Test that all node IDs are unique."""
        edges, nodes = create_manhattan_cartography(n_x=5, n_y=5)

        node_ids = nodes["id"].tolist()
        assert len(node_ids) == len(set(node_ids))

    def test_edge_ids_are_unique(self):
        """Test that all edge IDs are unique."""
        edges, nodes = create_manhattan_cartography(n_x=5, n_y=5)

        edge_ids = edges["id"].tolist()
        assert len(edge_ids) == len(set(edge_ids))

    def test_all_edges_are_bidirectional(self):
        """Test that for every edge u->v there exists an edge v->u."""
        edges, nodes = create_manhattan_cartography(n_x=4, n_y=4)

        # Create a set of (source, target) pairs
        edge_pairs = set(zip(edges["source"], edges["target"]))

        # Check that for every (u, v) there exists (v, u)
        for source, target in edge_pairs:
            assert (target, source) in edge_pairs

    def test_node_types(self):
        """Test that all nodes have the correct type."""
        edges, nodes = create_manhattan_cartography(n_x=3, n_y=3)

        # All nodes should have type "N/A"
        assert all(nodes["type"] == "N/A")

    def test_edge_attributes(self):
        """Test that edges have correct attributes."""
        maxspeed = 60.0
        edges, nodes = create_manhattan_cartography(n_x=3, n_y=3, maxspeed=maxspeed)

        # Check nlanes
        assert all(edges["nlanes"] == 1)

        # Check type
        assert all(edges["type"] == "primary")

        # Check maxspeed
        assert all(edges["maxspeed"] == maxspeed)

        # Check that all edges have names
        assert all(edges["name"].str.startswith("grid_street_"))

    def test_edge_length_calculation(self):
        """Test that edge lengths are calculated correctly."""
        spacing = 1000.0  # 1 km
        edges, nodes = create_manhattan_cartography(n_x=3, n_y=3, spacing=spacing)

        # All edges should have approximately the same length (spacing)
        # Allow for small numerical errors
        assert all(abs(edges["length"] - spacing) < 1.0)

    def test_center_coordinates(self):
        """Test that the grid is centered at the specified coordinates."""
        center_lat, center_lon = 45.0, 10.0
        edges, nodes = create_manhattan_cartography(
            n_x=5, n_y=5, center_lat=center_lat, center_lon=center_lon
        )

        # Calculate the centroid of all nodes
        mean_lat = nodes.geometry.y.mean()
        mean_lon = nodes.geometry.x.mean()

        # Check that the centroid is close to the specified center
        # Allow for small numerical errors
        assert abs(mean_lat - center_lat) < 0.01
        assert abs(mean_lon - center_lon) < 0.01

    def test_geometry_types(self):
        """Test that geometries have the correct types."""
        edges, nodes = create_manhattan_cartography(n_x=3, n_y=3)

        # All nodes should have Point geometries
        assert all(nodes.geometry.geom_type == "Point")

        # All edges should have LineString geometries
        assert all(edges.geometry.geom_type == "LineString")

    def test_crs(self):
        """Test that GeoDataFrames have the correct CRS."""
        edges, nodes = create_manhattan_cartography(n_x=3, n_y=3)

        # Both should use EPSG:4326 (WGS 84)
        assert edges.crs == "EPSG:4326"
        assert nodes.crs == "EPSG:4326"

    def test_edge_connectivity(self):
        """Test that edges connect valid nodes."""
        edges, nodes = create_manhattan_cartography(n_x=4, n_y=4)

        node_ids = set(nodes["id"])

        # All edge sources and targets should be valid node IDs
        assert all(edges["source"].isin(node_ids))
        assert all(edges["target"].isin(node_ids))

    def test_no_self_loops(self):
        """Test that there are no self-loops (edges from a node to itself)."""
        edges, nodes = create_manhattan_cartography(n_x=5, n_y=5)

        # No edge should have the same source and target
        assert all(edges["source"] != edges["target"])

    def test_small_grid(self):
        """Test with a minimal grid size."""
        edges, nodes = create_manhattan_cartography(n_x=2, n_y=2)

        # 2x2 grid has 4 nodes
        assert len(nodes) == 4

        # 2x2 grid has 4 undirected edges, so 8 directed edges
        assert len(edges) == 8

    def test_custom_spacing(self):
        """Test with custom spacing."""
        spacing1 = 500.0
        spacing2 = 2000.0

        edges1, _ = create_manhattan_cartography(n_x=3, n_y=3, spacing=spacing1)
        edges2, _ = create_manhattan_cartography(n_x=3, n_y=3, spacing=spacing2)

        # Edges with larger spacing should have larger lengths
        assert edges1["length"].mean() < edges2["length"].mean()

    def test_rectangular_grid(self):
        """Test with a rectangular (non-square) grid."""
        n_x, n_y = 10, 3
        edges, nodes = create_manhattan_cartography(n_x=n_x, n_y=n_y)

        assert len(nodes) == n_x * n_y

        # Calculate expected edges
        expected_edges = 2 * (n_y * (n_x - 1) + n_x * (n_y - 1))
        assert len(edges) == expected_edges


class TestToFoliumMap:
    """Tests for to_folium_map function."""

    @pytest.fixture
    def sample_graph(self):
        """Create a sample graph for testing."""
        edges, nodes = create_manhattan_cartography(n_x=3, n_y=3)
        return graph_from_gdfs(edges, nodes)

    def test_returns_folium_map(self, sample_graph):
        """Test that the function returns a folium.Map object."""
        result = to_folium_map(sample_graph)
        assert isinstance(result, folium.Map)

    def test_edges_only(self, sample_graph):
        """Test visualization with edges only (default)."""
        result = to_folium_map(sample_graph, which="edges")
        assert isinstance(result, folium.Map)
        # Check that the map has children (the edges)
        assert len(result._children) > 0

    def test_nodes_only(self, sample_graph):
        """Test visualization with nodes only."""
        result = to_folium_map(sample_graph, which="nodes")
        assert isinstance(result, folium.Map)
        assert len(result._children) > 0

    def test_both_edges_and_nodes(self, sample_graph):
        """Test visualization with both edges and nodes."""
        result = to_folium_map(sample_graph, which="both")
        assert isinstance(result, folium.Map)
        # Should have more children than edges-only or nodes-only
        edges_only = to_folium_map(sample_graph, which="edges")
        nodes_only = to_folium_map(sample_graph, which="nodes")
        # 'both' should have children from edges and nodes combined
        # (minus the base tile layer which is common)
        assert len(result._children) >= len(edges_only._children)
        assert len(result._children) >= len(nodes_only._children)

    def test_map_center_location(self, sample_graph):
        """Test that the map is centered correctly."""
        result = to_folium_map(sample_graph)
        # The map should be centered around the mean of node coordinates
        # For a Manhattan grid centered at (0, 0), the center should be near (0, 0)
        location = result.location
        assert location is not None
        assert len(location) == 2
        # Check that location is reasonable (near 0,0 for default manhattan grid)
        assert -1 < location[0] < 1  # latitude
        assert -1 < location[1] < 1  # longitude

    def test_default_which_parameter(self, sample_graph):
        """Test that default 'which' parameter is 'edges'."""
        default_result = to_folium_map(sample_graph)
        edges_result = to_folium_map(sample_graph, which="edges")
        # Both should produce maps with the same number of children
        assert len(default_result._children) == len(edges_result._children)


class TestForbiddenTurns:
    """Tests for lane_mapping and forbidden_turns inference (uses Bologna, which has turn:lanes data)."""

    @pytest.fixture(scope="class")
    def bologna_cartography(self):
        """Fetch and process a Bologna cartography known to include turn:lanes tags."""
        return get_cartography("Bologna, Emilia-Romagna, Italy")

    def test_lane_mapping_column_has_values(self, bologna_cartography):
        """Test that at least some edges have a non-null lane_mapping."""
        _, edges, _ = bologna_cartography
        assert "lane_mapping" in edges.columns
        assert edges["lane_mapping"].notna().any()

    def test_lane_mapping_entries_are_lists_of_strings(self, bologna_cartography):
        """Test that lane_mapping entries are well-formed lists of non-empty strings."""
        _, edges, _ = bologna_cartography
        lane_mappings = edges["lane_mapping"].dropna()
        assert len(lane_mappings) > 0
        for mapping in lane_mappings:
            assert isinstance(mapping, list)
            assert len(mapping) > 0
            for lane in mapping:
                assert isinstance(lane, str)
                assert len(lane) > 0
                # "through" should have been normalized to "straight"
                assert "through" not in lane

    def test_forbidden_turns_column_has_values(self, bologna_cartography):
        """Test that at least some edges have inferred forbidden_turns."""
        _, edges, _ = bologna_cartography
        assert "forbidden_turns" in edges.columns
        assert edges["forbidden_turns"].notna().any()

    def test_forbidden_turns_reference_valid_edge_ids(self, bologna_cartography):
        """Test that forbidden_turns entries are lists of valid, existing edge ids."""
        _, edges, _ = bologna_cartography
        valid_ids = set(edges["id"])
        forbidden = edges["forbidden_turns"].dropna()
        for entry in forbidden:
            assert isinstance(entry, list)
            assert len(entry) > 0
            for edge_id in entry:
                assert edge_id in valid_ids

    def test_no_forbidden_turns_without_lane_mapping(self, bologna_cartography):
        """Edges without a lane_mapping should never have forbidden_turns set."""
        _, edges, _ = bologna_cartography
        no_mapping = edges[edges["lane_mapping"].isna()]
        assert no_mapping["forbidden_turns"].isna().all()

    def test_infer_forbidden_turns_disabled(self):
        """Test that disabling infer_forbidden_turns skips forbidden_turns computation entirely."""
        _, edges, _ = get_cartography(
            "Bologna, Emilia-Romagna, Italy", infer_forbidden_turns=False
        )
        assert "forbidden_turns" not in edges.columns


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
