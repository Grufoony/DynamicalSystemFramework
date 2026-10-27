from pathlib import Path
import geopandas as gpd

from global_constants import COMMON_FOLDER, WORK_FOLDER, DEFAULT_CRS

gdf_edges = gpd.read_file(WORK_FOLDER / "0_edges.geojson")
gdf_nodes = gpd.read_file(WORK_FOLDER / "0_nodes.geojson")

# Genera mappa interattiva HTML
m = gdf_edges.explore(color="blue", weight=2, name="Archi Pedonali")
gdf_nodes.explore(m=m, color="red", marker_kwds={"radius": 2}, name="Nodi")

# Salva il file HTML
html_output = "network_map.html"
m.save(html_output)

print(f"Map saved to {html_output}")