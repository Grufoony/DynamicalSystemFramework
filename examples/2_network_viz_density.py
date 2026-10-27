"""Visualize the average street occupancy of the simulated day on an interactive map.

This script reads the SQLite database produced by 1_trial_simulation.py
(venezia_pedestrian.db), computes the average density of every street over all
the saved time steps, and draws the road network with folium. Streets are
colored (and made thicker) according to how much they were used, so the most
crowded ones stand out immediately.

Density unit: DSF stores `density_vpk`, i.e. agents per kilometre of street.
In pedestrian mode this means "pedestrians per km of street".
"""

import argparse
import html
import sqlite3
from pathlib import Path

import branca.colormap as cm
import folium
import geopandas as gpd
import pandas as pd
from folium.plugins import Fullscreen, MiniMap

from global_constants import WORK_FOLDER

# Table and column names of the DSF database (verified on venezia_pedestrian.db)
TABLE = "road_data"
ID_COL = "street_id"
DENSITY_COL = "density_vpk"
SIM_COL = "simulation_id"

DENSITY_UNIT = "pedestrians / km"

# Basemaps with explicit URLs: they do not need any API key
CARTO_ATTR = (
    '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> '
    'contributors &copy; <a href="https://carto.com/attributions">CARTO</a>'
)
BASEMAPS = [
    (
        "Dark",
        "https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png",
        CARTO_ATTR,
    ),
    (
        "Light",
        "https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png",
        CARTO_ATTR,
    ),
    (
        "OpenStreetMap",
        "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
    ),
]

# Colors
UNUSED_COLOR = "#59606e"
COLORMAP_COLORS = ["#2b83ba", "#abdda4", "#ffffbf", "#fdae61", "#d7191c"]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot the average daily street occupancy of a DSF simulation with folium."
    )
    parser.add_argument(
        "--db",
        type=Path,
        default=Path("venezia_pedestrian.db"),
        help="Path to the simulation database (default: ./venezia_pedestrian.db)",
    )
    parser.add_argument(
        "--edges",
        type=Path,
        default=WORK_FOLDER / "0_edges.geojson",
        help="Path to the edge GeoJSON file (default: WORK_FOLDER/0_edges.geojson)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=WORK_FOLDER / "2_density_map.html",
        help="Output HTML map (default: WORK_FOLDER/2_density_map.html)",
    )
    parser.add_argument(
        "--simulation-id",
        type=int,
        default=None,
        help="Simulation id to use (default: the latest one in the database).",
    )
    parser.add_argument(
        "--top",
        type=int,
        default=10,
        help="Number of most used streets to show in the map panel and console.",
    )
    parser.add_argument(
        "--clip-quantile",
        type=float,
        default=0.98,
        help="Upper quantile used to cap the color scale, so that a few outliers "
        "do not wash out the rest of the map.",
    )
    return parser.parse_args()


def load_average_density(db_path: Path, simulation_id: int | None):
    """Return the simulation id and the mean density of each street over the day.

    The average is computed directly by SQLite, so the millions of rows
    stored in road_data are never loaded in memory.
    """
    with sqlite3.connect(db_path) as conn:
        if simulation_id is None:
            simulation_id = conn.execute(
                f"SELECT MAX({SIM_COL}) FROM {TABLE}"
            ).fetchone()[0]
            if simulation_id is None:
                raise RuntimeError(f"Table '{TABLE}' is empty: no data was saved.")
        print(f"Using simulation id {simulation_id}")

        data = pd.read_sql_query(
            f"SELECT {ID_COL} AS street_id, AVG({DENSITY_COL}) AS avg_density, "
            f"COUNT(*) AS n_steps FROM {TABLE} WHERE {SIM_COL} = ? GROUP BY {ID_COL}",
            conn,
            params=(simulation_id,),
        )

    if data.empty:
        raise RuntimeError(f"No data found for simulation id {simulation_id}.")

    print(
        f"Computed the average density of {len(data)} streets "
        f"over {int(data['n_steps'].max())} time steps."
    )
    return simulation_id, data.set_index("street_id")["avg_density"]


def clean_name(name) -> str:
    """Make OSM street names readable (underscores -> spaces, handle missing)."""
    if name is None or (isinstance(name, float) and pd.isna(name)) or name == "":
        return "Unnamed street"
    return str(name).replace("_", " ")


def build_panel(title: str, subtitle: str, top_rows: list[tuple[str, float, float | None]]):
    """Build the HTML overlays (title box and top-N ranking) as one folium Element."""
    rows = "".join(
        f"<tr><td style='padding:2px 8px 2px 0;color:#9aa3b2'>{i}.</td>"
        f"<td style='padding:2px 8px 2px 0'>{html.escape(name)}</td>"
        f"<td style='padding:2px 0;text-align:right;font-weight:600'>{dens:.1f}</td></tr>"
        for i, (name, dens, _) in enumerate(top_rows, start=1)
    )
    css = (
        "font-family:'Helvetica Neue',Arial,sans-serif;background:rgba(20,24,32,0.88);"
        "color:#f1f3f7;border-radius:10px;box-shadow:0 2px 12px rgba(0,0,0,0.45);"
        "z-index:9999;position:fixed;"
    )
    return folium.Element(
        f"""
        <div style="{css}top:12px;left:56px;padding:10px 16px;">
            <div style="font-size:16px;font-weight:700">{html.escape(title)}</div>
            <div style="font-size:12px;color:#9aa3b2;margin-top:2px">{html.escape(subtitle)}</div>
        </div>
        <div style="{css}bottom:32px;left:12px;padding:10px 14px;font-size:12px;max-width:320px">
            <div style="font-size:13px;font-weight:700;margin-bottom:6px">
                Most used streets <span style="font-weight:400;color:#9aa3b2">({DENSITY_UNIT})</span>
            </div>
            <table style="border-collapse:collapse">{rows}</table>
        </div>
        """
    )


def main():
    args = parse_args()

    if not args.db.exists():
        raise FileNotFoundError(
            f"Database not found at {args.db.resolve()}. Run 1_trial_simulation.py first."
        )
    if not args.edges.exists():
        raise FileNotFoundError(
            f"Edges GeoJSON not found at {args.edges.resolve()}. "
            "Run 0_build_venezia_geojson.py first."
        )
    if not 0 < args.clip_quantile <= 1:
        raise ValueError("--clip-quantile must be in the interval (0, 1].")

    simulation_id, avg_density = load_average_density(args.db, args.simulation_id)

    # Load the geometries and attach the average density to each street
    gdf = gpd.read_file(args.edges).set_geometry("geometry")
    gdf["id"] = gdf["id"].astype(int)
    gdf = gdf.merge(avg_density, left_on="id", right_index=True, how="left")
    gdf["avg_density"] = gdf["avg_density"].fillna(0.0)

    # Folium needs WGS84 coordinates
    if gdf.crs is not None and gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs(epsg=4326)

    # Human readable fields
    gdf["street"] = (
        gdf["name"].map(clean_name) if "name" in gdf.columns else "Unnamed street"
    )
    has_length = "length" in gdf.columns
    if has_length:
        # Average number of pedestrians simultaneously on the street
        gdf["avg_pedestrians"] = gdf["avg_density"] * gdf["length"] / 1000.0
        gdf["length_m"] = gdf["length"].round(0).astype(int)

    used_mask = gdf["avg_density"] > 0
    n_used = int(used_mask.sum())
    print(f"{n_used} streets used, {len(gdf) - n_used} never used.")

    # Console summary of the most used streets
    top_df = gdf.sort_values("avg_density", ascending=False).head(args.top)
    print(f"\nTop {args.top} most used streets (average density, {DENSITY_UNIT}):")
    print(
        top_df[["id", "street", "avg_density"]]
        .rename(columns={"avg_density": f"avg_density [{DENSITY_UNIT}]"})
        .to_string(index=False)
    )

    # Color scale capped at a high quantile to keep the contrast readable
    vmax = float(gdf.loc[used_mask, "avg_density"].quantile(args.clip_quantile))
    if vmax <= 0:
        vmax = float(gdf["avg_density"].max()) or 1.0
    colormap = cm.LinearColormap(
        COLORMAP_COLORS,
        vmin=0.0,
        vmax=vmax,
        caption=(
            f"Average density over the simulated day [{DENSITY_UNIT}] "
            f"(scale capped at the {args.clip_quantile:.0%} quantile)"
        ),
    )

    # Map with API-key-free basemaps
    minx, miny, maxx, maxy = gdf.total_bounds
    fmap = folium.Map(
        location=[(miny + maxy) / 2, (minx + maxx) / 2],
        zoom_start=14,
        tiles=None,
        control_scale=True,
    )
    for i, (name, url, attr) in enumerate(BASEMAPS):
        folium.TileLayer(
            tiles=url,
            attr=attr,
            name=name,
            subdomains="abcd" if "cartocdn" in url else "abc",
            max_zoom=19,
            show=(i == 0),
        ).add_to(fmap)
    fmap.fit_bounds([[miny, minx], [maxy, maxx]])

    # Tooltip configuration
    tooltip_fields = ["street", "avg_density"]
    tooltip_aliases = ["Street", f"Avg density [{DENSITY_UNIT}]"]
    if has_length:
        tooltip_fields += ["avg_pedestrians", "length_m"]
        tooltip_aliases += ["Avg pedestrians on street", "Length [m]"]
    tooltip_fields.append("id")
    tooltip_aliases.append("Street id")

    gdf["avg_density"] = gdf["avg_density"].round(2)
    if has_length:
        gdf["avg_pedestrians"] = gdf["avg_pedestrians"].round(2)

    def make_tooltip():
        return folium.GeoJsonTooltip(
            fields=tooltip_fields,
            aliases=tooltip_aliases,
            sticky=True,
            style=(
                "font-family:'Helvetica Neue',Arial,sans-serif;font-size:12px;"
                "background-color:rgba(20,24,32,0.92);color:#f1f3f7;"
                "border:none;border-radius:6px;padding:6px 10px;"
            ),
        )

    def highlight(_):
        return {"weight": 8, "color": "#ffffff", "opacity": 1.0}

    plot_cols = list(dict.fromkeys(tooltip_fields)) + ["geometry"]

    # Layer 1: streets never used (thin, faint)
    unused = gdf.loc[~used_mask, plot_cols]
    if not unused.empty:
        folium.GeoJson(
            unused,
            name="Unused streets",
            style_function=lambda _: {"color": UNUSED_COLOR, "weight": 1.0, "opacity": 0.45},
            tooltip=make_tooltip(),
            highlight_function=highlight,
        ).add_to(fmap)

    # Layer 2: used streets, sorted so that the busiest are drawn on top
    used = gdf.loc[used_mask, plot_cols].sort_values("avg_density")

    def used_style(feature):
        value = feature["properties"].get("avg_density") or 0.0
        norm = min(value / vmax, 1.0)
        return {
            "color": colormap(min(value, vmax)),
            "weight": 2.0 + 4.0 * norm,
            "opacity": 0.65 + 0.35 * norm,
        }

    folium.GeoJson(
        used,
        name="Average density",
        style_function=used_style,
        tooltip=make_tooltip(),
        highlight_function=highlight,
    ).add_to(fmap)

    # Colorbar, extra controls and info panels
    colormap.add_to(fmap)
    Fullscreen().add_to(fmap)
    MiniMap(toggle_display=True, minimized=True, position="bottomright").add_to(fmap)
    folium.LayerControl(collapsed=False).add_to(fmap)

    top_rows = [
        (row["street"], row["avg_density"], row.get("avg_pedestrians"))
        for _, row in top_df.iterrows()
    ]
    fmap.get_root().html.add_child(
        build_panel(
            title="Pedestrian density - Venezia",
            subtitle=f"Average over the simulated day | simulation {simulation_id} | "
            f"{n_used}/{len(gdf)} streets used",
            top_rows=top_rows,
        )
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fmap.save(args.output.as_posix())
    print(f"\nMap saved to {args.output.resolve()}")


if __name__ == "__main__":
    main()