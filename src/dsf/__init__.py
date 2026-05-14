import sys

from dsf_cpp import (
    __version__ as __version__,
    logging as logging,
    mobility as mobility,
    mdt as mdt,
)

from .cartography import (
    get_cartography as get_cartography,
    graph_from_gdfs as graph_from_gdfs,
    graph_to_gdfs as graph_to_gdfs,
    create_manhattan_cartography as create_manhattan_cartography,
    to_folium_map as to_folium_map,
)

sys.modules.setdefault(__name__ + ".mobility", mobility)
sys.modules.setdefault(__name__ + ".mdt", mdt)
