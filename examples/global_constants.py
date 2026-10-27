import os
from pathlib import Path
# ============================================================================
# DIRECTORY PATHS (relative to pipeline/ directory)
# ============================================================================

# Get current username
CURRENT_USER = os.getenv("USER")  # Current system username
DATA_01 = Path("data")  # Base directory for raw data files (absolute path)
DATA01_DIR_BASE = Path(
    DATA_01
)  # Base directory for raw data files (absolute path)

# COMMON_FOLDER = DATA01_DIR_BASE / "common"  # Common files directory (absolute path)
COMMON_FOLDER = Path("data")  # Common files directory (relative path)

WORK_FOLDER = DATA01_DIR_BASE # Output files directory

SCRIPTS_DIR = Path("scripts")  # Directory for pipeline scripts (relative path)


DEFAULT_CRS = "EPSG:4326"  # Default Coordinate Reference System for geospatial data (WGS 84)
