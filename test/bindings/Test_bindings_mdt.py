import numpy as np
import pytest

from dsf import mdt

EXPECTED_COLUMNS = [
    "uid",
    "trajectory_id",
    "lon",
    "lat",
    "timestamp_in",
    "timestamp_out",
]


class InvalidShapeFrame:
    columns = ["uid", "timestamp", "lat", "lon"]

    def to_numpy(self):
        return np.array([1.0, 2.0, 3.0], dtype=np.float64)


def test_trajectory_collection_from_pandas_to_pandas(trajectory_pandas_df):
    collection = mdt.TrajectoryCollection(trajectory_pandas_df)
    collection.filter(0.5, 150.0, 0)

    result = collection.to_pandas()

    assert list(result.columns) == EXPECTED_COLUMNS
    assert len(result) > 0
    assert {1, 2}.issubset(set(result["uid"].astype(int).tolist()))


def test_trajectory_collection_from_polars_to_polars(trajectory_polars_df):
    polars = pytest.importorskip("polars")

    collection = mdt.TrajectoryCollection(trajectory_polars_df)
    collection.filter(0.5, 150.0, 0)

    result = collection.to_polars()

    assert isinstance(result, polars.DataFrame)
    assert result.columns == EXPECTED_COLUMNS
    assert result.height > 0
    assert {1, 2}.issubset(set(result["uid"].cast(polars.Int64).to_list()))


def test_trajectory_collection_to_csv_writes_expected_header(
    trajectory_pandas_df, tmp_path
):
    collection = mdt.TrajectoryCollection(trajectory_pandas_df)
    collection.filter(0.5, 150.0, 0)

    output_file = tmp_path / "trajectory_export.csv"
    collection.to_csv(str(output_file))

    assert output_file.exists()

    header = output_file.read_text(encoding="utf-8").splitlines()[0]
    assert header == "uid;trajectory_id;lon;lat;timestamp_in;timestamp_out"


def test_trajectory_collection_rejects_non_2d_numpy_input():
    with pytest.raises(RuntimeError, match="2D numpy array"):
        mdt.TrajectoryCollection(InvalidShapeFrame())
