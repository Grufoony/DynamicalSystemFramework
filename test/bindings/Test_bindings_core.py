import dsf
from dsf import logging
import dsf_cpp
import pytest


def test_root_module_reexports_cpp_bindings():
    assert dsf.__version__ == dsf_cpp.__version__
    assert dsf.mobility is dsf_cpp.mobility
    assert dsf.mdt is dsf_cpp.mdt
    assert hasattr(dsf_cpp, "Measurement")


def test_log_level_round_trip():
    logging.set_level(logging.LogLevel.DEBUG)

    logging.set_level(logging.LogLevel.INFO)


def test_log_to_file_accepts_string_path(tmp_path):
    logging.to_file(str(tmp_path / "dsf_bindings.log"))


def test_measurement_properties_are_mutable():
    measurement = dsf_cpp.Measurement(12.5, 3.0, 10)

    assert measurement.mean == pytest.approx(12.5)
    assert measurement.std == pytest.approx(3.0)
    assert measurement.n == 10
    assert isinstance(measurement.is_valid, bool)

    measurement.mean = 7.25
    measurement.std = 1.5
    measurement.n = 4
    measurement.is_valid = False

    assert measurement.mean == pytest.approx(7.25)
    assert measurement.std == pytest.approx(1.5)
    assert measurement.n == 4
    assert measurement.is_valid is False
