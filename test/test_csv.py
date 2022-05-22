import pytest
from rtk_mapper.csv import CSVFormat, EUFSFormat, RTKFormat, InvalidCSVFormat
from rtk_mapper.map import MarkerType


class TestCSV:
    def test_get_marker_type(self):
        assert CSVFormat.get_marker_type("blue") == MarkerType.BLUE
        assert CSVFormat.get_marker_type("yellow") == MarkerType.YELLOW
        assert CSVFormat.get_marker_type("orange") == MarkerType.ORANGE
        assert CSVFormat.get_marker_type("big_orange") == MarkerType.BIG_ORANGE
        assert CSVFormat.get_marker_type("car_start") == MarkerType.CAR_START

    def test_get_marker_str(self):
        assert CSVFormat.get_marker_str(MarkerType.BLUE) == "blue"
        assert CSVFormat.get_marker_str(MarkerType.YELLOW) == "yellow"
        assert CSVFormat.get_marker_str(MarkerType.ORANGE) == "orange"
        assert CSVFormat.get_marker_str(MarkerType.BIG_ORANGE) == "big_orange"
        assert CSVFormat.get_marker_str(MarkerType.CAR_START) == "car_start"
