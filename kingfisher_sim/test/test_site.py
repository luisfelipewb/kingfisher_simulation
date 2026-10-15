# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""kingfisher_localization's lac_symphonie site matches the lake world: same datum, same points."""

import os

from ament_index_python.packages import get_package_share_directory
from lac_symphonie.geo import read_origin_from_sdf
import pytest
import yaml


def _yaml(package, *path):
    with open(os.path.join(get_package_share_directory(package), *path)) as f:
        return yaml.safe_load(f)


SITE = _yaml('kingfisher_localization', 'config', 'sites', 'lac_symphonie.yaml')


def test_datum_is_world_origin():
    sdf = os.path.join(get_package_share_directory('lac_symphonie'), 'worlds', 'lac_symphonie.sdf')
    d = SITE['datum']
    assert (d['latitude'], d['longitude'], d['altitude']) == pytest.approx(
        read_origin_from_sdf(sdf), abs=1e-9)


def test_points_match_lake():
    assert SITE['points'] == _yaml('lac_symphonie', 'config', 'geo_points.yaml')['points']
