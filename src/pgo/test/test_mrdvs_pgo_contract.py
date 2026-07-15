from pathlib import Path

import pytest
import yaml


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize(
    'relative_path',
    [
        'src/interface/package.xml',
        'src/interface/srv/SaveMaps.srv',
        'src/pgo/package.xml',
        'src/pgo/LICENSE',
        'src/pgo/src/pgo_node.cpp',
        'src/pgo/src/pgos/simple_pgo.cpp',
    ],
)
def test_upstream_pgo_package_layout_is_present(relative_path):
    path = WORKSPACE_ROOT / relative_path
    assert path.is_file(), f'missing upstream PGO file: {relative_path}'


def load_workspace_yaml(relative_path):
    path = WORKSPACE_ROOT / relative_path
    assert path.is_file(), f'missing YAML file: {relative_path}'
    return yaml.safe_load(path.read_text())


def test_fastlio2_pgo_profile_only_changes_the_world_frame():
    baseline = load_workspace_yaml('src/fastlio2/config/mrdvs.yaml')
    pgo_profile = load_workspace_yaml('src/fastlio2/config/mrdvs_pgo.yaml')

    assert baseline['world_frame'] == 'map'
    assert pgo_profile['world_frame'] == 'lio_local'
    assert {**pgo_profile, 'world_frame': 'map'} == baseline


def test_mrdvs_pgo_configuration_matches_the_online_loop_contract():
    config = load_workspace_yaml('src/pgo/config/mrdvs.yaml')

    assert config == {
        'cloud_topic': '/fastlio2/body_cloud',
        'odom_topic': '/fastlio2/lio_odom',
        'map_frame': 'map',
        'local_frame': 'lio_local',
        'key_pose_delta_deg': 10,
        'key_pose_delta_trans': 0.5,
        'loop_search_radius': 1.0,
        'loop_time_tresh': 60.0,
        'loop_score_tresh': 0.15,
        'loop_submap_half_range': 5,
        'submap_resolution': 0.1,
        'min_loop_detect_duration': 5.0,
    }
