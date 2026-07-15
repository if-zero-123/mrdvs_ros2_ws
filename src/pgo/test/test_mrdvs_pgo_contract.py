import ast
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
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
        'src/pgo/src/pgo_outputs.cpp',
        'src/pgo/src/pgo_outputs.h',
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
        'optimized_map_resolution': 0.1,
        'optimized_map_publish_period': 1.0,
    }


def test_mrdvs_pgo_full_launch_exposes_the_isolated_pipeline_contract():
    relative_path = 'src/pgo/launch/mrdvs_pgo_full_launch.py'
    path = WORKSPACE_ROOT / relative_path
    assert path.is_file(), f'missing PGO launch file: {relative_path}'

    source = path.read_text()
    for required_token in (
        "FindPackageShare('lx_camera_ros')",
        "FindPackageShare('fastlio2')",
        "FindPackageShare('pgo')",
        "'mrdvs_pgo.yaml'",
        "'mrdvs.yaml'",
        "package='pgo'",
        "package='rviz2'",
        "'mrdvs_pgo_optimized.rviz'",
    ):
        assert required_token in source
    assert 'fast_livo' not in source

    spec = importlib.util.spec_from_file_location('mrdvs_pgo_full_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    defaults = {
        entity.name: perform_substitutions(context, entity.default_value)
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert defaults == {
        'enable_rviz': 'true',
        'camera_ip': '192.168.100.82',
        'fastlio_delay': '3.0',
    }


def test_pgo_full_launch_isolates_fastlio_child_launch_arguments():
    path = WORKSPACE_ROOT / 'src/pgo/launch/mrdvs_pgo_full_launch.py'
    tree = ast.parse(path.read_text())
    timer_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'TimerAction'
    ]
    assert len(timer_calls) == 1

    actions_keyword = next(
        keyword
        for keyword in timer_calls[0].keywords
        if keyword.arg == 'actions'
    )
    assert isinstance(actions_keyword.value, ast.List)
    first_action = actions_keyword.value.elts[0]
    assert isinstance(first_action, ast.Call)
    assert isinstance(first_action.func, ast.Name)
    assert first_action.func.id == 'GroupAction'
    scoped_keyword = next(
        keyword for keyword in first_action.keywords if keyword.arg == 'scoped'
    )
    assert isinstance(scoped_keyword.value, ast.Constant)
    assert scoped_keyword.value.value is True


def test_pgo_message_time_guard_has_a_deterministic_initial_value():
    source = (WORKSPACE_ROOT / 'src/pgo/src/pgo_node.cpp').read_text()
    assert 'double last_message_time = 0.0;' in source


def test_pgo_publishes_corrected_pose_and_path_topics():
    source = (WORKSPACE_ROOT / 'src/pgo/src/pgo_node.cpp').read_text()

    for topic in (
        '/pgo/optimized_odom',
        '/pgo/optimized_path',
    ):
        assert topic in source


def test_pgo_publishes_a_rate_limited_optimized_map():
    source = (WORKSPACE_ROOT / 'src/pgo/src/pgo_node.cpp').read_text()

    assert '/pgo/optimized_map' in source
    assert 'optimized_map_resolution' in source
    assert 'optimized_map_publish_period' in source


def test_optimized_pgo_rviz_shows_live_scan_and_corrected_outputs():
    rviz_path = WORKSPACE_ROOT / 'src/pgo/rviz/mrdvs_pgo_optimized.rviz'
    assert rviz_path.is_file()
    source = rviz_path.read_text()

    for required_topic in (
        '/fastlio2/body_cloud',
        '/pgo/optimized_odom',
        '/pgo/optimized_path',
        '/pgo/optimized_map',
        '/pgo/loop_markers',
    ):
        assert required_topic in source

    assert 'Class: rviz_default_plugins/Odometry' in source
    assert 'Fixed Frame: map' in source
    assert 'Reference Frame: mrdvs_imu' in source
    assert '/fastlio2/world_cloud' not in source
    assert '/fastlio2/lio_path' not in source


def test_readme_documents_the_corrected_pgo_visualization_topics():
    source = (WORKSPACE_ROOT / 'README.md').read_text()

    for topic in (
        '/pgo/optimized_odom',
        '/pgo/optimized_path',
        '/pgo/optimized_map',
    ):
        assert topic in source
