import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


SCRIPT_PATH = Path(__file__).parents[1] / 'scripts' / 'mrdvs_imu_diagnostics.py'
SPEC = importlib.util.spec_from_file_location('mrdvs_imu_diagnostics', SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def add_stationary_sample(analyzer, timestamp, arrival):
    analyzer.add_sample(
        sensor_timestamp=timestamp,
        arrival_elapsed=arrival,
        acceleration=(0.0, 9.81, 0.0),
        angular_velocity=(0.0, 0.0, 0.0),
    )


def test_duplicate_is_dropped_without_resetting_new_initialization():
    analyzer = MODULE.ImuDiagnosticsAnalyzer(required_samples=4)

    add_stationary_sample(analyzer, 10.000, 0.000)
    add_stationary_sample(analyzer, 10.005, 0.005)
    add_stationary_sample(analyzer, 10.005, 0.010)
    add_stationary_sample(analyzer, 10.010, 0.015)
    add_stationary_sample(analyzer, 10.015, 0.020)

    summary = analyzer.build_summary()
    assert summary['duplicate_count'] == 1
    assert summary['old_policy_completed'] is False
    assert summary['new_policy_completed'] is True
    assert summary['old_policy_timestamp_resets'] == 1
    assert summary['new_policy_timestamp_resets'] == 0
    assert analyzer.rows[2].action == 'drop_without_reset'


def test_large_backward_jump_resets_new_stream_and_old_progress():
    analyzer = MODULE.ImuDiagnosticsAnalyzer(required_samples=4, max_gap_sec=0.2)

    add_stationary_sample(analyzer, 10.000, 0.000)
    add_stationary_sample(analyzer, 10.005, 0.005)
    add_stationary_sample(analyzer, 9.700, 0.010)

    summary = analyzer.build_summary()
    assert summary['large_jump_count'] == 1
    assert summary['old_policy_timestamp_resets'] == 1
    assert summary['old_policy_progress_samples'] == 0
    assert summary['new_policy_timestamp_resets'] == 1
    assert summary['new_policy_progress_samples'] == 1
    assert analyzer.rows[-1].action == 'reset_stream'


def test_old_policy_keeps_watermark_after_large_backward_drop():
    analyzer = MODULE.ImuDiagnosticsAnalyzer(required_samples=3, max_gap_sec=0.2)

    add_stationary_sample(analyzer, 10.0, 0.0)
    add_stationary_sample(analyzer, 10.1, 0.1)
    add_stationary_sample(analyzer, 9.7, 0.2)
    add_stationary_sample(analyzer, 9.8, 0.3)
    add_stationary_sample(analyzer, 9.9, 0.4)
    add_stationary_sample(analyzer, 10.2, 0.5)

    summary = analyzer.build_summary()
    assert summary['old_policy_timestamp_resets'] == 3
    assert summary['old_policy_progress_samples'] == 1
    assert summary['old_policy_completed'] is False
    assert summary['new_policy_timestamp_resets'] == 1
    assert summary['new_policy_completed'] is True


@pytest.mark.parametrize(
    'kwargs',
    [
        {'gravity_magnitude': 0.0},
        {'gravity_magnitude': math.nan},
        {'max_gyro_norm': -0.1},
        {'max_gyro_norm': math.inf},
        {'max_acc_norm_error': -0.1},
        {'max_acc_norm_error': math.nan},
        {'max_gap_sec': math.inf},
    ],
)
def test_analyzer_rejects_nonfinite_or_out_of_range_configuration(kwargs):
    with pytest.raises(ValueError):
        MODULE.ImuDiagnosticsAnalyzer(**kwargs)


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('duration', 0.0),
        ('duration', -1.0),
        ('duration', math.nan),
        ('duration', math.inf),
        ('wait_timeout', 0.0),
        ('wait_timeout', math.nan),
    ],
)
def test_collection_rejects_invalid_capture_timing(field, value):
    args = SimpleNamespace(
        topic='/lx_camera_node/LxCamera_Imu', duration=30.0, wait_timeout=10.0
    )
    setattr(args, field, value)

    with pytest.raises(ValueError):
        MODULE.validate_collection_args(args)


def test_parse_args_reports_invalid_numeric_parameter(monkeypatch, capsys):
    monkeypatch.setattr(
        sys,
        'argv',
        [
            'mrdvs_imu_diagnostics',
            '--output-dir',
            '/tmp/unused',
            '--duration',
            'nan',
        ],
    )

    with pytest.raises(SystemExit) as error:
        MODULE.parse_args()

    assert error.value.code == 2
    assert 'duration must be finite and positive' in capsys.readouterr().err


def test_output_files_include_summary_samples_and_chart(tmp_path):
    analyzer = MODULE.ImuDiagnosticsAnalyzer(required_samples=4)
    for index in range(8):
        add_stationary_sample(analyzer, 10.0 + index * 0.005, index * 0.005)

    paths = MODULE.write_outputs(analyzer, tmp_path)

    assert paths['summary'].is_file()
    assert paths['samples'].is_file()
    assert paths['chart'].is_file()
    assert paths['chart'].stat().st_size > 0
    summary = json.loads(paths['summary'].read_text(encoding='utf-8'))
    assert summary['message_count'] == 8
    assert summary['new_policy_completed'] is True
