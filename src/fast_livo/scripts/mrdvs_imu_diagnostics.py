#!/usr/bin/env python3

import argparse
import csv
import json
import math
from pathlib import Path
import time
from typing import NamedTuple

import matplotlib.pyplot as plt
import numpy as np

plt.switch_backend('Agg')


class SampleRow(NamedTuple):
    arrival_elapsed: float
    sensor_timestamp: float
    dt_sec: float
    action: str
    gyro_norm: float
    acc_norm: float
    stationary: bool
    old_progress_samples: int
    new_progress_samples: int


class ImuDiagnosticsAnalyzer:
    def __init__(
        self,
        required_samples=600,
        gravity_magnitude=9.81,
        max_gyro_norm=0.10,
        max_acc_norm_error=0.75,
        max_gap_sec=0.2,
    ):
        required_samples = int(required_samples)
        gravity_magnitude = float(gravity_magnitude)
        max_gyro_norm = float(max_gyro_norm)
        max_acc_norm_error = float(max_acc_norm_error)
        max_gap_sec = float(max_gap_sec)

        if required_samples <= 0:
            raise ValueError('required_samples must be positive')
        if not math.isfinite(gravity_magnitude) or gravity_magnitude <= 0.0:
            raise ValueError('gravity_magnitude must be finite and positive')
        if not math.isfinite(max_gyro_norm) or max_gyro_norm < 0.0:
            raise ValueError('max_gyro_norm must be finite and non-negative')
        if not math.isfinite(max_acc_norm_error) or max_acc_norm_error < 0.0:
            raise ValueError('max_acc_norm_error must be finite and non-negative')
        if not math.isfinite(max_gap_sec) or max_gap_sec <= 0.0:
            raise ValueError('max_gap_sec must be finite and positive')
        self.required_samples = required_samples
        self.gravity_magnitude = gravity_magnitude
        self.max_gyro_norm = max_gyro_norm
        self.max_acc_norm_error = max_acc_norm_error
        self.max_gap_sec = max_gap_sec
        self.rows = []
        self.old_last_timestamp = None
        self.new_last_timestamp = None
        self.duplicate_count = 0
        self.rollback_count = 0
        self.large_jump_count = 0
        self.nonfinite_count = 0
        self.gyro_threshold_count = 0
        self.acc_threshold_count = 0
        self.old_policy_timestamp_resets = 0
        self.new_policy_timestamp_resets = 0
        self.old_policy_sensor_resets = 0
        self.new_policy_sensor_resets = 0
        self.old_progress = 0
        self.new_progress = 0
        self.old_max_progress = 0
        self.new_max_progress = 0
        self.old_policy_completed = False
        self.new_policy_completed = False
        self.old_completion_elapsed = None
        self.new_completion_elapsed = None

    @staticmethod
    def _norm(values):
        return math.sqrt(sum(value * value for value in values))

    def _classify_old_timestamp(self, timestamp):
        if self.old_last_timestamp is None or self.old_last_timestamp <= 0.0:
            return math.nan, 'accept'
        dt_sec = timestamp - self.old_last_timestamp
        if dt_sec <= 0.0:
            return dt_sec, 'drop_and_reset'
        if dt_sec > self.max_gap_sec:
            return dt_sec, 'reset_stream'
        return dt_sec, 'accept'

    def _classify_new_timestamp(self, timestamp):
        if self.new_last_timestamp is None or self.new_last_timestamp <= 0.0:
            return math.nan, 'accept'
        dt_sec = timestamp - self.new_last_timestamp
        if abs(dt_sec) > self.max_gap_sec:
            return dt_sec, 'reset_stream'
        if dt_sec <= 0.0:
            return dt_sec, 'drop_without_reset'
        return dt_sec, 'accept'

    def _update_policy_progress(self, old_action, new_action, stationary, arrival_elapsed):
        if not self.old_policy_completed:
            if old_action != 'accept':
                self.old_progress = 0
                self.old_policy_timestamp_resets += 1
            elif not stationary:
                self.old_progress = 0
                self.old_policy_sensor_resets += 1
            else:
                self.old_progress += 1

            if old_action == 'reset_stream' and stationary:
                self.old_progress = 1

            if self.old_progress >= self.required_samples:
                self.old_progress = self.required_samples
                self.old_policy_completed = True
                self.old_completion_elapsed = arrival_elapsed

        if not self.new_policy_completed:
            if new_action == 'reset_stream':
                self.new_progress = 0
                self.new_policy_timestamp_resets += 1
                if stationary:
                    self.new_progress = 1
            elif new_action == 'drop_without_reset':
                pass
            elif not stationary:
                self.new_progress = 0
                self.new_policy_sensor_resets += 1
            else:
                self.new_progress += 1

            if self.new_progress >= self.required_samples:
                self.new_progress = self.required_samples
                self.new_policy_completed = True
                self.new_completion_elapsed = arrival_elapsed

        self.old_max_progress = max(self.old_max_progress, self.old_progress)
        self.new_max_progress = max(self.new_max_progress, self.new_progress)

    def add_sample(self, sensor_timestamp, arrival_elapsed, acceleration, angular_velocity):
        values = (sensor_timestamp, arrival_elapsed, *acceleration, *angular_velocity)
        finite = all(math.isfinite(value) for value in values)
        gyro_norm = self._norm(angular_velocity) if finite else math.nan
        acc_norm = self._norm(acceleration) if finite else math.nan
        _, old_action = self._classify_old_timestamp(sensor_timestamp)
        timestamp_dt, new_action = self._classify_new_timestamp(sensor_timestamp)

        if self.new_last_timestamp is not None and self.new_last_timestamp > 0.0:
            if timestamp_dt == 0.0:
                self.duplicate_count += 1
            if timestamp_dt < 0.0:
                self.rollback_count += 1
            if abs(timestamp_dt) > self.max_gap_sec:
                self.large_jump_count += 1

        if not finite:
            self.nonfinite_count += 1
        gyro_over = finite and gyro_norm > self.max_gyro_norm
        acc_over = finite and abs(acc_norm - self.gravity_magnitude) > self.max_acc_norm_error
        self.gyro_threshold_count += int(gyro_over)
        self.acc_threshold_count += int(acc_over)
        stationary = finite and not gyro_over and not acc_over

        self._update_policy_progress(old_action, new_action, stationary, arrival_elapsed)

        self.rows.append(
            SampleRow(
                arrival_elapsed=float(arrival_elapsed),
                sensor_timestamp=float(sensor_timestamp),
                dt_sec=float(timestamp_dt),
                action=new_action,
                gyro_norm=float(gyro_norm),
                acc_norm=float(acc_norm),
                stationary=stationary,
                old_progress_samples=self.old_progress,
                new_progress_samples=self.new_progress,
            )
        )

        if old_action != 'drop_and_reset':
            self.old_last_timestamp = sensor_timestamp
        if new_action != 'drop_without_reset':
            self.new_last_timestamp = sensor_timestamp

    @staticmethod
    def _distribution(values, prefix, scale=1.0):
        finite_values = np.asarray(
            [value * scale for value in values if math.isfinite(value)], dtype=float
        )
        if finite_values.size == 0:
            return {
                f'{prefix}_min': None,
                f'{prefix}_mean': None,
                f'{prefix}_median': None,
                f'{prefix}_p95': None,
                f'{prefix}_p99': None,
                f'{prefix}_max': None,
            }
        return {
            f'{prefix}_min': float(np.min(finite_values)),
            f'{prefix}_mean': float(np.mean(finite_values)),
            f'{prefix}_median': float(np.median(finite_values)),
            f'{prefix}_p95': float(np.percentile(finite_values, 95)),
            f'{prefix}_p99': float(np.percentile(finite_values, 99)),
            f'{prefix}_max': float(np.max(finite_values)),
        }

    def build_summary(self):
        message_count = len(self.rows)
        duration = 0.0
        if message_count > 1:
            duration = self.rows[-1].arrival_elapsed - self.rows[0].arrival_elapsed
        transition_count = max(message_count - 1, 0)
        observed_rate = transition_count / duration if duration > 0.0 else None
        positive_intervals = [row.dt_sec for row in self.rows if row.dt_sec > 0.0]
        gyro_norms = [row.gyro_norm for row in self.rows]
        acc_norms = [row.acc_norm for row in self.rows]
        summary = {
            'message_count': message_count,
            'transition_count': transition_count,
            'capture_duration_sec': duration,
            'observed_rate_hz': observed_rate,
            'duplicate_count': self.duplicate_count,
            'duplicate_rate_percent': (
                self.duplicate_count / transition_count * 100.0 if transition_count else 0.0
            ),
            'rollback_count': self.rollback_count,
            'large_jump_count': self.large_jump_count,
            'nonfinite_count': self.nonfinite_count,
            'gyro_threshold_count': self.gyro_threshold_count,
            'acc_threshold_count': self.acc_threshold_count,
            'old_policy_timestamp_resets': self.old_policy_timestamp_resets,
            'new_policy_timestamp_resets': self.new_policy_timestamp_resets,
            'old_policy_sensor_resets': self.old_policy_sensor_resets,
            'new_policy_sensor_resets': self.new_policy_sensor_resets,
            'old_policy_completed': self.old_policy_completed,
            'new_policy_completed': self.new_policy_completed,
            'old_policy_completion_sec': self.old_completion_elapsed,
            'new_policy_completion_sec': self.new_completion_elapsed,
            'old_policy_progress_samples': self.old_progress,
            'new_policy_progress_samples': self.new_progress,
            'old_policy_max_progress_samples': self.old_max_progress,
            'new_policy_max_progress_samples': self.new_max_progress,
            'required_samples': self.required_samples,
            'max_gap_sec': self.max_gap_sec,
            'max_gyro_norm': self.max_gyro_norm,
            'max_acc_norm_error': self.max_acc_norm_error,
        }
        summary.update(self._distribution(positive_intervals, 'positive_dt_ms', 1000.0))
        summary.update(self._distribution(gyro_norms, 'gyro_norm'))
        summary.update(self._distribution(acc_norms, 'acc_norm'))
        return summary


def write_outputs(analyzer, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / 'summary.json'
    samples_path = output_dir / 'samples.csv'
    chart_path = output_dir / 'imu_diagnostics.png'

    summary = analyzer.build_summary()
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
    )

    with samples_path.open('w', newline='', encoding='utf-8') as output_file:
        writer = csv.writer(output_file)
        writer.writerow(SampleRow._fields)
        writer.writerows(analyzer.rows)

    _write_chart(analyzer, summary, chart_path)
    return {'summary': summary_path, 'samples': samples_path, 'chart': chart_path}


def _write_chart(analyzer, summary, chart_path):
    elapsed = np.asarray([row.arrival_elapsed for row in analyzer.rows], dtype=float)
    dt_ms = np.asarray([row.dt_sec * 1000.0 for row in analyzer.rows], dtype=float)
    gyro_norm = np.asarray([row.gyro_norm for row in analyzer.rows], dtype=float)
    acc_norm = np.asarray([row.acc_norm for row in analyzer.rows], dtype=float)
    old_progress = np.asarray(
        [row.old_progress_samples / analyzer.required_samples * 100.0 for row in analyzer.rows]
    )
    new_progress = np.asarray(
        [row.new_progress_samples / analyzer.required_samples * 100.0 for row in analyzer.rows]
    )
    duplicate_mask = np.asarray([row.dt_sec == 0.0 for row in analyzer.rows], dtype=bool)
    rollback_mask = np.asarray([row.dt_sec < 0.0 for row in analyzer.rows], dtype=bool)

    figure, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True, constrained_layout=True)
    axes[0].plot(elapsed, dt_ms, linewidth=0.7, color='#3366cc', label='timestamp dt')
    if duplicate_mask.any():
        axes[0].scatter(
            elapsed[duplicate_mask],
            dt_ms[duplicate_mask],
            color='#dc3912',
            s=24,
            label='duplicate',
        )
    if rollback_mask.any():
        axes[0].scatter(
            elapsed[rollback_mask], dt_ms[rollback_mask], color='#ff9900', s=24, label='rollback'
        )
    axes[0].axhline(5.0, color='#109618', linestyle='--', linewidth=1.0, label='5 ms nominal')
    axes[0].set_ylabel('Timestamp dt (ms)')
    axes[0].legend(loc='upper right')
    axes[0].grid(alpha=0.25)

    axes[1].plot(elapsed, old_progress, color='#dc3912', label='old strict policy')
    axes[1].plot(elapsed, new_progress, color='#109618', label='fixed policy')
    axes[1].set_ylabel('Initialization (%)')
    axes[1].set_ylim(-2.0, 102.0)
    axes[1].legend(loc='lower right')
    axes[1].grid(alpha=0.25)

    axes[2].plot(elapsed, gyro_norm, linewidth=0.7, color='#990099', label='gyro norm')
    axes[2].axhline(
        analyzer.max_gyro_norm, color='#dc3912', linestyle='--', label='stationary threshold'
    )
    axes[2].set_ylabel('Gyro norm (rad/s)')
    axes[2].legend(loc='upper right')
    axes[2].grid(alpha=0.25)

    axes[3].plot(elapsed, acc_norm, linewidth=0.7, color='#0099c6', label='acc norm')
    axes[3].axhline(analyzer.gravity_magnitude, color='#109618', linestyle='--', label='gravity')
    axes[3].axhline(
        analyzer.gravity_magnitude - analyzer.max_acc_norm_error,
        color='#dc3912',
        linestyle=':',
        label='stationary band',
    )
    axes[3].axhline(
        analyzer.gravity_magnitude + analyzer.max_acc_norm_error,
        color='#dc3912',
        linestyle=':',
    )
    axes[3].set_ylabel('Acc norm (m/s²)')
    axes[3].set_xlabel('Capture elapsed time (s)')
    axes[3].legend(loc='upper right')
    axes[3].grid(alpha=0.25)

    figure.suptitle(
        'MRDVS IMU Diagnostics | '
        f"messages={summary['message_count']}  duplicates={summary['duplicate_count']}  "
        f"rollback={summary['rollback_count']}  large jumps={summary['large_jump_count']}"
    )
    figure.savefig(chart_path, dpi=160)
    plt.close(figure)


def validate_collection_args(args):
    if not str(args.topic).strip():
        raise ValueError('topic must not be empty')
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise ValueError('duration must be finite and positive')
    if not math.isfinite(args.wait_timeout) or args.wait_timeout <= 0.0:
        raise ValueError('wait_timeout must be finite and positive')


def collect_ros_samples(args):
    validate_collection_args(args)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    analyzer = ImuDiagnosticsAnalyzer(
        required_samples=args.required_samples,
        gravity_magnitude=args.gravity,
        max_gyro_norm=args.max_gyro_norm,
        max_acc_norm_error=args.max_acc_norm_error,
        max_gap_sec=args.max_gap_sec,
    )

    class ImuDiagnosticsNode(Node):
        def __init__(self):
            super().__init__('mrdvs_imu_diagnostics')
            self.first_arrival = None
            self.create_subscription(Imu, args.topic, self.on_imu, qos_profile_sensor_data)

        def on_imu(self, message):
            arrival = time.monotonic()
            if self.first_arrival is None:
                self.first_arrival = arrival
            timestamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
            analyzer.add_sample(
                sensor_timestamp=timestamp,
                arrival_elapsed=arrival - self.first_arrival,
                acceleration=(
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ),
                angular_velocity=(
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                ),
            )

    rclpy.init()
    node = ImuDiagnosticsNode()
    wait_deadline = time.monotonic() + args.wait_timeout
    try:
        while rclpy.ok() and node.first_arrival is None and time.monotonic() < wait_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.first_arrival is None:
            raise RuntimeError(f'no IMU data received from {args.topic}')

        capture_deadline = node.first_arrival + args.duration
        while rclpy.ok() and time.monotonic() < capture_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return analyzer


def parse_args():
    parser = argparse.ArgumentParser(
        description='Collect and analyze MRDVS IMU timing and stationary initialization'
    )
    parser.add_argument('--topic', default='/lx_camera_node/LxCamera_Imu')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--wait-timeout', type=float, default=10.0)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--required-samples', type=int, default=600)
    parser.add_argument('--gravity', type=float, default=9.81)
    parser.add_argument('--max-gyro-norm', type=float, default=0.10)
    parser.add_argument('--max-acc-norm-error', type=float, default=0.75)
    parser.add_argument('--max-gap-sec', type=float, default=0.2)
    args = parser.parse_args()
    try:
        validate_collection_args(args)
        ImuDiagnosticsAnalyzer(
            required_samples=args.required_samples,
            gravity_magnitude=args.gravity,
            max_gyro_norm=args.max_gyro_norm,
            max_acc_norm_error=args.max_acc_norm_error,
            max_gap_sec=args.max_gap_sec,
        )
    except ValueError as error:
        parser.error(str(error))
    return args


def main():
    args = parse_args()
    analyzer = collect_ros_samples(args)
    paths = write_outputs(analyzer, args.output_dir)
    print(json.dumps(analyzer.build_summary(), ensure_ascii=False, indent=2))
    print(f"summary={paths['summary']}")
    print(f"samples={paths['samples']}")
    print(f"chart={paths['chart']}")


if __name__ == '__main__':
    main()
