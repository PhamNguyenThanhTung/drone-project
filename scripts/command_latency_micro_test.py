#!/usr/bin/env python3
"""Run the required +vx/-vx/zero command latency micro-test.

The stack must already be running and the vehicle should be airborne. The
script publishes directly to /teleop/cmd_vel and matches its publish records
with MotionArbiter's monotonic trace records.
"""

import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MicroPublisher(Node):
    def __init__(self, trace_path):
        super().__init__('command_latency_micro_test')
        self.publisher = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
        self.trace_path = Path(trace_path)
        self.trace_path.parent.mkdir(parents=True, exist_ok=True)
        self.trace = self.trace_path.open('a', buffering=1)

    def publish_command(self, vx):
        msg = Twist()
        msg.linear.x = float(vx)
        wall = time.time()
        mono = time.monotonic()
        ros = self.get_clock().now().nanoseconds / 1e9
        self.publisher.publish(msg)
        self.trace.write(json.dumps({
            'phase': 'published',
            'source': 'command_latency_micro_test',
            'wall_time': wall,
            'monotonic_time': mono,
            'ros_time': ros,
            'vx': 0.0,
            'vy': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'test_command_vx': float(vx),
        }, separators=(',', ':')) + '\n')
        return mono

    def close(self):
        self.trace.close()


def _records(path, start_mono, end_mono):
    rows = []
    if not path.exists():
        return rows
    for line in path.read_text(errors='replace').splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        mono = row.get('monotonic_time')
        if isinstance(mono, (int, float)) and start_mono - 1.0 <= mono <= end_mono + 3.0:
            rows.append(row)
    return rows


def _first_motion(samples, command_vx):
    if not samples:
        return None
    baseline = math.hypot(samples[0].get('vx_ned', 0.0), samples[0].get('vy_ned', 0.0))
    for sample in samples:
        speed = math.sqrt(sum(float(sample.get(k, 0.0)) ** 2 for k in ('vx_ned', 'vy_ned', 'vz_ned')))
        if abs(command_vx) > 0.05:
            if speed >= max(0.15, 0.25 * abs(command_vx)):
                return sample
        elif abs(speed - baseline) >= 0.10 or speed <= 0.15:
            return sample
    return None


def build_results(rows, published):
    received = [r for r in rows if r.get('phase') == 'received'
                and r.get('source') == 'motion_arbiter']
    decisions = {r.get('generation'): r for r in rows if r.get('phase') == 'decision'}
    sends = {r.get('generation'): r for r in rows if r.get('phase') == 'mavlink_send'}
    samples = {}
    for row in rows:
        if row.get('phase') == 'vehicle_sample':
            samples.setdefault(row.get('generation'), []).append(row)
    used = set()
    results = []
    for pub in published:
        vx = float(pub['test_command_vx'])
        match = None
        for row in received:
            generation = row.get('generation')
            if generation in used:
                continue
            if abs(float(row.get('vx', 0.0)) - vx) < 1e-6:
                delta = float(row['monotonic_time']) - float(pub['monotonic_time'])
                if -0.05 <= delta <= 1.5:
                    match = row
                    break
        record = {
            'command_vx': vx,
            'publish_mono': pub['monotonic_time'],
            'receive_mono': None,
            'decision_mono': None,
            'mavlink_mono': None,
            'motion_mono': None,
            'publish_receive_ms': None,
            'receive_decision_ms': None,
            'decision_mavlink_ms': None,
            'mavlink_motion_ms': None,
            'total_ms': None,
            'state': None,
            'result': 'NO_RECEIVE',
        }
        if match is None:
            results.append(record)
            continue
        generation = match.get('generation')
        used.add(generation)
        receive_mono = float(match['monotonic_time'])
        record['receive_mono'] = receive_mono
        decision = decisions.get(generation)
        send = sends.get(generation)
        record['publish_receive_ms'] = (receive_mono - pub['monotonic_time']) * 1000.0
        record['state'] = (decision or {}).get('state')
        if decision is None:
            record['result'] = 'NO_DECISION'
            results.append(record)
            continue
        decision_mono = float(decision['decision_mono'])
        record['decision_mono'] = decision_mono
        record['receive_decision_ms'] = (decision_mono - receive_mono) * 1000.0
        if send is None:
            record['result'] = 'NO_MAVLINK_SEND'
            results.append(record)
            continue
        send_mono = float(send['mavlink_send_mono'])
        record['mavlink_mono'] = send_mono
        record['decision_mavlink_ms'] = (send_mono - decision_mono) * 1000.0
        motion = _first_motion(samples.get(generation, []), vx)
        if motion is not None:
            motion_mono = float(motion['monotonic_time'])
            record['motion_mono'] = motion_mono
            record['mavlink_motion_ms'] = (motion_mono - send_mono) * 1000.0
            record['total_ms'] = (motion_mono - pub['monotonic_time']) * 1000.0
            record['result'] = 'MEASURED'
        else:
            record['result'] = 'NO_MOTION_SAMPLE'
        results.append(record)
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=5)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--vx', type=float, default=1.0)
    parser.add_argument('--trace', default=os.environ.get(
        'COMMAND_LATENCY_LOG', 'logs/command_latency_trace.jsonl'))
    args = parser.parse_args()
    trace_path = Path(args.trace)
    node = None
    published = []
    start_mono = time.monotonic()
    try:
        rclpy.init()
        node = MicroPublisher(trace_path)
        time.sleep(1.0)
        for trial in range(1, args.trials + 1):
            for command in (0.0, args.vx, -args.vx, 0.0):
                published.append({
                    'trial': trial,
                    'test_command_vx': command,
                    'monotonic_time': node.publish_command(command),
                })
                deadline = time.monotonic() + args.interval
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(2.5)
        end_mono = time.monotonic()
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    rows = _records(trace_path, start_mono, end_mono)
    results = build_results(rows, published)
    out_path = trace_path.with_name('COMMAND_LATENCY_VALIDATION.csv')
    import csv
    fields = ['trial', 'command', 'publish_time', 'receive_time', 'decision_time',
              'mavlink_time', 'motion_time', 'publish_receive_ms',
              'receive_decision_ms', 'decision_mavlink_ms', 'mavlink_motion_ms',
              'total_ms', 'state', 'result']
    with out_path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for index, row in enumerate(results):
            writer.writerow({
                'trial': index // 4 + 1,
                'command': row['command_vx'],
                'publish_time': row['publish_mono'],
                'receive_time': row['receive_mono'] or '',
                'decision_time': row['decision_mono'] or '',
                'mavlink_time': row['mavlink_mono'] or '',
                'motion_time': row['motion_mono'] or '',
                **{key: '' if row[key] is None else round(row[key], 3)
                   for key in ('publish_receive_ms', 'receive_decision_ms',
                               'decision_mavlink_ms', 'mavlink_motion_ms', 'total_ms')},
                'state': row['state'] or '',
                'result': row['result'],
            })
    measured = [r for r in results if r['total_ms'] is not None]
    print(f'Trace: {trace_path}')
    print(f'Validation CSV: {out_path}')
    print(f'Commands: {len(results)}, measured motion: {len(measured)}')
    if measured:
        totals = sorted(r['total_ms'] for r in measured)
        p50 = totals[(len(totals) - 1) // 2]
        p95 = totals[min(len(totals) - 1, math.ceil(len(totals) * 0.95) - 1)]
        print(f'TOTAL ms: P50={p50:.1f} P95={p95:.1f} MAX={max(totals):.1f}')
    else:
        print('TOTAL ms: no vehicle-motion samples; status remains UNKNOWN/FAIL')


if __name__ == '__main__':
    main()
