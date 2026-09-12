#!/usr/bin/env python3
"""Sample process CPU, threads, affinity and host load during Phase 3 runs."""

import argparse
import csv
import os
import re
import subprocess
import time


PATTERNS = {
    'px4': re.compile(r'(^|/)(px4)(\s|$)'),
    'gazebo': re.compile(r'gz sim|ruby.*person_tracking'),
    'yolo': re.compile(r'yolo_detector'),
    'bridge': re.compile(r'parameter_bridge'),
    'motion_arbiter': re.compile(r'motion_arbiter'),
    'hud': re.compile(r'live_camera_hud'),
}


def processes():
    result = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            cmdline = open(f'/proc/{entry}/cmdline', 'rb').read().replace(b'\0', b' ').decode(errors='replace').strip()
            stat = open(f'/proc/{entry}/stat').read().split()
            if len(stat) < 39:
                continue
            pid = int(entry)
            name = next((key for key, pat in PATTERNS.items() if pat.search(cmdline)), None)
            if name is None:
                continue
            affinity = ''
            try:
                affinity = ','.join(map(str, os.sched_getaffinity(pid)))
            except OSError:
                pass
            result.setdefault(name, []).append({
                'pid': pid,
                'cmd': cmdline[:240],
                'utime': int(stat[13]),
                'stime': int(stat[14]),
                'threads': int(stat[19]),
                'psr': stat[38],
                'affinity': affinity,
            })
        except (OSError, ValueError, IndexError):
            continue
    return result


def load_snapshot():
    try:
        return os.getloadavg()
    except OSError:
        return (0.0, 0.0, 0.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--output', default='logs/phase3_process_samples.csv')
    args = parser.parse_args()
    hz = os.sysconf(os.sysconf_names['SC_CLK_TCK'])
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    rows = []
    previous = {}
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        now = time.monotonic()
        snapshot = processes()
        load1, load5, load15 = load_snapshot()
        for role, items in snapshot.items():
            for item in items:
                key = item['pid']
                old = previous.get(key)
                cpu = 0.0
                if old is not None:
                    wall = max(1e-6, now - old['mono'])
                    cpu = ((item['utime'] + item['stime']) - old['ticks']) / hz / wall * 100.0
                rows.append({
                    'monotonic_time': round(now, 6),
                    'role': role,
                    'pid': item['pid'],
                    'cpu_percent': round(cpu, 3),
                    'threads': item['threads'],
                    'psr': item['psr'],
                    'affinity': item['affinity'],
                    'load1': load1,
                    'load5': load5,
                    'load15': load15,
                    'cmd': item['cmd'],
                })
                previous[key] = {'ticks': item['utime'] + item['stime'], 'mono': now}
        time.sleep(args.interval)
    fields = ['monotonic_time', 'role', 'pid', 'cpu_percent', 'threads', 'psr',
              'affinity', 'load1', 'load5', 'load15', 'cmd']
    with open(args.output, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f'wrote {len(rows)} samples to {args.output}')


if __name__ == '__main__':
    main()
