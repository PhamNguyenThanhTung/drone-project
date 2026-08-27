#!/usr/bin/env python3
"""Inject a native PX4 SITL failure through MAV_CMD_INJECT_FAILURE."""

import argparse
import sys

from pymavlink import mavutil


UNITS = {
    'gyro': 0, 'accel': 1, 'mag': 2, 'baro': 3, 'gps': 4,
    'optical_flow': 5, 'vio': 6, 'distance_sensor': 7, 'airspeed': 8,
    'battery': 100, 'motor': 101, 'servo': 102, 'avoidance': 103,
    'rc_signal': 104, 'mavlink_signal': 105,
}
TYPES = {'ok': 0, 'off': 1, 'stuck': 2, 'garbage': 3, 'wrong': 4,
         'slow': 5, 'delayed': 6, 'intermittent': 7}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('unit', choices=UNITS)
    parser.add_argument('failure_type', choices=TYPES)
    parser.add_argument('--instance', type=int, default=0)
    parser.add_argument('--connection', default='udpin:0.0.0.0:14540')
    args = parser.parse_args()
    link = mavutil.mavlink_connection(args.connection, source_system=255)
    heartbeat = link.wait_heartbeat(timeout=10)
    if heartbeat is None:
        sys.exit('No PX4 heartbeat received')
    link.mav.command_long_send(
        link.target_system, link.target_component, 420, 0,
        UNITS[args.unit], TYPES[args.failure_type], args.instance, 0, 0, 0, 0)
    ack = link.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack is None or ack.command != 420 or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        sys.exit(f'Failure injection rejected: {ack}')
    print(f'Injected {args.unit}={args.failure_type}, instance={args.instance}')


if __name__ == '__main__':
    main()
