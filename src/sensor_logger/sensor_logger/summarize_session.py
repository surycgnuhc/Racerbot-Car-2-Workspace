"""
summarize_session.py

Turn one recorded session into labels and a verdict:

    ros2 run sensor_logger summarize_session data_logs/turn_left/20261016-141500
    ros2 run sensor_logger summarize_session <dir> --json     # for scripts/agents

Writes two files next to the data:
    labels.csv     stamp,label for every odometry sample
    summary.json   the category check, label seconds, IMU and scan health

Exit status: 0 the session matches its category (or the category is
unchecked, like free_drive); 1 it does not; 2 the folder is not a
readable session.

No rclpy: runs anywhere Python does, including a laptop with a copied
session folder.
"""

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

from .maneuvers import Thresholds, check_session, label_series, validate_category
from .session import (SCAN_FIXED_COLUMNS, SOURCE_UNKNOWN, SOURCES, ImuStats,
                      write_json_atomic)


class SessionError(Exception):
    """The folder is not a readable session."""


def _complete_lines(handle, cut):
    """Every line that ends in a newline.

    A hard kill can leave the last line half written, and a row cut inside
    its final number ("0.512" -> "0.5") still has the right column count,
    so it would parse into a wrong value. A line with no newline is never
    complete: it is counted in cut[0] and skipped.
    """
    for line in handle:
        if line.endswith('\n'):
            yield line
        else:
            cut[0] += 1


def _rows(path, columns):
    """Parsed float rows of `columns` from a CSV, and the malformed count."""
    rows, cut = [], [0]
    malformed = 0
    with open(path, newline='') as handle:
        reader = csv.reader(_complete_lines(handle, cut))
        header = next(reader, None)
        if header is None:
            return rows, cut[0]
        try:
            index = [header.index(c) for c in columns]
        except ValueError as exc:
            raise SessionError(f'{path.name}: missing column ({exc})') from exc
        for raw in reader:
            if len(raw) != len(header):
                malformed += 1
                continue
            try:
                rows.append([float(raw[i]) for i in index])
            except ValueError:
                malformed += 1
    return rows, malformed + cut[0]


def _scan_health(path):
    beams = rows = malformed = finite = total = 0
    first = last = None
    cut = [0]
    with open(path, newline='') as handle:
        reader = csv.reader(_complete_lines(handle, cut))
        header = next(reader, None)
        if header is not None:
            beams = len(header) - len(SCAN_FIXED_COLUMNS)
            for raw in reader:
                if len(raw) != len(header):
                    malformed += 1
                    continue
                try:
                    stamp = float(raw[0])
                    ranges = [float(r) for r in raw[len(SCAN_FIXED_COLUMNS):]]
                except ValueError:
                    malformed += 1
                    continue
                rows += 1
                first = stamp if first is None else first
                last = stamp
                total += len(ranges)
                finite += sum(1 for r in ranges if math.isfinite(r))
    span = (last - first) if rows > 1 else 0.0
    return {'scans': rows, 'beams': beams, 'malformed': malformed + cut[0],
            'rate_hz': round((rows - 1) / span, 2) if span > 0 else None,
            'finite_fraction': round(finite / total, 4) if total else None}


def _validate_source(source, publishers):
    if source not in SOURCES:
        raise ValueError(f'source {source!r} is not one of {", ".join(SOURCES)}')
    if not (isinstance(publishers, dict)
            and all(isinstance(names, list) and all(isinstance(n, str) for n in names)
                    for names in publishers.values())):
        raise ValueError('source_publishers must map each topic to a list of node names')


def summarize(session_dir, thresholds: Thresholds = Thresholds()):
    session_dir = Path(session_dir)
    meta_path = session_dir / 'metadata.json'
    if not meta_path.is_file():
        raise SessionError(f'{session_dir} has no metadata.json; not a session folder')
    try:
        metadata = json.loads(meta_path.read_text())
        category = metadata['category']
        validate_category(category)
        # Recordings made before the field existed say nothing: 'unknown'.
        source = metadata.get('source', SOURCE_UNKNOWN)
        publishers = metadata.get('source_publishers', {})
        _validate_source(source, publishers)
    except (ValueError, KeyError) as exc:
        raise SessionError(f'{meta_path}: unreadable ({exc})') from exc

    odom, odom_bad = _rows(session_dir / 'odom.csv', ('stamp', 'speed', 'yaw_rate'))
    t = [r[0] for r in odom]
    speed = [r[1] for r in odom]
    try:
        labels = label_series(t, speed, [r[2] for r in odom], thresholds)
    except ValueError as exc:
        raise SessionError(f'odom.csv: {exc}') from exc
    with open(session_dir / 'labels.csv', 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(('stamp', 'label'))
        writer.writerows(zip(t, labels))
    check = check_session(category, t, speed, labels)

    accel_unit = metadata.get('units', {}).get('imu_accel', 'g')
    stats = ImuStats(accel_unit)
    imu, imu_bad = _rows(session_dir / 'imu.csv', ('ax', 'ay', 'az', 'gx', 'gy', 'gz'))
    for row in imu:
        stats.add(*row)

    summary = {
        'session': str(session_dir),
        'category': category,
        'notes': metadata.get('notes', ''),
        'source': source,
        'source_publishers': publishers,
        'check': asdict(check),
        'thresholds': asdict(thresholds),
        'odom': {'samples': len(odom), 'malformed': odom_bad},
        'imu': dict(stats.as_dict(), malformed=imu_bad),
        'scan': _scan_health(session_dir / 'scan.csv'),
    }
    write_json_atomic(session_dir / 'summary.json', summary)
    return summary


def render(summary) -> str:
    check = summary['check']
    if not check['checked']:
        verdict = 'UNCHECKED (category has no test vector)'
    else:
        verdict = 'PASS' if check['passed'] else 'FAIL'
    lines = [f'{summary["session"]}',
             f'  category  {summary["category"]}: {verdict}']
    lines += [f'    - {f}' for f in check['failures']]
    publishers = sorted(set().union(*summary['source_publishers'].values()))
    lines.append(f'  source    {summary["source"]}'
                 + (f' ({", ".join(publishers)})' if publishers else ''))
    seconds = ', '.join(f'{k} {v:.1f}s' for k, v in
                        sorted(check['label_seconds'].items(), key=lambda kv: -kv[1]))
    lines.append(f'  labels    {seconds or "none"}  '
                 f'(peak {check["peak_speed_mps"]:.2f} m/s over {check["duration_sec"]:.1f}s)')
    imu = summary['imu']
    lines.append(f'  imu       {imu["verdict"]}, {imu["samples"]} samples, '
                 f'mean |accel| {imu["mean_accel_norm"]} {imu["accel_unit"]}')
    scan = summary['scan']
    lines.append(f'  scan      {scan["scans"]} scans x {scan["beams"]} beams, '
                 f'{scan["rate_hz"]} Hz, finite {scan["finite_fraction"]}')
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('session_dir')
    parser.add_argument('--json', action='store_true', help='print summary.json')
    args = parser.parse_args(argv)
    try:
        summary = summarize(args.session_dir)
    except (SessionError, OSError) as exc:
        print(f'summarize_session: {exc}', file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2) if args.json else render(summary))
    return 0 if summary['check']['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
