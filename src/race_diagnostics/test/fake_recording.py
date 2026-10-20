"""
fake_recording.py

Writes a synthetic recording in the same shape record_run.py produces
(<run>/bag/bag_0.mcap, bag_1.mcap, ...) for testing the bag_* tools.
There is no real recording in the repo -- they live on the car under
~/.ros/racerbot_runs/ -- so the tests need data whose right answers are
known by construction.

It is deliberately messy, because real recordings are:

  timing jitter    every message's header stamp wobbles around its nominal
                   period, and it arrives after a variable delay. Both are
                   *bounded* (truncated), so a test can compute exactly how
                   far a measured gap or lag is allowed to stray.
  sensor noise     Gaussian noise on position, speed, voltage, current and
                   every LiDAR beam.
  bad beams        ~1% of beams NaN and ~1% inf in every scan, as the
                   Hokuyo reports missing returns.
  glitches         single-sample electrical spikes on the battery voltage
                   and single-beam short readings on the LiDAR -- the kind
                   of noise a naive threshold search reports as events.

and it contains five real events, recorded in FakeTruth so a test can
check a tool found them without a hand-typed number:

  odom dropout     no /odom at all for 0.6 s
  battery dip      voltage drops to 10.2 V for 2 s
  near wall        front beams read 0.30 m for 1 s, and /drive commands 0
  lag spike        /odom data arrives up to 0.4 s old, then catches up
  LB released      /joy button 4 (LB) is 0 for 2 s

Seeded, so the same seed always writes the same bytes. Needs the
`mcap` and `mcap-ros2-support` libraries (see bag_io.py). Run directly to
get a recording to try the tools on:

    python3 src/race_diagnostics/test/fake_recording.py /tmp/fake_run
"""

import calendar
import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path

from mcap_ros2.writer import Writer

NS = 1_000_000_000
SEP = '\n' + '=' * 80 + '\n'

# --- Message definitions, as rosbag2 stores them in the recording --------
# Copied from the .msg files (comments dropped): std_msgs, geometry_msgs,
# nav_msgs, sensor_msgs from ROS 2; vesc_msgs from
# src/f1tenth_system/vesc/vesc_msgs/msg/; ackermann_msgs from upstream.

_DEPS = {
    'std_msgs/Header': 'builtin_interfaces/Time stamp\nstring frame_id',
    'builtin_interfaces/Time': 'int32 sec\nuint32 nanosec',
    'geometry_msgs/PoseWithCovariance': 'Pose pose\nfloat64[36] covariance',
    'geometry_msgs/Pose': 'Point position\nQuaternion orientation',
    'geometry_msgs/Point': 'float64 x\nfloat64 y\nfloat64 z',
    'geometry_msgs/Quaternion': 'float64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1',
    'geometry_msgs/TwistWithCovariance': 'Twist twist\nfloat64[36] covariance',
    'geometry_msgs/Twist': 'Vector3 linear\nVector3 angular',
    'geometry_msgs/Vector3': 'float64 x\nfloat64 y\nfloat64 z',
    'ackermann_msgs/AckermannDrive': (
        'float32 steering_angle\nfloat32 steering_angle_velocity\nfloat32 speed\n'
        'float32 acceleration\nfloat32 jerk'),
    'vesc_msgs/VescState': (
        'int32 FAULT_CODE_NONE=0\nint32 FAULT_CODE_OVER_VOLTAGE=1\n'
        'int32 FAULT_CODE_UNDER_VOLTAGE=2\n'
        'float64 temp_fet\nfloat64 temp_motor\nfloat64 current_motor\n'
        'float64 current_input\nfloat64 avg_id\nfloat64 avg_iq\nfloat64 duty_cycle\n'
        'float64 speed\nfloat64 voltage_input\nfloat64 charge_drawn\nfloat64 charge_regen\n'
        'float64 energy_drawn\nfloat64 energy_regen\nint32 displacement\n'
        'int32 distance_traveled\nint32 fault_code\nfloat64 pid_pos_now\n'
        'int32 controller_id\nfloat64 ntc_temp_mos1\nfloat64 ntc_temp_mos2\n'
        'float64 ntc_temp_mos3\nfloat64 avg_vd\nfloat64 avg_vq'),
}

_ROOTS = {
    'std_msgs/msg/String': ('string data', []),
    'nav_msgs/msg/Odometry': (
        'std_msgs/Header header\nstring child_frame_id\n'
        'geometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist',
        ['std_msgs/Header', 'builtin_interfaces/Time', 'geometry_msgs/PoseWithCovariance',
         'geometry_msgs/Pose', 'geometry_msgs/Point', 'geometry_msgs/Quaternion',
         'geometry_msgs/TwistWithCovariance', 'geometry_msgs/Twist', 'geometry_msgs/Vector3']),
    'ackermann_msgs/msg/AckermannDriveStamped': (
        'std_msgs/Header header\nAckermannDrive drive',
        ['std_msgs/Header', 'builtin_interfaces/Time', 'ackermann_msgs/AckermannDrive']),
    'sensor_msgs/msg/LaserScan': (
        'std_msgs/Header header\nfloat32 angle_min\nfloat32 angle_max\n'
        'float32 angle_increment\nfloat32 time_increment\nfloat32 scan_time\n'
        'float32 range_min\nfloat32 range_max\nfloat32[] ranges\nfloat32[] intensities',
        ['std_msgs/Header', 'builtin_interfaces/Time']),
    'sensor_msgs/msg/Joy': (
        'std_msgs/Header header\nfloat32[] axes\nint32[] buttons',
        ['std_msgs/Header', 'builtin_interfaces/Time']),
    'vesc_msgs/msg/VescStateStamped': (
        'std_msgs/Header  header\nVescState state',
        ['std_msgs/Header', 'builtin_interfaces/Time', 'vesc_msgs/VescState']),
}


def msgdef(datatype):
    root, deps = _ROOTS[datatype]
    return root + ''.join(f'{SEP}MSG: {d}\n{_DEPS[d]}' for d in deps)


# --- What the recording contains -----------------------------------------

@dataclass
class FakeTruth:
    """Everything the generator decided, for tests to check against."""
    seed: int
    duration_s: float
    beams: int
    split_at_s: float
    start_ns: int
    # nominal rates, Hz
    rates: dict = field(default_factory=lambda: {
        '/odom': 50.0, '/drive': 40.0, '/scan': 40.0, '/sensors/core': 50.0, '/joy': 20.0})
    # bounds on the jitter: header stamp offset is within +-stamp_jitter_s of
    # nominal; arrival is stamp + latency, latency within [lo, hi]
    stamp_jitter_s: float = 0.002
    latency_s: dict = field(default_factory=lambda: {
        '/odom': (0.015, 0.020), '/drive': (0.002, 0.005), '/scan': (0.025, 0.035),
        '/sensors/core': (0.004, 0.008), '/joy': (0.001, 0.003)})
    # events [start, end) in seconds from the start of the recording
    odom_dropout: tuple = (12.0, 12.6)
    battery_dip: tuple = (20.0, 22.0)
    battery_dip_volts: float = 10.2
    battery_nominal_volts: float = 11.8
    battery_noise_volts: float = 0.03
    voltage_spike_volts: float = 10.0
    wall: tuple = (30.0, 31.0)
    wall_range_m: float = 0.30
    wall_half_angle_rad: float = math.radians(10.0)
    scan_noise_m: float = 0.01
    lag_spike: tuple = (40.0, 41.0)
    lag_spike_extra_s: float = 0.4
    lb_released: tuple = (50.0, 52.0)
    speed_mps: float = 2.0
    # filled in while generating: the nominal times of the one-sample glitches
    voltage_spike_stamps_s: list = field(default_factory=list)
    scan_glitch_stamps_s: list = field(default_factory=list)
    counts: dict = field(default_factory=dict)


VOLTAGE_SPIKES_NEAR_S = (5.13, 17.41, 33.07, 48.9)   # outside the dip
SCAN_GLITCHES_NEAR_S = (8.2, 25.55, 44.1)            # outside the wall
SCAN_GLITCH_RANGE_M = 0.10


def _in(t, span):
    return span[0] <= t < span[1]


class _Gen:
    def __init__(self, truth):
        self.truth = truth
        self.rng = random.Random(truth.seed)
        self.out = []          # (arrival_s, topic, datatype, message dict)

    def jitter(self, bound):
        return max(-bound, min(bound, self.rng.gauss(0.0, bound / 2)))

    def latency(self, topic):
        lo, hi = self.truth.latency_s[topic]
        return self.rng.uniform(lo, hi)

    def stamps(self, topic):
        """(nominal, jittered) header stamps for one topic, first one period in.

        Events are decided on the nominal time, so an event's edges are
        exact multiples of the period whatever the jitter did."""
        rate = self.truth.rates[topic]
        k = 1
        while k / rate < self.truth.duration_s - 0.05:
            yield k / rate, k / rate + self.jitter(self.truth.stamp_jitter_s)
            k += 1

    def emit(self, arrival, topic, datatype, msg):
        self.out.append((arrival, topic, datatype, msg))
        self.truth.counts[topic] = self.truth.counts.get(topic, 0) + 1

    def header(self, stamp, frame):
        ns = self.truth.start_ns + round(stamp * NS)
        return {'stamp': {'sec': ns // NS, 'nanosec': ns % NS}, 'frame_id': frame}

    def moving(self, t):
        return not _in(t, self.truth.wall)

    def speed(self, t):
        return self.truth.speed_mps if self.moving(t) else 0.0


def generate(truth):
    g = _Gen(truth)
    T = truth
    radius = 3.0
    wheelbase = 0.36      # docs/hardware-reference.md

    g.emit(0.0, '/robot_description', 'std_msgs/msg/String',
           {'data': '<robot name="racerbot_car_2"/>'})

    # /odom: circling; a dropout; a lag spike that decays as it catches up
    heading = 0.0
    last = 0.0
    for nominal, stamp in g.stamps('/odom'):
        heading += g.speed(nominal) * (nominal - last) / radius
        last = nominal
        if _in(nominal, T.odom_dropout):
            continue
        lag = g.latency('/odom')
        if _in(nominal, T.lag_spike):
            fraction_left = (T.lag_spike[1] - nominal) / (T.lag_spike[1] - T.lag_spike[0])
            lag += T.lag_spike_extra_s * fraction_left
        n = 0.01
        g.emit(stamp + lag, '/odom', 'nav_msgs/msg/Odometry', {
            'header': g.header(stamp, 'odom'), 'child_frame_id': 'base_link',
            'pose': {
                'pose': {
                    'position': {'x': radius * math.sin(heading) + g.rng.gauss(0, n),
                                 'y': radius * (1 - math.cos(heading)) + g.rng.gauss(0, n),
                                 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(heading / 2),
                                    'w': math.cos(heading / 2)}},
                'covariance': [0.0] * 36},
            'twist': {'twist': {
                'linear': {'x': g.speed(nominal) + g.rng.gauss(0, 0.02), 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': g.speed(nominal) / radius}},
                'covariance': [0.0] * 36},
        })

    # /drive: constant-curvature circle, zero speed at the wall
    steer = math.atan(wheelbase / radius)
    for nominal, stamp in g.stamps('/drive'):
        g.emit(stamp + g.latency('/drive'), '/drive', 'ackermann_msgs/msg/AckermannDriveStamped', {
            'header': g.header(stamp, 'base_link'),
            'drive': {'steering_angle': steer, 'steering_angle_velocity': 0.0,
                      'speed': g.speed(nominal), 'acceleration': 0.0, 'jerk': 0.0}})

    # /scan: noisy room, NaN/inf beams, a wall ahead, single-beam glitches
    angle_min, angle_max = -2.35619449, 2.35619449
    inc = (angle_max - angle_min) / (T.beams - 1)
    glitches = list(SCAN_GLITCHES_NEAR_S)
    for nominal, stamp in g.stamps('/scan'):
        ranges = []
        for i in range(T.beams):
            angle = angle_min + i * inc
            roll = g.rng.random()
            if roll < 0.01:
                ranges.append(math.nan)
            elif roll < 0.02:
                ranges.append(math.inf)
            elif _in(nominal, T.wall) and abs(angle) <= T.wall_half_angle_rad:
                ranges.append(T.wall_range_m + g.jitter(T.scan_noise_m))
            else:
                ranges.append(2.0 + 1.5 * abs(math.sin(angle)) + g.rng.gauss(0, T.scan_noise_m))
        if glitches and nominal >= glitches[0]:
            glitches.pop(0)
            ranges[g.rng.randrange(T.beams)] = SCAN_GLITCH_RANGE_M
            T.scan_glitch_stamps_s.append(nominal)
        g.emit(stamp + g.latency('/scan'), '/scan', 'sensor_msgs/msg/LaserScan', {
            'header': g.header(stamp, 'laser'), 'angle_min': angle_min, 'angle_max': angle_max,
            'angle_increment': inc, 'time_increment': 0.0, 'scan_time': 0.025,
            'range_min': 0.02, 'range_max': 30.0, 'ranges': ranges, 'intensities': []})

    # /sensors/core: voltage sags with current; a 2 s dip; one-sample spikes
    spikes = list(VOLTAGE_SPIKES_NEAR_S)
    for nominal, stamp in g.stamps('/sensors/core'):
        current = (8.0 if g.moving(nominal) else 1.0) + g.rng.gauss(0, 0.3)
        volts = T.battery_dip_volts if _in(nominal, T.battery_dip) else T.battery_nominal_volts
        volts += g.jitter(2 * T.battery_noise_volts)
        if spikes and nominal >= spikes[0]:
            spikes.pop(0)
            volts = T.voltage_spike_volts
            T.voltage_spike_stamps_s.append(nominal)
        g.emit(stamp + g.latency('/sensors/core'), '/sensors/core',
               'vesc_msgs/msg/VescStateStamped', {
                   'header': g.header(stamp, 'vesc'),
                   'state': {
                       'temp_fet': 35.0, 'temp_motor': 30.0 + 15.0 * nominal / T.duration_s,
                       'current_motor': current, 'current_input': current * 0.6,
                       'avg_id': 0.0, 'avg_iq': current,
                       'duty_cycle': 0.2 if g.moving(nominal) else 0.0,
                       # vesc.yaml's speed_to_erpm_gain is negative -- see CLAUDE.md
                       'speed': -4447.983786 * g.speed(nominal), 'voltage_input': volts,
                       'charge_drawn': 0.0, 'charge_regen': 0.0, 'energy_drawn': 0.0,
                       'energy_regen': 0.0, 'displacement': 0, 'distance_traveled': 0,
                       'fault_code': 0, 'pid_pos_now': 0.0, 'controller_id': 0,
                       'ntc_temp_mos1': 35.0, 'ntc_temp_mos2': 35.0, 'ntc_temp_mos3': 35.0,
                       'avg_vd': 0.0, 'avg_vq': 0.0}})

    # /joy: LB (button 4) held except for a 2 s release
    for nominal, stamp in g.stamps('/joy'):
        buttons = [0] * 11
        buttons[4] = 0 if _in(nominal, T.lb_released) else 1
        g.emit(stamp + g.latency('/joy'), '/joy', 'sensor_msgs/msg/Joy', {
            'header': g.header(stamp, 'joy'),
            'axes': [g.jitter(0.02) for _ in range(8)], 'buttons': buttons})

    g.out.sort(key=lambda m: m[0])
    return g.out


def write_fake_run(run_dir, seed=0, duration_s=60.0, beams=1081, split_at_s=30.0):
    """Write a fake recording into run_dir/bag/ and return its FakeTruth."""
    start_ns = calendar.timegm((2026, 10, 19, 12, 0, 0)) * NS
    truth = FakeTruth(seed=seed, duration_s=duration_s, beams=beams,
                      split_at_s=split_at_s, start_ns=start_ns)
    messages = generate(truth)
    bag_dir = Path(run_dir) / 'bag'
    bag_dir.mkdir(parents=True, exist_ok=True)
    parts = [[m for m in messages if m[0] < split_at_s],
             [m for m in messages if m[0] >= split_at_s]]
    for index, part in enumerate(parts):
        with open(bag_dir / f'bag_{index}.mcap', 'wb') as stream:
            writer = Writer(stream)
            schemas = {}
            for arrival, topic, datatype, msg in part:
                if datatype not in schemas:
                    schemas[datatype] = writer.register_msgdef(datatype, msgdef(datatype))
                t = start_ns + round(arrival * NS)
                writer.write_message(topic, schemas[datatype], msg, log_time=t, publish_time=t)
            writer.finish()
    return truth


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(f'usage: {sys.argv[0]} OUTPUT_RUN_DIRECTORY')
    t = write_fake_run(sys.argv[1])
    print(f'wrote a fake {t.duration_s:.0f} s recording to {sys.argv[1]}/bag/')
    print(f'events: odom dropout {t.odom_dropout}, battery dip {t.battery_dip}, '
          f'wall {t.wall}, lag spike {t.lag_spike}, LB released {t.lb_released}')
