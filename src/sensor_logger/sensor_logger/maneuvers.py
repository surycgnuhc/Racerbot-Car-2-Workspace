"""
maneuvers.py

The test-vector catalogue for sensor logging: what each recording
category (turn_left, start, stop, ...) means, how to drive it, and how to
check afterwards that a recording actually contains what its category
claims.

Two levels, deliberately kept apart:

  * A *label* is what the car is doing at one instant -- idle,
    accelerating, braking, straight, turn_left, turn_right, reverse.
    label_series() computes one per odometry sample.
  * A *category* is what a whole recording is meant to be -- the test
    vector the driver set out to capture. It is chosen up front with the
    logger's `category` parameter, and check_session() verifies it
    against the labels afterwards, so a "turn_left" folder that is
    actually full of straight-line driving is caught rather than trusted.

Labels come from /odom, not the IMU. The VESC IMU on this car reports all
zeros (docs/hardware-reference.md, measured 2026-08-19), so the IMU cannot
be what tells us the car was turning. /odom's speed is measured (ERPM) and
its yaw rate is speed * tan(commanded steering) / wheelbase
(vesc_to_odom.cpp), which is enough to tell the maneuvers apart.

Pure Python, no rclpy: runs under a bare pytest with nothing sourced.
"""

import math
from dataclasses import dataclass, field

# Per-sample labels.
IDLE = 'idle'
ACCELERATING = 'accelerating'
BRAKING = 'braking'
STRAIGHT = 'straight'
TURN_LEFT = 'turn_left'
TURN_RIGHT = 'turn_right'
REVERSE = 'reverse'
INVALID = 'invalid'     # NaN/inf speed or yaw rate

LABELS = (IDLE, ACCELERATING, BRAKING, STRAIGHT, TURN_LEFT, TURN_RIGHT,
          REVERSE, INVALID)


@dataclass(frozen=True)
class Thresholds:
    """Where one label ends and the next begins."""

    # Below this |speed| the car counts as stopped. vesc_to_odom clamps
    # |speed| < 0.05 m/s to exactly 0 (vesc_to_odom.cpp), so anything
    # under 0.10 is that clamp or wheel creep, not driving.
    still_speed_mps: float = 0.10
    # |dv/dt| at or above this is a deliberate speed change (start/stop),
    # not cruising.
    accel_mps2: float = 0.50
    # dv/dt is measured over this much history. One 50 Hz odom step is
    # too noisy to difference on its own.
    accel_window_sec: float = 0.30
    # |yaw rate / speed| -- path curvature, 1/m -- at or above this is a
    # turn. 0.25 1/m is a 4 m radius, about 5 deg of steering on the
    # 0.36 m wheelbase; full lock is 0.74 1/m (the 1.35 m minimum radius,
    # docs/hardware-reference.md). Curvature, unlike yaw rate, does not
    # change with speed.
    turn_curvature_per_m: float = 0.25


def _finite(value) -> bool:
    return value is not None and math.isfinite(value)


def _label_one(speed, yaw_rate, accel, th: Thresholds) -> str:
    if not (_finite(speed) and _finite(yaw_rate)):
        return INVALID
    if speed <= -th.still_speed_mps:
        return REVERSE
    if accel is not None and accel <= -th.accel_mps2:
        return BRAKING
    if accel is not None and accel >= th.accel_mps2:
        return ACCELERATING
    if abs(speed) < th.still_speed_mps:
        return IDLE
    # ROS REP-103: z is up, so positive yaw rate is counter-clockwise --
    # a left turn when driving forwards.
    curvature = yaw_rate / speed
    if curvature >= th.turn_curvature_per_m:
        return TURN_LEFT
    if curvature <= -th.turn_curvature_per_m:
        return TURN_RIGHT
    return STRAIGHT


def label_series(t, speed, yaw_rate, thresholds: Thresholds = Thresholds()):
    """One label per sample. t in seconds, non-decreasing and finite.

    Acceleration at sample i is the speed change over the preceding
    accel_window_sec. Until at least half a window of history exists it
    is unknown, and the sample is labelled from speed and yaw rate alone.
    """
    n = len(t)
    if len(speed) != n or len(yaw_rate) != n:
        raise ValueError(
            f't, speed and yaw_rate must be the same length '
            f'(got {n}, {len(speed)}, {len(yaw_rate)})')
    for i, stamp in enumerate(t):
        if not math.isfinite(stamp):
            raise ValueError(f'timestamp t[{i}]={stamp} is not finite')
        if i and stamp < t[i - 1]:
            raise ValueError(
                f'timestamps must be non-decreasing; t[{i}]={stamp} < t[{i - 1}]={t[i - 1]}')

    window = thresholds.accel_window_sec
    labels = []
    j = 0
    for i in range(n):
        while t[i] - t[j] > window:
            j += 1
        span = t[i] - t[j]
        accel = None
        if span >= window / 2 and _finite(speed[i]) and _finite(speed[j]):
            accel = (speed[i] - speed[j]) / span
        labels.append(_label_one(speed[i], yaw_rate[i], accel, thresholds))
    return labels


def label_seconds(t, labels):
    """Seconds spent under each label. Sample i owns [t[i], t[i+1])."""
    seconds = {}
    for i in range(len(t) - 1):
        seconds[labels[i]] = seconds.get(labels[i], 0.0) + (t[i + 1] - t[i])
    return seconds


@dataclass(frozen=True)
class Category:
    """One test vector: what to drive, and what the data must then show."""

    description: str
    protocol: str                   # what the driver actually does
    min_label_sec: dict = field(default_factory=dict)    # label -> at least
    max_label_sec: dict = field(default_factory=dict)    # label -> at most
    min_label_fraction: dict = field(default_factory=dict)  # of the session
    starts_idle_sec: float = 0.0    # first N s must all be idle
    ends_idle_sec: float = 0.0      # last N s must all be idle
    min_peak_speed_mps: float = 0.0  # max |speed| reached
    checked: bool = True


CATEGORIES = {
    'idle': Category(
        description='Parked, powered, nobody touching anything.',
        protocol='Bringup running, car on the floor, LB NOT held. Record 10 s. '
                 'This is also the IMU reference: at rest it must show gravity.',
        min_label_fraction={IDLE: 0.95}),
    'start': Category(
        description='Pull away from rest in a straight line.',
        protocol='Stopped for at least 1 s, then accelerate straight to about '
                 '1 m/s and hold it for a second.',
        min_label_sec={ACCELERATING: 0.3},
        starts_idle_sec=0.5, min_peak_speed_mps=0.5),
    'stop': Category(
        description='Come to a full stop from a straight run.',
        protocol='Drive straight at about 1 m/s, release the throttle to a full '
                 'stop, and stay stopped for at least 1 s.',
        min_label_sec={BRAKING: 0.3},
        ends_idle_sec=0.5, min_peak_speed_mps=0.5),
    'straight': Category(
        description='Constant speed, wheel centred.',
        protocol='About 1 m/s, steering centred, for at least 3 s.',
        min_label_sec={STRAIGHT: 2.0},
        max_label_sec={TURN_LEFT: 0.5, TURN_RIGHT: 0.5},
        min_peak_speed_mps=0.5),
    'turn_left': Category(
        description='Constant-speed left turn.',
        protocol='About 1 m/s with steering held left (full lock is a 1.35 m '
                 'radius circle) for at least 3 s.',
        min_label_sec={TURN_LEFT: 2.0},
        max_label_sec={TURN_RIGHT: 0.5},
        min_peak_speed_mps=0.5),
    'turn_right': Category(
        description='Constant-speed right turn.',
        protocol='About 1 m/s with steering held right for at least 3 s.',
        min_label_sec={TURN_RIGHT: 2.0},
        max_label_sec={TURN_LEFT: 0.5},
        min_peak_speed_mps=0.5),
    'reverse': Category(
        description='Straight reverse.',
        protocol='Back up slowly in a straight line for at least 2 s.',
        min_label_sec={REVERSE: 1.0},
        min_peak_speed_mps=0.3),
    'free_drive': Category(
        description='Anything else. Labelled, but not checked against a vector.',
        protocol='Drive however you like.',
        checked=False),
}


def validate_category(name: str) -> Category:
    if name not in CATEGORIES:
        raise ValueError(
            f'unknown category {name!r}; expected one of: {", ".join(CATEGORIES)}')
    return CATEGORIES[name]


@dataclass
class CheckResult:
    category: str
    checked: bool
    passed: bool
    failures: list
    label_seconds: dict
    duration_sec: float
    peak_speed_mps: float


def check_session(category: str, t, speed, labels) -> CheckResult:
    """Does a recording contain the maneuver its category claims?"""
    spec = validate_category(category)
    if len(t) != len(labels) or len(t) != len(speed):
        raise ValueError(
            f't, speed and labels must be the same length '
            f'(got {len(t)}, {len(speed)}, {len(labels)})')
    seconds = label_seconds(t, labels)
    duration = t[-1] - t[0] if t else 0.0
    finite_speeds = [abs(v) for v in speed if _finite(v)]
    peak = max(finite_speeds) if finite_speeds else 0.0
    result = CheckResult(category, spec.checked, True, [], seconds, duration, peak)

    if not spec.checked:
        return result
    failures = result.failures
    if len(t) < 2:
        failures.append(f'only {len(t)} odometry sample(s); nothing to check')
        result.passed = False
        return result

    for label, need in spec.min_label_sec.items():
        got = seconds.get(label, 0.0)
        if got < need:
            failures.append(f'{label}: {got:.2f}s, needs at least {need:.2f}s')
    for label, limit in spec.max_label_sec.items():
        got = seconds.get(label, 0.0)
        if got > limit:
            failures.append(f'{label}: {got:.2f}s, allowed at most {limit:.2f}s')
    for label, need in spec.min_label_fraction.items():
        got = seconds.get(label, 0.0) / duration if duration > 0 else 0.0
        if got < need:
            failures.append(f'{label}: {100 * got:.0f}% of the session, '
                            f'needs at least {100 * need:.0f}%')
    if spec.starts_idle_sec:
        if duration < spec.starts_idle_sec:
            failures.append(f'session is {duration:.2f}s, shorter than the '
                            f'{spec.starts_idle_sec:.2f}s idle start it needs')
        elif any(lab != IDLE for ts, lab in zip(t, labels)
                 if ts < t[0] + spec.starts_idle_sec):
            failures.append(f'not idle for the first {spec.starts_idle_sec:.2f}s')
    if spec.ends_idle_sec:
        if duration < spec.ends_idle_sec:
            failures.append(f'session is {duration:.2f}s, shorter than the '
                            f'{spec.ends_idle_sec:.2f}s idle end it needs')
        elif any(lab != IDLE for ts, lab in zip(t, labels)
                 if ts > t[-1] - spec.ends_idle_sec):
            failures.append(f'not idle for the last {spec.ends_idle_sec:.2f}s')
    if peak < spec.min_peak_speed_mps:
        failures.append(f'peak speed {peak:.2f} m/s, needs at least '
                        f'{spec.min_peak_speed_mps:.2f} m/s')

    result.passed = not failures
    return result
