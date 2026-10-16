"""
Tests for maneuvers.py: the per-sample labels and the test-vector
categories (idle, start, stop, straight, turn_left, turn_right, reverse).

The vectors below are closed-form profiles -- a speed held or ramped at a
known rate, a known path curvature -- so every expected label follows from
the numbers in Thresholds, not from running the code and copying its
output. Pure Python, runs with nothing sourced:

    python3 -m pytest src/sensor_logger/test/ -v
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sensor_logger.maneuvers import (  # noqa: E402
    ACCELERATING, BRAKING, CATEGORIES, IDLE, INVALID, REVERSE, STRAIGHT,
    TURN_LEFT, TURN_RIGHT, Thresholds, check_session, label_seconds,
    label_series)

TH = Thresholds()
DT = 0.02          # 50 Hz odometry
FULL_LOCK = 0.74   # 1/m: tan(0.26 rad) / 0.36 m wheelbase, docs/hardware-reference.md


def profile(*segments):
    """Build (t, speed, yaw_rate) from segments.

    ('hold', seconds, speed, curvature) or
    ('ramp', seconds, v_from, v_to, curvature); yaw_rate = speed * curvature,
    exactly what vesc_to_odom computes (speed * tan(steer) / wheelbase).
    """
    t, speed, yaw = [], [], []
    now = 0.0
    for seg in segments:
        kind, seconds = seg[0], seg[1]
        steps = round(seconds / DT)
        for k in range(steps):
            if kind == 'hold':
                v, curv = seg[2], seg[3]
            else:
                v0, v1, curv = seg[2], seg[3], seg[4]
                v = v0 + (v1 - v0) * k / steps
            t.append(now)
            speed.append(v)
            yaw.append(v * curv)
            now = round(now + DT, 9)
    return t, speed, yaw


# The test vectors. Each is how the category's protocol looks in /odom
# when driven the way CATEGORIES[...].protocol says.
VECTORS = {
    'idle': profile(('hold', 3.0, 0.0, 0.0)),
    # 1 m/s^2 ramp: twice the accel threshold, so unambiguous.
    'start': profile(('hold', 1.0, 0.0, 0.0), ('ramp', 1.0, 0.0, 1.0, 0.0),
                     ('hold', 1.0, 1.0, 0.0)),
    'stop': profile(('hold', 1.0, 1.0, 0.0), ('ramp', 1.0, 1.0, 0.0, 0.0),
                    ('hold', 1.5, 0.0, 0.0)),
    'straight': profile(('hold', 3.0, 1.0, 0.0)),
    'turn_left': profile(('hold', 3.0, 1.0, +FULL_LOCK)),
    'turn_right': profile(('hold', 3.0, 1.0, -FULL_LOCK)),
    'reverse': profile(('hold', 2.0, -0.5, 0.0)),
}
CHECKED = [name for name, cat in CATEGORIES.items() if cat.checked]


def test_every_checked_category_has_a_vector():
    assert sorted(VECTORS) == sorted(CHECKED)


@pytest.mark.parametrize('vector', sorted(VECTORS))
def test_each_vector_passes_its_own_category_and_no_other(vector):
    """The core property: a recording is recognised as the maneuver it is,
    and NOT as any other. Passing only its own category is what makes the
    folder name trustworthy."""
    t, speed, yaw = VECTORS[vector]
    labels = label_series(t, speed, yaw)
    passed = {cat for cat in CHECKED if check_session(cat, t, speed, labels).passed}
    assert passed == {vector}


def _label_at(vector, when):
    t, speed, yaw = VECTORS[vector]
    labels = label_series(t, speed, yaw)
    return labels[min(range(len(t)), key=lambda i: abs(t[i] - when))]


@pytest.mark.parametrize('vector,when,expected', [
    ('start', 0.5, IDLE),           # still parked
    ('start', 1.5, ACCELERATING),   # mid-ramp: dv/dt = 1 m/s^2 >= 0.5
    ('start', 2.8, STRAIGHT),       # cruise, ramp out of the 0.3 s window
    ('stop', 0.5, STRAIGHT),
    ('stop', 1.5, BRAKING),         # dv/dt = -1 m/s^2
    ('stop', 3.0, IDLE),
    ('turn_left', 1.5, TURN_LEFT),
    ('turn_right', 1.5, TURN_RIGHT),
    ('reverse', 1.0, REVERSE),
])
def test_vector_labels_at_known_instants(vector, when, expected):
    assert _label_at(vector, when) == expected


def test_positive_yaw_rate_is_a_left_turn():
    """REP-103: z up, so +yaw rate is counter-clockwise = left, forwards."""
    labels = label_series([0.0], [1.0], [+0.5])
    assert labels == [TURN_LEFT]
    assert label_series([0.0], [1.0], [-0.5]) == [TURN_RIGHT]


@pytest.mark.parametrize('speed', [0.5, 1.0, 2.0])
def test_turn_detection_uses_curvature_not_yaw_rate(speed):
    """Same steering, different speed, same label. At 0.5 m/s full lock
    is only 0.37 rad/s of yaw, which a yaw-rate threshold tuned at speed
    would miss."""
    assert label_series([0.0], [speed], [speed * FULL_LOCK]) == [TURN_LEFT]


@pytest.mark.parametrize('yaw_rate,expected', [
    (+0.25, TURN_LEFT),                       # exactly at 0.25 1/m (v = 1)
    (math.nextafter(0.25, 0.0), STRAIGHT),    # one float step below
    (-0.25, TURN_RIGHT),
    (math.nextafter(-0.25, 0.0), STRAIGHT),
])
def test_curvature_threshold_boundary(yaw_rate, expected):
    assert TH.turn_curvature_per_m == 0.25
    assert label_series([0.0], [1.0], [yaw_rate]) == [expected]


@pytest.mark.parametrize('speed,expected', [
    (0.10, STRAIGHT),                         # at still_speed: moving
    (math.nextafter(0.10, 0.0), IDLE),        # just under: stopped
    (0.0, IDLE),
    (-0.10, REVERSE),
    (math.nextafter(-0.10, 0.0), IDLE),
])
def test_still_speed_boundary(speed, expected):
    assert TH.still_speed_mps == 0.10
    assert label_series([0.0], [speed], [0.0]) == [expected]


# Accel over 0.25 s (exact in binary): dv = 0.125 m/s is exactly 0.5 m/s^2.
@pytest.mark.parametrize('v0,v1,expected', [
    (1.0, 1.125, ACCELERATING),
    (1.0, 1.12, STRAIGHT),        # 0.48 m/s^2
    (1.125, 1.0, BRAKING),
    (1.12, 1.0, STRAIGHT),
])
def test_accel_threshold_boundary(v0, v1, expected):
    assert TH.accel_mps2 == 0.5
    assert label_series([0.0, 0.25], [v0, v1], [0.0, 0.0])[1] == expected


def test_accel_needs_half_a_window_of_history():
    """A 1 m/s jump in 0.1 s is 10 m/s^2 -- but 0.1 s is under half the
    0.3 s window, so it is too little history to difference, and the
    sample is labelled by speed alone. At 0.15 s (exactly half) it counts."""
    assert label_series([0.0, 0.1], [1.0, 2.0], [0.0, 0.0])[1] == STRAIGHT
    assert label_series([0.0, 0.15], [1.0, 2.0], [0.0, 0.0])[1] == ACCELERATING


def test_samples_older_than_the_window_are_forgotten():
    """At t=0.75 the window holds only t=0.5 (same speed), so the car is
    cruising. Differencing against t=0 instead would give 1.33 m/s^2."""
    labels = label_series([0.0, 0.5, 0.75], [0.0, 1.0, 1.0], [0.0, 0.0, 0.0])
    assert labels[2] == STRAIGHT


def test_reverse_outranks_braking():
    """Slowing down while backing up is still reversing."""
    labels = label_series([0.0, 0.25], [-1.0, -0.5], [0.0, 0.0])
    assert labels[1] == REVERSE


@pytest.mark.parametrize('speed,yaw', [
    (math.nan, 0.0), (math.inf, 0.0), (-math.inf, 0.0),
    (1.0, math.nan), (1.0, math.inf),
])
def test_non_finite_samples_are_invalid(speed, yaw):
    assert label_series([0.0], [speed], [yaw]) == [INVALID]


def test_a_nan_inside_the_window_is_not_differenced():
    """Sample 1 differences against sample 0, which is NaN, so it gets no
    acceleration and is labelled by speed: straight, not accelerating."""
    labels = label_series([0.0, 0.25], [math.nan, 2.0], [0.0, 0.0])
    assert labels == [INVALID, STRAIGHT]


def test_empty_and_single_sample():
    assert label_series([], [], []) == []
    assert label_series([5.0], [0.0], [0.0]) == [IDLE]


def test_duplicate_timestamps_are_allowed():
    assert label_series([1.0, 1.0], [1.0, 1.0], [0.0, 0.0]) == [STRAIGHT, STRAIGHT]


def test_rejects_mismatched_lengths():
    with pytest.raises(ValueError, match='same length'):
        label_series([0.0, 1.0], [0.0], [0.0, 0.0])


def test_rejects_decreasing_time():
    with pytest.raises(ValueError, match='non-decreasing'):
        label_series([0.0, 1.0, 0.5], [0.0] * 3, [0.0] * 3)


@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_rejects_non_finite_time(bad):
    with pytest.raises(ValueError, match='not finite'):
        label_series([bad], [0.0], [0.0])


def test_label_seconds_assigns_each_interval_to_its_start():
    # sample 0 owns [0,1), sample 1 owns [1,3); the last sample owns nothing.
    assert label_seconds([0.0, 1.0, 3.0], ['a', 'a', 'b']) == {'a': 3.0}
    assert label_seconds([0.0, 1.0, 3.0], ['a', 'b', 'b']) == {'a': 1.0, 'b': 2.0}
    assert label_seconds([], []) == {}
    assert label_seconds([2.0], ['a']) == {}


# --- check_session at its boundaries, on hand-built label lists ---------
# 0.25 s steps are exact in binary, so label seconds are exact sums.

def _steps(n):
    return [0.25 * i for i in range(n)]


def test_unknown_category_is_refused():
    with pytest.raises(ValueError, match="unknown category 'turnleft'"):
        check_session('turnleft', [0.0], [0.0], [IDLE])


def test_min_label_seconds_boundary():
    # 9 samples = 8 intervals = 2.00 s of turn_left: exactly the minimum.
    t = _steps(9)
    assert check_session('turn_left', t, [1.0] * 9, [TURN_LEFT] * 9).passed
    # 8 samples = 1.75 s: short.
    result = check_session('turn_left', t[:8], [1.0] * 8, [TURN_LEFT] * 8)
    assert not result.passed
    assert result.failures == ['turn_left: 1.75s, needs at least 2.00s']


def test_max_label_seconds_boundary():
    # 2.0 s left then exactly 0.5 s right (allowed), then 0.75 s (not).
    ok = [TURN_LEFT] * 8 + [TURN_RIGHT] * 2 + [TURN_LEFT]
    assert check_session('turn_left', _steps(11), [1.0] * 11, ok).passed
    bad = [TURN_LEFT] * 8 + [TURN_RIGHT] * 3 + [TURN_LEFT]
    result = check_session('turn_left', _steps(12), [1.0] * 12, bad)
    assert result.failures == ['turn_right: 0.75s, allowed at most 0.50s']


def test_idle_fraction_boundary():
    # 20 intervals: 19 idle = 95% passes, 18 idle = 90% fails.
    labels = [IDLE] * 19 + [STRAIGHT, IDLE]
    assert check_session('idle', _steps(21), [0.0] * 21, labels).passed
    labels = [IDLE] * 18 + [STRAIGHT, STRAIGHT, IDLE]
    result = check_session('idle', _steps(21), [0.0] * 21, labels)
    assert result.failures == ['idle: 90% of the session, needs at least 95%']


def _start_labels():
    # 0.5 s idle (t=0, 0.25), then 0.5 s accelerating, then straight.
    return [IDLE, IDLE, ACCELERATING, ACCELERATING, STRAIGHT, STRAIGHT]


def test_start_needs_an_idle_beginning():
    speed = [0.0, 0.0, 0.3, 0.6, 1.0, 1.0]
    assert check_session('start', _steps(6), speed, _start_labels()).passed
    labels = _start_labels()
    labels[1] = STRAIGHT          # moving at t=0.25, inside the first 0.5 s
    result = check_session('start', _steps(6), speed, labels)
    assert result.failures == ['not idle for the first 0.50s']


def test_start_needs_a_real_peak_speed():
    speed = [0.0, 0.0, 0.2, 0.4, 0.49, 0.49]   # never reaches 0.5 m/s
    result = check_session('start', _steps(6), speed, _start_labels())
    assert result.failures == ['peak speed 0.49 m/s, needs at least 0.50 m/s']


def test_stop_needs_an_idle_end():
    # Ends at t=1.5, so the last 0.5 s is (1.0, 1.5]: t=1.0 itself, still
    # braking, is just outside it -- the mirror of start's [0, 0.5).
    labels = [STRAIGHT, STRAIGHT, BRAKING, BRAKING, BRAKING, IDLE, IDLE]
    speed = [1.0, 1.0, 0.7, 0.4, 0.1, 0.0, 0.0]
    assert check_session('stop', _steps(7), speed, labels).passed
    labels[5] = STRAIGHT          # t=1.25 is inside the last 0.5 s
    result = check_session('stop', _steps(7), speed, labels)
    assert result.failures == ['not idle for the last 0.50s']


def test_a_session_too_short_for_its_idle_window_fails():
    result = check_session('start', [0.0, 0.25], [0.0, 0.0], [IDLE, IDLE])
    assert 'session is 0.25s, shorter than the 0.50s idle start it needs' in result.failures


def test_reverse_peak_speed_counts_magnitude():
    t = _steps(9)
    result = check_session('reverse', t, [-0.5] * 9, [REVERSE] * 9)
    assert result.passed
    assert result.peak_speed_mps == 0.5


@pytest.mark.parametrize('n', [0, 1])
def test_checked_category_with_no_data_fails(n):
    result = check_session('straight', [0.0] * n, [1.0] * n, [STRAIGHT] * n)
    assert not result.passed
    assert result.failures == [f'only {n} odometry sample(s); nothing to check']


def test_free_drive_is_labelled_but_never_checked():
    result = check_session('free_drive', [], [], [])
    assert (result.checked, result.passed, result.failures) == (False, True, [])


def test_check_rejects_mismatched_lengths():
    with pytest.raises(ValueError, match='same length'):
        check_session('idle', [0.0, 1.0], [0.0, 0.0], [IDLE])
