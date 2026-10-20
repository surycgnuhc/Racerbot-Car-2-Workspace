"""
bag_io.py

Opens a recording made by record_run.py and reads messages out of it --
the only part of the bag_* tools that touches files. Everything it
returns is plain data for bag_query.py to work on.

Reads MCAP, the rosbag2 storage format ROS 2 Jazzy records by default,
using the pure-Python `mcap` and `mcap-ros2-support` libraries rather than
rosbag2_py. That way the tools need no sourced ROS, run on a laptop that
has only a copied run folder, and decode messages from the definitions
stored inside the recording itself -- so a message type this machine has
never built (vesc_msgs, say) still decodes. Install, once per machine:

    pip install --user --break-system-packages mcap==1.6.0 mcap-ros2-support==0.6.0

(--break-system-packages is Ubuntu 24.04's PEP 668 opt-in; with --user it
writes only under ~/.local, never into apt's files.)

Accepts any of: a run directory (containing bag/), the bag directory
itself, or a single .mcap file. A recording split into several files
(bag_0.mcap, bag_1.mcap, ...) is read as one, merged by arrival time.
"""

import heapq
import re
from pathlib import Path

from race_diagnostics.bag_query import schema_has_header

INSTALL_HINT = ('pip install --user --break-system-packages '
                'mcap==1.6.0 mcap-ros2-support==0.6.0')


class BagError(Exception):
    """The recording cannot be opened. The message says what to do."""


def _require_mcap():
    try:
        from mcap.exceptions import McapError
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        raise BagError('reading recordings needs the mcap libraries, which are not '
                       f'installed. Install them once with:\n  {INSTALL_HINT}') from None
    return make_reader, DecoderFactory, McapError


def _damaged(file, exc):
    return BagError(f'{file} is damaged or was cut off mid-write (the recorder was '
                    f'killed, or the disk filled): {exc}. The mcap CLI\'s '
                    '`mcap recover` can usually rescue the messages before the damage.')


def _split_index(path):
    """bag_10.mcap sorts after bag_9.mcap."""
    numbers = re.findall(r'\d+', path.stem)
    return (int(numbers[-1]) if numbers else -1, path.name)


def find_mcap_files(path):
    """The .mcap files that make up the recording at `path`, in split order."""
    path = Path(path).expanduser()
    if not path.exists():
        raise BagError(f'{path} does not exist')
    if path.is_file():
        if path.suffix != '.mcap':
            raise BagError(f'{path} is not an .mcap file')
        return [path]
    candidates = [path / 'bag', path]
    for directory in candidates:
        files = sorted(directory.glob('*.mcap'), key=_split_index)
        if files:
            return files
    if any(d.is_dir() and any(d.glob('*.db3')) for d in candidates):
        raise BagError(f'the recording in {path} is in SQLite (.db3) format; these tools '
                       'read MCAP, the ROS 2 Jazzy default. Convert it with '
                       '`ros2 bag convert` or re-record with `-s mcap`.')
    raise BagError(f'no recording (.mcap files) found in {path} or {path / "bag"}')


class Bag:
    """One recording, possibly split across several .mcap files."""

    def __init__(self, path):
        self.files = find_mcap_files(path)
        self._make_reader, self._decoder_factory, self._mcap_error = _require_mcap()
        self.topics = {}             # topic -> {'type', 'count', 'header'}
        self.start_ns = None
        self.end_ns = None
        for file in self.files:
            try:
                with open(file, 'rb') as stream:
                    summary = self._make_reader(stream).get_summary()
            except self._mcap_error as exc:
                raise _damaged(file, exc) from None
            if summary is None:
                raise _damaged(file, 'it has no summary section')
            stats = summary.statistics
            if stats is not None and stats.message_count:
                if self.start_ns is None or stats.message_start_time < self.start_ns:
                    self.start_ns = stats.message_start_time
                if self.end_ns is None or stats.message_end_time > self.end_ns:
                    self.end_ns = stats.message_end_time
            counts = stats.channel_message_counts if stats is not None else {}
            for channel_id, channel in summary.channels.items():
                schema = summary.schemas.get(channel.schema_id)
                entry = self.topics.setdefault(channel.topic, {
                    'type': schema.name if schema else '',
                    'count': 0,
                    'header': bool(schema) and schema_has_header(
                        schema.data.decode('utf-8', 'replace')),
                })
                entry['count'] += counts.get(channel_id, 0)
        if self.start_ns is None:
            raise BagError(f'the recording in {path} contains no messages')

    @property
    def duration_s(self):
        return (self.end_ns - self.start_ns) / 1e9

    def check_topics(self, topics):
        """Raise BagError naming the real topics if any requested one is absent."""
        missing = [t for t in topics if t not in self.topics]
        if missing:
            raise BagError(f'not in this recording: {", ".join(missing)}\n'
                           f'topics it has: {", ".join(sorted(self.topics))}')

    def _time_bounds(self, start_s, end_s):
        start = None if start_s is None else self.start_ns + int(start_s * 1e9)
        end = None if end_s is None else self.start_ns + int(end_s * 1e9)
        return start, end

    def raw(self, topics=None, start_s=None, end_s=None):
        """(topic, arrival_ns, serialized bytes), in arrival order, undecoded."""
        start, end = self._time_bounds(start_s, end_s)

        def one(file):
            with open(file, 'rb') as stream:
                reader = self._make_reader(stream)
                try:
                    for _schema, channel, message in reader.iter_messages(
                            topics=topics, starting_at=start, ending_before=end):
                        yield channel.topic, message.log_time, message.data
                except self._mcap_error as exc:
                    raise _damaged(file, exc) from None

        return heapq.merge(*(one(f) for f in self.files), key=lambda m: m[1])

    def decoded(self, topics=None, start_s=None, end_s=None):
        """(topic, arrival_ns, decoded message), in arrival order."""
        start, end = self._time_bounds(start_s, end_s)

        def one(file):
            with open(file, 'rb') as stream:
                reader = self._make_reader(stream, decoder_factories=[self._decoder_factory()])
                try:
                    for _schema, channel, message, decoded in reader.iter_decoded_messages(
                            topics=topics, starting_at=start, ending_before=end):
                        yield channel.topic, message.log_time, decoded
                except self._mcap_error as exc:
                    raise _damaged(file, exc) from None

        return heapq.merge(*(one(f) for f in self.files), key=lambda m: m[1])
