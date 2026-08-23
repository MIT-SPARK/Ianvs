import heapq
import itertools
import functools

import tqdm
import tf2_ros
import rosbag2_py
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rclpy.logging import get_logger, get_logging_severity_from_string
from rosidl_runtime_py.utilities import get_message


def _all_nonempty(queues):
    status = [bool(q) for q in queues]
    return functools.reduce(lambda x, y: x and y, status, True)


def _valid_set(stamps, max_diff_ns):
    for lhs, rhs in itertools.combinations(stamps.items(), 2):
        diff_ns = abs(lhs[1] - rhs[1])
        if diff_ns >= max_diff_ns:
            return False

    return True


class ProgressTracker:
    """Time-based progress bar for iterating through rosbag."""

    def __init__(self, bag, start_time_ns=None, enabled=True):
        rel_start = start_time_ns if start_time_ns else bag.start_time
        rel_duration = bag.duration - (rel_start - bag.start_time)

        self.total_s = rel_duration * 1.0e-9
        self.enabled = enabled
        self.pbar = None
        self.last_stamp = None

    def __enter__(self):
        self.pbar = None
        self.last_stamp = None
        if self.enabled:
            fmt = "{l_bar}{bar}| {n:.2f}/{total:.2f} [s] [{elapsed}<{remaining}, {rate_fmt}{postfix}]"
            self.pbar = tqdm.tqdm(
                total=self.total_s, unit=" bag seconds", bar_format=fmt
            )

        return self

    def update(self, stamp):
        if self.pbar is not None and self.last_stamp is not None:
            self.pbar.update((stamp - self.last_stamp) * 1.0e-9)

        self.last_stamp = stamp

    def __exit__(self, exc_type, exc_msg, traceback):
        if self.pbar is not None:
            self.pbar.close()


class BagReader:
    """Wrapper around SequentialReader to provide slighter cleaner interface."""

    def __init__(self, bag_path):
        self._path = bag_path
        self._bag = rosbag2_py.SequentialReader()

    @property
    def start_time(self):
        return self._bag.get_metadata().starting_time.nanoseconds  # type: ignore

    @property
    def duration(self):
        return self._bag.get_metadata().duration.nanoseconds  # type: ignore

    @property
    def topics(self):
        return [x for x in self._typenames]

    def open(self):
        level = get_logging_severity_from_string("WARN")
        logger = get_logger("rosbag2_storage")
        logger.set_level(level)
        self._bag.open_uri(str(self._path))
        self._typenames = {}
        for info in self._bag.get_all_topics_and_types():  # type: ignore
            try:
                self._typenames[info.name] = get_message(info.type)
            except Exception as e:
                logger.error(f"Unknown message type: {e}")

    def close(self):
        self._bag.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def lookup_static_transform(self, parent, child):
        """Find the transform between the parent and child."""
        buffer = tf2_ros.BufferCore()
        for _, msg, _ in self.read_messages(["/tf_static"], progress=False):
            for x in msg.transforms:
                buffer.set_transform_static(x, "rosbag")

        return buffer.lookup_transform_core(parent, child, Time()).transform

    def read_messages(self, topics=None, start_time_ns=None, progress=True):
        self._bag.seek(0)

        if topics is not None:
            missing = [x for x in topics if x not in self._typenames]
            if len(missing) > 0:
                logger = get_logger("rosbag2_storage")
                logger.warning(f"Could not find {missing} (available: {self.topics})")
                return None

            self._bag.set_filter(rosbag2_py.StorageFilter(topics=topics))

        with ProgressTracker(self, start_time_ns, enabled=progress) as tracker:
            while self._bag.has_next():
                topic, data, t = self._bag.read_next()
                if start_time_ns is not None and t < start_time_ns:
                    continue

                tracker.update(t)
                yield topic, deserialize_message(data, self._typenames[topic]), t

    def read_synced_messages(
        self, topics, max_diff_ns=0, start_time_ns=None, progress=True
    ):
        logger = get_logger("rosbag2_storage")

        self._bag.seek(0)
        missing = [x for x in topics if x not in self._typenames]
        if len(missing) > 0:
            logger.warning(f"Could not find {missing} (available: {self.topics})")
            return None

        unknown = set([])
        queues = {t: list() for t in topics}
        self._bag.set_filter(rosbag2_py.StorageFilter(topics=topics))
        with ProgressTracker(self, start_time_ns, enabled=progress) as tracker:
            while self._bag.has_next():
                topic, data, t = self._bag.read_next()
                if start_time_ns is not None and t < start_time_ns:
                    continue

                tracker.update(t)
                queue = queues.get(topic)
                if queue is None and topic not in unknown:
                    logger.warning(f"Unexpected topic '{topic}'")
                    unknown.add(topic)
                    continue

                msg = deserialize_message(data, self._typenames[topic])
                heapq.heappush(queue, msg)

                while _all_nonempty(queues):
                    stamps = {t: q[0] for t, q in queues.items()}
                    if not _valid_set(stamps, max_diff_ns):
                        min_topic, _ = min(stamps.items(), key=lambda x: x[1])
                        queues[min_topic].pop(0)
                        continue

                    yield {t: q.pop(0) for t, q in queues.items()}
