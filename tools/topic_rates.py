"""Measure arrival rates for live ROS 2 topics."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message


@dataclass
class TopicStats:
    """Online arrival-time statistics for one ROS 2 topic."""

    samples: int = 0
    first_ns: int | None = None
    last_ns: int | None = None
    interval_count: int = 0
    mean_interval_ns: float = 0.0
    interval_m2: float = 0.0
    min_interval_ns: int | None = None
    max_interval_ns: int | None = None

    def add(self, now_ns: int) -> None:
        self.samples += 1

        if self.first_ns is None:
            self.first_ns = now_ns
            self.last_ns = now_ns
            return

        assert self.last_ns is not None

        interval_ns = now_ns - self.last_ns
        self.last_ns = now_ns
        self.interval_count += 1

        if (
            self.min_interval_ns is None
            or interval_ns < self.min_interval_ns
        ):
            self.min_interval_ns = interval_ns

        if (
            self.max_interval_ns is None
            or interval_ns > self.max_interval_ns
        ):
            self.max_interval_ns = interval_ns

        delta = interval_ns - self.mean_interval_ns
        self.mean_interval_ns += delta / self.interval_count
        self.interval_m2 += (
            delta * (interval_ns - self.mean_interval_ns)
        )

    @property
    def rate_hz(self) -> float | None:
        if (
            self.interval_count == 0
            or self.first_ns is None
            or self.last_ns is None
            or self.last_ns <= self.first_ns
        ):
            return None

        elapsed_s = (self.last_ns - self.first_ns) / 1e9
        return self.interval_count / elapsed_s

    @property
    def min_interval_ms(self) -> float | None:
        if self.min_interval_ns is None:
            return None

        return self.min_interval_ns / 1e6

    @property
    def max_interval_ms(self) -> float | None:
        if self.max_interval_ns is None:
            return None

        return self.max_interval_ns / 1e6

    @property
    def std_interval_ms(self) -> float | None:
        if self.interval_count == 0:
            return None

        variance_ns2 = self.interval_m2 / self.interval_count
        return math.sqrt(max(variance_ns2, 0.0)) / 1e6


def normalize_prefix(prefix: str) -> str:
    if not prefix.startswith("/"):
        prefix = "/" + prefix

    if not prefix.endswith("/"):
        prefix += "/"

    return prefix


def normalize_topic(topic: str) -> str:
    if topic.startswith("/"):
        return topic

    return "/fmu/out/" + topic


def is_internal_ros_topic(topic: str) -> bool:
    return topic in {
        "/parameter_events",
        "/rosout",
    }


def discover_topics(
    node: Node,
    timeout_s: float,
) -> dict[str, list[str]]:
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

        topics = dict(node.get_topic_names_and_types())

        if topics:
            return topics

    return dict(node.get_topic_names_and_types())


def spin_for(node: Node, duration_s: float) -> None:
    deadline = time.monotonic() + duration_s

    while rclpy.ok():
        remaining = deadline - time.monotonic()

        if remaining <= 0.0:
            return

        rclpy.spin_once(
            node,
            timeout_sec=min(remaining, 0.1),
        )


def format_value(value: float | None, digits: int) -> str:
    if value is None:
        return "-"

    return f"{value:.{digits}f}"


def print_results(
    topics: list[str],
    stats: dict[str, TopicStats],
) -> None:
    width = max(
        len("TOPIC"),
        *(len(topic) for topic in topics),
    )

    print()
    print(
        f"{'TOPIC':<{width}}  "
        f"{'RATE_HZ':>10}  "
        f"{'MIN_MS':>9}  "
        f"{'MAX_MS':>9}  "
        f"{'STD_MS':>9}  "
        f"{'SAMPLES':>8}"
    )

    for topic in topics:
        result = stats[topic]

        print(
            f"{topic:<{width}}  "
            f"{format_value(result.rate_hz, 3):>10}  "
            f"{format_value(result.min_interval_ms, 3):>9}  "
            f"{format_value(result.max_interval_ms, 3):>9}  "
            f"{format_value(result.std_interval_ms, 3):>9}  "
            f"{result.samples:>8}"
        )


def write_csv(
    path: Path,
    topics: list[str],
    topic_types: dict[str, str],
    stats: dict[str, TopicStats],
) -> None:
    if not path.parent.is_dir():
        raise ValueError(
            f"CSV parent directory does not exist: {path.parent}"
        )

    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)

        writer.writerow(
            [
                "topic",
                "type",
                "samples",
                "rate_hz",
                "min_interval_ms",
                "max_interval_ms",
                "std_interval_ms",
            ]
        )

        for topic in topics:
            result = stats[topic]

            writer.writerow(
                [
                    topic,
                    topic_types[topic],
                    result.samples,
                    (
                        ""
                        if result.rate_hz is None
                        else f"{result.rate_hz:.6f}"
                    ),
                    (
                        ""
                        if result.min_interval_ms is None
                        else f"{result.min_interval_ms:.6f}"
                    ),
                    (
                        ""
                        if result.max_interval_ms is None
                        else f"{result.max_interval_ms:.6f}"
                    ),
                    (
                        ""
                        if result.std_interval_ms is None
                        else f"{result.std_interval_ms:.6f}"
                    ),
                ]
            )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Measure ROS 2 topic arrival rates concurrently. With no "
            "topic arguments, all live non-internal topics that have "
            "publishers are measured."
        )
    )

    parser.add_argument(
        "topics",
        nargs="*",
        help=(
            "Specific topic names. Bare names resolve under /fmu/out/, "
            "for example 'vehicle_angular_velocity'."
        ),
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=5.0,
        help="Measurement duration in seconds (default: 5).",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=1.0,
        help=(
            "Subscription warm-up time in seconds before statistics "
            "are collected (default: 1)."
        ),
    )
    parser.add_argument(
        "--prefix",
        action="append",
        dest="prefixes",
        help=(
            "Restrict automatic discovery to this topic prefix. "
            "May be repeated. Without --prefix, all live non-internal "
            "ROS topics are considered."
        ),
    )
    parser.add_argument(
        "--show-zero",
        action="store_true",
        help=(
            "Also show selected topics that produced no samples during "
            "the measurement window."
        ),
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="Also write the measurements to a CSV file.",
    )

    args = parser.parse_args()

    if args.duration <= 0.0:
        parser.error("--duration must be greater than zero.")

    if args.warmup < 0.0:
        parser.error("--warmup must be non-negative.")

    return args


def main() -> int:
    args = parse_arguments()

    prefixes = tuple(
        normalize_prefix(prefix)
        for prefix in (args.prefixes or [])
    )

    rclpy.init()

    node = Node("px4_topic_rate_monitor")

    try:
        discovered = discover_topics(node, timeout_s=3.0)

        if args.topics:
            requested = [
                normalize_topic(topic)
                for topic in args.topics
            ]

            missing = [
                topic
                for topic in requested
                if topic not in discovered
            ]

            if missing:
                for topic in missing:
                    print(
                        f"ERROR: topic is not present in the ROS graph: "
                        f"{topic}",
                        file=sys.stderr,
                    )

                return 1

            topics = sorted(set(requested))

        else:
            candidates = sorted(
                topic
                for topic in discovered
                if not is_internal_ros_topic(topic)
                and (
                    not prefixes
                    or any(
                        topic.startswith(prefix)
                        for prefix in prefixes
                    )
                )
            )

            # ROS graph discovery includes topics that only have subscribers.
            # PX4 exposes DDS readers for /fmu/in/* even when nothing is
            # publishing to them. Only treat a discovered topic as a live
            # source when the ROS graph reports at least one publisher.
            topics = [
                topic
                for topic in candidates
                if node.get_publishers_info_by_topic(topic)
            ]

        if not topics:
            print(
                "ERROR: no live ROS topics with publishers were found.",
                file=sys.stderr,
            )
            return 1

        stats = {
            topic: TopicStats()
            for topic in topics
        }

        topic_types: dict[str, str] = {}
        subscriptions = []

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        for topic in topics:
            types = discovered[topic]

            if len(types) != 1:
                print(
                    f"ERROR: expected one type for {topic}, "
                    f"found {types}",
                    file=sys.stderr,
                )
                return 1

            type_name = types[0]

            try:
                message_type = get_message(type_name)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                print(
                    f"ERROR: cannot load {type_name} for {topic}: {exc}",
                    file=sys.stderr,
                )
                return 1

            topic_types[topic] = type_name

            def callback(
                _message: object,
                *,
                topic_name: str = topic,
            ) -> None:
                stats[topic_name].add(time.monotonic_ns())

            subscriptions.append(
                node.create_subscription(
                    message_type,
                    topic,
                    callback,
                    qos,
                )
            )

        print(
            f"Measuring {len(topics)} topic(s): "
            f"{args.warmup:.1f} s warm-up + "
            f"{args.duration:.1f} s measurement"
        )

        if args.warmup > 0.0:
            spin_for(node, args.warmup)

        for topic in topics:
            stats[topic] = TopicStats()

        spin_for(node, args.duration)

        report_topics = (
            topics
            if args.show_zero
            else [
                topic
                for topic in topics
                if stats[topic].samples > 0
            ]
        )

        if not report_topics:
            print(
                "ERROR: no samples were received during the "
                "measurement window.",
                file=sys.stderr,
            )
            return 1

        print_results(report_topics, stats)

        if args.csv is not None:
            write_csv(
                args.csv,
                report_topics,
                topic_types,
                stats,
            )
            print()
            print(f"Wrote CSV: {args.csv}")

        return 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
