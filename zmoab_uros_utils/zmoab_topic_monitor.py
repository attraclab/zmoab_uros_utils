#!/usr/bin/env python3

import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import (
    Int8,
    Int8MultiArray,
    Int16MultiArray,
    Int32MultiArray,
    UInt32MultiArray,
    UInt16MultiArray,
    Float32,
)
from sensor_msgs.msg import Imu


class TopicState:
    def __init__(self, name):
        self.name = name
        self.count = 0
        self.last_count = 0
        self.last_time = None
        self.last_period = None
        self.max_gap = 0.0
        self.stale_count = 0
        self.first_msg_time = None

    def tick(self):
        now = time.monotonic()

        if self.last_time is not None:
            gap = now - self.last_time
            self.last_period = gap
            self.max_gap = max(self.max_gap, gap)
        else:
            self.first_msg_time = now

        self.last_time = now
        self.count += 1

    def age(self):
        if self.last_time is None:
            return None
        return time.monotonic() - self.last_time

    def hz_since_last_report(self, report_dt):
        diff = self.count - self.last_count
        self.last_count = self.count
        return diff / max(report_dt, 1e-6)


class ZmoabTopicMonitor(Node):
    def __init__(self):
        super().__init__("zmoab_topic_monitor")

        self.declare_parameter("stale_sec", 1.0)
        self.declare_parameter("debug_stale_sec", 3.0)
        self.declare_parameter("report_sec", 1.0)
        self.declare_parameter("log_dir", os.path.expanduser("~/zmoab_uros_debug/logs"))
        self.declare_parameter("use_best_effort", False)

        self.stale_sec = float(self.get_parameter("stale_sec").value)
        self.debug_stale_sec = float(self.get_parameter("debug_stale_sec").value)
        self.report_sec = float(self.get_parameter("report_sec").value)
        self.log_dir = self.get_parameter("log_dir").value
        self.use_best_effort = bool(self.get_parameter("use_best_effort").value)

        os.makedirs(self.log_dir, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.log_dir, f"zmoab_topic_monitor_{stamp}.csv")
        self.event_path = os.path.join(self.log_dir, f"zmoab_events_{stamp}.log")
        self.debug_csv_path = os.path.join(self.log_dir, f"zmoab_debug_{stamp}.csv")

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "wall_time",
            "monotonic",
            "topic",
            "count_total",
            "hz_report",
            "age_sec",
            "last_period_sec",
            "max_gap_sec",
            "status",
        ])
        self.csv_file.flush()

        self.debug_csv_file = open(self.debug_csv_path, "w", newline="")
        self.debug_csv_writer = csv.writer(self.debug_csv_file)
        self.debug_csv_writer.writerow([
            "wall_time",
            "monotonic",

            "esp_millis",
            "timer_callback_count",
            "executor_spin_count",
            "pub_error_count",
            "last_pub_error_code",
            "last_failed_pub_id",
            "agent_ping_ok_count",
            "agent_ping_fail_count",
            "free_heap",
            "min_free_heap",
            "last_timer_callback_time_us",
            "max_timer_callback_time_us",

            "pwm_pub_ok_count",
            "sbus_pub_ok_count",
            "imu_pub_ok_count",
            "rpm_pub_ok_count",
            "error_pub_ok_count",
            "enc_pub_ok_count",
            "cart_mode_pub_ok_count",
            "switch_pub_ok_count",
            "vin_pub_ok_count",
            "sbus_fs_pub_ok_count",
            "debug_pub_ok_count",
            "timer_watchdog_trip_count",
        ])
        self.debug_csv_file.flush()
        

        if self.use_best_effort:
            self.qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        else:
            self.qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )

        self.topics = {
            "/zmoab/cart_mode": Int8,
            "/zmoab/encoder": Int32MultiArray,
            "/zmoab/error": Int16MultiArray,
            "/zmoab/imu": Imu,
            "/zmoab/pwm_in": Int16MultiArray,
            "/zmoab/rpm_fb": Int16MultiArray,
            "/zmoab/sbus_rc_ch": UInt16MultiArray,
            "/zmoab/switch": Int8MultiArray,
            "/zmoab/vin": Float32,
            "/zmoab/debug": UInt32MultiArray,
        }

        self.states = {name: TopicState(name) for name in self.topics.keys()}
        self.prev_all_stale = False
        self.last_report_mono = time.monotonic()

        for topic_name, msg_type in self.topics.items():
            if topic_name == "/zmoab/debug":
                callback = self.debug_callback
            else:
                callback = self.make_callback(topic_name)

            self.create_subscription(
                msg_type,
                topic_name,
                callback,
                self.qos,
            )

        self.create_timer(self.report_sec, self.report)

        self.get_logger().info(f"Logging CSV to: {self.csv_path}")
        self.get_logger().info(f"Logging events to: {self.event_path}")
        self.get_logger().info(f"Stale threshold: {self.stale_sec:.2f} sec")
        self.get_logger().info(f"Debug stale threshold: {self.debug_stale_sec:.2f} sec")
        self.get_logger().info(f"QoS reliability: {'BEST_EFFORT' if self.use_best_effort else 'RELIABLE'}")
        self.get_logger().info(f"Logging debug CSV to: {self.debug_csv_path}")

    def make_callback(self, topic_name):
        def cb(msg):
            self.states[topic_name].tick()
        return cb

    def log_event(self, text):
        line = f"{datetime.now().isoformat(timespec='seconds')} {text}"
        self.get_logger().warn(line)
        with open(self.event_path, "a") as f:
            f.write(line + "\n")

    def report(self):
        now = time.monotonic()
        report_dt = now - self.last_report_mono
        self.last_report_mono = now

        wall = datetime.now().isoformat(timespec="seconds")

        stale_topics = []
        never_topics = []

        for topic, state in self.states.items():
            age = state.age()

            topic_stale_sec = self.debug_stale_sec if topic == "/zmoab/debug" else self.stale_sec

            if age is None:
                status = "NEVER_RECEIVED"
                never_topics.append(topic)
                age_out = ""
            elif age > topic_stale_sec:
                status = "STALE"
                stale_topics.append(topic)
                state.stale_count += 1
                age_out = f"{age:.3f}"
            else:
                status = "OK"
                age_out = f"{age:.3f}"

            hz = state.hz_since_last_report(report_dt)

            self.csv_writer.writerow([
                wall,
                f"{now:.3f}",
                topic,
                state.count,
                f"{hz:.3f}",
                age_out,
                "" if state.last_period is None else f"{state.last_period:.3f}",
                f"{state.max_gap:.3f}",
                status,
            ])
            

        self.csv_file.flush()

        normal_topic_names = [t for t in self.states.keys() if t != "/zmoab/debug"]

        normal_dead = all(
            (self.states[t].age() is None) or (self.states[t].age() > self.stale_sec)
            for t in normal_topic_names
        )

        debug_age = self.states["/zmoab/debug"].age()
        debug_dead = (debug_age is None) or (debug_age > self.debug_stale_sec)

        all_dead = normal_dead

        if normal_dead and not debug_dead:
            self.log_event("NORMAL_TOPICS_STOPPED_BUT_DEBUG_ALIVE")

        if normal_dead and debug_dead:
            self.log_event("NORMAL_TOPICS_AND_DEBUG_STOPPED")

        self.prev_all_stale = all_dead

        ok_count = len(self.states) - len(stale_topics) - len(never_topics)
        self.get_logger().info(
            f"OK={ok_count}, STALE={len(stale_topics)}, NEVER={len(never_topics)}"
        )

        if stale_topics:
            self.log_event("STALE_TOPICS: " + ", ".join(stale_topics))

    def debug_callback(self, msg):
        topic_name = "/zmoab/debug"
        self.states[topic_name].tick()

        data = list(msg.data)

        # Expecting at least 23 values based on your ESP32 debug layout
        if len(data) < 23:
            self.log_event(f"DEBUG_MSG_TOO_SHORT len={len(data)} data={data}")
            return

        wall = datetime.now().isoformat(timespec="seconds")
        now = time.monotonic()

        # Field 23 (timer_watchdog_trip_count) is only present on newer firmware
        # that grew the debug array to 24. Stay backward compatible with 23-field
        # firmware by defaulting to 0.
        watchdog_trip = data[23] if len(data) >= 24 else 0

        self.debug_csv_writer.writerow([
            wall,
            f"{now:.3f}",

            data[0],   # esp_millis
            data[1],   # timer_callback_count
            data[2],   # executor_spin_count
            data[3],   # pub_error_count
            data[4],   # last_pub_error_code
            data[5],   # last_failed_pub_id
            data[6],   # agent_ping_ok_count
            data[7],   # agent_ping_fail_count
            data[8],   # free_heap
            data[9],   # min_free_heap
            data[10],  # last_timer_callback_time_us
            data[11],  # max_timer_callback_time_us

            data[12],  # pwm_pub_ok_count
            data[13],  # sbus_pub_ok_count
            data[14],  # imu_pub_ok_count
            data[15],  # rpm_pub_ok_count
            data[16],  # error_pub_ok_count
            data[17],  # enc_pub_ok_count
            data[18],  # cart_mode_pub_ok_count
            data[19],  # switch_pub_ok_count
            data[20],  # vin_pub_ok_count
            data[21],  # sbus_fs_pub_ok_count
            data[22],  # debug_pub_ok_count
            watchdog_trip,  # timer_watchdog_trip_count (0 on older firmware)
        ])
        self.debug_csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass

        try:
            self.debug_csv_file.close()
        except Exception:
            pass

        super().destroy_node()


def main():
    rclpy.init()
    node = ZmoabTopicMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()