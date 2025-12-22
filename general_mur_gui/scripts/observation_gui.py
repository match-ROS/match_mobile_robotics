#!/usr/bin/env python3
"""Interactive GUI to observe UR-related twist topics."""

import json
import sys
import threading
from collections import deque
from pathlib import Path

import rospy
from geometry_msgs.msg import Twist
from python_qt_binding import QtCore, QtWidgets  # type: ignore[attr-defined]
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# Available twist components that can be plotted
TWIST_COMPONENTS = (
    "linear.x",
    "linear.y",
    "linear.z",
    "angular.x",
    "angular.y",
    "angular.z",
)

DEFAULT_SELECTED_COMPONENTS = ("linear.x", "linear.y", "linear.z")

# Topics that appear in complete_ur_trajectory_follower_ff_only.launch
DEFAULT_TWIST_TOPICS = [
    "/ur_twist_mir_compensation",
    "/ur_twist_world_in_mir",
    "/laser_profile_offset_cmd_vel",
    "/orthogonal_twist",
    "/ur_twist_direction_world",
    "/ur_rotation_twist_world",
    "/ur_twist_world",
    "/ur_error_world",
    "/orthogonal_error",
]


class TwistBuffer:
    """Stores recent samples for a single twist topic."""

    def __init__(self, max_len):
        self.max_len = max_len
        self.timestamps = deque(maxlen=max_len)
        self.data = {component: deque(maxlen=max_len) for component in TWIST_COMPONENTS}
        self.lock = threading.Lock()

    def append(self, stamp, msg):
        values = {
            "linear.x": msg.linear.x,
            "linear.y": msg.linear.y,
            "linear.z": msg.linear.z,
            "angular.x": msg.angular.x,
            "angular.y": msg.angular.y,
            "angular.z": msg.angular.z,
        }
        with self.lock:
            self.timestamps.append(stamp)
            for component, component_values in self.data.items():
                component_values.append(values[component])

    def get_series(self, component):
        with self.lock:
            if not self.timestamps:
                return [], []
            times = list(self.timestamps)
            base = times[0]
            rel_times = [t - base for t in times]
            values = list(self.data[component])
        return rel_times, values

    def clear(self):
        with self.lock:
            self.timestamps.clear()
            for component_values in self.data.values():
                component_values.clear()


class ObservationGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        rospy.init_node("observation_gui", anonymous=True)

        robot_name = str(rospy.get_param("~robot_name", "mur620d"))
        prefix_ur_param = rospy.get_param("~prefix_ur", "UR10_r/")
        prefix_ur = str(prefix_ur_param)
        if not prefix_ur.endswith("/"):
            prefix_ur += "/"

        combined_topic = f"/{robot_name}/{prefix_ur}twist_controller/command_collision_free"

        twist_topics = list(DEFAULT_TWIST_TOPICS)
        twist_topics.append(combined_topic)
        extra_topics = rospy.get_param("~twist_topics", [])
        if isinstance(extra_topics, (list, tuple, set)):
            twist_topics.extend([str(topic) for topic in extra_topics if topic])
        elif extra_topics:
            twist_topics.append(str(extra_topics))
        self.twist_topics = sorted({topic for topic in twist_topics if topic})

        self.max_points = self._coerce_to_int(rospy.get_param("~max_points", 2000), 2000)
        self.update_interval_ms = self._coerce_to_int(rospy.get_param("~update_interval_ms", 200), 200)
        default_config_path = Path.home() / ".ros" / "observation_gui_config.json"
        config_param = rospy.get_param("~config_path", str(default_config_path))
        self.config_path = Path(str(config_param))

        self.buffers = {topic: TwistBuffer(self.max_points) for topic in self.twist_topics}

        self._subscribers = [
            rospy.Subscriber(topic, Twist, self._make_callback(topic), queue_size=10)
            for topic in self.twist_topics
        ]

        self._build_ui()
        self._start_update_timer()
        self._load_config(initial=True)

        rospy.loginfo("observation_gui ready. Monitoring %d twist topics.", len(self.twist_topics))

    def _make_callback(self, topic):
        def _callback(msg):
            self.buffers[topic].append(rospy.get_time(), msg)

        return _callback

    @staticmethod
    def _coerce_to_int(value, default):
        if isinstance(value, int):
            return value
        if isinstance(value, float):
            return int(value)
        try:
            return int(str(value))
        except (TypeError, ValueError):
            rospy.logwarn("Invalid integer parameter '%s', falling back to %d", value, default)
            return default

    def _build_ui(self):
        self.setWindowTitle("Observation GUI - Twist Monitor")
        self.resize(1100, 700)

        main_layout = QtWidgets.QVBoxLayout(self)

        controls_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(controls_layout)

        self.topic_list = QtWidgets.QListWidget()
        self.topic_list.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        for topic in self.twist_topics:
            self._insert_topic_item(topic, checked=True)
        topic_group = QtWidgets.QGroupBox("Twist Topics")
        topic_group_layout = QtWidgets.QVBoxLayout(topic_group)
        topic_group_layout.addWidget(self.topic_list)

        topic_buttons_layout = QtWidgets.QHBoxLayout()
        self.select_all_button = QtWidgets.QPushButton("Select All")
        self.select_all_button.clicked.connect(lambda: self._set_all_topics(True))
        self.deselect_all_button = QtWidgets.QPushButton("Deselect All")
        self.deselect_all_button.clicked.connect(lambda: self._set_all_topics(False))
        topic_buttons_layout.addWidget(self.select_all_button)
        topic_buttons_layout.addWidget(self.deselect_all_button)
        topic_group_layout.addLayout(topic_buttons_layout)

        add_topic_layout = QtWidgets.QHBoxLayout()
        self.custom_topic_input = QtWidgets.QLineEdit()
        self.custom_topic_input.setPlaceholderText("Topic name (e.g., /my_twist_topic)")
        self.add_topic_button = QtWidgets.QPushButton("Add Topic")
        self.add_topic_button.clicked.connect(self._handle_add_topic)
        add_topic_layout.addWidget(self.custom_topic_input)
        add_topic_layout.addWidget(self.add_topic_button)
        topic_group_layout.addLayout(add_topic_layout)
        controls_layout.addWidget(topic_group, stretch=2)

        component_group = QtWidgets.QGroupBox("Components")
        component_layout = QtWidgets.QVBoxLayout(component_group)
        self.component_checks = {}
        for component in TWIST_COMPONENTS:
            cb = QtWidgets.QCheckBox(component)
            cb.setChecked(component in DEFAULT_SELECTED_COMPONENTS)
            self.component_checks[component] = cb
            component_layout.addWidget(cb)
        controls_layout.addWidget(component_group, stretch=1)

        button_layout = QtWidgets.QVBoxLayout()
        self.clear_button = QtWidgets.QPushButton("Clear Data")
        self.clear_button.clicked.connect(self._clear_data)
        button_layout.addWidget(self.clear_button)

        self.save_config_button = QtWidgets.QPushButton("Save Config")
        self.save_config_button.clicked.connect(self._save_config)
        button_layout.addWidget(self.save_config_button)

        self.load_config_button = QtWidgets.QPushButton("Load Config")
        self.load_config_button.clicked.connect(lambda: self._load_config())
        button_layout.addWidget(self.load_config_button)
        button_layout.addStretch()
        controls_layout.addLayout(button_layout)

        self.figure = Figure(figsize=(8, 5))
        self.canvas = FigureCanvas(self.figure)
        self.axes = self.figure.add_subplot(111)
        self.axes.set_xlabel("Time [s]")
        self.axes.set_ylabel("Twist value")
        self.axes.grid(True)
        main_layout.addWidget(self.canvas, stretch=1)

    def _set_all_topics(self, enabled):
        state = QtCore.Qt.Checked if enabled else QtCore.Qt.Unchecked
        for index in range(self.topic_list.count()):
            self.topic_list.item(index).setCheckState(state)

    def _handle_add_topic(self):
        topic = self.custom_topic_input.text().strip()
        if not topic:
            return
        self._add_topic(topic, auto_select=True)
        self.custom_topic_input.clear()

    def _add_topic(self, topic, auto_select):
        topic = topic.strip()
        if not topic:
            return
        if topic in self.buffers:
            self._set_topic_check_state(topic, auto_select)
            return
        self.twist_topics.append(topic)
        self.twist_topics.sort()
        self.buffers[topic] = TwistBuffer(self.max_points)
        self._subscribers.append(
            rospy.Subscriber(topic, Twist, self._make_callback(topic), queue_size=10)
        )
        self._insert_topic_item(topic, auto_select)

    def _insert_topic_item(self, topic, checked):
        item = QtWidgets.QListWidgetItem(topic)
        item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
        item.setCheckState(QtCore.Qt.Checked if checked else QtCore.Qt.Unchecked)
        insert_row = self.topic_list.count()
        for index in range(self.topic_list.count()):
            if self.topic_list.item(index).text() > topic:
                insert_row = index
                break
        self.topic_list.insertItem(insert_row, item)

    def _find_topic_item(self, topic):
        for index in range(self.topic_list.count()):
            item = self.topic_list.item(index)
            if item.text() == topic:
                return item
        return None

    def _set_topic_check_state(self, topic, enabled):
        item = self._find_topic_item(topic)
        if item is None:
            return
        item.setCheckState(QtCore.Qt.Checked if enabled else QtCore.Qt.Unchecked)

    def _start_update_timer(self):
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._update_plot)
        self.timer.start(self.update_interval_ms)

    def _selected_topics(self):
        result = []
        for index in range(self.topic_list.count()):
            item = self.topic_list.item(index)
            if item.checkState() == QtCore.Qt.Checked:
                result.append(item.text())
        return result

    def _selected_components(self):
        return [name for name, checkbox in self.component_checks.items() if checkbox.isChecked()]

    def _gather_config_payload(self):
        topics_state = []
        for index in range(self.topic_list.count()):
            item = self.topic_list.item(index)
            topics_state.append(
                {
                    "name": item.text(),
                    "enabled": item.checkState() == QtCore.Qt.Checked,
                }
            )
        components_state = {
            name: checkbox.isChecked() for name, checkbox in self.component_checks.items()
        }
        return {
            "topics": topics_state,
            "components": components_state,
        }

    def _save_config(self):
        payload = self._gather_config_payload()
        try:
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            with self.config_path.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
            rospy.loginfo("Saved observation_gui config to %s", self.config_path)
        except OSError as exc:
            rospy.logerr("Failed to save config to %s: %s", self.config_path, exc)

    def _load_config(self, initial=False):
        if not self.config_path.exists():
            if not initial:
                rospy.logwarn("Config file %s not found", self.config_path)
            return
        try:
            with self.config_path.open("r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, json.JSONDecodeError) as exc:
            rospy.logerr("Failed to load config %s: %s", self.config_path, exc)
            return

        topics = data.get("topics", [])
        if isinstance(topics, list) and topics:
            self._set_all_topics(False)
            for entry in topics:
                topic_name = str(entry.get("name", "")).strip()
                if not topic_name:
                    continue
                enabled = bool(entry.get("enabled", True))
                self._add_topic(topic_name, auto_select=enabled)

        components = data.get("components", {})
        if isinstance(components, dict):
            for name, state in components.items():
                checkbox = self.component_checks.get(name)
                if checkbox is not None:
                    checkbox.setChecked(bool(state))

        rospy.loginfo("Loaded observation_gui config from %s", self.config_path)

    def _clear_data(self):
        for buffer in self.buffers.values():
            buffer.clear()
        self.axes.clear()
        self.axes.set_xlabel("Time [s]")
        self.axes.set_ylabel("Twist value")
        self.axes.grid(True)
        self.canvas.draw_idle()

    def _update_plot(self):
        if rospy.is_shutdown():
            QtWidgets.QApplication.quit()
            return

        topics = self._selected_topics()
        components = self._selected_components()

        self.axes.clear()
        self.axes.set_xlabel("Time [s]")
        self.axes.set_ylabel("Twist value")
        self.axes.grid(True)

        if not topics or not components:
            self.axes.set_title("Select at least one topic and one component")
            self.canvas.draw_idle()
            return

        plotted_any = False
        for topic in topics:
            buffer = self.buffers.get(topic)
            if buffer is None:
                continue
            for component in components:
                times, values = buffer.get_series(component)
                if not times:
                    continue
                label = f"{topic} [{component}]"
                self.axes.plot(times, values, label=label)
                plotted_any = True

        if plotted_any:
            self.axes.legend(loc="upper right", fontsize="small")
        else:
            self.axes.set_title("Waiting for data...")

        self.canvas.draw_idle()


def main():
    app = QtWidgets.QApplication(sys.argv)
    gui = ObservationGUI()
    gui.show()

    rospy.on_shutdown(app.quit)
    sys.exit(app.exec_())


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
