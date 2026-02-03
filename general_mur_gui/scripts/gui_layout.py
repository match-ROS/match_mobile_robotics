from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QGridLayout,
    QSlider,
    QLineEdit,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QTableWidget,
    QCheckBox,
    QTableWidgetItem,
    QGroupBox,
    QTabWidget,
    QSpinBox,
    QDoubleSpinBox,
    QTextEdit,
    QComboBox,
    QDoubleSpinBox,
    QDialogButtonBox,
    QFormLayout,
    QDialog,
    QMessageBox,
    QButtonGroup,
    QRadioButton
)
from PyQt5 import QtCore
from PyQt5.QtCore import QTimer, pyqtSignal, pyqtSlot
from typing import Any, cast
from PyQt5.QtGui import QIcon
from ros_interface import start_status_update, ur_follow_trajectory, open_rviz, launch_drivers, quit_drivers, turn_on_arm_controllers, turn_on_twist_controllers, stop_mir_motion, stop_idx_advancer, stop_ur_motion, stop_all_but_drivers
from ros_interface import enable_all_urs, move_to_home_pose, parse_mir_path, parse_ur_path, move_mir_to_start_pose, move_ur_to_start_pose, mir_follow_trajectory, increment_path_index, target_broadcaster
from ros_interface import ROSInterface
import os
import math
import html
import json


Qt = cast(Any, QtCore.Qt)


class EnterSpinBox(QSpinBox):
    """QSpinBox that emits returnPressed when user presses Enter."""
    returnPressed = pyqtSignal()

    def keyPressEvent(self, event):
        key_return = getattr(Qt, 'Key_Return', None)
        key_enter = getattr(Qt, 'Key_Enter', None)
        if event.key() in [k for k in (key_return, key_enter) if k is not None]:
            self.returnPressed.emit()
        super().keyPressEvent(event)


class ROSGui(QWidget):
    path_idx = pyqtSignal(int)
    medians = pyqtSignal(float, float)
    ros_log_signal = pyqtSignal(str, str, str)  # level, node, text
    
    def __init__(self):
        super().__init__()

        # ROS + window
        self.ros_interface = ROSInterface(self)
        self.setWindowTitle("General MuR GUI")
        self.setWindowIcon(QIcon(os.path.join(os.path.dirname(__file__), '../img/Logo.png')))
        self.setGeometry(100, 100, 1600, 850)
        main_layout = QHBoxLayout()
        # Left column
        left_layout = QVBoxLayout()
        self.status_label = QLabel("Controller Status: Not Checked")
        self.status_label.setStyleSheet("border: 1px solid black; padding: 5px;")
        left_layout.addWidget(self.status_label)
        # Battery
        self.battery_group = QGroupBox("Battery Status"); self.battery_layout = QVBoxLayout(); self.battery_group.setLayout(self.battery_layout); left_layout.addWidget(self.battery_group); self.battery_labels = {}
        # Selection
        selection_group = QGroupBox("Robot and UR Selection"); selection_layout = QHBoxLayout(); robot_layout = QVBoxLayout()
        self.robots = {n: QCheckBox(n) for n in ["mur620a", "mur620b", "mur620c", "mur620d"]}
        for cb in self.robots.values(): robot_layout.addWidget(cb)
        for name, cb in self.robots.items(): cb.stateChanged.connect(lambda _, r=name: self.ros_interface.check_and_subscribe_battery())
        for robot in self.robots.keys():
            row = QHBoxLayout(); row.addWidget(QLabel(robot)); mir_label = QLabel("MiR: –"); ur_label = QLabel("UR: –"); row.addWidget(mir_label); row.addWidget(ur_label); self.battery_labels[robot] = (mir_label, ur_label); self.battery_layout.addLayout(row)
        ur_layout = QVBoxLayout(); ur_layout.addWidget(QLabel("Select URs:")); self.ur10_l = QCheckBox("UR10_l"); self.ur10_r = QCheckBox("UR10_r"); self.ur10_l.setChecked(False); self.ur10_r.setChecked(True); ur_layout.addWidget(self.ur10_l); ur_layout.addWidget(self.ur10_r)
        selection_layout.addLayout(robot_layout); selection_layout.addLayout(ur_layout); selection_group.setLayout(selection_layout); left_layout.addWidget(selection_group)
        
        # Setup
        setup_group = QGroupBox("Setup Functions"); setup_layout = QVBoxLayout();
        setup_buttons = {
            "Check Status": lambda: start_status_update(self),
            "Launch Drivers": lambda: launch_drivers(self),
            "Prepare Driver Cleanup Channel": lambda: self.ros_interface.prime_driver_cleanup_sessions(),
            "Launch Keyence Scanner": lambda: self.ros_interface.launch_keyence_scanner(),
            "Launch Flow Sensor Bridge": lambda: self.ros_interface.launch_flow_sensor_bridge(),
            "Start Dynamixel Driver": lambda: self.ros_interface.start_dynamixel_driver(),
            "Stop Dynamixel Driver": lambda: self.ros_interface.stop_dynamixel_driver(),
            "Launch Strand Center Camera": lambda: self.ros_interface.launch_strand_center_app(),
            "Open RVIZ": lambda: open_rviz(self),
            "Start Roscore": lambda: self.ros_interface.start_roscore(),
            "Start Mocap": lambda: self.ros_interface.start_mocap(),
            "Start Sync": lambda: self.ros_interface.start_sync(),
        }
        for text, fn in setup_buttons.items():
            b = QPushButton(text); 
            if text=="Launch Keyence Scanner": self.btn_keyence=b
            if text=="Launch Flow Sensor Bridge":
                self.btn_flow_sensor=b
                b.setContextMenuPolicy(Qt.CustomContextMenu)
                b.customContextMenuRequested.connect(self._handle_flow_sensor_right_click)
            if text=="Launch Drivers":
                self.btn_launch_drivers=b
                b.setContextMenuPolicy(Qt.CustomContextMenu)
                b.customContextMenuRequested.connect(self._handle_launch_drivers_right_click)
            if text == "Start Roscore": self.btn_roscore = b
            elif text == "Start Mocap": self.btn_mocap = b
            elif text == "Start Sync": self.btn_sync = b
            b.clicked.connect(lambda _, f=fn: f()); b.setStyleSheet("background-color: lightgray;"); setup_layout.addWidget(b)
        
        self.workspace_input = QLineEdit(); default_path = self.get_relative_workspace_path(); self.workspace_input.setText(default_path); self.workspace_input.setPlaceholderText("Enter workspace name"); setup_layout.addWidget(QLabel("Workspace Name:")); setup_layout.addWidget(self.workspace_input); setup_group.setLayout(setup_layout); left_layout.addWidget(setup_group)
        main_layout.addLayout(left_layout)
        # Right column
        right_layout = QVBoxLayout(); controller_group = QGroupBox("Controller Functions"); controller_layout = QVBoxLayout(); controller_buttons = {
            "Enable all URs": lambda: enable_all_urs(self),
            "Turn on Arm Controllers": lambda: turn_on_arm_controllers(self),
            "Turn on Twist Controllers": lambda: turn_on_twist_controllers(self),
            "Move to Home Pose Left": lambda: move_to_home_pose(self, "UR10_l"),
            "Move to Home Pose Right": lambda: move_to_home_pose(self, "UR10_r"),
        }
        for text, fn in controller_buttons.items(): btn = QPushButton(text); btn.clicked.connect(lambda _, f=fn: f()); controller_layout.addWidget(btn)
        controller_group.setLayout(controller_layout); right_layout.addWidget(controller_group)
        
        # --- ROS log console on the far right ---
        log_group = QGroupBox("ROS Messages")
        log_layout = QVBoxLayout()

        self.ros_log_text = QTextEdit()
        self.ros_log_text.setReadOnly(True)
        self.ros_log_text.setLineWrapMode(QTextEdit.NoWrap)

        log_layout.addWidget(self.ros_log_text)

        # --- Log level filter checkboxes ---
        filter_layout = QHBoxLayout()
        self.chk_log_error = QCheckBox("Error")
        self.chk_log_warn = QCheckBox("Warning")
        self.chk_log_info = QCheckBox("Info")
        self.chk_log_debug = QCheckBox("Debug")

        # Default: Error/Warn/Info an, Debug aus
        self.chk_log_error.setChecked(True)
        self.chk_log_warn.setChecked(True)
        self.chk_log_info.setChecked(True)
        self.chk_log_debug.setChecked(False)

        # Clear-Button
        self.btn_log_clear = QPushButton("Clear")
        self.btn_log_clear.clicked.connect(self._clear_ros_log)

        # Filter-Checkboxen triggern Ansicht neu
        for cb in (self.chk_log_error, self.chk_log_warn,
                   self.chk_log_info, self.chk_log_debug):
            cb.stateChanged.connect(self._rebuild_ros_log_view)

        filter_layout.addWidget(self.chk_log_error)
        filter_layout.addWidget(self.chk_log_warn)
        filter_layout.addWidget(self.chk_log_info)
        filter_layout.addWidget(self.chk_log_debug)
        filter_layout.addWidget(self.btn_log_clear)

        log_layout.addLayout(filter_layout)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

        self.setLayout(main_layout)

        # connect ROS log signal after widgets exist
        self._ros_log_buffer = []
        self._ros_log_update_timer = QTimer(self)
        self._ros_log_update_timer.setSingleShot(True)
        self._ros_log_update_timer.setInterval(120)
        self._ros_log_update_timer.timeout.connect(self._flush_ros_log_view)
        self.ros_log_signal.connect(self._append_ros_log)

        
        self.setLayout(main_layout)
        # Timer
        self.status_timer = QTimer(); self.status_timer.timeout.connect(self.ros_interface.update_button_status); self.status_timer.start(2000)

    def closeEvent(self, event):
        """Stop periodic updates and tear down ROS before the app quits."""
        try:
            if hasattr(self, "status_timer"):
                self.status_timer.stop()
        except Exception as exc:
            print(f"Failed to stop status timer: {exc}")
        try:
            self.ros_interface.shutdown()
        except Exception as exc:
            print(f"Failed to shut down ROS interface: {exc}")
        super().closeEvent(event)

    def _handle_launch_drivers_right_click(self, _pos):
        """Stop driver terminals when the launch button is right-clicked."""
        quit_drivers(self)

    def _handle_flow_sensor_right_click(self, _pos):
        """Stop the flow sensor bridge when its button is right-clicked."""
        self.ros_interface.stop_flow_sensor_bridge()

    def _handle_start_signal_button(self):
        """Trigger the latched start condition via the ROS interface."""
        if hasattr(self, "ros_interface"):
            self.ros_interface.trigger_start_signal()

    def _publish_current_index(self):
        """Send the currently selected index back onto /path_index."""
        if not hasattr(self, "idx_spin"):
            return
        self.ros_interface.publish_path_index(self.idx_spin.value())

    def update_start_signal_visual(self, active: bool):
        button = getattr(self, "btn_start_signal", None)
        if button is None:
            return
        if active:
            button.setText("Start Signal ACTIVE (click to retrigger)")
            button.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold;")
        else:
            button.setText("Trigger Start Signal")
            button.setStyleSheet("background-color: #4caf50; color: white;")

  
    def _ros_log_level_enabled(self, level: str) -> bool:
        level = level.upper()
        if level == "ERROR":
            return self.chk_log_error.isChecked()
        if level in ("WARN", "WARNING"):
            return self.chk_log_warn.isChecked()
        if level == "INFO":
            return self.chk_log_info.isChecked()
        if level == "DEBUG":
            return self.chk_log_debug.isChecked()
        # Unbekannt – behandel wie INFO
        return self.chk_log_info.isChecked()


    @pyqtSlot(str, str, str)
    def _append_ros_log(self, level: str, node: str, text: str):
        """Append one ROS log entry to the buffer and refresh view."""
        if not hasattr(self, "_ros_log_buffer"):
            self._ros_log_buffer = []

        # Buffer enthält Tuples
        self._ros_log_buffer.append((level, node, text))
        self._ros_log_buffer = self._ros_log_buffer[-400:]

        self._schedule_ros_log_refresh()


    def _schedule_ros_log_refresh(self):
        timer = getattr(self, "_ros_log_update_timer", None)
        if timer is None:
            self._rebuild_ros_log_view()
            return
        timer.start()


    def _flush_ros_log_view(self):
        timer = getattr(self, "_ros_log_update_timer", None)
        if timer is not None and timer.isActive():
            timer.stop()
        self._rebuild_ros_log_view()


    def _rebuild_ros_log_view(self):
        """Rebuild the log text widget from the buffer, applying filters + colors."""
        html_lines = []

        for level, node, text in self._ros_log_buffer:
            if not self._ros_log_level_enabled(level):
                continue

            line = f"[{level}] {node}: {text}"
            escaped = html.escape(line)

            lvl = level.upper()
            if lvl in ("WARN", "WARNING"):
                escaped = escaped.replace(
                    "[WARN]",
                    '<span style="color:#d9a400; font-weight:bold;">[WARN]</span>'
                )
            elif lvl == "ERROR":
                escaped = escaped.replace(
                    "[ERROR]",
                    '<span style="color:#c00000; font-weight:bold;">[ERROR]</span>'
                )

            html_lines.append(escaped)

        hsb = self.ros_log_text.horizontalScrollBar()
        prev_h_value = hsb.value() if hsb is not None else 0
        h_was_at_end = bool(hsb and prev_h_value >= hsb.maximum())

        self.ros_log_text.setHtml("<br>".join(html_lines))

        if hsb is not None:
            if h_was_at_end:
                hsb.setValue(hsb.maximum())
            else:
                hsb.setValue(min(prev_h_value, hsb.maximum()))

        sb = self.ros_log_text.verticalScrollBar()
        if sb is not None:
            sb.setValue(sb.maximum())


    def _clear_ros_log(self):
        """Clear all buffered log messages and the view."""
        self._ros_log_buffer = []
        self.ros_log_text.clear()
        if hasattr(self, "_ros_log_update_timer"):
            self._ros_log_update_timer.stop()
        
    def open_ur_settings(self):
        dlg = URFollowSettingsDialog(self, initial_settings=self.ur_follow_settings)
        if dlg.exec_() == QDialog.Accepted:
            # Retrieve and store new settings
            self.ur_follow_settings = dlg.getValues()
            print("New UR Follow settings:", self.ur_follow_settings)

    def set_idx_metric(self, text):
            self.idx_metric = text

    def get_full_workspace_path(self):
        import os
        current_path = os.path.abspath(__file__)

        while current_path != "/":
            if os.path.basename(current_path) == "src":
                return os.path.dirname(current_path)  # Absoluter Pfad zum Workspace
            current_path = os.path.dirname(current_path)

        return os.path.expanduser("~/catkin_ws")  # Fallback


    def get_relative_workspace_path(self):
        full_path = self.get_full_workspace_path()
        home_path = os.path.expanduser("~")
        if full_path.startswith(home_path):
            return os.path.relpath(full_path, home_path)
        return full_path

    def update_virtual_object_pose(self, pose):
        """Updates the GUI table with the latest virtual object pose."""
        for col in range(6):
            self.table.setItem(8, col, QTableWidgetItem(str(round(pose[col], 4))))
    
    def get_selected_robots(self):
        return [name for name, checkbox in self.robots.items() if checkbox.isChecked()]

    def get_selected_urs(self):
        ur_prefixes = []
        if self.ur10_l.isChecked():
            ur_prefixes.append("UR10_l")
        if self.ur10_r.isChecked():
            ur_prefixes.append("UR10_r")
        return ur_prefixes

    def get_workspace_name(self):
        return self.workspace_input.text().strip()

    def get_override_value(self):
        return self.override_slider.value()

    def _handle_turbo_mode_toggle(self, enabled: bool):
        if not hasattr(self, 'override_slider'):
            return
        max_value = 200 if enabled else 100
        self.override_slider.setMaximum(max_value)
        if self.override_slider.value() > max_value:
            self.override_slider.setValue(max_value)

    def _handle_ludicrous_mode_toggle(self, enabled: bool):
        if not hasattr(self, 'override_slider'):
            return
        max_value = 300 if enabled else (200 if self.is_turbo_mode_enabled() else 100)
        self.override_slider.setMaximum(max_value)
        if self.override_slider.value() > max_value:
            self.override_slider.setValue(max_value)

    def is_debug_enabled(self):
        chk = getattr(self, "chk_log_debug", None)
        return bool(chk and chk.isChecked())



