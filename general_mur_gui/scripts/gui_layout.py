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

    def is_turbo_mode_enabled(self):
        chk = getattr(self, "turbo_mode_checkbox", None)
        return bool(chk and chk.isChecked())

    def is_ludicrous_mode_enabled(self):
        chk = getattr(self, "ludicrous_mode_checkbox", None)
        return bool(chk and chk.isChecked())

    def get_tcp_offset_xyz(self):
        if not hasattr(self, 'tcp_offset_spins'):
            return [0.0, 0.0, 0.0]
        return [spin.value() for spin in self.tcp_offset_spins]

    def get_tcp_offset_sixd(self):
        xyz = self.get_tcp_offset_xyz()
        phi = self.get_tcp_phi_radians()
        return xyz + [0.0, 0.0, phi]

    def get_tcp_phi_radians(self):
        if not hasattr(self, 'tcp_phi_spin'):
            return 0.0
        return math.radians(self.tcp_phi_spin.value())

    def get_spray_distance(self):
        return self.spray_distance_spin.value()

    def open_rosbag_settings(self):
        dlg = RosbagSettingsDialog(self, self.topic_settings)
        if dlg.exec_() == QDialog.Accepted:
            self.topic_settings = dlg.get_settings()

    def _handle_orth_pid_toggle(self, enabled: bool):
        self.orth_pid_state_label.setText("On" if enabled else "Off")
        if enabled:
            self.ros_interface.start_orthogonal_pid_controller()
        else:
            self.ros_interface.stop_orthogonal_pid_controller()

    def _open_orth_pid_settings(self):
        dlg = OrthogonalPIDDialog(self, self.ros_interface)
        dlg.exec_()


class URFollowSettingsDialog(QDialog):
    def __init__(self, parent=None, initial_settings=None):
        super().__init__(parent)
        self.setWindowTitle("UR Follow Trajectory Settings")

        # Default settings
        init = initial_settings or {}
        idx_metrics = ["virtual line", "radius", "collinear"]
        idx_metric = init.get('idx_metric', idx_metrics[0])
        threshold = init.get('threshold', 0.010)

        form = QFormLayout()        
        self.dropdown_idx_metric = QComboBox()
        self.dropdown_idx_metric.addItems(idx_metrics)
        self.dropdown_idx_metric.setCurrentText(idx_metric)  # Set default text
        self.dropdown_idx_metric.setStyleSheet("background-color: lightgray;")
        # self.dropdown_idx_metric.currentTextChanged.connect(lambda text: self.set_idx_metric(text))
        form.addRow("Index Metric:", self.dropdown_idx_metric)

        self.spin_threshold = QDoubleSpinBox()
        self.spin_threshold.setRange(0.0, 0.2)  # Set range for the spin box
        self.spin_threshold.setSingleStep(0.002)
        self.spin_threshold.setDecimals(3)  # Set number of decimal places
        self.spin_threshold.setValue(threshold)
        self.spin_threshold.setSuffix(" m")
        self.spin_threshold.setStyleSheet("background-color: lightgray;")
        form.addRow("Threshold:", self.spin_threshold)

        # OK / Cancel
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        # Layout
        layout = QVBoxLayout()
        layout.addLayout(form)
        layout.addWidget(buttons)
        self.setLayout(layout)

    def getValues(self):
        return {
            'idx_metric': self.dropdown_idx_metric.currentText(),
            'threshold': self.spin_threshold.value(),
        }


class ServoCalibrationDialog(QDialog):
    def __init__(self, parent, calib, state_provider=None, save_callback=None):
        super().__init__(parent)
        self.setWindowTitle("Servo Calibration")
        self._orig = calib
        self._data = {
            'left': calib['left'].copy(),
            'right': calib['right'].copy()
        }
        self._state_provider = state_provider
        self._save_callback = save_callback
        form = QFormLayout()
        # Left
        self.left_min = QSpinBox(); self.left_min.setRange(0,4095); self.left_min.setValue(self._data['left']['min'])
        self.left_zero = QSpinBox(); self.left_zero.setRange(0,4095); self.left_zero.setValue(self._data['left']['zero'])
        self.left_max = QSpinBox(); self.left_max.setRange(0,4095); self.left_max.setValue(self._data['left']['max'])
        form.addRow(QLabel("Left Min"), self._wrap_spin_with_buttons(
            self.left_min,
            [
                ("Use Live", lambda _, spin=self.left_min: self._apply_live_position('left', spin)),
            ],
        ))
        form.addRow(QLabel("Left Zero"), self._wrap_spin_with_buttons(
            self.left_zero,
            [
                ("Use Live", lambda _, spin=self.left_zero: self._apply_live_position('left', spin)),
                ("Auto Range", lambda _, side='left': self._auto_calc_range(side)),
            ],
        ))
        form.addRow(QLabel("Left Max"), self._wrap_spin_with_buttons(
            self.left_max,
            [
                ("Use Live", lambda _, spin=self.left_max: self._apply_live_position('left', spin)),
            ],
        ))
        # Right
        self.right_min = QSpinBox(); self.right_min.setRange(0,4095); self.right_min.setValue(self._data['right']['min'])
        self.right_zero = QSpinBox(); self.right_zero.setRange(0,4095); self.right_zero.setValue(self._data['right']['zero'])
        self.right_max = QSpinBox(); self.right_max.setRange(0,4095); self.right_max.setValue(self._data['right']['max'])
        form.addRow(QLabel("Right Min"), self._wrap_spin_with_buttons(
            self.right_min,
            [
                ("Use Live", lambda _, spin=self.right_min: self._apply_live_position('right', spin)),
            ],
        ))
        form.addRow(QLabel("Right Zero"), self._wrap_spin_with_buttons(
            self.right_zero,
            [
                ("Use Live", lambda _, spin=self.right_zero: self._apply_live_position('right', spin)),
                ("Auto Range", lambda _, side='right': self._auto_calc_range(side)),
            ],
        ))
        form.addRow(QLabel("Right Max"), self._wrap_spin_with_buttons(
            self.right_max,
            [
                ("Use Live", lambda _, spin=self.right_max: self._apply_live_position('right', spin)),
            ],
        ))
        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        save_btn = buttons.addButton("Save Defaults", QDialogButtonBox.ActionRole)
        buttons.accepted.connect(self._on_accept)
        buttons.rejected.connect(self.reject)
        if save_btn is not None:
            save_btn.clicked.connect(self._handle_save_defaults)
        layout = QVBoxLayout()
        layout.addLayout(form)
        layout.addWidget(buttons)
        self.setLayout(layout)

    def _wrap_spin_with_buttons(self, spinbox, button_defs):
        container = QWidget(); row = QHBoxLayout(container); row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(spinbox)
        for text, handler in button_defs:
            btn = QPushButton(text); btn.setAutoDefault(False); btn.setDefault(False); btn.setMaximumWidth(120)
            btn.clicked.connect(handler)
            row.addWidget(btn)
        row.addStretch(1)
        return container

    def _apply_live_position(self, side: str, spinbox: QSpinBox):
        value = self._fetch_live_position(side)
        if value is None:
            self._show_missing_live_data_warning(side)
            return
        spinbox.setValue(self._wrap_raw(value))

    def _fetch_live_position(self, side: str):
        if self._state_provider is None:
            return None
        getter = getattr(self._state_provider, 'get_latest_servo_position', None)
        if not callable(getter):
            return None
        try:
            value = getter(side)
        except Exception as exc:
            print(f"Failed to fetch servo state for {side}: {exc}")
            return None
        if value is None:
            return None
        if isinstance(value, (int, float)):
            return int(round(float(value)))
        return None

    def _show_missing_live_data_warning(self, side: str):
        QMessageBox.warning(self, "Servo State Unavailable", f"No live position available for the {side} servo.")

    def _auto_calc_range(self, side: str):
        if side == 'left':
            zero_spin, min_spin, max_spin = self.left_zero, self.left_min, self.left_max
        else:
            zero_spin, min_spin, max_spin = self.right_zero, self.right_min, self.right_max
        zero_spin.interpretText()
        zero_value = zero_spin.value()
        if side == 'left':
            min_offset = 1643
            max_delta = 557
        else:
            min_offset = -1643
            max_delta = -557
        min_value = self._wrap_raw(zero_value + min_offset)
        max_value = self._wrap_raw(min_value + max_delta)
        min_spin.setValue(min_value)
        max_spin.setValue(max_value)

    @staticmethod
    def _wrap_raw(value: int) -> int:
        return int(value) % 4096

    def _handle_save_defaults(self):
        if self._save_callback is None:
            QMessageBox.warning(self, "Save Unsupported", "Saving defaults is unavailable in this context.")
            return
        payload = self.get_values()
        try:
            self._save_callback(payload)
        except Exception as exc:
            QMessageBox.critical(self, "Save Failed", f"Could not save calibration defaults:\n{exc}")
            return
        QMessageBox.information(self, "Defaults Saved", "Servo calibration defaults have been updated.")

    def _on_accept(self):
        # Basic validation: min <= zero <= max
        if not (self.left_min.value() <= self.left_zero.value() <= self.left_max.value()):
            # silently clamp
            z = min(max(self.left_zero.value(), self.left_min.value()), self.left_max.value())
            self.left_zero.setValue(z)
        if not (self.right_min.value() <= self.right_zero.value() <= self.right_max.value()):
            z = min(max(self.right_zero.value(), self.right_min.value()), self.right_max.value())
            self.right_zero.setValue(z)
        self.accept()

    def get_values(self):
        return {
            'left': {
                'min': self.left_min.value(),
                'zero': self.left_zero.value(),
                'max': self.left_max.value(),
            },
            'right': {
                'min': self.right_min.value(),
                'zero': self.right_zero.value(),
                'max': self.right_max.value(),
            }
        }

class RosbagSettingsDialog(QDialog):
    def __init__(self, parent, topic_settings):
        super().__init__(parent)
        self.setWindowTitle("Rosbag Settings")
        self.topic_settings = topic_settings  # {"topic": {"local": True, "remote": True}}

        layout = QVBoxLayout()
        grid = QGridLayout()
        grid.addWidget(QLabel("Topic"), 0, 0)
        grid.addWidget(QLabel("GUI-PC"), 0, 1)
        grid.addWidget(QLabel("MuR"), 0, 2)

        self.box_local = {}
        self.box_remote = {}

        row = 1
        for topic, opts in topic_settings.items():
            t_label = QLabel(topic)
            cb_local = QCheckBox();  cb_local.setChecked(opts["local"])
            cb_remote = QCheckBox(); cb_remote.setChecked(opts["remote"])
            grid.addWidget(t_label, row, 0)
            grid.addWidget(cb_local, row, 1)
            grid.addWidget(cb_remote, row, 2)
            self.box_local[topic] = cb_local
            self.box_remote[topic] = cb_remote
            row += 1

        layout.addLayout(grid)
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)
        self.setLayout(layout)

    def get_settings(self):
        out = {}
        for topic in self.topic_settings:
            out[topic] = {
                "local": self.box_local[topic].isChecked(),
                "remote": self.box_remote[topic].isChecked(),
            }
        return out


class ComponentTransformDialog(QDialog):
    TRANSLATION_KEYS = ("tx", "ty", "tz")
    ROTATION_KEYS = ("rx", "ry", "rz")

    def __init__(self, parent, component_names, selected_component, transform_loader=None):
        super().__init__(parent)
        self.setWindowTitle("Component Selection & Transform")
        self._transform_loader = transform_loader
        self._transform_cache = {}
        self._blocking = False

        layout = QVBoxLayout()
        form = QFormLayout()

        self.component_combo = QComboBox()
        self.component_combo.addItems(component_names)
        if selected_component and selected_component in component_names:
            idx = component_names.index(selected_component)
            self.component_combo.setCurrentIndex(idx)
        elif component_names:
            self.component_combo.setCurrentIndex(0)
        form.addRow("Component", self.component_combo)

        self.translation_fields = {}
        for key, label in zip(self.TRANSLATION_KEYS, ("X Offset (m)", "Y Offset (m)", "Z Offset (m)")):
            spin = QDoubleSpinBox()
            spin.setRange(-10.0, 10.0)
            spin.setDecimals(4)
            spin.setSingleStep(0.001)
            spin.setSuffix(" m")
            self.translation_fields[key] = spin
            form.addRow(label, spin)

        self.rotation_fields = {}
        for key, label in zip(self.ROTATION_KEYS, ("Roll (deg)", "Pitch (deg)", "Yaw (deg)")):
            spin = QDoubleSpinBox()
            spin.setRange(-360.0, 360.0)
            spin.setDecimals(3)
            spin.setSingleStep(1.0)
            spin.setSuffix(" °")
            self.rotation_fields[key] = spin
            form.addRow(label, spin)

        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self.setLayout(layout)

        self._current_component = self.component_combo.currentText()
        self.component_combo.currentTextChanged.connect(self._handle_component_changed)
        self._load_component_values(self._current_component)

    def _handle_component_changed(self, new_component):
        self._store_current_values()
        self._current_component = new_component
        self._load_component_values(new_component)

    def _normalize_transform(self, data):
        normalized = {key: 0.0 for key in self.TRANSLATION_KEYS + self.ROTATION_KEYS}
        if isinstance(data, dict):
            for key in normalized.keys():
                value = data.get(key)
                try:
                    normalized[key] = float(value)
                except (TypeError, ValueError):
                    normalized[key] = 0.0
        return normalized

    def _ensure_transform_cached(self, component_name):
        name = component_name or ""
        cached = self._transform_cache.get(name)
        if cached is not None:
            return cached

        payload = {}
        if callable(self._transform_loader):
            try:
                payload = self._transform_loader(name) or {}
            except Exception as exc:
                print(f"Failed to load transform for {name}: {exc}")
                payload = {}

        cached = self._normalize_transform(payload)
        self._transform_cache[name] = cached
        return cached

    def _load_component_values(self, component_name):
        if self._blocking:
            return
        transform = self._ensure_transform_cached(component_name)
        self._blocking = True
        for key, spin in self.translation_fields.items():
            spin.setValue(transform.get(key, 0.0))
        for key, spin in self.rotation_fields.items():
            spin.setValue(math.degrees(transform.get(key, 0.0)))
        self._blocking = False

    def _collect_field_values(self):
        values = {}
        for key, spin in self.translation_fields.items():
            values[key] = float(spin.value())
        for key, spin in self.rotation_fields.items():
            values[key] = math.radians(float(spin.value()))
        return values


class OrthogonalPIDDialog(QDialog):
    FIELD_SPECS = [
        ("stamped", "Stamped Twist", "bool"),
        ("Kp_linear_x", "Kp linear x", "float"),
        ("Ki_linear_x", "Ki linear x", "float"),
        ("Kd_linear_x", "Kd linear x", "float"),
        ("Kp_linear_y", "Kp linear y", "float"),
        ("Ki_linear_y", "Ki linear y", "float"),
        ("Kd_linear_y", "Kd linear y", "float"),
        ("Kp_linear_z", "Kp linear z", "float"),
        ("Ki_linear_z", "Ki linear z", "float"),
        ("Kd_linear_z", "Kd linear z", "float"),
        ("Kp_angular_x", "Kp angular x", "float"),
        ("Ki_angular_x", "Ki angular x", "float"),
        ("Kd_angular_x", "Kd angular x", "float"),
        ("Kp_angular_y", "Kp angular y", "float"),
        ("Ki_angular_y", "Ki angular y", "float"),
        ("Kd_angular_y", "Kd angular y", "float"),
        ("Kp_angular_z", "Kp angular z", "float"),
        ("Ki_angular_z", "Ki angular z", "float"),
        ("Kd_angular_z", "Kd angular z", "float"),
    ]

    def __init__(self, parent, ros_interface):
        super().__init__(parent)
        self.setWindowTitle("Orthogonal PID Settings")
        self.ros_interface = ros_interface
        self._raw_config = {}
        self._controls = {}

        layout = QVBoxLayout()
        form = QFormLayout()

        for key, label, kind in self.FIELD_SPECS:
            if kind == "bool":
                widget = QCheckBox()
            else:
                spin = QDoubleSpinBox()
                spin.setRange(-50.0, 50.0)
                spin.setDecimals(4)
                spin.setSingleStep(0.01)
                widget = spin
            self._controls[key] = widget
            form.addRow(label, widget)

        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        self.btn_save = buttons.addButton("Save", QDialogButtonBox.ActionRole)
        self.btn_reload = buttons.addButton("Reload", QDialogButtonBox.ActionRole)
        self.btn_save.clicked.connect(self._handle_save)
        self.btn_reload.clicked.connect(self._load_values)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.setLayout(layout)
        self._load_values()

    def _load_values(self):
        try:
            cfg = self.ros_interface.load_orthogonal_pid_config()
        except Exception as exc:
            QMessageBox.critical(self, "Load Failed", f"Could not read PID config:\n{exc}")
            cfg = {}

        self._raw_config = cfg if isinstance(cfg, dict) else {}
        for key, widget in self._controls.items():
            value = self._raw_config.get(key)
            if isinstance(widget, QCheckBox):
                widget.setChecked(bool(value))
            elif isinstance(widget, QDoubleSpinBox):
                try:
                    widget.setValue(float(value))
                except (TypeError, ValueError):
                    widget.setValue(0.0)

    def _collect_values(self):
        values = {}
        for key, widget in self._controls.items():
            if isinstance(widget, QCheckBox):
                values[key] = bool(widget.isChecked())
            elif isinstance(widget, QDoubleSpinBox):
                values[key] = float(widget.value())
        return values

    def _handle_save(self):
        updated = dict(self._raw_config)
        updated.update(self._collect_values())
        try:
            self.ros_interface.save_orthogonal_pid_config(updated)
        except Exception as exc:
            QMessageBox.critical(self, "Save Failed", f"Could not write PID config:\n{exc}")
            return
        QMessageBox.information(self, "Saved", "Orthogonal PID parameters have been saved.")

    def _store_current_values(self):
        name = self._current_component or ""
        if not name:
            return
        self._transform_cache[name] = self._collect_field_values()

    def accept(self):
        self._store_current_values()
        super().accept()

    def get_selection(self):
        name = (self._current_component or self.component_combo.currentText() or "").strip()
        if not name and self.component_combo.count() > 0:
            name = self.component_combo.itemText(0).strip()
        transform = self._transform_cache.get(name)
        if transform is None:
            transform = self._collect_field_values()
            self._transform_cache[name] = transform
        return name, dict(transform)
