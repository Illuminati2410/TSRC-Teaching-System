import csv
import json
import re
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation as R

from tracker_core import TrackerReader


SURVIVE_EXE = r"C:\Users\Shaurya\libsurvive\build-win\Release\survive-cli.exe"
TARGET_DEVICE = "WM0"
RENDER_FPS = 60
GRID_SPACING_MM = 100
GRID_EXTENT_MM = 2000
TRACKER_AXIS_LENGTH_MM = 120
GLOBAL_AXIS_LENGTH_MM = 70
MAX_TRAIL_SEGMENTS = 800

# Display tracker axes directly with no remapping:
#   Red   = +X
#   Green = +Y
#   Blue  = +Z
DISPLAY_BASIS = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=float,
)


@dataclass
class CapturePoint:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


class TrackerViewWidget(gl.GLViewWidget):
    mouse_world_changed = QtCore.pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.setMouseTracking(True)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        mouse_pos = event.pos()
        world = self.world_point_on_ground(mouse_pos.x(), mouse_pos.y())
        self.mouse_world_changed.emit(world)

    def world_point_on_ground(self, px, py, plane_z=0.0):
        width = max(1, self.width())
        height = max(1, self.height())
        region = (0, 0, width, height)
        viewport = (0, 0, width, height)

        ndc_x = (2.0 * px / width) - 1.0
        ndc_y = 1.0 - (2.0 * py / height)

        near = QtGui.QVector4D(ndc_x, ndc_y, -1.0, 1.0)
        far = QtGui.QVector4D(ndc_x, ndc_y, 1.0, 1.0)

        matrix = self.projectionMatrix(region, viewport) * self.viewMatrix()
        inverse, invertible = matrix.inverted()
        if not invertible:
            return None

        near_world = inverse * near
        far_world = inverse * far

        if near_world.w() == 0 or far_world.w() == 0:
            return None

        near_world /= near_world.w()
        far_world /= far_world.w()

        origin = np.array([near_world.x(), near_world.y(), near_world.z()], dtype=float)
        target = np.array([far_world.x(), far_world.y(), far_world.z()], dtype=float)
        direction = target - origin

        if abs(direction[2]) < 1e-9:
            return None

        t = (plane_z - origin[2]) / direction[2]
        if t < 0:
            return None

        return origin + direction * t


class LiveTrackerPoseViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live Vive Tracker Pose Viewer")
        self.resize(1400, 900)

        # Create tracker reader
        self.reader = TrackerReader(SURVIVE_EXE, TARGET_DEVICE)
        self.reader.start()

        self.view = TrackerViewWidget()
        self.view.setBackgroundColor((18, 20, 24))
        self.view.opts["distance"] = 1.4
        self.view.opts["elevation"] = 22
        self.view.opts["azimuth"] = -55
        #self.view.mouse_world_changed.connect(self._update_mouse_overlay)
        self.setCentralWidget(self.view)

        self.latest_pose = None
        self.last_rendered_seq = -1
        self.last_button_pressed = False
        self.manual_button_pressed = False
        self.captured_points = []
        self.trail_segments = deque()
        self._last_trail_position = None

        self._build_scene()
        self._build_overlays()

        self.render_timer = QtCore.QTimer(self)
        self.render_timer.timeout.connect(self._render_latest_pose)
        self.render_timer.start(int(1000 / RENDER_FPS))

    def closeEvent(self, event):
        self.reader.stop()
        super().closeEvent(event)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_C:
            self.capture_current_pose()
            event.accept()
            return
        if event.key() == QtCore.Qt.Key_B:
            self._manual_button_toggle()
            event.accept()
            return
        super().keyPressEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        margin = 12
        self.info_panel.move(margin, margin)
        self.controls_panel.move(self.width() - self.controls_panel.width() - margin, margin)
        self.mouse_panel.move(margin, self.height() - self.mouse_panel.height() - margin)

    def _build_scene(self):
        grid = gl.GLGridItem()
        spacing_m = GRID_SPACING_MM / 1000.0
        extent_m = GRID_EXTENT_MM / 1000.0
        grid.setSize(x=extent_m, y=extent_m)
        grid.setSpacing(x=spacing_m, y=spacing_m)
        self.view.addItem(grid)

        self.global_axes = self._create_axes(GLOBAL_AXIS_LENGTH_MM / 1000.0, width=4)
        self.tracker_axes = self._create_axes(TRACKER_AXIS_LENGTH_MM / 1000.0, width=7)

        for item in self.global_axes + self.tracker_axes:
            self.view.addItem(item)

        corner_offset = (extent_m * 0.5) - (GLOBAL_AXIS_LENGTH_MM / 1000.0) - (spacing_m * 0.5)
        global_origin = np.array(
            [-corner_offset, -corner_offset, GLOBAL_AXIS_LENGTH_MM / 1000.0 * 0.35],
            dtype=float,
        )
        self._set_axes_pose(self.global_axes, global_origin, np.eye(3))

        self.capture_scatter = gl.GLScatterPlotItem(
            pos=np.empty((0, 3), dtype=float),
            size=12,
            color=(1.0, 0.85, 0.2, 1.0),
            pxMode=False,
        )
        self.view.addItem(self.capture_scatter)

    def _build_overlays(self):
        panel_style = (
            "background-color: rgba(8, 10, 14, 185);"
            "color: white;"
            "border: 1px solid rgba(255,255,255,40);"
            "border-radius: 6px;"
        )

        self.info_panel = QtWidgets.QFrame(self.view)
        self.info_panel.setStyleSheet(panel_style)
        info_layout = QtWidgets.QVBoxLayout(self.info_panel)
        info_layout.setContentsMargins(10, 8, 10, 8)
        info_layout.setSpacing(4)

        self.pose_label = QtWidgets.QLabel("Waiting for tracker pose...")
        self.pose_label.setStyleSheet("font-size: 14px;")
        self.pose_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.pose_label.setTextFormat(QtCore.Qt.RichText)
        info_layout.addWidget(self.pose_label)

        self.status_label = QtWidgets.QLabel("Starting tracker...")
        self.status_label.setStyleSheet("font-size: 12px; color: #9cc7ff;")
        info_layout.addWidget(self.status_label)
        self.info_panel.adjustSize()

        self.controls_panel = QtWidgets.QFrame(self.view)
        self.controls_panel.setStyleSheet(panel_style)
        controls_layout = QtWidgets.QVBoxLayout(self.controls_panel)
        controls_layout.setContentsMargins(10, 8, 10, 8)
        controls_layout.setSpacing(6)

        self.capture_button = QtWidgets.QPushButton("Capture Point (C)")
        self.capture_button.clicked.connect(self.capture_current_pose)
        controls_layout.addWidget(self.capture_button)

        self.export_csv_button = QtWidgets.QPushButton("Export CSV")
        self.export_csv_button.clicked.connect(self.export_csv)
        controls_layout.addWidget(self.export_csv_button)

        self.export_json_button = QtWidgets.QPushButton("Export JSON")
        self.export_json_button.clicked.connect(self.export_json)
        controls_layout.addWidget(self.export_json_button)
        self.controls_panel.adjustSize()

        self.mouse_panel = QtWidgets.QFrame(self.view)
        self.mouse_panel.setStyleSheet(panel_style)
        mouse_layout = QtWidgets.QVBoxLayout(self.mouse_panel)
        mouse_layout.setContentsMargins(10, 8, 10, 8)
        self.mouse_label = QtWidgets.QLabel("Mouse:\nX = -- mm\nY = -- mm\nZ = -- mm")
        self.mouse_label.setStyleSheet("font-size: 13px;")
        mouse_layout.addWidget(self.mouse_label)
        self.mouse_panel.adjustSize()

        self.resizeEvent(QtGui.QResizeEvent(self.size(), self.size()))

    def _create_axes(self, length_m, width):
        x_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [length_m, 0, 0]], dtype=float),
            color=(1.0, 0.2, 0.2, 1.0),
            width=width,
            antialias=True,
        )
        y_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, length_m, 0]], dtype=float),
            color=(0.2, 1.0, 0.2, 1.0),
            width=width,
            antialias=True,
        )
        z_axis = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 0, length_m]], dtype=float),
            color=(0.2, 0.5, 1.0, 1.0),
            width=width,
            antialias=True,
        )
        x_axis.line_length_m = length_m
        y_axis.line_length_m = length_m
        z_axis.line_length_m = length_m
        return (x_axis, y_axis, z_axis)

    def _set_axes_pose(self, axes, origin, rotation_matrix):
        axis_length = getattr(axes[0], "line_length_m", TRACKER_AXIS_LENGTH_MM / 1000.0)

        x_end = origin + rotation_matrix[:, 0] * axis_length
        y_end = origin + rotation_matrix[:, 1] * axis_length
        z_end = origin - rotation_matrix[:, 2] * axis_length

        axes[0].setData(pos=np.array([origin, x_end], dtype=float))
        axes[1].setData(pos=np.array([origin, y_end], dtype=float))
        axes[2].setData(pos=np.array([origin, z_end], dtype=float))

    def _transform_position_to_view(self, position):
        return DISPLAY_BASIS @ position

    def _transform_rotation_to_view(self, rotation_matrix):
        return DISPLAY_BASIS @ rotation_matrix @ DISPLAY_BASIS.T

    def _render_latest_pose(self):
        sample = self.reader.get_latest_pose()
        if sample is None or sample.seq == self.last_rendered_seq:
            return

        self.latest_pose = sample
        self.last_rendered_seq = sample.seq

        tracker_position = np.array([sample.x, sample.y, sample.z], dtype=float)
        tracker_rotation = R.from_quat(
            [sample.qx, sample.qy, sample.qz, sample.qw]
        ).as_matrix()

        position = self._transform_position_to_view(tracker_position)
        rotation = self._transform_rotation_to_view(tracker_rotation)

        # DEBUG TRANSLATION SMOOTHNESS
        if hasattr(self, "_prev_pos"):
            step = np.linalg.norm(position - self._prev_pos)
            print(f"step = {step:.6f}")

        self._prev_pos = position.copy()

        self._set_axes_pose(self.tracker_axes, position, rotation)

        effective_button = sample.button_pressed or self.manual_button_pressed
        self._append_trail_segment(position, effective_button)
        self._update_pose_overlay(sample, effective_button)
        self.mouse_label.setText(
            "\n".join(
                [
                    "Tracker Position",
                    f"X = {sample.x * 1000:.1f} mm",
                    f"Y = {sample.y * 1000:.1f} mm",
                    f"Z = {sample.z * 1000:.1f} mm",
                ]
            )
        )

        if effective_button and not self.last_button_pressed:
            self.capture_current_pose()

        self.last_button_pressed = effective_button

    def capture_current_pose(self):
        if self.latest_pose is None:
            self._set_status("No pose available to capture")
            return

        point = CapturePoint(
            x=self.latest_pose.x,
            y=self.latest_pose.y,
            z=self.latest_pose.z,
            qx=self.latest_pose.qx,
            qy=self.latest_pose.qy,
            qz=self.latest_pose.qz,
            qw=self.latest_pose.qw,
        )
        self.captured_points.append(point)
        self._update_capture_markers()
        self._set_status(f"Captured point {len(self.captured_points)}")

    def export_csv(self):
        if not self.captured_points:
            self._set_status("No captured points to export")
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export Teach Points as CSV",
            "teach_points.csv",
            "CSV Files (*.csv)",
        )
        if not path:
            return

        with open(path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(asdict(self.captured_points[0]).keys()))
            writer.writeheader()
            for point in self.captured_points:
                writer.writerow(asdict(point))

        self._set_status(f"Exported CSV: {path}")

    def export_json(self):
        if not self.captured_points:
            self._set_status("No captured points to export")
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export Teach Points as JSON",
            "teach_points.json",
            "JSON Files (*.json)",
        )
        if not path:
            return

        with open(path, "w") as handle:
            json.dump([asdict(point) for point in self.captured_points], handle, indent=2)

        self._set_status(f"Exported JSON: {path}")

    def _append_trail_segment(self, position, button_pressed):
        if self._last_trail_position is None:
            self._last_trail_position = position
            return

        color = (0.2, 1.0, 0.35, 1.0) if button_pressed else (1.0, 1.0, 1.0, 0.85)
        segment = gl.GLLinePlotItem(
            pos=np.array([self._last_trail_position, position], dtype=float),
            color=color,
            width=2,
            antialias=True,
        )
        self.view.addItem(segment)
        self.trail_segments.append(segment)
        self._last_trail_position = position

        while len(self.trail_segments) > MAX_TRAIL_SEGMENTS:
            old_segment = self.trail_segments.popleft()
            self.view.removeItem(old_segment)

    def _update_capture_markers(self):
        if not self.captured_points:
            self.capture_scatter.setData(pos=np.empty((0, 3), dtype=float))
            return

        positions = np.array([[p.x, p.y, p.z] for p in self.captured_points], dtype=float)
        self.capture_scatter.setData(
            pos=positions,
            size=np.full(len(positions), 0.02, dtype=float),
            color=np.tile(np.array([[1.0, 0.82, 0.15, 1.0]]), (len(positions), 1)),
            pxMode=False,
        )
        effective_button = self.latest_pose.button_pressed if self.latest_pose is not None else False
        effective_button = effective_button or self.manual_button_pressed
        self._update_pose_overlay(self.latest_pose, effective_button)

    def _update_pose_overlay(self, sample, effective_button=None):
        if sample is None:
            return

        if effective_button is None:
            effective_button = sample.button_pressed or self.manual_button_pressed

        button_text = "Pressed" if effective_button else "Released"
        button_color = "#73ff8f" if effective_button else "#ffffff"
        if self.reader.is_button_available():
            button_source = "tracker"
        elif self.manual_button_pressed:
            button_source = "manual"
        else:
            button_source = "manual/unknown"

        self.pose_label.setText(
            "\n".join(
                [
                    f"Grid spacing = {GRID_SPACING_MM} mm",
                    f"X = {sample.x * 1000:.1f} mm",
                    f"Y = {sample.y * 1000:.1f} mm",
                    f"Z = {sample.z * 1000:.1f} mm",
                    f"qx = {sample.qx:.4f}",
                    f"qy = {sample.qy:.4f}",
                    f"qz = {sample.qz:.4f}",
                    f"qw = {sample.qw:.4f}",
                    f"Button = <span style='color:{button_color}'>{button_text}</span> ({button_source})",
                    f"Captured points = {len(self.captured_points)}",
                    "Keyboard: C capture, B toggle button",
                ]
            )
        )

    def _update_mouse_overlay(self, world_point):

            if self.latest_pose is None:
                self.mouse_label.setText(
                    "Tracker Position\nX = -- mm\nY = -- mm\nZ = -- mm"
                )
                return

            self.mouse_label.setText(
                "\n".join(
                    [
                        "Tracker Position",
                        f"X = {self.latest_pose.x * 1000:.1f} mm",
                        f"Y = {self.latest_pose.y * 1000:.1f} mm",
                        f"Z = {self.latest_pose.z * 1000:.1f} mm",
                    ]
                )
            )

    def _set_status(self, text):
        self.status_label.setText(text)

    def _manual_button_toggle(self):
        self.manual_button_pressed = not self.manual_button_pressed
        if self.latest_pose is not None:
            effective_button = self.latest_pose.button_pressed or self.manual_button_pressed
            self._update_pose_overlay(self.latest_pose, effective_button)
        self._set_status(f"Manual button {'pressed' if self.manual_button_pressed else 'released'}")


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = LiveTrackerPoseViewer()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
