"""Lightweight map-frame top view for cooperative handling."""

from dataclasses import dataclass
import math
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


WORLD_FRAME = "map"
OBJECT_FRAME = "virtual_object/base_link"
ARM_PREFIX = {"l": "UR10_l", "r": "UR10_r"}
ARM_LINKS = (
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "tool0",
)
ROBOT_COLORS = {
    "mur620a": "#D55E00",
    "mur620b": "#0072B2",
    "mur620c": "#009E73",
    "mur620d": "#A64D79",
}
PIXELS_PER_METER = 100.0
STALE_SECONDS = 2.0


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ViewSnapshot:
    robots: tuple
    sides: tuple
    poses: dict
    problems: dict


def frame_names(robots, sides):
    """Frames needed for the selected MuRs and arms, in diagnostic priority order."""
    if not robots:
        return ()
    names = []
    for robot in robots:
        names.append(f"{robot}/base_link")
        for side in sides:
            prefix = ARM_PREFIX[side]
            names.append(f"{robot}/{prefix}/virtual_object_target_tcp")
            names.extend(f"{robot}/{prefix}/{link}" for link in ARM_LINKS)
    names.append(OBJECT_FRAME)
    return tuple(names)


def pose_from_transform(stamped, now_ns):
    """Return a planar pose or a reason why a transform cannot be drawn."""
    stamp = stamped.header.stamp
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    if stamp_ns and now_ns - stamp_ns > STALE_SECONDS * 1_000_000_000:
        return None, f"stale TF ({(now_ns - stamp_ns) / 1e9:.1f} s)"
    translation = stamped.transform.translation
    rotation = stamped.transform.rotation
    values = (translation.x, translation.y, rotation.x, rotation.y, rotation.z, rotation.w)
    if not all(math.isfinite(value) for value in values):
        return None, "invalid TF"
    norm = math.sqrt(sum(value * value for value in
                         (rotation.x, rotation.y, rotation.z, rotation.w)))
    if norm < 1e-9:
        return None, "invalid rotation"
    x, y, z, w = (rotation.x / norm, rotation.y / norm,
                  rotation.z / norm, rotation.w / norm)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return Pose2D(translation.x, translation.y, yaw), None


def collect_snapshot(buffer, now_ns, robots, sides):
    """Collect one nonblocking snapshot from the latest available TF values."""
    robots = tuple(robots)
    sides = tuple(sides)
    poses = {}
    problems = {}
    for frame in frame_names(robots, sides):
        try:
            stamped = buffer.lookup_transform(WORLD_FRAME, frame, Time())
        except TransformException:
            problems[frame] = "missing TF"
            continue
        pose, problem = pose_from_transform(stamped, now_ns)
        if problem:
            problems[frame] = problem
        else:
            poses[frame] = pose
    return ViewSnapshot(robots, sides, poses, problems)


class TopViewRosWorker(QtCore.QThread):
    """Keep all TF subscriptions and lookups outside the Qt GUI thread."""

    snapshot = QtCore.pyqtSignal(object)
    log = QtCore.pyqtSignal(str)

    def __init__(self, robots=(), sides=()):
        super().__init__()
        self._selection = (tuple(robots), tuple(sides))
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reset_tf = threading.Event()

    def set_selection(self, robots, sides):
        with self._lock:
            self._selection = (tuple(robots), tuple(sides))

    def shutdown(self):
        self._stop.set()

    def reset_tf_cache(self):
        """Discard transforms from stopped publishers and resubscribe to live static TF."""
        self._reset_tf.set()

    def run(self):
        deadline = time.monotonic() + 5.0
        while not rclpy.ok() and not self._stop.is_set() and time.monotonic() < deadline:
            self.msleep(20)
        if not rclpy.ok() or self._stop.is_set():
            self.log.emit("[view] ROS context unavailable; map view has no TF data")
            return

        node = None
        listener = None
        executor = None
        try:
            node = rclpy.create_node("cooperative_top_view")
            buffer = Buffer(cache_time=Duration(seconds=10), node=node)
            listener = TransformListener(buffer, node, spin_thread=False, qos=10)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            next_snapshot = 0.0
            while rclpy.ok() and not self._stop.is_set():
                if self._reset_tf.is_set():
                    listener.unregister()
                    node.destroy_service(buffer.srv)
                    buffer = Buffer(cache_time=Duration(seconds=10), node=node)
                    listener = TransformListener(buffer, node, spin_thread=False, qos=10)
                    self._reset_tf.clear()
                executor.spin_once(timeout_sec=0.002)
                now = time.monotonic()
                if now < next_snapshot:
                    continue
                with self._lock:
                    robots, sides = self._selection
                self.snapshot.emit(
                    collect_snapshot(buffer, node.get_clock().now().nanoseconds, robots, sides)
                )
                next_snapshot = now + 0.2
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception as exc:
            self.log.emit(f"[view] TF worker stopped: {exc}")
        finally:
            if executor is not None:
                if node is not None:
                    executor.remove_node(node)
                executor.shutdown()
            if listener is not None:
                listener.unregister()
            if node is not None:
                node.destroy_node()


def scene_point(pose):
    """Project ROS map XY into Qt scene coordinates."""
    return QtCore.QPointF(pose.x * PIXELS_PER_METER, -pose.y * PIXELS_PER_METER)


class RobotGlyph(QtWidgets.QGraphicsItem):
    """Planar silhouette based on the MiR 600 top-module footprint."""

    def __init__(self, color):
        super().__init__()
        self.color = QtGui.QColor(color)
        self.setZValue(1)

    def boundingRect(self):
        return QtCore.QRectF(-78, -56, 156, 112)

    def paint(self, painter, _option, _widget=None):
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setPen(QtGui.QPen(QtGui.QColor("#344054"), 1.5))
        painter.setBrush(QtGui.QColor("#4B5563"))
        painter.drawRoundedRect(QtCore.QRectF(-23, -53, 46, 12), 4, 4)
        painter.drawRoundedRect(QtCore.QRectF(-23, 41, 46, 12), 4, 4)
        painter.setPen(QtGui.QPen(self.color, 2.5))
        painter.setBrush(QtGui.QColor("#EEF2F6"))
        painter.drawRoundedRect(QtCore.QRectF(-67.5, -45.5, 135, 91), 16, 16)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(self.color)
        painter.drawRoundedRect(QtCore.QRectF(58, -28, 6, 56), 3, 3)


class RobotBadge(QtWidgets.QGraphicsItem):
    """Keep the colored MuR letter readable at overview zoom levels."""

    def __init__(self, letter, color):
        super().__init__()
        self.letter = letter
        self.color = QtGui.QColor(color)
        self.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations)
        self.setZValue(7)

    def boundingRect(self):
        return QtCore.QRectF(-17, -17, 34, 34)

    def paint(self, painter, _option, _widget=None):
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setPen(QtGui.QPen(self.color, 2))
        painter.setBrush(QtGui.QColor("#FFFFFF"))
        painter.drawEllipse(self.boundingRect())
        painter.setPen(self.color)
        painter.setFont(QtGui.QFont("Sans Serif", 16, QtGui.QFont.Bold))
        painter.drawText(self.boundingRect(), QtCore.Qt.AlignCenter, self.letter)


class FrameAxes(QtWidgets.QGraphicsItem):
    """Screen-readable X/Y axes at a map-frame pose."""

    def __init__(self, yaw, length):
        super().__init__()
        self.length = length
        self.setRotation(-math.degrees(yaw))
        self.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations)
        self.setZValue(5.5)

    def boundingRect(self):
        return QtCore.QRectF(-4, -self.length - 4, self.length + 8, self.length + 8)

    def paint(self, painter, _option, _widget=None):
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        for end, color in (
            (QtCore.QPointF(self.length, 0), "#D92D20"),
            (QtCore.QPointF(0, -self.length), "#039855"),
        ):
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
            painter.drawLine(QtCore.QLineF(QtCore.QPointF(0, 0), end))
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor(color))
            painter.drawEllipse(end, 2.5, 2.5)


class TopViewCanvas(QtWidgets.QGraphicsView):
    manual_navigation = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRenderHints(
            QtGui.QPainter.Antialiasing | QtGui.QPainter.TextAntialiasing
        )
        self.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setBackgroundBrush(QtGui.QColor("#FAFBFD"))
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

    def wheelEvent(self, event):
        self.manual_navigation.emit()
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)
        event.accept()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.manual_navigation.emit()
        super().mousePressEvent(event)

    def drawBackground(self, painter, rect):
        super().drawBackground(painter, rect)
        step = PIXELS_PER_METER * 0.5
        painter.setPen(QtGui.QPen(QtGui.QColor("#E4E9F0"), 0))
        x = math.floor(rect.left() / step) * step
        while x <= rect.right():
            painter.drawLine(QtCore.QLineF(x, rect.top(), x, rect.bottom()))
            x += step
        y = math.floor(rect.top() / step) * step
        while y <= rect.bottom():
            painter.drawLine(QtCore.QLineF(rect.left(), y, rect.right(), y))
            y += step
        painter.setPen(QtGui.QPen(QtGui.QColor("#AAB6C5"), 1))
        painter.drawLine(QtCore.QLineF(0, rect.top(), 0, rect.bottom()))
        painter.drawLine(QtCore.QLineF(rect.left(), 0, rect.right(), 0))


class TopViewPanel(QtWidgets.QGroupBox):
    """A resizable, zoomable view of selected robots and coordinate frames."""

    def __init__(self, parent=None):
        super().__init__("Cooperative map view", parent)
        self.setMinimumHeight(220)
        self._scene = QtWidgets.QGraphicsScene(self)
        self._content_rect = QtCore.QRectF(-200, -150, 400, 300)
        self._selection = None
        self._fit_pending = True
        self._manual_navigation = False

        layout = QtWidgets.QVBoxLayout(self)
        toolbar = QtWidgets.QHBoxLayout()
        toolbar.addWidget(QtWidgets.QLabel("map: +X →   +Y ↑   ·   grid 0.5 m"))
        self.legend = QtWidgets.QLabel()
        toolbar.addWidget(self.legend, 1)
        fit_button = QtWidgets.QPushButton("Fit")
        fit_button.setToolTip("Fit all available robots, arms, targets and object")
        fit_button.clicked.connect(self.fit_content)
        toolbar.addWidget(fit_button)
        layout.addLayout(toolbar)

        self.canvas = TopViewCanvas(self)
        self.canvas.setScene(self._scene)
        self.canvas.manual_navigation.connect(self._mark_manual_navigation)
        layout.addWidget(self.canvas, 1)

        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.update_snapshot(ViewSnapshot((), (), {}, {}))

    def _mark_manual_navigation(self):
        self._fit_pending = False
        self._manual_navigation = True

    def fit_content(self):
        self.canvas.fitInView(self._content_rect, QtCore.Qt.KeepAspectRatio)
        self._fit_pending = False
        self._manual_navigation = False

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not self._manual_navigation:
            QtCore.QTimer.singleShot(0, self.fit_content)

    def update_snapshot(self, snapshot):
        selection = (snapshot.robots, snapshot.sides)
        if selection != self._selection:
            self._fit_pending = True
            self._manual_navigation = False
            self._selection = selection
        self._scene.clear()

        if not snapshot.robots:
            self.legend.clear()
            self.status.setText("No MuR selected")
            label = self._scene.addText("Select a MuR to show its map frames.")
            label.setDefaultTextColor(QtGui.QColor("#667085"))
            label.setPos(-label.boundingRect().width() / 2, -15)
            self._content_rect = QtCore.QRectF(-200, -150, 400, 300)
            self._scene.setSceneRect(self._content_rect)
            if self._fit_pending:
                self.fit_content()
            return

        self.legend.setText("   ".join(
            f'<span style="color:{ROBOT_COLORS[robot]}"><b>{robot[-1]}</b></span>'
            for robot in snapshot.robots
        ))
        positions = []
        for robot in snapshot.robots:
            base_frame = f"{robot}/base_link"
            base = snapshot.poses.get(base_frame)
            if base is not None:
                self._draw_robot(robot, base)
                origin = scene_point(base)
                positions.extend((QtCore.QPointF(origin.x() - 85, origin.y() - 65),
                                  QtCore.QPointF(origin.x() + 85, origin.y() + 65)))
            for side in snapshot.sides:
                positions.extend(self._draw_arm(snapshot, robot, side))
                target_frame = f"{robot}/{ARM_PREFIX[side]}/virtual_object_target_tcp"
                target = snapshot.poses.get(target_frame)
                if target is not None:
                    self._draw_target(robot, side, target)
                    positions.append(scene_point(target))

        object_pose = snapshot.poses.get(OBJECT_FRAME)
        if object_pose is not None:
            self._draw_object(object_pose)
            positions.append(scene_point(object_pose))

        if not positions:
            label = self._scene.addText("Waiting for map TF…")
            label.setDefaultTextColor(QtGui.QColor("#667085"))
            label.setPos(-label.boundingRect().width() / 2, -15)
        self._content_rect = self._bounds(positions)
        self._scene.setSceneRect(self._content_rect)
        self._set_status(snapshot.problems)
        if self._fit_pending or (
            not self._manual_navigation
            and not self.canvas.mapToScene(
                self.canvas.viewport().rect()
            ).boundingRect().contains(self._content_rect)
        ):
            self.fit_content()

    @staticmethod
    def _bounds(points):
        if not points:
            return QtCore.QRectF(-200, -150, 400, 300)
        xs = [point.x() for point in points]
        ys = [point.y() for point in points]
        width = max(400.0, max(xs) - min(xs) + 100.0)
        height = max(300.0, max(ys) - min(ys) + 100.0)
        return QtCore.QRectF((min(xs) + max(xs) - width) / 2,
                             (min(ys) + max(ys) - height) / 2, width, height)

    @staticmethod
    def _short_frame(frame):
        if frame == OBJECT_FRAME:
            return "object"
        robot, _, rest = frame.partition("/")
        short = robot[-1]
        if rest == "base_link":
            return f"{short}/base"
        for side, prefix in ARM_PREFIX.items():
            if rest.startswith(prefix + "/"):
                link = rest[len(prefix) + 1:]
                if link == "virtual_object_target_tcp":
                    link = "target"
                return f"{short}/{side.upper()} {link}"
        return frame

    def _set_status(self, problems):
        if not problems:
            self.status.setText("All requested TF frames available")
            self.status.setToolTip("")
            return
        important = [
            (frame, reason) for frame, reason in problems.items()
            if frame == OBJECT_FRAME
            or frame.endswith("/virtual_object_target_tcp")
            or (frame.endswith("/base_link") and "/UR10_" not in frame)
        ]
        shown = important or list(problems.items())
        preview = ", ".join(
            f"{self._short_frame(frame)} ({reason})" for frame, reason in shown[:4]
        )
        remaining = len(problems) - min(4, len(shown))
        if remaining:
            preview += f" … (+{remaining} arm/other frames)"
        self.status.setText(f"TF unavailable: {preview}")
        self.status.setToolTip(
            "\n".join(f"{frame}: {reason}" for frame, reason in problems.items())
        )

    def _draw_robot(self, robot, pose):
        point = scene_point(pose)
        color = ROBOT_COLORS[robot]
        glyph = RobotGlyph(color)
        glyph.setPos(point)
        glyph.setRotation(-math.degrees(pose.yaw))
        glyph.setData(0, f"robot:{robot}")
        self._scene.addItem(glyph)

        badge = RobotBadge(robot[-1], color)
        badge.setPos(point)
        self._scene.addItem(badge)
        self._draw_axes(pose, f"{robot[-1]}/base_link", color, 25)

    def _draw_arm(self, snapshot, robot, side):
        prefix = f"{robot}/{ARM_PREFIX[side]}"
        color = QtGui.QColor(ROBOT_COLORS[robot])
        points = []
        pen = QtGui.QPen(color, 5, QtCore.Qt.SolidLine if side == "l"
                         else QtCore.Qt.DashLine, QtCore.Qt.RoundCap)
        previous = None
        for link in ARM_LINKS:
            pose = snapshot.poses.get(f"{prefix}/{link}")
            if pose is None:
                previous = None
                continue
            point = scene_point(pose)
            points.append(point)
            if previous is not None:
                segment = self._scene.addLine(QtCore.QLineF(previous, point), pen)
                segment.setZValue(2)
            radius = 5 if link == "tool0" else 3
            joint = self._scene.addEllipse(
                point.x() - radius, point.y() - radius, radius * 2, radius * 2,
                QtGui.QPen(color, 1), QtGui.QBrush(QtGui.QColor("#FFFFFF"))
            )
            joint.setZValue(2.5)
            if link == "tool0":
                label = self._scene.addSimpleText(
                    side.upper(), QtGui.QFont("Sans Serif", 8, QtGui.QFont.Bold)
                )
                label.setBrush(color)
                label.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations)
                label.setPos(point.x() + 5, point.y() - 11)
                label.setZValue(3)
            previous = point
        return points

    def _draw_target(self, robot, side, pose):
        point = scene_point(pose)
        color = QtGui.QColor(ROBOT_COLORS[robot])
        marker = self._scene.addEllipse(
            -10, -10, 20, 20,
            QtGui.QPen(color, 2, QtCore.Qt.DashLine), QtGui.QBrush(QtCore.Qt.NoBrush)
        )
        marker.setPos(point)
        marker.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations)
        marker.setData(0, f"target:{robot}:{side}")
        marker.setZValue(5)
        self._draw_axes(pose, f"{robot[-1]}/{side.upper()} target", color.name(), 18)

    def _draw_object(self, pose):
        point = scene_point(pose)
        color = QtGui.QColor("#B8860B")
        diamond = QtGui.QPolygonF((
            QtCore.QPointF(0, -12),
            QtCore.QPointF(12, 0),
            QtCore.QPointF(0, 12),
            QtCore.QPointF(-12, 0),
        ))
        marker = self._scene.addPolygon(
            diamond, QtGui.QPen(color, 2), QtGui.QBrush(QtGui.QColor("#FFF3BF"))
        )
        marker.setPos(point)
        marker.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations)
        marker.setData(0, "object")
        marker.setZValue(5)
        self._draw_axes(pose, "object", color.name(), 26)

    def _draw_axes(self, pose, label, color, length):
        point = scene_point(pose)
        axes = FrameAxes(pose.yaw, length)
        axes.setPos(point)
        self._scene.addItem(axes)
        caption = self._scene.addSimpleText(label, QtGui.QFont("Sans Serif", 8))
        caption.setBrush(QtGui.QColor(color))
        caption.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations)
        caption.setPos(point.x() + 13, point.y() + 9)
        caption.setZValue(6)
