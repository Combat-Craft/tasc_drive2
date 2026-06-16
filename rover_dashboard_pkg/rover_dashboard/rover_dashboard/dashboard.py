import json
import math
import os
import sys
import threading
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except Exception:
    rclpy = None
    Node = object
    String = object
    ROS_AVAILABLE = False

from PySide6.QtCore import QObject, QProcess, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QProgressBar,
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

try:
    from .shared_topics import (
        DEFAULT_LIGHTS,
        DEFAULT_MOTORS,
        TOPIC_RELAY_COMMAND,
        TOPIC_GUI_HEARTBEAT,
        TOPIC_GUI_TELEMETRY,
    )
except ImportError:
    from shared_topics import (  # type: ignore
        DEFAULT_LIGHTS,
        DEFAULT_MOTORS,
        TOPIC_RELAY_COMMAND,
        TOPIC_GUI_HEARTBEAT,
        TOPIC_GUI_TELEMETRY,
    )

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def format_elapsed(seconds: int) -> str:
    hours, remainder = divmod(seconds, 3600)
    minutes, secs = divmod(remainder, 60)
    return f'{hours:02d}:{minutes:02d}:{secs:02d}'


def humanize_joint_name(name: str) -> str:
    mapping = {
        'front_left_wheel_joint': 'Front Left Wheel',
        'middle_left_wheel_joint': 'Middle Left Wheel',
        'rear_left_wheel_joint': 'Rear Left Wheel',
        'front_right_wheel_joint': 'Front Right Wheel',
        'middle_right_wheel_joint': 'Middle Right Wheel',
        'rear_right_wheel_joint': 'Rear Right Wheel',
        'left_headlight': 'Left Headlight',
        'right_headlight': 'Right Headlight',
    }
    return mapping.get(name, name)


class TelemetryBridge(QObject):
    telemetry_received = Signal(dict)
    heartbeat_received = Signal(str)
    ros_status_changed = Signal(str)


class RosDashboardNode(Node):
    def __init__(self, bridge: TelemetryBridge):
        super().__init__('rover_dashboard_ui')
        self.bridge = bridge
        self.telemetry_sub = self.create_subscription(String, TOPIC_GUI_TELEMETRY, self._telemetry_cb, 10)
        self.heartbeat_sub = self.create_subscription(String, TOPIC_GUI_HEARTBEAT, self._heartbeat_cb, 10)
        self.command_pub = self.create_publisher(String, TOPIC_RELAY_COMMAND, 10)
        self.bridge.ros_status_changed.emit('ROS 2 connected')

    def _telemetry_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.bridge.telemetry_received.emit(payload)
        except Exception as exc:
            self.get_logger().error(f'Failed to parse telemetry JSON: {exc}')

    def _heartbeat_cb(self, msg: String) -> None:
        self.bridge.heartbeat_received.emit(msg.data)

    def send_command(self, command: Dict) -> None:
        msg = String()
        msg.data = json.dumps(command)
        self.command_pub.publish(msg)


class RosSpinThread(threading.Thread):
    def __init__(self, node: RosDashboardNode):
        super().__init__(daemon=True)
        self.node = node
        self._running = True

    def run(self) -> None:
        while self._running and ROS_AVAILABLE:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def stop(self) -> None:
        self._running = False


@dataclass
class ComponentState:
    name: str
    display_name: str
    kind: str
    enabled: bool = False  # This is the DISPLAY state (what dashboard shows)
    real_enabled: bool = False  # This is the REAL relay state
    attached: bool = False
    channel: int = 0
    voltage: float = 0.0
    current: float = 0.0
    position: float = 0.0
    velocity: float = 0.0
    temperature: float = 0.0
    health: str = 'offline'
    fault: str = 'No data'
    relay_fault: str = 'No data'
    motor_fault: str = 'No data'
    last_command: str = 'None'
    last_update: str = '--'

    @property
    def is_motor(self) -> bool:
        return self.kind == 'motor'

    @property
    def status_label(self) -> str:
        return 'ON' if self.enabled else 'OFF'


class StatusBadge(QLabel):
    COLORS = {
        'healthy': '#1db954',
        'warning': '#f0ad4e',
        'fault': '#e74c3c',
        'offline': '#6c757d',
        'connected': '#1db954',
    }

    def __init__(self, text: str = 'OFFLINE', status_key: str = 'offline') -> None:
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(92)
        self.set_status(text, status_key)

    def set_status(self, text: str, status_key: str) -> None:
        color = self.COLORS.get(status_key, '#6c757d')
        self.setText(text)
        self.setStyleSheet(
            f'background:{color}; color:white; border-radius:10px; padding:4px 10px; font-weight:700;'
        )


class MetricPill(QFrame):
    def __init__(self, label: str, value: str = '--') -> None:
        super().__init__()
        self.setObjectName('metricPill')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        self.title = QLabel(label)
        self.title.setObjectName('metricTitle')
        self.value = QLabel(value)
        self.value.setObjectName('metricValue')
        layout.addWidget(self.title)
        layout.addWidget(self.value)

    def set_value(self, value: str) -> None:
        self.value.setText(value)


class TopologyComponentBlock(QFrame):
    toggled = Signal(str, bool)
    details_requested = Signal(str)

    def __init__(self, component_name: str) -> None:
        super().__init__()
        self.component_name = component_name
        self.state = ComponentState(
            name=component_name,
            display_name=humanize_joint_name(component_name),
            kind='motor' if component_name in DEFAULT_MOTORS else 'light',
        )
        self.setObjectName('componentBlock')
        self._build_ui()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        title_row = QHBoxLayout()
        self.title = QLabel(self.state.display_name)
        self.title.setObjectName('topologyTitle')
        self.title.setWordWrap(True)
        self.badge = StatusBadge('OFFLINE', 'offline')
        title_row.addWidget(self.title)
        title_row.addStretch(1)
        title_row.addWidget(self.badge)
        root.addLayout(title_row)

        self.summary = QLabel('Power OFF')
        self.summary.setWordWrap(True)
        self.summary.setObjectName('topologySummary')
        root.addWidget(self.summary)

        self.stats = QLabel('No telemetry')
        self.stats.setWordWrap(True)
        self.stats.setObjectName('topologyStats')
        root.addWidget(self.stats)

        self.fault_label = QLabel('Fault: No data')
        self.fault_label.setWordWrap(True)
        self.fault_label.setObjectName('faultLabel')
        root.addWidget(self.fault_label)

        button_row = QHBoxLayout()
        button_row.setSpacing(8)
        self.toggle_button = QPushButton('Power ON')
        self.toggle_button.setMinimumHeight(34)
        self.toggle_button.clicked.connect(self._emit_toggle)
        self.detail_button = QPushButton('Details')
        self.detail_button.setMinimumHeight(34)
        self.detail_button.clicked.connect(lambda: self.details_requested.emit(self.component_name))
        button_row.addWidget(self.toggle_button)
        button_row.addWidget(self.detail_button)
        root.addLayout(button_row)

    def _emit_toggle(self) -> None:
        # Send the REAL state (opposite of what dashboard shows)
        self.toggled.emit(self.component_name, not self.state.real_enabled)

    def update_state(self, state: ComponentState) -> None:
        self.state = state
        self.title.setText(state.display_name)
        self.badge.set_status(state.health.upper(), state.health)
        self.summary.setText(f'Power {state.status_label}   Ch {state.channel}   Bus {state.voltage:0.1f} V')
        if state.is_motor:
            self.stats.setText(
                f'Temp {state.temperature:0.1f} C   Vel {state.velocity:0.2f} rad/s\n'
                f'Pos {state.position:0.2f} rad   Cur {state.current:0.2f} A'
            )
        else:
            self.stats.setText('Relay control active.\nOn/off state only for now.')
        fault_text = state.fault if state.fault and state.fault != 'None' else 'Ready'
        self.fault_label.setText(f'Status: {fault_text}')
        self.toggle_button.setText('Power OFF' if state.enabled else 'Power ON')


class TopologyGraphView(QFrame):
    toggled = Signal(str, bool)
    details_requested = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName('topologyGraph')
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(1320, 980)
        self.components: Dict[str, TopologyComponentBlock] = {}
        self.battery = QFrame(self)
        self.battery.setObjectName('systemBlock')
        self.battery_label = QLabel('Battery', self.battery)
        self.battery_value = QLabel('-- V', self.battery)
        self.battery_value.setObjectName('systemValue')

        self.bus = QFrame(self)
        self.bus.setObjectName('systemBlock')
        self.bus_label = QLabel('Relay Bus', self.bus)
        self.bus_value = QLabel('-- A', self.bus)
        self.bus_value.setObjectName('systemValue')

    def ensure_components(self, component_names) -> None:
        for name in component_names:
            if name in self.components:
                continue
            block = TopologyComponentBlock(name)
            block.setParent(self)
            block.toggled.connect(self.toggled.emit)
            block.details_requested.connect(self.details_requested.emit)
            self.components[name] = block
            block.show()
        self._layout_children()

    def update_summary(self, battery_voltage: float, total_current: float) -> None:
        self.battery_value.setText(f'{battery_voltage:0.2f} V')
        self.bus_value.setText(f'{total_current:0.2f} A')

    def update_component_state(self, state: ComponentState) -> None:
        self.ensure_components([state.name])
        self.components[state.name].update_state(state)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._layout_children()

    def _layout_children(self) -> None:
        width = max(self.width(), self.minimumWidth())
        height = max(self.height(), self.minimumHeight())
        if width <= 0 or height <= 0:
            return

        battery_rect = self._rect(width * 0.39, height * 0.03, width * 0.22, height * 0.10)
        bus_rect = self._rect(width * 0.35, height * 0.18, width * 0.30, height * 0.11)
        self.battery.setGeometry(*battery_rect)
        self.bus.setGeometry(*bus_rect)
        self._layout_system_block(self.battery, self.battery_label, self.battery_value)
        self._layout_system_block(self.bus, self.bus_label, self.bus_value)

        positions = {
            'Left Headlight': self._rect(width * 0.03, height * 0.05, width * 0.27, height * 0.24),
            'Right Headlight': self._rect(width * 0.70, height * 0.05, width * 0.27, height * 0.24),
            'front_left_wheel_joint': self._rect(width * 0.02, height * 0.32, width * 0.30, height * 0.20),
            'middle_left_wheel_joint': self._rect(width * 0.02, height * 0.52, width * 0.30, height * 0.20),
            'rear_left_wheel_joint': self._rect(width * 0.02, height * 0.72, width * 0.30, height * 0.20),
            'front_right_wheel_joint': self._rect(width * 0.68, height * 0.32, width * 0.30, height * 0.20),
            'middle_right_wheel_joint': self._rect(width * 0.68, height * 0.52, width * 0.30, height * 0.20),
            'rear_right_wheel_joint': self._rect(width * 0.68, height * 0.72, width * 0.30, height * 0.20),
        }

        for name, block in self.components.items():
            rect = positions.get(name, self._rect(width * 0.34, height * 0.60, width * 0.32, height * 0.24))
            block.setGeometry(*rect)

    def _layout_system_block(self, frame: QFrame, title: QLabel, value: QLabel) -> None:
        title.setGeometry(10, 8, frame.width() - 20, 22)
        value.setGeometry(10, 34, frame.width() - 20, 28)

    def _rect(self, x: float, y: float, w: float, h: float):
        return int(x), int(y), int(w), int(h)

    def paintEvent(self, event) -> None:
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        pen = QPen(QColor('#4e6278'))
        pen.setWidth(3)
        painter.setPen(pen)

        battery_center = self.battery.geometry().center()
        bus_center = self.bus.geometry().center()
        painter.drawLine(battery_center, bus_center)

        for name, block in self.components.items():
            center = block.geometry().center()
            painter.drawLine(bus_center, center)


class UrdfVisualLauncher(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName('sideCard')
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        title = QLabel('3D View')
        title.setObjectName('cardTitle')
        description = QLabel('Open the real RViz URDF view.')
        description.setObjectName('metaLabel')
        self.status_label = QLabel('RViz status: not launched')
        self.status_label.setObjectName('metaLabel')
        self.open_button = QPushButton('Open RViz')
        self.open_button.clicked.connect(self.launch_rviz)
        layout.addWidget(title)
        layout.addWidget(description)
        layout.addWidget(self.status_label)
        layout.addWidget(self.open_button)

    def launch_rviz(self) -> None:
        if get_package_share_directory is None:
            self.status_label.setText('RViz status: ament index unavailable')
            return
        try:
            description_share = get_package_share_directory('drive_description')
        except Exception as exc:
            self.status_label.setText(f'RViz status: drive_description missing ({exc})')
            return
        rviz_config = os.path.join(description_share, 'rviz', 'drive_model.rviz')
        launched = QProcess.startDetached('rviz2', ['-d', rviz_config])
        self.status_label.setText('RViz status: launched' if launched else 'RViz status: launch failed')


class DetailPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__('Component Details')
        layout = QVBoxLayout(self)
        self.title = QLabel('Select a component')
        self.title.setObjectName('detailTitle')
        self.info = QTextEdit()
        self.info.setReadOnly(True)
        self.info.setMinimumHeight(240)
        layout.addWidget(self.title)
        layout.addWidget(self.info)

    def render_component(self, state: ComponentState) -> None:
        self.title.setText(state.display_name)
        if state.is_motor:
            text = (
                f'Type: Motor\n'
                f'Joint name: {state.name}\n'
                f'Powered: {state.enabled}\n'
                f'Attached: {state.attached}\n'
                f'Relay channel: {state.channel}\n'
                f'Bus voltage: {state.voltage:0.2f} V\n'
                f'Relay current: {state.current:0.2f} A\n'
                f'Position: {state.position:0.4f} rad\n'
                f'Velocity: {state.velocity:0.4f} rad/s\n'
                f'Temperature: {state.temperature:0.1f} C\n'
                f'Health: {state.health}\n'
                f'Fault: {state.fault}\n'
                f'Relay fault: {state.relay_fault}\n'
                f'Motor fault: {state.motor_fault}\n'
                f'Last command: {state.last_command}\n'
                f'Last motor update: {state.last_update}\n'
            )
        else:
            text = (
                f'Type: Headlight\n'
                f'Name: {state.name}\n'
                f'Powered: {state.enabled}\n'
                f'Relay channel: {state.channel}\n'
                f'Bus voltage: {state.voltage:0.2f} V\n'
                f'Health: {state.health}\n'
                f'Fault: {state.fault}\n'
                f'Last command: {state.last_command}\n'
            )
        self.info.setPlainText(text)


class HeaderBar(QFrame):
    kill_requested = Signal()
    turn_on_all_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName('headerBar')
        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)

        title_col = QVBoxLayout()
        title = QLabel('Rover Topology & Power Dashboard')
        title.setObjectName('mainTitle')
        subtitle = QLabel('Motor and headlight controls with inline telemetry')
        subtitle.setObjectName('subTitle')
        title_col.addWidget(title)
        title_col.addWidget(subtitle)
        layout.addLayout(title_col)

        layout.addStretch(1)
        self.ros_badge = StatusBadge('DEMO MODE', 'offline')
        self.heartbeat_label = QLabel('Elapsed: 00:00:00')
        self.mode_badge = StatusBadge('SAFE', 'warning')
        
        self.turn_on_all_button = QPushButton('TURN ON ALL')
        self.turn_on_all_button.setObjectName('turnOnAllButton')
        self.turn_on_all_button.clicked.connect(self.turn_on_all_requested.emit)
        
        self.kill_button = QPushButton('SOFTWARE KILL')
        self.kill_button.setObjectName('killButton')
        self.kill_button.clicked.connect(self.kill_requested.emit)
        
        layout.addWidget(self.ros_badge)
        layout.addWidget(self.heartbeat_label)
        layout.addWidget(self.mode_badge)
        layout.addWidget(self.turn_on_all_button)
        layout.addWidget(self.kill_button)


class SummaryStrip(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName('summaryStrip')
        layout = QHBoxLayout(self)
        self.items = {
            'battery': MetricPill('Battery', '-- V'),
            'current': MetricPill('Total Relay Current', '-- A'),
            'motors': MetricPill('Motors Online', '--'),
            'faults': MetricPill('Fault Count', '--'),
        }
        for item in self.items.values():
            layout.addWidget(item)

        self.battery_bar = QProgressBar()
        self.battery_bar.setFormat('Battery %p%')
        self.battery_bar.setValue(0)
        self.battery_bar.setMinimumWidth(180)
        layout.addWidget(self.battery_bar)

    def update_summary(self, battery_voltage: float, total_current: float, motors_online: int, fault_count: int) -> None:
        self.items['battery'].set_value(f'{battery_voltage:0.2f} V')
        self.items['current'].set_value(f'{total_current:0.2f} A')
        self.items['motors'].set_value(str(motors_online))
        self.items['faults'].set_value(str(fault_count))
        pct = int(clamp((battery_voltage - 20.0) / 8.0 * 100.0, 0.0, 100.0))
        self.battery_bar.setValue(pct)


class LogPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__('Event Log')
        layout = QVBoxLayout(self)
        self.text = QTextEdit()
        self.text.setReadOnly(True)
        layout.addWidget(self.text)

    def add_entry(self, entry: str) -> None:
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.text.append(f'[{timestamp}] {entry}')


class DashboardWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('Rover Dashboard')
        self.resize(1680, 980)
        self.bridge = TelemetryBridge()
        self.bridge.telemetry_received.connect(self.on_telemetry)
        self.bridge.heartbeat_received.connect(self.on_heartbeat)
        self.bridge.ros_status_changed.connect(self.on_ros_status)

        self.ros_node: Optional[RosDashboardNode] = None
        self.spin_thread: Optional[RosSpinThread] = None
        self.demo_timer = None
        self.elapsed_timer = QTimer(self)
        self.elapsed_timer.timeout.connect(self._update_elapsed_label)
        self.elapsed_timer.start(1000)
        self.gui_launch_time = datetime.now()
        self.seen_backend_heartbeat = False
        self.current_selected: Optional[str] = None
        self.latest_payload = {}
        
        # Auto-connect timer variables
        self.connect_timer = None
        self.connect_timeout = None
        self.connect_attempts = 0

        self.component_states: Dict[str, ComponentState] = {
            name: ComponentState(name=name, display_name=humanize_joint_name(name), kind='motor')
            for name in DEFAULT_MOTORS
        }
        for name in DEFAULT_LIGHTS:
            self.component_states[name] = ComponentState(name=name, display_name=name, kind='light')

        self._build_ui()
        self._apply_styles()
        self._setup_ros_or_demo()
        self.topology.ensure_components(list(self.component_states.keys()))
        self._update_elapsed_label()

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        self.header = HeaderBar()
        self.header.kill_requested.connect(self.on_kill_requested)
        self.header.turn_on_all_requested.connect(self.on_turn_on_all_requested) 
        root.addWidget(self.header)

        self.summary = SummaryStrip()
        root.addWidget(self.summary)

        main_splitter = QSplitter(Qt.Horizontal)
        main_splitter.setChildrenCollapsible(False)
        root.addWidget(main_splitter, 1)

        topology_box = QGroupBox('Rover Topology')
        topology_layout = QVBoxLayout(topology_box)
        topology_layout.setContentsMargins(8, 18, 8, 8)
        topology_scroll = QScrollArea()
        topology_scroll.setWidgetResizable(False)
        topology_scroll.setFrameShape(QFrame.NoFrame)
        topology_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        topology_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.topology = TopologyGraphView()
        self.topology.toggled.connect(self.on_toggle_component)
        self.topology.details_requested.connect(self.on_show_details)
        topology_scroll.setWidget(self.topology)
        topology_layout.addWidget(topology_scroll)
        main_splitter.addWidget(topology_box)

        side_column = QWidget()
        side_layout = QVBoxLayout(side_column)
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.setSpacing(12)

        self.rviz_panel = UrdfVisualLauncher()
        side_layout.addWidget(self.rviz_panel)
        self.detail_panel = DetailPanel()
        side_layout.addWidget(self.detail_panel, 1)
        main_splitter.addWidget(side_column)
        main_splitter.setStretchFactor(0, 4)
        main_splitter.setStretchFactor(1, 1)
        main_splitter.setSizes([1180, 360])

        self.log_panel = LogPanel()
        root.addWidget(self.log_panel)

        self.setCentralWidget(central)

    def _apply_styles(self) -> None:
        self.setStyleSheet(
            """
            QWidget { background: #0f1720; color: #e5e7eb; font-family: Arial; font-size: 13px; }
            QMainWindow { background: #0f1720; }
            QGroupBox {
                border: 1px solid #263241; border-radius: 12px; margin-top: 12px;
                font-weight: 700; padding-top: 12px; background: #111827;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
            #headerBar, #summaryStrip, #componentBlock, #topologyGraph, #metricPill, #sideCard, #systemBlock {
                background: #111827; border: 1px solid #263241; border-radius: 12px;
            }
            #mainTitle { font-size: 24px; font-weight: 800; }
            #subTitle, #metaLabel, #topologyStats, #topologySummary { color: #c7d0dc; }
            #cardTitle, #detailTitle { font-size: 16px; font-weight: 700; }
            #topologyTitle { font-size: 14px; font-weight: 700; color: #eef4ff; }
            #topologyStats { font-size: 13px; line-height: 1.35em; }
            #metaLabel, #topologySummary { font-size: 12px; }
            #metricTitle { color: #93a3b8; font-size: 11px; }
            #metricValue, #systemValue { font-size: 18px; font-weight: 700; }
            #faultLabel { color: #f8d7da; font-size: 12px; }
            QPushButton {
                background: #1f6feb; border: none; border-radius: 10px; padding: 9px 12px;
                font-weight: 700;
            }
            QPushButton:hover { background: #388bfd; }
            #componentBlock QPushButton { padding: 6px 8px; font-size: 11px; }
            QPushButton#killButton { background: #e74c3c; min-width: 160px; }
            QPushButton#killButton:hover { background: #ff6655; }
            #turnOnAllButton { background: #1db954; min-width: 160px; }
            #turnOnAllButton:hover { background: #26c456; }
            QTextEdit {
                background: #0b1220; border: 1px solid #263241; border-radius: 10px;
            }
            QProgressBar {
                background: #0b1220; border: 1px solid #263241; border-radius: 8px; text-align: center;
                min-height: 24px;
            }
            QProgressBar::chunk { background: #1db954; border-radius: 8px; }
            """
        )

    def on_turn_on_all_requested(self) -> None:
        confirm = QMessageBox.question(
            self,
            'Confirm Turn On All',
            'Turn on all motors? This will enable all motor power outputs.',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if confirm != QMessageBox.Yes:
            return
        
        self.log_panel.add_entry('TURN ON ALL requested - will keep trying until motors connect...')
        
        # Disable button while trying
        self.header.turn_on_all_button.setEnabled(False)
        self.header.turn_on_all_button.setText('CONNECTING...')
        
        # Create a timer that keeps publishing every 1 second
        self.connect_attempts = 0
        self.connect_timer = QTimer(self)
        self.connect_timer.timeout.connect(self._publish_turn_on_all)
        self.connect_timer.start(1000)  # Every 1 second
        
        # 10 second timeout max attempts
        self.connect_timeout = QTimer(self)
        self.connect_timeout.setSingleShot(True)
        self.connect_timeout.timeout.connect(self._stop_connecting)
        self.connect_timeout.start(10000)  # 10 seconds max

    def _publish_turn_on_all(self) -> None:
        """Keep publishing turn_on_all command until motors connect"""
        self.connect_attempts += 1
        
        command = {
            'type': 'turn_on_all',
            'timestamp': datetime.now().isoformat(),
        }
        
        if self.ros_node:
            self.ros_node.send_command(command)
            self.log_panel.add_entry(f'Published TURN ON ALL (attempt {self.connect_attempts})')
        
        # Check if motors are connected via telemetry
        motors_connected = False
        for state in self.component_states.values():
            if state.is_motor and state.attached and state.real_enabled == False:  # relay OFF = motors ON
                motors_connected = True
                break
        
        if motors_connected or self.connect_attempts >= 10:  # Stop if connected or 10 attempts
            self._stop_connecting()
            if motors_connected:
                self.log_panel.add_entry(f'✅ Motors connected after {self.connect_attempts} attempts!')
                self.header.mode_badge.set_status('ARMED', 'healthy')
                self.header.turn_on_all_button.setText('MOTORS ON')
                self.header.turn_on_all_button.setStyleSheet('background: #1db954; min-width: 160px;')
            else:
                self.log_panel.add_entry('❌ Timeout (10s): Motors did not connect. Check hardware.')

    def _stop_connecting(self) -> None:
        """Stop the connection attempts"""
        if self.connect_timer:
            self.connect_timer.stop()
        if self.connect_timeout:
            self.connect_timeout.stop()
        
        self.header.turn_on_all_button.setEnabled(True)
        if self.header.turn_on_all_button.text() != 'MOTORS ON':
            self.header.turn_on_all_button.setText('TURN ON ALL')
            self.header.turn_on_all_button.setStyleSheet('')

    def _setup_ros_or_demo(self) -> None:
        if ROS_AVAILABLE:
            try:
                rclpy.init(args=None)
                self.ros_node = RosDashboardNode(self.bridge)
                self.spin_thread = RosSpinThread(self.ros_node)
                self.spin_thread.start()
                self.log_panel.add_entry('ROS 2 initialized. Waiting for topology telemetry...')
                return
            except Exception as exc:
                self.log_panel.add_entry(f'ROS 2 unavailable, starting demo mode: {exc}')

        self.header.ros_badge.set_status('DEMO MODE', 'offline')
        self.demo_timer = QTimer(self)
        self.demo_timer.timeout.connect(self._emit_demo_payload)
        self.demo_timer.start(500)
        self.log_panel.add_entry('Running internal dashboard demo mode.')

    def _emit_demo_payload(self) -> None:
        t = datetime.now().timestamp()
        self.on_telemetry(build_fake_payload(t))
        self.on_heartbeat(datetime.now().strftime('%H:%M:%S'))

    def _update_elapsed_label(self) -> None:
        elapsed = int((datetime.now() - self.gui_launch_time).total_seconds())
        self.header.heartbeat_label.setText(f'Elapsed: {format_elapsed(elapsed)}')

    def on_ros_status(self, text: str) -> None:
        self.header.ros_badge.set_status('ROS 2', 'connected')
        self.log_panel.add_entry(text)

    def on_heartbeat(self, beat: str) -> None:
        if beat and beat != '--' and not self.seen_backend_heartbeat:
            self.seen_backend_heartbeat = True
            self.log_panel.add_entry('Backend heartbeat detected.')

    def on_telemetry(self, payload: Dict) -> None:
        self.latest_payload = payload
        relays = payload.get('subsystems', {})
        motors = payload.get('motors', {})
        component_names = list(dict.fromkeys(list(relays.keys()) + list(motors.keys()) + list(self.component_states.keys())))
        self.topology.ensure_components(component_names)

        fault_count = 0
        motors_online = 0

        for name in component_names:
            relay_data = relays.get(name, {})
            motor_data = motors.get(name, {})
            state = self.component_states.get(
                name,
                ComponentState(
                    name=name,
                    display_name=humanize_joint_name(name),
                    kind='motor' if name in DEFAULT_MOTORS else 'light',
                ),
            )

            # Store the REAL relay state
            real_enabled = bool(relay_data.get('enabled', state.real_enabled))
            state.real_enabled = real_enabled
            # DISPLAY is inverted (light ON = show OFF)
            state.enabled = not real_enabled
            
            state.channel = int(relay_data.get('channel', state.channel))
            state.voltage = float(relay_data.get('voltage', state.voltage))
            state.current = float(relay_data.get('current', state.current))
            state.last_command = relay_data.get('last_command', state.last_command)
            state.relay_fault = relay_data.get('fault', 'Relay telemetry unavailable')

            if state.is_motor:
                state.position = float(motor_data.get('position', state.position))
                state.velocity = float(motor_data.get('velocity', state.velocity))
                state.temperature = float(motor_data.get('temperature', state.temperature))
                state.attached = bool(motor_data.get('attached', state.attached))
                state.last_update = motor_data.get('last_update', state.last_update)
                state.motor_fault = motor_data.get('fault', 'Motor telemetry unavailable')
                relay_health = relay_data.get('health', 'offline' if not real_enabled else 'healthy')
                motor_health = motor_data.get('health', 'offline' if not state.attached else 'healthy')

                if relay_health == 'fault' or motor_health == 'fault':
                    state.health = 'fault'
                elif relay_health == 'warning' or motor_health == 'warning':
                    state.health = 'warning'
                elif real_enabled and state.attached:
                    state.health = 'healthy'
                elif real_enabled or state.attached:
                    state.health = 'warning'
                else:
                    state.health = 'offline'

                fault_parts = []
                if state.relay_fault not in {'None', 'Not energized', 'Relay telemetry unavailable'}:
                    fault_parts.append(state.relay_fault)
                if state.motor_fault not in {'None', 'Motor telemetry unavailable'}:
                    fault_parts.append(state.motor_fault)
                if not fault_parts:
                    if not real_enabled:
                        fault_parts.append('Power path is off')
                    elif not state.attached:
                        fault_parts.append('Motor controller not attached')
                    else:
                        fault_parts.append('None')
                state.fault = ' | '.join(fault_parts)

                if state.attached:
                    motors_online += 1
            else:
                state.attached = real_enabled
                state.temperature = 0.0
                state.position = 0.0
                state.velocity = 0.0
                state.motor_fault = 'N/A'
                relay_health = relay_data.get('health', 'offline' if not real_enabled else 'healthy')
                state.health = relay_health
                state.fault = relay_data.get('fault', 'Headlight control ready')

            self.component_states[name] = state
            self.topology.update_component_state(state)
            if state.health == 'fault':
                fault_count += 1

        bus = payload.get('bus', {})
        battery = float(bus.get('battery_voltage', 0.0))
        total_current = float(bus.get('total_current', 0.0))
        self.summary.update_summary(battery, total_current, motors_online, fault_count)
        self.topology.update_summary(battery, total_current)

        mode = payload.get('meta', {}).get('mode', 'SAFE')
        self.header.mode_badge.set_status(mode, 'healthy' if mode == 'ARMED' else 'warning')

        if self.current_selected and self.current_selected in self.component_states:
            self.detail_panel.render_component(self.component_states[self.current_selected])

    def on_toggle_component(self, name: str, enable: bool) -> None:
        # enable here is the REAL state we want (True = turn relay ON = motor ON)
        command = {
            'type': 'set_power',
            'target': name,
            'enable': enable,
            'timestamp': datetime.now().isoformat(),
        }
        self.log_panel.add_entry(f"Command sent: {'ENABLE' if enable else 'DISABLE'} {humanize_joint_name(name)}")
        if name in self.component_states:
            self.component_states[name].last_command = (
                f"{'ENABLE' if enable else 'DISABLE'} @ {datetime.now().strftime('%H:%M:%S')}"
            )
        if self.ros_node:
            self.ros_node.send_command(command)

    def on_show_details(self, name: str) -> None:
        self.current_selected = name
        if name in self.component_states:
            self.detail_panel.render_component(self.component_states[name])
            self.log_panel.add_entry(f'Detail view opened for {humanize_joint_name(name)}')

    def on_kill_requested(self) -> None:
        confirm = QMessageBox.question(
            self,
            'Confirm software kill',
            'Turn off all motors?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if confirm != QMessageBox.Yes:
            return
        
        # Motors OFF = Relays ON
        command = {
            'type': 'set_power',
            'target': 'all',
            'enable': True,   # True = relays ON = motors OFF
            'timestamp': datetime.now().isoformat(),
        }
        self.log_panel.add_entry('SOFTWARE KILL - turning motors OFF (relays ON)')
        if self.ros_node:
            self.ros_node.send_command(command)
        
        # Update local state
        for state in self.component_states.values():
            state.enabled = False
            state.real_enabled = True
            state.health = 'offline'
            state.fault = 'Power path is off'
            state.last_command = f'MOTORS OFF @ {datetime.now().strftime("%H:%M:%S")}'
            self.topology.update_component_state(state)
        
        self.header.mode_badge.set_status('SAFE', 'warning')

    def closeEvent(self, event) -> None:
        try:
            if self.spin_thread:
                self.spin_thread.stop()
            if self.ros_node:
                self.ros_node.destroy_node()
            if ROS_AVAILABLE and rclpy is not None:
                rclpy.shutdown()
        except Exception:
            pass
        super().closeEvent(event)


def build_fake_payload(t: float) -> Dict:
    battery_voltage = 27.2 + 0.3 * math.sin(t / 9.0)
    payload = {
        'meta': {
            'mode': 'ARMED' if int(t) % 20 > 3 else 'SAFE',
        },
        'bus': {
            'battery_voltage': battery_voltage,
            'total_current': 3.8 + 0.8 * math.sin(t / 4.0),
        },
        'subsystems': {},
        'motors': {},
    }

    for idx, name in enumerate(DEFAULT_MOTORS):
        enabled = (idx + int(t)) % 4 != 0
        payload['subsystems'][name] = {
            'state': 'ON' if enabled else 'OFF',
            'enabled': enabled,
            'channel': idx,
            'voltage': 24.0 if enabled else 0.0,
            'current': 0.6 + 0.15 * math.sin(t / (idx + 2)) if enabled else 0.0,
            'temperature': 30.0 + idx,
            'health': 'healthy' if enabled else 'offline',
            'fault': 'None' if enabled else 'Power path is off',
            'last_command': 'AUTO DEMO',
        }
        payload['motors'][name] = {
            'position': math.sin(t / (idx + 2)) * 2.0,
            'velocity': math.cos(t / (idx + 2)) * 3.0,
            'temperature': 34.0 + idx * 1.8 + 1.5 * math.sin(t / 6.0),
            'attached': enabled,
            'health': 'healthy' if enabled else 'offline',
            'fault': 'None' if enabled else 'Motor not attached',
            'last_update': datetime.now().strftime('%H:%M:%S'),
        }

    for index, name in enumerate(DEFAULT_LIGHTS, start=4):
        enabled = int(t) % (index + 3) > 2
        payload['subsystems'][name] = {
            'state': 'ON' if enabled else 'OFF',
            'enabled': enabled,
            'channel': index,
            'voltage': 12.0 if enabled else 0.0,
            'current': 0.1 if enabled else 0.0,
            'temperature': 0.0,
            'health': 'healthy' if enabled else 'offline',
            'fault': 'None' if enabled else 'Light is off',
            'last_command': 'AUTO DEMO',
        }

    return payload


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName('Rover Dashboard')
    app.setFont(QFont('Arial', 10))
    window = DashboardWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
