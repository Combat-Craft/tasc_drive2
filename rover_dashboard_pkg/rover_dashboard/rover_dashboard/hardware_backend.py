import json
import sys
from datetime import datetime
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from .shared_topics import (
        DEFAULT_LIGHTS,
        DEFAULT_MOTORS,
        DEFAULT_SUBSYSTEMS,
        TOPIC_GUI_COMMAND,
        TOPIC_GUI_HEARTBEAT,
        TOPIC_GUI_TELEMETRY,
        TOPIC_MOTOR_TELEMETRY,
        TOPIC_RELAY_COMMAND,
        TOPIC_RELAY_HEARTBEAT,
        TOPIC_RELAY_STATE,
    )
except ImportError:
    from shared_topics import (  # type: ignore
        DEFAULT_LIGHTS,
        DEFAULT_MOTORS,
        DEFAULT_SUBSYSTEMS,
        TOPIC_GUI_COMMAND,
        TOPIC_GUI_HEARTBEAT,
        TOPIC_GUI_TELEMETRY,
        TOPIC_MOTOR_TELEMETRY,
        TOPIC_RELAY_COMMAND,
        TOPIC_RELAY_HEARTBEAT,
        TOPIC_RELAY_STATE,
    )


def now_string() -> str:
    return datetime.now().strftime('%H:%M:%S')


class HardwareBackendNode(Node):
    def __init__(self) -> None:
        super().__init__('rover_dashboard_backend')
        self.telemetry_pub = self.create_publisher(String, TOPIC_GUI_TELEMETRY, 10)
        self.heartbeat_pub = self.create_publisher(String, TOPIC_GUI_HEARTBEAT, 10)
        self.relay_command_pub = self.create_publisher(String, TOPIC_RELAY_COMMAND, 10)

        self.command_sub = self.create_subscription(String, TOPIC_GUI_COMMAND, self.command_cb, 10)
        self.relay_state_sub = self.create_subscription(String, TOPIC_RELAY_STATE, self.relay_state_cb, 10)
        self.relay_heartbeat_sub = self.create_subscription(String, TOPIC_RELAY_HEARTBEAT, self.relay_heartbeat_cb, 10)
        self.motor_telemetry_sub = self.create_subscription(String, TOPIC_MOTOR_TELEMETRY, self.motor_telemetry_cb, 10)

        self.timer = self.create_timer(0.25, self.publish_payload)

        self.kill_active = False
        self.last_relay_heartbeat = '--'
        self.last_motor_update = '--'
        self.bus_voltage = 0.0
        self.subsystems = self._build_default_subsystems(DEFAULT_SUBSYSTEMS)
        self.motors = self._build_default_motors(DEFAULT_MOTORS)

    def _build_default_subsystems(self, names: List[str]) -> Dict[str, Dict]:
        return {
            name: {
                'state': 'OFF',
                'enabled': False,
                'channel': index,
                'voltage': 0.0,
                'current': 0.0,
                'temperature': 0.0,
                'health': 'offline',
                'fault': 'Waiting for relay controller',
                'last_command': 'INIT',
            }
            for index, name in enumerate(names)
        }

    def _build_default_motors(self, names: List[str]) -> Dict[str, Dict]:
        return {
            name: {
                'position': 0.0,
                'velocity': 0.0,
                'temperature': 0.0,
                'attached': False,
                'health': 'offline',
                'fault': 'Waiting for motor telemetry',
                'last_update': '--',
            }
            for name in names
        }

    def relay_state_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Received malformed relay state JSON')
            return

        relays = payload.get('relays', [])
        self.bus_voltage = float(payload.get('bus_voltage', self.bus_voltage))
        if not relays:
            return

        updated = {}
        for relay in relays:
            if not relay.get('configured', True):
                continue
            name = relay.get('name') or f"Relay {relay.get('channel', '?')}"
            updated[name] = {
                'state': 'ON' if relay.get('enabled', False) else 'OFF',
                'enabled': bool(relay.get('enabled', False)),
                'channel': int(relay.get('channel', 0)),
                'voltage': float(relay.get('voltage', 0.0)),
                'current': float(relay.get('current', 0.0)),
                'temperature': float(relay.get('temperature', 0.0)),
                'health': relay.get('health', 'healthy' if relay.get('enabled', False) else 'offline'),
                'fault': relay.get('fault', 'None'),
                'last_command': relay.get('last_command', 'NONE'),
            }

        if updated:
            self.subsystems = updated

    def relay_heartbeat_cb(self, msg: String) -> None:
        self.last_relay_heartbeat = msg.data or now_string()

    def motor_telemetry_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Received malformed motor telemetry JSON')
            return

        motors = {}
        for motor in payload.get('motors', []):
            name = motor.get('name')
            if not name:
                continue
            attached = bool(motor.get('attached', False))
            motors[name] = {
                'position': float(motor.get('position', 0.0)),
                'velocity': float(motor.get('velocity', 0.0)),
                'temperature': float(motor.get('temperature', 0.0)),
                'attached': attached,
                'health': motor.get('health', 'healthy' if attached else 'offline'),
                'fault': motor.get('fault', 'None' if attached else 'No attachment'),
                'last_update': motor.get('last_update', now_string()),
            }

        if motors:
            self.motors.update(motors)
            self.last_motor_update = now_string()

    def command_cb(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Received malformed command JSON')
            return

        cmd_type = command.get('type')
        if cmd_type == 'set_power':
            target = command.get('target')
            if target in self.subsystems:
                self.subsystems[target]['last_command'] = (
                    f"{'ENABLE' if command.get('enable', False) else 'DISABLE'} @ {now_string()}"
                )
            self.relay_command_pub.publish(msg)
            return

        if cmd_type == 'turn_on_all':
            self.kill_active = False  # Clear kill state
            # Create individual commands for all subsystems
            for subsystem_name in self.subsystems.keys():
                enable_command = {
                    'type': 'set_power',
                    'target': subsystem_name,
                    'enable': True,
                    'timestamp': command.get('timestamp', datetime.now().isoformat()),
                }
                relay_msg = String()
                relay_msg.data = json.dumps(enable_command)
                self.relay_command_pub.publish(relay_msg)
                # Update local state
                self.subsystems[subsystem_name]['enabled'] = True
                self.subsystems[subsystem_name]['state'] = 'ON'
                self.subsystems[subsystem_name]['health'] = 'healthy'
                self.subsystems[subsystem_name]['fault'] = 'None'
                self.subsystems[subsystem_name]['last_command'] = f'ENABLE ALL @ {now_string()}'
            self.get_logger().info('Turn On All command executed')

        if cmd_type == 'software_kill':
            self.kill_active = True
            kill_command = {
                'type': 'software_kill',
                'timestamp': command.get('timestamp', datetime.now().isoformat()),
            }
            relay_msg = String()
            relay_msg.data = json.dumps(kill_command)
            self.relay_command_pub.publish(relay_msg)
            for subsystem in self.subsystems.values():
                subsystem['enabled'] = False
                subsystem['state'] = 'OFF'
                subsystem['health'] = 'offline'
                subsystem['fault'] = 'Software kill active'
                subsystem['last_command'] = f'KILLED @ {now_string()}'

    def publish_payload(self) -> None:
        total_current = sum(float(item.get('current', 0.0)) for item in self.subsystems.values())
        fault_count = sum(
            1
            for item in list(self.subsystems.values()) + list(self.motors.values())
            if item.get('health') == 'fault'
        )

        payload = {
            'meta': {
                'node_count': len(self.subsystems) + len(self.motors) + 3,
                'mode': 'SAFE' if self.kill_active else 'ARMED',
                'fault_count': fault_count,
                'relay_heartbeat': self.last_relay_heartbeat,
                'motor_heartbeat': self.last_motor_update,
            },
            'bus': {
                'battery_voltage': max(
                    [self.bus_voltage] +
                    [float(item.get('voltage', 0.0)) for item in self.subsystems.values()]
                ),
                'total_current': total_current,
            },
            'rails': {
                '24V': max(
                    [
                        float(item.get('voltage', 0.0))
                        for item in self.subsystems.values()
                        if float(item.get('voltage', 0.0)) >= 18.0
                    ]
                    or [0.0]
                ),
                '12V': max(
                    [
                        float(item.get('voltage', 0.0))
                        for item in self.subsystems.values()
                        if 8.0 <= float(item.get('voltage', 0.0)) < 18.0
                    ]
                    or [0.0]
                ),
                '5V': max(
                    [
                        float(item.get('voltage', 0.0))
                        for item in self.subsystems.values()
                        if float(item.get('voltage', 0.0)) < 8.0
                    ]
                    or [0.0]
                ),
            },
            'subsystems': self.subsystems,
            'motors': self.motors,
        }

        telemetry = String()
        telemetry.data = json.dumps(payload)
        self.telemetry_pub.publish(telemetry)

        heartbeat = String()
        heartbeat.data = now_string()
        self.heartbeat_pub.publish(heartbeat)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareBackendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)

