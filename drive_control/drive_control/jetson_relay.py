#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import time
import threading

# GPIO Pin mapping
RELAY_PINS = {
    'front_left_wheel_joint': 32,
    'middle_left_wheel_joint': 33,
    'rear_left_wheel_joint': 31,
    'front_right_wheel_joint': 29,
    'middle_right_wheel_joint': 15,
    'rear_right_wheel_joint': 7,
}

# Which relays are actually configured
CONFIGURED_RELAYS = [
    'front_left_wheel_joint',
    'middle_left_wheel_joint',
    'rear_left_wheel_joint', 
    'front_right_wheel_joint',
    'middle_right_wheel_joint',
    'rear_right_wheel_joint',
]

class JetsonRelayNode(Node):
    def __init__(self):
        super().__init__('jetson_relay')
        
        # GPIO Setup
        GPIO.setmode(GPIO.BOARD)
        self.relay_states = {}
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Forcing all relays OFF on startup...")
        self.get_logger().info("=" * 60)
        
        # Step 1: Setup all pins as output
        for name, pin in RELAY_PINS.items():
            GPIO.setup(pin, GPIO.OUT)
            self.get_logger().info(f"Setup {name} on pin {pin}")
        
        # Step 2: Force ALL pins to HIGH (OFF) - for active-LOW relays
        # GPIO.HIGH = relay OFF for most relay modules
        for name, pin in RELAY_PINS.items():
            GPIO.output(pin, GPIO.HIGH)  # HIGH = OFF for active-LOW relays
            self.relay_states[name] = False
            self.get_logger().info(f"FORCED OFF: {name} on pin {pin} (GPIO HIGH)")
        
        # Step 3: Extra delay and second force
        time.sleep(0.1)
        for name, pin in RELAY_PINS.items():
            GPIO.output(pin, GPIO.HIGH)  # Keep them OFF
        
        # Step 4: Verify they are actually OFF
        time.sleep(0.1)
        for name, pin in RELAY_PINS.items():
            current_state = GPIO.input(pin)
            # For active-LOW: HIGH = OFF, LOW = ON
            is_off = (current_state == GPIO.HIGH)
            self.get_logger().info(f"Verified {name} on pin {pin} is {'OFF' if is_off else 'ON!!!'}")
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("All relays forced OFF at startup!")
        self.get_logger().info("=" * 60)
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/rover/relay_board/state', 10)
        self.heartbeat_pub = self.create_publisher(String, '/rover/relay_board/heartbeat', 10)
        
        # Subscriber
        self.command_sub = self.create_subscription(
            String, 
            '/rover/relay_board/command',
            self.command_callback, 
            10
        )
        
        # Timer for state publishing (every 2 seconds)
        self.timer = self.create_timer(2.0, self.publish_state)
        
        # Heartbeat timer (1 Hz)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        
        self.get_logger().info("Jetson Relay Node Started - Ready to receive commands!")
    
    def set_relay(self, name: str, enabled: bool):
        if name not in RELAY_PINS:
            self.get_logger().warn(f"Unknown relay: {name}")
            return False
        
        pin = RELAY_PINS[name]
        # For active-LOW relays: HIGH = OFF, LOW = ON
        if enabled:
            GPIO.output(pin, GPIO.LOW)   # LOW turns relay ON
            self.get_logger().info(f"Relay {name} -> ON (GPIO LOW)")
        else:
            GPIO.output(pin, GPIO.HIGH)  # HIGH turns relay OFF
            self.get_logger().info(f"Relay {name} -> OFF (GPIO HIGH)")
        
        self.relay_states[name] = enabled
        return True
    
    def command_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            cmd_type = cmd.get('type')
            
            if cmd_type == 'software_kill':
                # KILL = motors OFF = relays ON = GPIO LOW
                for name in CONFIGURED_RELAYS:
                    self.set_relay(name, True)   # True = relay ON = motors OFF
                self.get_logger().warn("SOFTWARE KILL - Motors OFF, Relay lights ON")
                
            elif cmd_type == 'turn_on_all':
                # TURN ON ALL = motors ON = relays OFF = GPIO HIGH
                for name in CONFIGURED_RELAYS:
                    self.set_relay(name, False)  # False = relay OFF = motors ON
                self.get_logger().warn("TURN ON ALL - Motors ON, Relay lights OFF")
                
            elif cmd_type == 'set_power':
                target = cmd.get('target')
                enable = cmd.get('enable', False)
                if target == 'all':
                    for name in CONFIGURED_RELAYS:
                        self.set_relay(name, enable)
                elif target in RELAY_PINS:
                    self.set_relay(target, enable)
                else:
                    self.get_logger().warn(f"Unknown target: {target}")
                    
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON command: {e}")
    
    def publish_state(self):
        relays_list = []
        for i, name in enumerate(CONFIGURED_RELAYS):
            enabled = self.relay_states.get(name, False)
            relays_list.append({
                "name": name,
                "channel": i,
                "configured": True,
                "enabled": enabled,
                "voltage": 24.0 if enabled else 0.0,
                "current": 0.02 if enabled else 0.0,
                "temperature": 25.0,
                "health": "healthy" if enabled else "offline",
                "fault": "None" if enabled else "Not energized",
                "last_command": "GPIO_DIRECT"
            })
        
        state_msg = {
            "bus_voltage": 12.5,
            "relays": relays_list
        }
        
        msg = String()
        msg.data = json.dumps(state_msg)
        self.state_pub.publish(msg)
    
    def publish_heartbeat(self):
        msg = String()
        msg.data = str(int(time.time()))
        self.heartbeat_pub.publish(msg)
    
    def __del__(self):
        self.get_logger().info("Cleaning up GPIO...")
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = JetsonRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()