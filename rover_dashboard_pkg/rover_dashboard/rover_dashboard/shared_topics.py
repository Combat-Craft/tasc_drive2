TOPIC_GUI_TELEMETRY = '/rover/gui/telemetry'
TOPIC_GUI_COMMAND = '/rover/gui/command'
TOPIC_GUI_HEARTBEAT = '/rover/gui/heartbeat'

TOPIC_RELAY_COMMAND = '/rover/relay_board/command'
TOPIC_RELAY_STATE = '/rover/relay_board/state'
TOPIC_RELAY_HEARTBEAT = '/rover/relay_board/heartbeat'

TOPIC_MOTOR_TELEMETRY = '/rover/drive/motor_telemetry'
TOPIC_MOTOR_POSITIONS = '/rover/drive/motor_positions'

DEFAULT_MOTORS = [
    'front_left_wheel_joint',
    'rear_left_wheel_joint',
    'middle_left_wheel_joint',
    'front_right_wheel_joint',
    'middle_right_wheel_joint',
    'rear_right_wheel_joint',
]
DEFAULT_LIGHTS = [
    'Left Headlight',
    'Right Headlight',
]
DEFAULT_SUBSYSTEMS = DEFAULT_MOTORS.copy() + DEFAULT_LIGHTS.copy()
