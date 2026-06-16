# shared_topics.py
# Motor definitions - Updated for 6 wheels
DEFAULT_MOTORS = [
    'front_left_wheel_joint',
    'middle_left_wheel_joint',    # ADDED
    'rear_left_wheel_joint',
    'front_right_wheel_joint',
    'middle_right_wheel_joint',   # ADDED
    'rear_right_wheel_joint',
]

DEFAULT_LIGHTS = [
    'left_headlight',
    'right_headlight',
]

DEFAULT_SUBSYSTEMS = DEFAULT_MOTORS + DEFAULT_LIGHTS

# Topic names
TOPIC_GUI_COMMAND = '/rover/gui/command'
TOPIC_GUI_HEARTBEAT = '/rover/gui/heartbeat'
TOPIC_GUI_TELEMETRY = '/rover/gui/telemetry'

TOPIC_RELAY_COMMAND = '/rover/relay_board/command'
TOPIC_RELAY_STATE = '/rover/relay_board/state'
TOPIC_RELAY_HEARTBEAT = '/rover/relay_board/heartbeat'

TOPIC_MOTOR_TELEMETRY = '/rover/drive/motor_telemetry'   