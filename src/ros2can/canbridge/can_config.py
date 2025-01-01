
ROBOT_CONF = {
    "ROOK" : {
        "CMD_MOTOR_LEFT": 0x203,
        "CMD_MOTOR_RIGHT": 0x204,

        "REPORT_MOTOR_LEFT_RPM": 0x303,
        "REPORT_MOTOR_RIGHT_RPM": 0x304,
        "REPORT_IMU": 0x302,
        "REPORT_BATTERY": 0x309
    },

    "PROBOT" : {
        'CMD_MOTOR_LEFT': 0x626,
        'CMD_MOTOR_RIGHT': 0x627,

        'REPORT_MOTOR_LEFT_BIT': 0x73C,
        'REPORT_MOTOR_RIGHT_BIT': 0x73D,
        'REPORT_MOTOR_LEFT_ODOM': 0x706,
        'REPORT_MOTOR_RIGHT_ODOM': 0x707,
        'REPORT_MOTOR_LEFT_RPM': 0x740,
        'REPORT_MOTOR_RIGHT_RPM': 0x741,
        'REPORT_BATTERY_STATUS': 0x708,
        'REPORT_MOTOR_LEFT_TEMP': 0x709,
        'REPORT_MOTOR_RIGHT_TEMP': 0x70A,
        'REPORT_BATTERY_BIT': 0x70E,
    }
}

def get_robot_conf(robot_name: str):
    return ROBOT_CONF.get(robot_name, {})