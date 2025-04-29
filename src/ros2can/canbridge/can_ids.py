from enum import Enum

class RookCANIDs(Enum):
    ID_1 = 0x100
    ID_2 = 0x101

class MtgrCANIDs(Enum):
    ID_1 = 0x200
    ID_2 = 0x201

class TigrCANIDs(Enum):
    JOINT_PAN = 0x402
    JOINT_SHOULDER = 0x403
    JOINT_ELBOW1 = 0x404
    JOINT_ELBOW2 = 0x405
    JOINT_WRIST = 0x406
    JOINT_GRIPPER = 0x407
    MOTOR_VELOCITY = 0x67C
    TILT_CAM = 0x101

    REPORT_PAN = 0x742
    REPORT_SHOULDER = 0x743
    REPORT_ELBOW1 = 0x744
    REPORT_ELBOW2 = 0x745
    REPORT_WRIST = 0x746
