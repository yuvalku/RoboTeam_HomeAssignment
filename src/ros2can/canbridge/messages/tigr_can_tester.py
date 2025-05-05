from tiger_message import TigrMessage, TigrManipulatorMessageIDs
import can
import time

def test_tigr_drive_msg():
    tigr_message = TigrMessage()

    bus = can.interface.Bus(channel='can0', bustype='socketcan')
    left_velocity = 500  # Example velocity for the left motor
    right_velocity = -500  # Example velocity for the right motor

    msg_left, msg_right = tigr_message.set_velocity_message(left_velocity, right_velocity)
    can_msg_left = can.Message(arbitration_id=msg_left[0], data=msg_left[1], is_extended_id=False)  
    can_msg_right = can.Message(arbitration_id=msg_right[0], data=msg_right[1], is_extended_id=False)
    while True:
        bus.send(can_msg_left)
        bus.send(can_msg_right)
        time.sleep(0.066)

def test_tigr_tilt_camera_msg():
    tigr_message = TigrMessage()
    bus = can.interface.Bus(channel='can0', bustype='socketcan')

    direction = 1  # Start by moving up
    cycle_start_time = time.time()

    while True:
        # Generate a new message every time
        msg_id, msg_data = tigr_message.set_tilt_camera_message(direction)
        
        # Create and send CAN message
        can_msg = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
        bus.send(can_msg)

        # Print for debugging
        print(f"Sent tilt angle: {tigr_message.current_camera_tilt_angle:.1f} degrees")

        # Every 2 seconds, change direction
        if time.time() - cycle_start_time >= 2.0:
            direction = 1 if direction == -1 else -1
            cycle_start_time = time.time()

        time.sleep(0.1)


def test_tigr_joint_speed_msg():
    tigr_message = TigrMessage()
    bus = can.interface.Bus(channel='can0', bustype='socketcan')

    # Example: Set all joints speed to 1000
    joints = [
        TigrManipulatorMessageIDs.JOINT_PAN,
        TigrManipulatorMessageIDs.JOINT_SHOULDER,
        TigrManipulatorMessageIDs.JOINT_ELBOW1,
        TigrManipulatorMessageIDs.JOINT_ELBOW2,
        TigrManipulatorMessageIDs.JOINT_WRIST,
    ]

    while True:
        msg_id, msg_data = tigr_message.set_joint_speed_message(TigrManipulatorMessageIDs.JOINT_PAN, speed=-200)
        can_msg = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
        bus.send(can_msg)
        print(f"Sent speed command to {TigrManipulatorMessageIDs.JOINT_PAN.name}")

        time.sleep(0.01)  # Send every second

def test_tigr_joint_abs_position_msg():
    tigr_message = TigrMessage()
    bus = can.interface.Bus(channel='can0', bustype='socketcan')
    position = 0  # 30 degrees (scaled x10)
    direction = 0

    msg_id, msg_data = tigr_message.set_joint_abs_position_message(TigrManipulatorMessageIDs.JOINT_SHOULDER, position, direction)
    can_msg = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    bus.send(can_msg)
    print(f"Sent position command to {TigrManipulatorMessageIDs.JOINT_SHOULDER.name}")
    while True:

        continue


if __name__ == "__main__":
    #test_tigr_drive_msg()
    #test_tigr_tilt_camera_msg()
    test_tigr_joint_speed_msg()
    #test_tigr_joint_abs_position_msg()