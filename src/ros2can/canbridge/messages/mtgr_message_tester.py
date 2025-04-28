from ros2can.canbridge.messages.mtgr_message import MtgrMessage, MtgrMessageIDs
from ros2can.canbridge.messages.mtgr_message import MtgrMessage, FLIPPER_CW, FLIPPER_CCW, FLIPPER_NONE
import can
import time

def test_mtgr_drive_msg():
    # Create an instance of MtgrMessage
    mtgr_message = MtgrMessage()

    # Test input: Set velocities for left and right motors
    left_velocity = 500  # Example velocity for the left motor
    right_velocity = -500  # Example velocity for the right motor
    bus = can.interface.Bus(channel='can0', bustype='socketcan')
    # Generate the velocity messages
    msg_left, msg_right = mtgr_message.set_velocity_message(left_velocity, right_velocity)
    can_msg_left = can.Message(arbitration_id=msg_left[0], data=msg_left[1], is_extended_id=False)
    can_msg_right = can.Message(arbitration_id=msg_right[0], data=msg_right[1], is_extended_id=False)

    bus.send(can_msg_left)
    bus.send(can_msg_right)

    # Print the results
    print("MTGR Velocity Message Test")
    print("--------------------------")
    print(f"Left Motor Message ID: {hex(msg_left[0])}")
    print(f"Left Motor Message Data: {bytes(msg_left[1])}")
    print(f"Right Motor Message: {msg_right}")



def test_flipper_rotation():
    # Create an instance of MtgrMessage
    mtgr_message = MtgrMessage()
    bus = can.interface.Bus(channel='can0', bustype='socketcan')
    print("Testing Flipper Rotation Messages")
    print("---------------------------------")

    # # Test 1: Synchronized flipper movement (both CW)
    # msg_id, msg_data = mtgr_message.set_flipper_rotation_message(left_move=FLIPPER_CW, is_sync=True)
    # print(f"Test 1 - Synchronized CW: ID={hex(msg_id)}, Data={msg_data}")
    # can_msg_left = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # can_msg_right = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # bus.send(can_msg_left)
    # bus.send(can_msg_right)
    # time.sleep(3)
   
   
    # Test 2: Synchronized flipper movement (no movement)
    # msg_id, msg_data = mtgr_message.set_flipper_rotation_message(left_move=FLIPPER_NONE, is_sync=True)
    # print(f"Test 2 - Synchronized None: ID={hex(msg_id)}, Data={msg_data}")
    # can_msg_left = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # can_msg_right = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # bus.send(can_msg_left)
    # bus.send(can_msg_right)
    # time.sleep(3)
   
   
    # Test 3: Independent flipper movement (left CW, right CW)
    # msg_id, msg_data = mtgr_message.set_flipper_rotation_message(left_move=FLIPPER_CW, right_move=FLIPPER_CW, is_sync=False)
    # print(f"Test 3 - Independent (Left CW, Right CCW): ID={hex(msg_id)}, Data={msg_data}")
    # can_msg_left = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # can_msg_right = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # bus.send(can_msg_left)
    # bus.send(can_msg_right)
    # time.sleep(3)
   
   
    # # Test 4: Independent flipper movement (left CCW, right CW)
    # msg_id, msg_data = mtgr_message.set_flipper_rotation_message(left_move=FLIPPER_CCW, right_move=FLIPPER_CCW, is_sync=False)
    # print(f"Test 4 - Independent (Left CCW, Right CCW): ID={hex(msg_id)}, Data={msg_data}")
    # can_msg_left = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # can_msg_right = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # bus.send(can_msg_left)
    # bus.send(can_msg_right)
    # time.sleep(3)
   
   
    # Test 5: Independent flipper movement (left CW, right None)
    # msg_id, msg_data = mtgr_message.set_flipper_rotation_message(left_move=FLIPPER_CW, right_move=FLIPPER_NONE, is_sync=False)
    # print(f"Test 5 - Independent (Left CW, Right None): ID={hex(msg_id)}, Data={msg_data}")
    # can_msg_left = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # can_msg_right = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # bus.send(can_msg_left)
    # bus.send(can_msg_right)
    # time.sleep(3)
   
   
    # # Test 6: Independent flipper movement (left None, right CCW)
    # msg_id, msg_data = mtgr_message.set_flipper_rotation_message(left_move=FLIPPER_NONE, right_move=FLIPPER_CCW, is_sync=False)
    # print(f"Test 6 - Independent (Left None, Right CCW): ID={hex(msg_id)}, Data={msg_data}")
    # can_msg_left = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # can_msg_right = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    # bus.send(can_msg_left)
    # bus.send(can_msg_right)
    # time.sleep(3)
   
   
    # # Test 7: No movement for both flippers
    msg_id, msg_data = mtgr_message.set_flipper_rotation_message(left_move=FLIPPER_NONE, right_move=FLIPPER_NONE, is_sync=False)
    print(f"Test 7 - No Movement: ID={hex(msg_id)}, Data={msg_data}")
    can_msg_left = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    can_msg_right = can.Message(arbitration_id=msg_id, data=msg_data, is_extended_id=False)
    bus.send(can_msg_left)
    bus.send(can_msg_right)
    # time.sleep(3)


def main():
    # test_mtgr_drive_msg()
    test_flipper_rotation()

# Run the test
if __name__ == "__main__":
    main()
