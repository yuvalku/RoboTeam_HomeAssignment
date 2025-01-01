
def parse_frame(robot_type, can_id, data):
    """
    Returns a dictionary of parsed fields, depending on the robot config.
    The calling node can map them to RobotStatus or otherwise.
    """
    parsed = {}

    match robot_type:
        case "ROOK":
            match can_id:
                case 'REPORT_MOTOR_LEFT_RPM' if len(data) >= 5:
                    rpm = int.from_bytes(data[1:3], 'little', signed=True)
                    load = int.from_bytes(data[3:5], 'little', signed=True)
                    parsed['left_rpm'] = rpm
                    parsed['left_load'] = load

                case 'REPORT_MOTOR_RIGHT_RPM'  if len(data) >= 5:
                    rpm = int.from_bytes(data[1:3], 'little', signed=True)
                    load = int.from_bytes(data[3:5], 'little', signed=True)
                    parsed['right_rpm'] = rpm
                    parsed['right_load'] = load    

                case 'REPORT_IMU' if len(data) >= 7:  # IMU
                    roll  = int.from_bytes(data[1:3], 'little', signed=False)
                    pitch = int.from_bytes(data[3:5], 'little', signed=False)
                    yaw   = int.from_bytes(data[5:7], 'little', signed=False)
                    parsed['roll_deg']  = roll
                    parsed['pitch_deg'] = pitch
                    parsed['yaw_deg']   = yaw

                case 'REPORT_BATTERY' if len(data) >= 7:
                    battery_id = data[1]
                    current = int.from_bytes(data[2:4], byteorder='little', signed=True)
                    voltage = int.from_bytes(data[4:6], byteorder='little', signed=True)
                    charge = data[6]
                    parsed['battery_id'] = battery_id
                    parsed['current'] = current
                    parsed['voltage'] = voltage
                    parsed['charge'] = charge

        case "PROBOT":
            match can_id:
                case 'REPORT_MOTOR_LEFT_BIT' if len(data) >=2: # Left BIT
                    bit_type = data[0]
                    bit_data = data[1]
                    parsed['left_bit_error'] = bit_data

                case 'REPORT_MOTOR_RIGHT_BIT' if len(data) >= 2:  # Right BIT
                    bit_type = data[0]
                    bit_data = data[1]
                    parsed['right_bit_error'] = bit_data

                # Odometer Data (Left: 0x706, Right: 0x707)
                case 'REPORT_MOTOR_LEFT_ODOM' if len(data) >= 5:
                    odom = int.from_bytes(data[1:5], 'little', signed=False)
                    parsed['left_odom'] = odom

                case 'REPORT_MOTOR_RIGHT_ODOM' if len(data) >= 5:
                    odom = int.from_bytes(data[1:5], 'little', signed=False)
                    parsed['right_odom'] = odom

                # RPM Data (Left: 0x740, Right: 0x741)
                case 'REPORT_MOTOR_LEFT_RPM' if len(data) >= 5:
                    rpm = int.from_bytes(data[1:3], 'little', signed=True)
                    current = int.from_bytes(data[3:5], 'little', signed=False)
                    parsed['left_rpm'] = rpm
                    parsed['left_current'] = current

                case 'REPORT_MOTOR_RIGHT_RPM' if len(data) >= 5:
                    rpm = int.from_bytes(data[1:3], 'little', signed=True)
                    current = int.from_bytes(data[3:5], 'little', signed=False)
                    parsed['right_rpm'] = rpm
                    parsed['right_current'] = current

                # Battery Status (0x708), Motor Temp (0x709/0x70A), etc.
                case 'REPORT_BATTERY_STATUS' if len(data) >= 7:
                    # battery data for Probot
                    pass

                case 'REPORT_MOTOR_LEFT_TEMP' if len(data) >= 4:
                    temp = int.from_bytes(data[1:3], 'little', signed=True)
                    heat = data[3]
                    parsed['left_temp'] = temp
                    parsed['left_heat'] = heat

                case 'REPORT_MOTOR_RIGHT_TEMP' if len(data) >= 4:
                    temp = int.from_bytes(data[1:3], 'little', signed=True)
                    heat = data[3]
                    parsed['right_temp'] = temp
                    parsed['right_heat'] = heat

                case 'REPORT_BATTERY_BIT' if len(data) >= 2:  # Battery BIT
                    bit_type = data[0]
                    bit_data = data[1]
                    parsed['battery_bit_error'] = bit_data

    return parsed