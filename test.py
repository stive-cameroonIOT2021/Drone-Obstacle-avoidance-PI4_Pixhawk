import time
from pymavlink import mavutil

# Connect to Pixhawk on /dev/serial0 (replace with actual path if different)
pixhawk_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
pixhawk_connection.wait_heartbeat()
print("Connected to Pixhawk.")

def arm_pixhawk():
    """
    Arm the Pixhawk to enable motor control.
    """
    pixhawk_connection.mav.command_long_send(
        pixhawk_connection.target_system, pixhawk_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Sent arm command.")
    time.sleep(2)

def disarm_pixhawk():
    """
    Disarm the Pixhawk to disable motor control.
    """
    pixhawk_connection.mav.command_long_send(
        pixhawk_connection.target_system, pixhawk_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Sent disarm command.")
    time.sleep(2)

def motor_test(motor_number, throttle):
    """
    Spin a specified motor at a given throttle.
    :param motor_number: The motor to test (1-4 for a quadcopter)
    :param throttle: Throttle value between 0 and 1000
    """
    pixhawk_connection.mav.command_long_send(
        pixhawk_connection.target_system, pixhawk_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        motor_number,  # Motor number
        mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,  # Test type
        throttle,  # Throttle level (percent)
        0, 0, 0, 0
    )
    print(f"Testing motor {motor_number} at {throttle}% throttle.")
    time.sleep(3)  # Run for 3 seconds per motor test

def read_rc_channels():
    """
    Read RC transmitter channels.
    """
    try:
        rc_msg = pixhawk_connection.recv_match(type='RC_CHANNELS', blocking=True)
        if rc_msg:
            channels = [rc_msg.chan1_raw, rc_msg.chan2_raw, rc_msg.chan3_raw, rc_msg.chan4_raw,
                        rc_msg.chan5_raw, rc_msg.chan6_raw, rc_msg.chan7_raw, rc_msg.chan8_raw]
            print("RC Channels:", channels)
        else:
            print("No RC data received.")
    except Exception as e:
        print(f"Error retrieving RC data: {e}")

# Main function to run tests
def run_tests():
    print("Arming Pixhawk...")
    arm_pixhawk()

    print("Testing motors...")
    for motor in range(1, 5):  # Assuming a quadcopter with 4 motors
        motor_test(motor, throttle=40)  # Test each motor at 50% throttle

    print("Disarming Pixhawk...")
    disarm_pixhawk()

    print("Reading RC Channels...")
    read_rc_channels()

# Run all tests
if __name__ == "__main__":
    run_tests()
