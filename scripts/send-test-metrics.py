# mavlink_sender.py
# This script sends random test temperature and water voltage as NAMED_VALUE_FLOAT messages over UDP.

import time
from pymavlink import mavutil
import random
import sys

def send_metrics(connection_string='udpout:192.168.1.82:14571', temp_sensor_name='temp', water_sensor_name='water_v', interval=2.0):
    """
    Connects via MAVLink and sends random temperature and water voltage periodically.

    :param connection_string: MAVLink connection (e.g., 'udpout:localhost:14550').
    :param temp_sensor_name: Sensor name for temperature (<= 10 characters).
    :param water_sensor_name: Sensor name for water voltage (<= 10 characters).
    :param interval: Send interval in seconds.
    """
    master = mavutil.mavlink_connection(connection_string, source_system=1, source_component=1)

    print(f"Sender connected to {connection_string}")

    # Send initial heartbeat to establish connection
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GENERIC,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,  # base_mode
        0,  # custom_mode
        0   # system_status
    )
    print("Sent initial heartbeat")

    last_heartbeat = time.time()
    while True:
        now = time.time()

        # Send heartbeat every 1 second for keepalive/debug
        if now - last_heartbeat >= 1.0:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,  # base_mode
                0,  # custom_mode
                0   # system_status
            )
            last_heartbeat = now

        # Generate random temperature (e.g., between 20.0 and 30.0 C)
        temp_value = random.uniform(20.0, 30.0)
        print(f"Generated temperature: {temp_value} C")

        # Generate random water voltage (e.g., between 0.0 and 5.0 V)
        water_value = random.uniform(0.0, 5.0)
        print(f"Generated water voltage: {water_value} V")

        # Send the temperature value
        time_boot_ms = int(now * 1000) % (2**32)
        master.mav.named_value_float_send(
            time_boot_ms,
            temp_sensor_name.encode('utf-8'),
            temp_value
        )
        print(f"Sent {temp_sensor_name}: {temp_value} at time {time_boot_ms}")

        # Send the water voltage value (reuse time_boot_ms for simplicity, or recalculate if needed)
        master.mav.named_value_float_send(
            time_boot_ms,
            water_sensor_name.encode('utf-8'),
            water_value
        )
        print(f"Sent {water_sensor_name}: {water_value} at time {time_boot_ms}")

        # Check for incoming messages (for debug)
        try:
            msg = master.recv_match(blocking=False)
            if msg:
                print(f"Sender received: {msg.get_type()} - {msg}")
        except Exception as e:
            print(f"Sender recv error: {e}")

        time.sleep(interval)

if __name__ == "__main__":
    connection_string = 'udpout:192.168.1.82:14571'
    if len(sys.argv) > 1:
        if len(sys.argv) == 2:
            connection_string = sys.argv[1]
        elif len(sys.argv) == 3:
            arg1 = sys.argv[1]
            arg2 = sys.argv[2]
            if '=' in arg1 and '=' in arg2:
                # Parse named arguments like host=192.168.1.104 port=14555
                if arg1.startswith('host='):
                    host = arg1.split('=')[1]
                else:
                    host = '192.168.1.82'  # fallback
                if arg2.startswith('port='):
                    port = arg2.split('=')[1]
                else:
                    port = '14571'  # fallback
            else:
                # Positional arguments: host port
                host = arg1
                port = arg2
            connection_string = f'udpout:{host}:{port}'
    send_metrics(connection_string=connection_string)
