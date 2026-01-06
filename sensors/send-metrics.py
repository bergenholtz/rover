# mavlink_sender.py
# This script sends real temperature from DHT11 and water voltage from ADS1115 as NAMED_VALUE_FLOAT messages over UDP.

import time
from pymavlink import mavutil
import adafruit_dht
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_ads1x15.ads1x15 import Pin  # Import Pin for channel constants

def send_metrics(connection_string='udpout:192.168.1.82:14551', temp_sensor_name='temp', water_sensor_name='water_v', interval=2.0):
    """
    Connects via MAVLink and sends real temperature and water voltage periodically.

    :param connection_string: MAVLink connection (e.g., 'udpout:localhost:14550').
    :param temp_sensor_name: Sensor name for temperature (<= 10 characters).
    :param water_sensor_name: Sensor name for water voltage (<= 10 characters).
    :param interval: Send interval in seconds (at least 2.0 for DHT11).
    """
    # Initialize DHT11 device (adjust pin if needed, e.g., board.D4 for physical pin 7)
    dht_device = adafruit_dht.DHT11(board.D4)

    # Initialize ADS1115 for water voltage
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    chan = AnalogIn(ads, Pin.A0)  # A0 pin (use Pin.A1 for A1, etc.)

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

        # Read real temperature from DHT11
        try:
            temp_value = dht_device.temperature
            print(f"Read temperature: {temp_value} C")
        except RuntimeError as error:
            # Errors like checksum failures or timeouts are common with DHT11; use last known or fallback
            print(f"Error reading DHT11: {error.args[0]}")
            temp_value = float('nan')  # Or use a last_known_value if you track it
        except Exception as e:
            print(f"Unexpected error reading DHT11: {e}")
            temp_value = float('nan')

        # Read water voltage from ADS1115
        try:
            water_value = chan.voltage
            print(f"Read water voltage: {water_value} V")
        except Exception as e:
            print(f"Error reading ADS1115: {e}")
            water_value = float('nan')

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
    send_metrics()
