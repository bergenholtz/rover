import time
from pymavlink import mavutil

# Connect to MAVProxy's local UDP output
mav = mavutil.mavlink_connection('udp:192.1681.1.104:14571')

# Wait for heartbeat to get vehicle sysid/compid
mav.wait_heartbeat()
print(f"Connected to system {mav.target_system}, component {mav.target_component}")

# Record start time for relative timestamp
start_time = time.time()

# Example sensor reading functions (replace with your actual code)
def read_temp_sensor():
    # Placeholder: e.g., read from DS18B20 or similar
    return 25.5  # Example value

def read_pressure_sensor():
    # Placeholder: e.g., read from BMP280
    return 1013.2  # Example value

def read_humidity_sensor():
    # Placeholder: e.g., read from DHT22
    return 60.0  # Example value

# Loop to send data every 1 second
while True:
    temp = read_temp_sensor()
    pressure = read_pressure_sensor()
    humidity = read_humidity_sensor()

    # Calculate relative time_boot_ms (milliseconds since script start)
    time_boot_ms = int((time.time() - start_time) * 1000)

    # Send each as a separate named value
    mav.mav.named_value_float_send(
        time_boot_ms,            # time_boot_ms
        b'Temp',                 # name
        temp                     # value
    )
    mav.mav.named_value_float_send(
        time_boot_ms,
        b'Pressure',
        pressure
    )
    mav.mav.named_value_float_send(
        time_boot_ms,
        b'Humidity',
        humidity
    )
    print(f"Sent Temp: {temp} at {time_boot_ms} ms")
    time.sleep(1)  # Adjust rate as needed (e.g., 0.5 for 2 Hz)
                                                                  
