# mavlink_receiver.py
# This script listens for MAVLink messages over UDP and displays name/value pairs from NAMED_VALUE_FLOAT messages.

from pymavlink import mavutil
import sys

def receive_metrics(connection_string='udpin:0.0.0.0:14550'):
    """
    Listens for MAVLink messages and prints name/value pairs.

    :param connection_string: MAVLink connection (e.g., 'udpin:0.0.0.0:14550' to listen on port 14550).
    """
    master = mavutil.mavlink_connection(connection_string, source_system=2, source_component=2)

    print(f"Receiver connected to {connection_string}")

    while True:
        try:
            msg = master.recv_match(blocking=True, timeout=5.0)  # Timeout to prevent indefinite block
            if msg:
                msg_type = msg.get_type()
                #print(f"Received message: {msg_type} - {msg}")

                if msg_type == 'NAMED_VALUE_FLOAT':
                    name = msg.name.rstrip('\0')  # Remove null padding (already a str)
                    value = msg.value
                    sys.stdout.write(f"\rProcessed: {name} = {value}    ")  # Overwrite line, add spaces to clear previous content
                    sys.stdout.flush()
                elif msg_type == 'HEARTBEAT':
                    #print()
                    #print("Processed: Heartbeat received")
                    pass  # Suppress output to avoid new lines
            #else:
                #print("No message received in timeout period")
        except Exception as e:
            print(f"Receiver error: {e}")

if __name__ == "__main__":
    receive_metrics()
