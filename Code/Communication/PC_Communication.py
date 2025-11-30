import serial
import socket
import threading
import time
import re
from teleop_client import init_teleop_client, send_tracking, cleanup_teleop, process_incoming

ESP_DATA = ""
UNITY_DATA = ""
 
#10.6.24.196 pitt guest wifi

#172.20.10.10 kevin hot
#192.168.1.250 kev wifi
#192.168.1.60 designlab wifi
#172.20.10.10 kev hotspot
init_teleop_client('192.168.1.60')  # Pi's IP address - dont change unless on different network

def handle_force_feedback(data):
        """Called when sensor data arrives from robot"""
        if data.get('type') == 'sensor':
            print(f"Force feedback received: {data}")
            # Send to your force feedback hardware here
            serial.Serial.write(f"FF{data['force_thumb']},{data['force_index']},{data['force_middle']},{data['force_ring']},{data['force_pinky']}\n".encode())

# --- Read ESP32 serial in a thread ---
def read_esp():
    global ESP_DATA
    try:
        esp = serial.Serial("COM3", 112500, timeout=1)
    except Exception as e:
        print("Failed to open serial:", e)
        return

    while True:
        try:
            line = esp.readline().decode().strip()
            if line:
                ESP_DATA = line
                
        except Exception as e:
            print("ESP read error:", e)
            time.sleep(0.1)

# --- Read Unity TCP in a thread ---
def read_unity(conn):
    global UNITY_DATA
    while True:
        try:
            data = conn.recv(1024).decode().strip()
            if not data:
                break
            UNITY_DATA = data
        except Exception as e:
            print("Unity read error:", e)
            break


def parse_unity_data(unity_data_str):
    """
    Parse Unity tracking data string and extract values.
    
    Args:
        unity_data_str: String in format "MP0MY0MR0HX0HY0HZ0KP351.5KY356.7KR8.2"
        
    Returns:
        dict: Dictionary containing all parsed values
    """
    try:
        # Define the identifiers in order
        identifiers = ['MP', 'MY', 'MR', 'HX', 'HY', 'HZ', 'KP', 'KY', 'KR']
        
        # Use regex to find all identifier-value pairs
        # Pattern matches: 2 letters followed by optional minus and digits with optional decimal
        pattern = r'([A-Z]{2})(-?\d+\.?\d*)'
        matches = re.findall(pattern, unity_data_str)
        
        # Create a dictionary from matches
        data_dict = {identifier: float(value) for identifier, value in matches}
        
        # Verify all expected identifiers are present
        for identifier in identifiers:
            if identifier not in data_dict:
                raise ValueError(f"Identifier {identifier} not found")
        
        # Map to meaningful variable names in the correct order
        unity_data = {
            'controller_pitch': data_dict['MP'],
            'controller_yaw': data_dict['MY'],
            'controller_roll': data_dict['MR'],
            'controller_x': data_dict['HX'],
            'controller_y': data_dict['HY'],
            'controller_z': data_dict['HZ'],
            'headset_pitch': data_dict['KP'],
            'headset_yaw': data_dict['KY'],
            'headset_roll': data_dict['KR']
        }
        
        return unity_data
        
    except (IndexError, ValueError) as e:
        print(f"Error parsing Unity data: {e}")
        print(f"Raw data: {unity_data_str}")
        return None


def parse_esp_data(esp_data_str):
    """
    Parse ESP32 finger tracking data string and extract values.
    
    Args:
        esp_data_str: String in format "FT90.12FI85.34FM80.56FR75.78FP70.90" or "FT90FI85FM80FR75FP70"
        
    Returns:
        dict: Dictionary containing all parsed values
    """
    try:
        # Define the identifiers in order
        identifiers = ['FT', 'FI', 'FM', 'FR', 'FP']
        
        # Use regex to find all identifier-value pairs
        # Pattern matches: 2 letters followed by optional minus and digits with optional decimal
        pattern = r'([A-Z]{2})(-?\d+\.?\d*)'
        matches = re.findall(pattern, esp_data_str)
        
        # Create a dictionary from matches
        data_dict = {identifier: float(value) for identifier, value in matches}
        
        # Verify all expected identifiers are present
        for identifier in identifiers:
            if identifier not in data_dict:
                raise ValueError(f"Identifier {identifier} not found")
        
        # Map to meaningful variable names
        esp_data = {
            'thumb': data_dict['FT'],
            'index': data_dict['FI'],
            'middle': data_dict['FM'],
            'ring': data_dict['FR'],
            'pinky': data_dict['FP']
        }
        
        return esp_data
        
    except (IndexError, ValueError) as e:
        print(f"Error parsing ESP data: {e}")
        print(f"Raw data: {esp_data_str}")
        return None
    



def main():
    global ESP_DATA, UNITY_DATA
    
    # Initialize connection to webserver
    print("Connecting to Pi...")
    if init_teleop_client('192.168.1.60', on_receive=handle_force_feedback):
        print("Connected!")

        # Start ESP thread
        threading.Thread(target=read_esp, daemon=True).start()
        
        # Start Unity TCP server
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("0.0.0.0", 5001)) # Listening to all network interfaces on computer, Port 5001 (127.0.0.1 Would be just local machine programs)
        server.listen(1)
        print("Waiting for Unity...")
        conn, _ = server.accept()
        print("Unity connected")

        # Start Unity reader in a separate thread
        threading.Thread(target=read_unity, args=(conn,), daemon=True).start()
        
        # Your main loop
        try:
            while True:
                
                unity_data_dict = parse_unity_data(UNITY_DATA)
                esp_data_dict = parse_esp_data(ESP_DATA)
                if not unity_data_dict or not esp_data_dict:
                    time.sleep(0.01)
                    continue 

                # Send to Pi
                send_tracking(
                    esp_data_dict['thumb'], esp_data_dict['index'], esp_data_dict['middle'], esp_data_dict['ring'], esp_data_dict['pinky'],
                    unity_data_dict['controller_x'], unity_data_dict['controller_y'], unity_data_dict['controller_z'],
                    unity_data_dict['controller_pitch'], unity_data_dict['controller_yaw'], unity_data_dict['controller_roll'],
                    unity_data_dict['headset_pitch'], unity_data_dict['headset_yaw'], unity_data_dict['headset_roll']
                )
                # Process incoming messages
                process_incoming()
                time.sleep(1/60)  # 60Hz
                
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            cleanup_teleop()
    else:
        print("Failed to connect!")

    

    

if __name__ == "__main__":
    main()
