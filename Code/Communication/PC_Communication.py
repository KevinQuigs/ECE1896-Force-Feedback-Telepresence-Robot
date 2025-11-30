import serial
import socket
import threading
import time
from teleop_client import init_teleop_client, send_tracking, cleanup_teleop, process_incoming

ESP_DATA = ""
UNITY_DATA = ""
 
#10.6.24.196 pitt guest wifi

#172.20.10.10 kevin hot
#192.168.1.250 kev wifi
init_teleop_client('172.20.10.10')  # Pi's IP address - dont change unless on different network

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
                '''
                # Haptic feedback example
                if ESP_DATA == "1":
                    esp.write(b"1\n")
                else:
                    esp.write(b"0\n")
                '''
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
        unity_data_str: String in format "MP45.23MY12.67MR-8.45HX1.23HY0.56HZ2.34KP15.67KY-5.43KR2.10"
        
    Returns:
        dict: Dictionary containing all parsed values
    """
    try:
        # Define the identifiers in order
        identifiers = ['MP', 'MY', 'MR', 'HX', 'HY', 'HZ', 'KP', 'KY', 'KR']
        
        values = []
        remaining = unity_data_str
        
        for i, identifier in enumerate(identifiers):
            # Find where this identifier starts
            start_idx = remaining.find(identifier)
            if start_idx == -1:
                raise ValueError(f"Identifier {identifier} not found")
            
            # Move past the identifier
            remaining = remaining[start_idx + len(identifier):]
            
            # Find where the next identifier starts (if not the last one)
            if i < len(identifiers) - 1:
                next_identifier = identifiers[i + 1]
                end_idx = remaining.find(next_identifier)
                if end_idx == -1:
                    raise ValueError(f"Next identifier {next_identifier} not found")
                value_str = remaining[:end_idx]
                remaining = remaining[end_idx:]
            else:
                # Last value - take everything remaining
                value_str = remaining
            
            values.append(float(value_str))
        
        # Map to meaningful variable names
        unity_data = {
            'controller_pitch': values[0],
            'controller_yaw': values[1],
            'controller_roll': values[2],
            'controller_x': values[3],
            'controller_y': values[4],
            'controller_z': values[5],
            'headset_pitch': values[6],
            'headset_yaw': values[7],
            'headset_roll': values[8]
        }
        
        return unity_data
        
    except (IndexError, ValueError) as e:
        print(f"Error parsing Unity data: {e}")
        return None


def parse_esp_data(esp_data_str):
    """
    Parse ESP32 finger tracking data string and extract values.
    
    Args:
        esp_data_str: String in format "FT90.12FI85.34FM80.56FR75.78FP70.90"
        
    Returns:
        dict: Dictionary containing all parsed values
    """
    try:
        # Define the identifiers in order
        identifiers = ['FT', 'FI', 'FM', 'FR', 'FP']
        
        values = []
        remaining = esp_data_str
        
        for i, identifier in enumerate(identifiers):
            # Find where this identifier starts
            start_idx = remaining.find(identifier)
            if start_idx == -1:
                raise ValueError(f"Identifier {identifier} not found")
            
            # Move past the identifier
            remaining = remaining[start_idx + len(identifier):]
            
            # Find where the next identifier starts (if not the last one)
            if i < len(identifiers) - 1:
                next_identifier = identifiers[i + 1]
                end_idx = remaining.find(next_identifier)
                if end_idx == -1:
                    raise ValueError(f"Next identifier {next_identifier} not found")
                value_str = remaining[:end_idx]
                remaining = remaining[end_idx:]
            else:
                # Last value - take everything remaining
                value_str = remaining
            
            values.append(float(value_str))
        
        # Map to meaningful variable names
        esp_data = {
            'thumb': values[0],
            'index': values[1],
            'middle': values[2],
            'ring': values[3],
            'pinky': values[4]
        }
        
        return esp_data
        
    except (IndexError, ValueError) as e:
        print(f"Error parsing ESP data: {e}")
        return None
    



def main():
    global ESP_DATA, UNITY_DATA
    
    

    # Initialize connection to webserver
    print("Connecting to Pi...")
    if init_teleop_client('172.20.10.10', on_receive=handle_force_feedback):
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
