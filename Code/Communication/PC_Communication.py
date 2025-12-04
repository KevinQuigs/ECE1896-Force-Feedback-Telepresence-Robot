import serial
import socket
import threading
import time
import keyboard
from teleop_client import init_teleop_client, send_tracking, cleanup_teleop, process_incoming

ESP_DATA = ""
UNITY_DATA = ""
force_feedback_serial = None

controller_offset_dict = {
        'pitch_offset': 0,
        'yaw_offset': 0,
        'roll_offset': 0
    }

hmd_offset_dict = {
        'pitch_offset': 0,
        'yaw_offset': 0,
        'roll_offset': 0
    }

#10.6.24.196 pitt guest wifi
#172.20.10.10 kevin hot
#192.168.1.250 kev wifi
#192.168.1.60 designlab wifi
#172.20.10.10 kev hotspot
init_teleop_client('192.168.1.60')  # Pi's IP address - dont change unless on different network

def init_force_feedback_serial():
    global force_feedback_serial
    try:
        force_feedback_serial = serial.Serial("COM3", 112500, timeout=1)  # Adjust COM port
        return True
    except Exception as e:
        print(f"Failed to open force feedback serial: {e}")
        return False

def handle_force_feedback(data):
    """Called when sensor data arrives from robot"""
    global force_feedback_serial
    if data.get('type') == 'sensor':
        # print(f"Force feedback received: {data}")
        # Send to your force feedback hardware here
        try:
            if force_feedback_serial and force_feedback_serial.is_open:
                force_feedback_serial.write(f"A{data['force_thumb']}B{data['force_index']}C{data['force_middle']}D{data['force_ring']}E{data['force_pinky']}\n".encode())
        except Exception as e:
            print(f"Error sending force feedback: {e}")

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
                print(f"Received from ESP: {ESP_DATA}")
                
        except Exception as e:
            print("ESP read error:", e)
            time.sleep(0.1)

# --- Read Unity TCP in a thread ---
def read_unity(conn):
    global UNITY_DATA
    buffer = ""
    while True:
        try:
            data = conn.recv(1024).decode()
            if not data:
                break
            
            buffer += data
            
            # Process complete lines (messages ending with \n)
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                if line.strip():  # Only update if not empty
                    UNITY_DATA = line.strip()
                    print(f"Received from Unity: {UNITY_DATA}")  # Debug print
                    
        except Exception as e:
            print("Unity read error:", e)
            break


def parse_unity_data(unity_data_str):
    """
    Parse Unity tracking data string and extract values.
    
    Args:
        unity_data_str: String in format "0.000,0.000,0.000,0.000,0.000,0.000,351.051,356.705,8.020"
        
    Returns:
        dict: Dictionary containing all parsed values
    """
    try:
        # Split by comma and convert to floats
        values = [float(x.strip()) for x in unity_data_str.split(',')]
        
        # Verify we have the correct number of values
        if len(values) != 9:
            raise ValueError(f"Expected 9 values, got {len(values)}")
        
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
        print(f"Raw data: '{unity_data_str}'")
        return None


def parse_esp_data(esp_data_str):
    """
    Parse ESP32 finger tracking data string and extract values.
    
    Args:
        esp_data_str: String in format "90.12,85.34,80.56,75.78,70.90, 0"
                                        Thumb,Index,Middle,Ring,Pinky,Calibration butt
        
    Returns:
        dict: Dictionary containing all parsed values
    """
    try:
        # Split by comma and convert to floats
        values = [float(x.strip()) for x in esp_data_str.split(',')]
        
        # Verify we have the correct number of values
        if len(values) != 6:
            raise ValueError(f"Expected 5 values, got {len(values)}")
        
        # Map to meaningful variable names
        esp_data = {
            'thumb': values[0],
            'index': values[1],
            'middle': values[2],
            'ring': values[3],
            'pinky': values[4],
            'calib': values[5]
        }
        
        return esp_data
        
    except (IndexError, ValueError) as e:
        print(f"Error parsing ESP data: {e}")
        print(f"Raw data: '{esp_data_str}'")
        return None  

def get_angle_offset(pitch, yaw, roll):
    # Capture current values as the new zero offsets
    
  #pitch -= pitch_offset;
  #yaw -= yaw_offset;
  #roll -= roll_offset;
    offset = {
        'pitch_offset': pitch,
        'yaw_offset': yaw,
        'roll_offset': roll
    }
        
    return offset



def main():
    global ESP_DATA, UNITY_DATA
    global controller_offset_dict, hmd_offset_dict
    
    # Initialize connection to webserver
    print("Connecting to Pi...")
    if init_teleop_client('192.168.1.60', on_receive=handle_force_feedback):
        print("Connected!")

        # Start ESP thread
        threading.Thread(target=read_esp, daemon=True).start()
        
        # Start Unity TCP server
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("0.0.0.0", 5001))
        server.listen(1)
        print("Waiting for Unity...")
        conn, _ = server.accept()
        print("Unity connected")

        # Start Unity reader in a separate thread
        threading.Thread(target=read_unity, args=(conn,), daemon=True).start()
        
        # Wait for initial data
        print("Waiting for initial data...")
        while not ESP_DATA or not UNITY_DATA:
            time.sleep(0.1)
        
        # Your main loop
        try:
            last_successful_send = time.time()
            while True:
                try:
                    
                    unity_data_dict = parse_unity_data(UNITY_DATA)
                    esp_data_dict = parse_esp_data(ESP_DATA)
                    
                    if not unity_data_dict or not esp_data_dict:
                        # Skip this iteration if parsing failed
                        time.sleep(0.01)
                        continue 
                    
                    '''
                    while(esp_data_dict['calib'] > 0):
                        print("Calibrating... Please close your fingers.")
                        controller_offset_dict = get_angle_offset(unity_data_dict['controller_pitch'], unity_data_dict['controller_yaw'], unity_data_dict['controller_roll'])
                        hmd_offset_dict = get_angle_offset(unity_data_dict['headset_pitch'], unity_data_dict['headset_yaw'], unity_data_dict['headset_roll'])
                    '''
                    while(keyboard.is_pressed('c')):
                        print("Calibrating... Please close your fingers.")
                        controller_offset_dict = get_angle_offset(unity_data_dict['controller_pitch'], unity_data_dict['controller_yaw'], unity_data_dict['controller_roll'])
                        hmd_offset_dict = get_angle_offset(unity_data_dict['headset_pitch'], unity_data_dict['headset_yaw'], unity_data_dict['headset_roll'])
                        

                    # Apply offsets
                    # Controller
                    unity_data_dict['controller_pitch'] -= controller_offset_dict['pitch_offset']
                    unity_data_dict['controller_yaw'] -= controller_offset_dict['yaw_offset']
                    unity_data_dict['controller_roll'] -= controller_offset_dict['roll_offset']
                    # HMD
                    unity_data_dict['headset_pitch'] -= hmd_offset_dict['pitch_offset']
                    unity_data_dict['headset_yaw'] -= hmd_offset_dict['yaw_offset']
                    unity_data_dict['headset_roll'] -= hmd_offset_dict['roll_offset']    

                    # Send to Pi
                    send_tracking(
                        esp_data_dict['thumb'], esp_data_dict['index'], esp_data_dict['middle'], esp_data_dict['ring'], esp_data_dict['pinky'],
                        unity_data_dict['controller_x'], unity_data_dict['controller_y'], unity_data_dict['controller_z'],
                        unity_data_dict['controller_pitch'], unity_data_dict['controller_yaw'], unity_data_dict['controller_roll'],
                        unity_data_dict['headset_pitch'], unity_data_dict['headset_yaw'], unity_data_dict['headset_roll']
                    )
                    last_successful_send = time.time()
                    
                    # Process incoming messages
                    process_incoming()
                    
                except Exception as e:
                    print(f"Error in main loop: {e}")
                    # Check if we've been unable to send for too long
                    if time.time() - last_successful_send > 5.0:
                        print("No successful sends for 5 seconds, reconnecting...")
                        break
                
                time.sleep(1/60)  # 60Hz
                
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            cleanup_teleop()
    else:
        print("Failed to connect!")

    

    

if __name__ == "__main__":
    main()
