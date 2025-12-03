import serial
import socket
import threading
import time

ESP_GLOVE_DATA = ""
UNITY_DATA = ""
ESP_HAND_FEEDBACK = ""

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

# --- Read ESP32 Glove serial in a thread ---
def read_esp_glove(esp_glove):
    global ESP_GLOVE_DATA
    while True:
        try:
            line = esp_glove.readline().decode().strip()
            if line:
                ESP_GLOVE_DATA = line
        except Exception as e:
            print("ESP Glove read error:", e)
            time.sleep(0.1)

# --- Read ESP32 Hand feedback (Hall effect values) in a thread ---
def read_esp_hand(esp_hand):
    global ESP_HAND_FEEDBACK
    while True:
        try:
            line = esp_hand.readline().decode().strip()
            if line:
                ESP_HAND_FEEDBACK = line
                print(f"Hall Effect from Hand: {line}")
        except Exception as e:
            print("ESP Hand read error:", e)
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

# --- Send Hall effect feedback to Unity in a thread ---
def send_to_unity(conn):
    global ESP_HAND_FEEDBACK
    last_feedback = ""
    while True:
        try:
            if ESP_HAND_FEEDBACK and ESP_HAND_FEEDBACK != last_feedback:
                conn.sendall((ESP_HAND_FEEDBACK + "\n").encode())
                last_feedback = ESP_HAND_FEEDBACK
        except Exception as e:
            print("Unity send error:", e)
            break
        time.sleep(0.01)

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

def main():
    global ESP_GLOVE_DATA, UNITY_DATA, ESP_HAND_FEEDBACK
    global controller_offset_dict, hmd_offset_dict

    # Open ESP32 Glove serial connection (reads potentiometer data)
    try:
        esp_glove = serial.Serial("COM3", 115200, timeout=1)
        print("ESP32 Glove connected on COM3")
    except Exception as e:
        print("Failed to open ESP Glove serial:", e)
        return

    # Open ESP32 Hand serial connection (sends concatenated data & receives Hall effect)
    try:
        esp_hand = serial.Serial("COM9", 115200, timeout=1)  # Change COM port as needed
        print("ESP32 Hand connected on COM9")
    except Exception as e:
        print("Failed to open ESP Hand serial:", e)
        return

    # Start ESP Glove reader thread
    threading.Thread(target=read_esp_glove, args=(esp_glove,), daemon=True).start()
    
    # Start ESP Hand reader thread (for Hall effect feedback)
    threading.Thread(target=read_esp_hand, args=(esp_hand,), daemon=True).start()

    # Start Unity TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", 5001))
    server.listen(1)
    print("Waiting for Unity...")

    conn, _ = server.accept()
    print("Unity connected")

    # Start Unity reader in a separate thread
    threading.Thread(target=read_unity, args=(conn,), daemon=True).start()
    
    # Start Unity sender thread (for Hall effect feedback)
    threading.Thread(target=send_to_unity, args=(conn,), daemon=True).start()

    # Main loop: concatenate and send to ESP Hand
    while True:
        # Concatenate ESP Glove pot data with Unity data
        concatenated_data = ESP_GLOVE_DATA + "," + UNITY_DATA if UNITY_DATA else ESP_GLOVE_DATA
        
        unity_data_dict = parse_unity_data(UNITY_DATA)
        esp_data_dict = parse_esp_data(ESP_GLOVE_DATA)

        if not unity_data_dict or not esp_data_dict:
            # Skip this iteration if parsing failed
            time.sleep(0.01)
            continue 
        
        
        while(esp_data_dict['calib'] > 0):
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

        
        esp_hand.write(f"{esp_data_dict['thumb']},{esp_data_dict['index']},{esp_data_dict['middle']},{esp_data_dict['ring']},{esp_data_dict['pinky']},{unity_data_dict['controller_x']},{unity_data_dict['controller_y']},{unity_data_dict['controller_z']},{unity_data_dict['controller_pitch']},{unity_data_dict['controller_yaw']},{unity_data_dict['controller_roll']},{unity_data_dict['headset_pitch']},{unity_data_dict['headset_yaw']},{unity_data_dict['headset_roll']}\n".encode())
        
        # Send concatenated string to ESP Hand
        #if concatenated_data:
        #    esp_hand.write((concatenated_data + "\n").encode())
        
        # Debug print
        #print(f"Glove: {ESP_GLOVE_DATA} | Unity: {UNITY_DATA} | Sent to Hand: {concatenated_data}")
        
        time.sleep(0.01)  # adjust rate as needed

if __name__ == "__main__":
    main()