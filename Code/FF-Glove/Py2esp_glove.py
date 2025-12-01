import serial
import socket
import threading
import time

ESP_GLOVE_DATA = ""
UNITY_DATA = ""

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

def main():
    global ESP_GLOVE_DATA, UNITY_DATA

    # Open ESP32 Glove serial connection (reads potentiometer data)
    try:
        esp_glove = serial.Serial("COM3", 115200, timeout=1)
        print("ESP32 Glove connected on COM3")
    except Exception as e:
        print("Failed to open ESP Glove serial:", e)
        return

    # Open ESP32 Hand serial connection (sends concatenated data)
    try:
        esp_hand = serial.Serial("COM9", 115200, timeout=1)  # Change COM port as needed
        print("ESP32 Hand connected on COM9")
    except Exception as e:
        print("Failed to open ESP Hand serial:", e)
        return

    # Start ESP Glove reader thread
    threading.Thread(target=read_esp_glove, args=(esp_glove,), daemon=True).start()

    # Start Unity TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", 5001))
    server.listen(1)
    print("Waiting for Unity...")

    conn, _ = server.accept()
    print("Unity connected")

    # Start Unity reader in a separate thread
    threading.Thread(target=read_unity, args=(conn,), daemon=True).start()

    # Main loop: concatenate and send to ESP Hand
    while True:
        # Concatenate ESP Glove pot data with Unity data
        concatenated_data = ESP_GLOVE_DATA + "," + UNITY_DATA if UNITY_DATA else ESP_GLOVE_DATA
        
        # Send concatenated string to ESP Hand
        if concatenated_data:
            esp_hand.write((concatenated_data + "\n").encode())
        
        # Debug print
        print(f"Glove: {ESP_GLOVE_DATA} | Unity: {UNITY_DATA} | Sent to Hand: {concatenated_data}")
        
        time.sleep(0.01)  # adjust rate as needed

if __name__ == "__main__":
    main()