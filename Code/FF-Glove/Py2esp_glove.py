import serial
import socket
import threading
import time

ESP_DATA = ""
UNITY_DATA = ""

# --- Read ESP32 serial in a thread ---
def read_esp():
    global ESP_DATA
    try:
        esp = serial.Serial("COM3", 9600, timeout=1)
    except Exception as e:
        print("Failed to open serial:", e)
        return

    while True:
        try:
            line = esp.readline().decode().strip()
            if line:
                ESP_DATA = line
                esp.write((ESP_DATA + "\n").encode())
                # Haptic feedback example
                # if ESP_DATA:
                #     esp.write(b"1\n")
                # else:
                #     esp.write(b"0\n")
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

def main():
    global ESP_DATA, UNITY_DATA

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

    # Main loop: concatenate and print messages
    while True:
        #FT_FI_FM_FR_FP_MP_MY_MR_HX_HY_HZ_KP_KY_KR
        concatenated = f"ESP: {ESP_DATA} | Unity: {UNITY_DATA}"
        print(concatenated)

        concatenated_data = ESP_DATA + UNITY_DATA
        print(f"String Sent: {concatenated_data}")
        time.sleep(1)  # adjust rate as needed

if __name__ == "__main__":
    main()
