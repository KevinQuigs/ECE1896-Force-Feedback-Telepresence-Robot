import serial
import socket
import threading

ESP_DATA = ""
UNITY_DATA = ""

# read ESP32 serial
def read_esp():
    global ESP_DATA
    esp = serial.Serial("COM4", 9600, timeout=1)
    while True:
        try:
            line = esp.readline().decode().strip()
            if line:
                ESP_DATA = line
                # print("ESP:", ESP_DATA)

                # HAPTIC FEEDBACK DATA BACK TO ESP
                if ESP_DATA == "1":
                    esp.write(b"1\n")
                else:
                    esp.write(b"0\n")

        except:
            pass


# ✅ read Unity TCP
def read_unity(conn):
    global UNITY_DATA
    while True:
        data = conn.recv(1024).decode().strip()
        if not data:
            break
        UNITY_DATA = data
        print("Unity:", UNITY_DATA)


def main():

    # Thread for ESP commmunication
    threading.Thread(target=read_esp, daemon=True).start()

    # Unity server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", 5001))
    server.listen(1)
    print("Waiting for Unity...")

    conn, _ = server.accept()
    print("Unity connected")
    read_unity(conn)

main()
