import math
import serial
import time

# --- Configure Serial Port ---
# Change 'COM3' to your actual serial port (e.g. '/dev/ttyUSB0' on Linux/Mac)
ser = serial.Serial('COM4', 115200, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

# --- Get user inputs ---
yaw_str = input("Enter yaw (degrees): ")
pitch_str = input("Enter pitch (degrees): ")

# --- Convert to floats ---
yaw_deg = float(yaw_str)
pitch_deg = float(pitch_str)

# --- Convert to radians ---
yaw_rad = math.radians(yaw_deg)
pitch_rad = math.radians(pitch_deg)

# --- Print values to serial port ---
ser.write(f"{yaw_rad:.4f},{pitch_rad:.4f}\n".encode())

# --- Optional: print to console too ---
print(f"Sent to serial: Yaw = {yaw_rad:.4f} rad, Pitch = {pitch_rad:.4f} rad")

# --- Close serial connection ---
#ser.close()