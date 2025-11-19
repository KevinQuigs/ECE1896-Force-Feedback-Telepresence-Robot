import serial
import time

# Adjust COM port to your Arduino
arduino = serial.Serial('COM7', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

while True:
    angle = input("Enter wrist angle (20-160) or 'q' to quit: ")
    if angle.lower() == 'q':
        break
    try:
        angle_val = int(angle)
        if 20 <= angle_val <= 160:
            arduino.write(f"{angle_val}\n".encode())
        else:
            print("Angle must be between 20 and 160")
    except ValueError:
        print("Please enter a valid integer.")

arduino.close()
