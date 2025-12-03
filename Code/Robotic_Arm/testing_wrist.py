import sys
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QPushButton, QComboBox, QTextEdit
)
from PyQt6.QtCore import Qt, QTimer

class FingerControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Hand Controller")

        # Make window bigger
        self.resize(800, 500)  # Width x Height
        # Optionally enforce a minimum size
        self.setMinimumSize(700, 450)

        # 5 fingers + wrist flex + wrist rotate
        self.labels_names = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Wrist Flex", "Wrist Rotate"]
        self.angles = [90] * len(self.labels_names)

        self.serial_conn = None
        self.init_ui()
        
        # Timer to read serial data
        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self.read_serial)
        self.serial_timer.start(50)  # Check every 50ms

    def init_ui(self):
        layout = QVBoxLayout()

        # Serial Port selection
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Serial Port:"))
        self.port_combo = QComboBox()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        port_layout.addWidget(self.port_combo)
        layout.addLayout(port_layout)

        # Connect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_serial)
        layout.addWidget(self.connect_button)

        # Sliders with angle display
        self.sliders = []
        self.angle_labels = []

        for i, name in enumerate(self.labels_names):
            finger_layout = QHBoxLayout()
            finger_layout.addWidget(QLabel(f"{name}:"))

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(180)
            slider.setValue(self.angles[i])
            slider.valueChanged.connect(lambda value, idx=i: self.update_angle(idx, value))
            finger_layout.addWidget(slider)
            self.sliders.append(slider)

            angle_label = QLabel(str(self.angles[i]))
            angle_label.setFixedWidth(40)  # Optional: make label width consistent
            finger_layout.addWidget(angle_label)
            self.angle_labels.append(angle_label)

            layout.addLayout(finger_layout)

        # Send button
        self.send_button = QPushButton("Send Angles")
        self.send_button.setFixedHeight(40)  # Optional: bigger button
        self.send_button.clicked.connect(self.send_angles)
        layout.addWidget(self.send_button)

        # Hall Effect Display
        hall_label = QLabel("Hall Effect Sensor Values:")
        layout.addWidget(hall_label)
        
        self.hall_display = QTextEdit()
        self.hall_display.setReadOnly(True)
        self.hall_display.setMaximumHeight(100)
        layout.addWidget(self.hall_display)

        self.setLayout(layout)

    def update_angle(self, idx, value):
        self.angles[idx] = value
        self.angle_labels[idx].setText(str(value))

    def connect_serial(self):
        port = self.port_combo.currentText()
        try:
            self.serial_conn = serial.Serial(port, 115200, timeout=0.1)
            self.connect_button.setText("Connected")
            self.connect_button.setEnabled(False)
        except Exception as e:
            self.connect_button.setText(f"Error: {e}")

    def read_serial(self):
        """Read incoming Hall effect data from COM3"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # Display the received data
                        current_text = self.hall_display.toPlainText()
                        
                        # Parse and format the Hall effect values
                        if ',' in line:
                            values = line.split(',')
                            if len(values) == 5:
                                formatted = f"Thumb: {values[0]}, Index: {values[1]}, Middle: {values[2]}, Ring: {values[3]}, Pinky: {values[4]}"
                                new_text = formatted + "\n" + current_text
                            else:
                                new_text = line + "\n" + current_text
                        else:
                            new_text = line + "\n" + current_text
                        
                        # Keep only last 10 lines
                        lines = new_text.split('\n')[:10]
                        self.hall_display.setText('\n'.join(lines))
                        
            except Exception as e:
                print(f"Error reading serial: {e}")

    def send_angles(self):
        if self.serial_conn and self.serial_conn.is_open:
            # Send as comma-separated values: "90,45,120,80,60,90,45\n"
            # Order: Thumb, Index, Middle, Ring, Pinky, WristFlex, WristRotate
            data = ",".join(str(angle) for angle in self.angles) + "\n"

            # data = "0,0,0,0,0,0,0,0,0.00,0,0,0,0,0" + "\n"
            self.serial_conn.write(data.encode())
            print(f"Sent: {data.strip()}")
        else:
            print("Serial not connected!")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = FingerControlGUI()
    gui.show()
    sys.exit(app.exec())