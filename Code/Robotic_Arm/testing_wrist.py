import sys
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QPushButton, QComboBox
)
from PyQt6.QtCore import Qt

class FingerControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Hand Controller")

        # Make window bigger
        self.resize(800, 400)  # Width x Height
        # Optionally enforce a minimum size
        self.setMinimumSize(700, 350)

        # 5 fingers + wrist flex + wrist rotate
        self.labels_names = ["Thumb", "Index", "Middle", "Ring", "Pinky", "Wrist Flex", "Wrist Rotate"]
        self.angles = [90] * len(self.labels_names)

        self.serial_conn = None
        self.init_ui()

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

        self.setLayout(layout)

    def update_angle(self, idx, value):
        self.angles[idx] = value
        self.angle_labels[idx].setText(str(value))

    def connect_serial(self):
        port = self.port_combo.currentText()
        try:
            self.serial_conn = serial.Serial(port, 115200, timeout=1)
            self.connect_button.setText("Connected")
            self.connect_button.setEnabled(False)
        except Exception as e:
            self.connect_button.setText(f"Error: {e}")

    def send_angles(self):
        if self.serial_conn and self.serial_conn.is_open:
            prefix = ["FT", "FI", "FM", "FR", "FP", "WF", "WR"]  # Thumb, Index, Middle, Ring, Pinky, Wrist Flex, Wrist Rotate
            data = "".join(f"{prefix[i]}{self.angles[i]}" for i in range(len(self.angles))) + "\n"
            self.serial_conn.write(data.encode())
            print(f"Sent: {data.strip()}")
        else:
            print("Serial not connected!")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = FingerControlGUI()
    gui.show()
    sys.exit(app.exec())
