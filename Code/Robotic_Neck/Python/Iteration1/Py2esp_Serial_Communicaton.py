import sys
from PyQt6.QtWidgets import (
    QApplication, QSlider, QFrame, QGridLayout, QTabWidget, QMainWindow,
    QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QWidget
)
from PyQt6.QtCore import QTimer, Qt
import serial

class TESTBENCH_Robotic_Neck(QMainWindow):
    def __init__(self):
        super().__init__()

        # Required information for pyserial to read from arduino
        self.serial_port = "COM4"
        self.baud_rate = 115200
        self.ser = None

        # Backend Variables
        # YAW - movement of the head left/right
        self.yaw = 0
        # PITCH - movement of the head up/down
        self.pitch = 0
        self.roll = 0

        # UI and serial setup
        self.init_ui()
        self.open_serial_port()

        # Timer to periodically write current pitch/yaw to serial
        self.timer = QTimer(self)
        # connect writing only if serial port is open
        if self.ser and getattr(self.ser, "is_open", False):
            self.timer.timeout.connect(self.write_to_serial)
        else:
            # still connect; write_to_serial guards against missing serial
            self.timer.timeout.connect(self.write_to_serial)
        self.timer.start(90)  # milliseconds

    def init_ui(self):
        # Set up main window properties
        self.total_width = 800
        self.total_height = 800
        self.setWindowTitle("ROBOTIC HEAD TESTBENCH")
        self.setGeometry(100, 100, self.total_width, self.total_height)

        # Initialize the tab widget
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Initialize "Home" tab
        self.home_tab = QWidget()
        self.init_home_tab()
        self.tabs.addTab(self.home_tab, "ESP32 Setup/Connection")

        # Initialize "Test Bench" tab
        self.test_bench_tab = QWidget()
        self.init_test_bench_tab()
        self.tabs.addTab(self.test_bench_tab, "Test Bench")

    def init_home_tab(self):
        layout_home = QGridLayout()
        layout_title = QVBoxLayout()
        layout_input = QVBoxLayout()
        layout_status = QVBoxLayout()

        # Title frame
        frame_title = QFrame()
        frame_title.setFrameShape(QFrame.Shape.StyledPanel)
        frame_title.setFrameShadow(QFrame.Shadow.Raised)
        frame_title.setStyleSheet("background-color: gray; border-radius: 8px;")
        frame_title.setFixedSize(self.total_width - 10, 50)

        # Input frame
        frame_input = QFrame()
        frame_input.setFrameShape(QFrame.Shape.StyledPanel)
        frame_input.setFrameShadow(QFrame.Shadow.Raised)
        frame_input.setStyleSheet("background-color: gray; border-radius: 8px;")
        frame_input.setFixedSize(self.total_width - 10, 150)

        # Status frame
        frame_status = QFrame()
        frame_status.setFrameShape(QFrame.Shape.StyledPanel)
        frame_status.setFrameShadow(QFrame.Shadow.Raised)
        frame_status.setStyleSheet("background-color: gray; border-radius: 8px;")
        frame_status.setFixedSize(self.total_width - 10, 50)

        # COM Port Input
        line_layout_com = QHBoxLayout()
        self.com_label = QLabel("COM Port: ")
        self.com_input = QLineEdit()
        self.com_input.setPlaceholderText("e.g., COM4")
        line_layout_com.addWidget(self.com_label)
        line_layout_com.addWidget(self.com_input)

        # Baud Rate Input
        line_layout_baud = QHBoxLayout()
        self.baud_label = QLabel("Baud Rate:")
        self.baud_input = QLineEdit()
        self.baud_input.setPlaceholderText("e.g., 9600 or 115200")
        line_layout_baud.addWidget(self.baud_label)
        line_layout_baud.addWidget(self.baud_input)

        # Connect Button
        line_layout_button = QHBoxLayout()
        self.connect_button = QPushButton("CONNECT")
        self.connect_button.setStyleSheet("background-color: #772CE8; border-radius: 5px; color: white;")
        self.connect_button.setFixedSize(self.total_width - 30, 40)
        self.connect_button.clicked.connect(self.open_serial_port)
        line_layout_button.addWidget(self.connect_button)
        line_layout_button.addStretch()

        # Status Connection Row
        line_layout_connection_status = QHBoxLayout()
        self.esp_connection_line = QLabel("Esp32 Connection Status: ")
        self.connection_status = QLabel("Serial port not connected.")
        self.connection_status.setStyleSheet("color: red;")
        line_layout_connection_status.addWidget(self.esp_connection_line)
        line_layout_connection_status.addWidget(self.connection_status)

        # Assemble frames
        layout_title.addLayout(self.create_header("ROBOTIC HEAD TESTBENCH", 15))
        layout_input.addLayout(line_layout_com)
        layout_input.addLayout(line_layout_baud)
        layout_input.addLayout(line_layout_button)
        layout_status.addLayout(line_layout_connection_status)

        frame_title.setLayout(layout_title)
        frame_input.setLayout(layout_input)
        frame_status.setLayout(layout_status)

        layout_home.addWidget(frame_title, 0, 0)
        layout_home.addWidget(frame_input, 1, 0)
        layout_home.addWidget(frame_status, 2, 0)

        self.home_tab.setLayout(layout_home)

    def init_test_bench_tab(self):
        layout2 = QVBoxLayout()

        # Output labels
        self.pitch_deg_tb = QLabel("Pitch (deg): 0")
        self.pitch_rad_tb = QLabel("Pitch (rad): 0.0000")
        self.yaw_deg_tb = QLabel("Yaw (deg): 0")
        self.yaw_rad_tb = QLabel("Yaw (rad): 0.0000")
        self.roll_deg_tb = QLabel("Roll (deg): 0")
        self.roll_rad_tb = QLabel("Roll (rad): 0.0000")
                                                                                                                                
        

        layout2.addLayout(self.create_header("Pitch Control:", 12))
        self.pitch_slider = self.create_slider(vertical=True, min_val=-16, max_val=26, step=1, init_val=5)
        layout2.addWidget(self.pitch_slider, alignment=Qt.AlignmentFlag.AlignHCenter)

        layout2.addLayout(self.create_header("Roll Control:", 12))
        self.roll_slider = self.create_slider(vertical=True, min_val=-24, max_val=23, step=1, init_val=5)
        layout2.addWidget(self.roll_slider, alignment=Qt.AlignmentFlag.AlignHCenter)

        layout2.addLayout(self.create_header("Yaw Control:", 12))
        self.yaw_slider = self.create_slider(vertical=False, min_val=0, max_val=180, step=1, init_val=90)
        layout2.addWidget(self.yaw_slider, alignment=Qt.AlignmentFlag.AlignHCenter)

        layout2.addLayout(self.create_header("OUTPUTS:", 12))
        layout2.addWidget(self.pitch_deg_tb)
        layout2.addWidget(self.pitch_rad_tb)
        layout2.addWidget(self.roll_deg_tb)
        layout2.addWidget(self.roll_rad_tb)
        layout2.addWidget(self.yaw_deg_tb)
        layout2.addWidget(self.yaw_rad_tb)

        self.test_bench_tab.setLayout(layout2)

        # Make sure sliders update UI and backend
        self.pitch_slider.valueChanged.connect(self.handle_pitch_change)
        self.yaw_slider.valueChanged.connect(self.handle_yaw_change)
        self.roll_slider.valueChanged.connect(self.handle_roll_change)

    def create_header(self, input_title, size):
        header_layout = QHBoxLayout()
        header = QLabel(input_title)
        font = header.font()
        font.setBold(True)
        font.setPointSize(size)
        header.setFont(font)
        header_layout.addWidget(header)
        return header_layout

    def create_slider(self, vertical: bool, min_val: int, max_val: int, step: int, init_val: int = 0) -> QSlider:
        if vertical:
            slider = QSlider(Qt.Orientation.Vertical)
        else:
            slider = QSlider(Qt.Orientation.Horizontal)

        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setSingleStep(step)
        slider.setValue(init_val)
        slider.setTickPosition(QSlider.TickPosition.TicksBothSides)
        slider.setTickInterval(15)
        slider.setPageStep(5)
        return slider

    # handlers that update both backend and UI
    def handle_yaw_change(self, value):
        self.yaw = value
        self.yaw_deg_tb.setText(f"Yaw (deg): {self.yaw}")
        self.yaw_rad_tb.setText(f"Yaw (rad): {self.degToRad(self.yaw):.4f}")

    def handle_pitch_change(self, value):
        self.pitch = value
        self.pitch_deg_tb.setText(f"Pitch (deg): {self.pitch}")
        self.pitch_rad_tb.setText(f"Pitch (rad): {self.degToRad(self.pitch):.4f}")

    def handle_roll_change(self, value):
        self.roll = value
        self.roll_deg_tb.setText(f"Roll (deg): {self.roll}")
        self.roll_rad_tb.setText(f"Roll (rad): {self.degToRad(self.roll):.4f}")
        


    def open_serial_port(self):
        # use values from inputs if provided
        if hasattr(self, "com_input") and self.com_input.text():
            self.serial_port = self.com_input.text()
        if hasattr(self, "baud_input") and self.baud_input.text():
            try:
                self.baud_rate = int(self.baud_input.text())
            except ValueError:
                self.connection_status.setText("Invalid baud rate")
                self.connection_status.setStyleSheet("color: red;")
                return

        try:
            self.ser = serial.Serial(self.serial_port, int(self.baud_rate), timeout=1)
            if self.ser.is_open:
                self.connection_status.setText("Serial port opened successfully.")
                self.connection_status.setStyleSheet("color: green;")
                print("Serial port opened successfully.")
        except (serial.SerialException, ValueError) as e:
            self.connection_status.setText("Error opening serial port")
            self.connection_status.setStyleSheet("color: red;")
            print("Error opening serial port:", e)
            self.ser = None

    def read_serial(self):
        # Disabled for now — you can implement parsing of incoming CSV lines here.
        # Keep in mind any variables you reference must exist on the class.
        pass

    def write_to_serial(self):
        # Continuously send backend variables to the serial port.
        if not self.ser or not getattr(self.ser, "is_open", False):
            # nothing to write to
            return

        # build command using numeric attributes (no parentheses)
        command = f"{self.degToRad(self.pitch)},{self.degToRad(self.roll)},{self.degToRad(self.yaw)}\n"
        try:
            self.ser.write(command.encode())
        except Exception as e:
            print("Error writing to serial:", e)

    def degToRad(self, degrees):
        return degrees * (3.141592653589793 / 180.0)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TESTBENCH_Robotic_Neck()
    window.show()
    sys.exit(app.exec())
