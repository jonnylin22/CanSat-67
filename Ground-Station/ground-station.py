import sys
import os
import time
import serial
import threading
from PyQt5 import QtCore, QtGui, QtWidgets, uic
import pyqtgraph as pg
from csv import writer, reader
from datetime import datetime
import pytz
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import random

TEAM_ID = "1083"

# TELEMETRY_FIELDS = ["TEAM_ID", "MISSION_TIME", "PACKET_COUNT", "MODE", "STATE", "ALTITUDE",
#                     "TEMPERATURE", "PRESSURE", "VOLTAGE", "CURRENT", "GYRO_R", "GYRO_P", "GYRO_Y", "ACCEL_R",
#                     "ACCEL_P", "ACCEL_Y", "HEADING", "GPS_TIME", "GPS_ALTITUDE",
#                     "GPS_LATITUDE", "GPS_LONGITUDE", "GPS_SATS","CMD_ECHO", "MAX_ALTITUDE",
#                     "CONTAINER_RELEASED", "PAYLOAD_RELEASED", "PARAGLIDER_EJECTED", "PARAGLIDER_ACTIVE", "TARGET_LATITUDE",
#                     "TARGET_LONGITUDE"]

TELEMETRY_FIELDS = ["TEAM_ID", "MISSION_TIME", "PACKET_COUNT", "MODE", "STATE", "ALTITUDE",
                    "TEMPERATURE", "PRESSURE", "VOLTAGE", "SOLAR_VOLTAGE", "CURRENT", "GYRO_R", "GYRO_P", "GYRO_Y", "ACCEL_R",
                    "ACCEL_P", "ACCEL_Y", "GPS_TIME", "GPS_ALTITUDE", "GPS_LATITUDE", "GPS_LONGITUDE", 
                    "GPS_SATS","CMD_ECHO", "HEADING", "MAX_ALTITUDE", "CONTAINER_RELEASED", "PAYLOAD_RELEASED",
                    "PARAGLIDER_EJECTED", "PARAGLIDER_ACTIVE", "TARGET_LATITUDE",
                    "TARGET_LONGITUDE", "TILT"]

# TELEMETRY_FIELDS = ["TEAM_ID", "MISSION_TIME", "PACKET_COUNT", "MODE", "STATE", "ALTITUDE",
#                     "TEMPERATURE", "PRESSURE", "VOLTAGE", "CURRENT", "GYRO_R", "GYRO_P", "GYRO_Y", "ACCEL_R",
#                     "ACCEL_P", "ACCEL_Y", "GPS_TIME", "GPS_ALTITUDE",
#                     "GPS_LATITUDE", "GPS_LONGITUDE", "GPS_SATS","CMD_ECHO", "HEADING", "MAX_ALTITUDE",
#                     "CONTAINER_RELEASED", "PAYLOAD_RELEASED", "PARAGLIDER_EJECTED", "PARAGLIDER_ACTIVE", "TARGET_LATITUDE",
#                     "TARGET_LONGITUDE", "TILT"]


current_time = time.time()
local_time = time.localtime(current_time)
readable_time = time.strftime("telemetry_%Y-%m-%d_%H-%M-%S", local_time)

sim = False
sim_enable = False
telemetry_on = True
csv_indexer = 0

packet_count = 0
lost_packet_count = 0
prev_mission_sec = -1
packets_sent = 0

# xbee communication parameters
BAUDRATE = 115200
COM_PORT = "COM3"    # USB0 on raspberry pi
# COM_PORT = "/dev/ttyUSB0"    # USB0 on raspberry pi

MAKE_CSV_FILE = True
 # Set to True to create a CSV log file of telemetry data, must be set before running the program to work
SER_DEBUG = True       # Set as True whenever testing without XBee connected

START_DELIMITER = "~"

ser = None
serialConnected = False

""" The following serial function is used when windows laptop is used for GS and COM_PORT is set to the correct port number for the Xbee """ 
def connect_Serial():
    global ser
    global serialConnected
    if (not SER_DEBUG):
        try:
            # ser = serial.Serial("/dev/tty.usbserial-AR0JQZCB", BAUDRATE, timeout=0.05)
            ser = serial.Serial("COM" + str(COM_PORT), BAUDRATE, timeout=0.05)
            serialConnected = True
            print("Connected to Xbee")
        except serial.serialutil.SerialException as e:
            if (serialConnected):
                print(f"Could not connect to Xbee: {e}")
            serialConnected = False


# """ The following serial function is used when raspberry pi or linux machine is used for GS and is set to COM_PORT = "/dev/ttyUSB0" """ 
# def connect_Serial():
#     global ser
#     global serialConnected
#     if (not SER_DEBUG):
#         try:
#             ser = serial.Serial(COM_PORT, BAUDRATE, timeout=0.05)
#             serialConnected = True
#             print("Connected to Xbee")
#         except serial.serialutil.SerialException as e:
#             if (serialConnected):
#                 print(f"Could not connect to Xbee: {e}")
#             serialConnected = False

def disconnect_Serial():
    global ser
    global serialConnected
    if (not SER_DEBUG):
        try:
            if serialConnected and ser.is_open:
                ser.close()
                print("Disconnected from Xbee")
            serialConnected = False
        except Exception as e:
            print(f"Error while disconnecting Xbee: {e}")
            serialConnected = False

# telemetry
# strings as keys and values as values, only last stored
# need to write all commands to csv files by last filled values
telemetry = {}
payload_released = False
paraglider_active = False
container_released = False
paraglider_ejected = False
just_sent_cxon = False

class GroundStationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        '''
        Initialize the Ground Station Window, and start timer loop for updating UI
        '''
        super().__init__()

        # Load the UI
        ui_path = os.path.join(os.path.dirname(__file__), "gui", "ground_station.ui")
        # ui_path = os.path.join(os.path.dirname(__file__), "new-gui", "testing.ui")
        uic.loadUi(ui_path, self)

        # self.showFullScreen()

        self.setup_UI()
        self.connect_buttons()

        self.init_graphs()

    def setup_UI(self):
        '''
        Set UI to initial State
        '''
        logo_pixmap = QtGui.QPixmap(os.path.join(os.path.dirname(__file__), "gui", "logo.png")).scaled(self.logo.size(), aspectRatioMode=True)
        self.logo.setPixmap(logo_pixmap)
        self.setWindowIcon(QtGui.QIcon(os.path.join(os.path.dirname(__file__), "gui", "logo.png")))
        self.title.setText("CanSat Ground Station - TEAM " + TEAM_ID)

        # Get telemetry labels
        self.telemetry_labels = {}
        for field in TELEMETRY_FIELDS + ["LOST_PACKET_COUNT"]:
            if field == "PARAGLIDER_EJECTED" or field == "TEAM_ID":
                continue
            label = self.telemetry_container_1.findChild(QtWidgets.QLabel, field)
            if (not label):
                label = self.telemetry_container_2.findChild(QtWidgets.QLabel, field)
                if (not label):
                    label = self.telemetry_container_3.findChild(QtWidgets.QLabel, field)
            self.telemetry_labels[field] = label

        # Set First Color of Telemetry Toggle button
        if telemetry_on:
            self.telemetry_toggle_button.setText("Telemetry Toggle: On")
            self.make_button_green(self.telemetry_toggle_button)
        else:
            self.telemetry_toggle_button.setText("Telemetry Toggle: Off")
            self.make_button_red(self.telemetry_toggle_button)

    def connect_buttons(self):
        '''
        Connect Buttons to their functions
        '''
        self.sim_enable_button.clicked.connect(lambda: self.handle_simulation("ENABLE"))
        self.sim_activate_button.clicked.connect(lambda: self.handle_simulation("ACTIVATE"))
        self.sim_disable_button.clicked.connect(lambda: self.handle_simulation("DISABLE"))
        self.reset_state_button.clicked.connect(self.reset_state)
        self.set_time_gps_button.clicked.connect(lambda: write_xbee("CMD," + TEAM_ID + ",ST,GPS"))
        self.set_time_utc_button.clicked.connect(lambda: write_xbee("CMD," + TEAM_ID + ",ST," + datetime.now(pytz.timezone("UTC")).strftime("%H:%M:%S")))
        self.calibrate_alt_button.clicked.connect(lambda: write_xbee("CMD," + TEAM_ID + ",CAL"))
        self.release_payload_button.clicked.connect(self.release_payload_clicked)
        self.eject_paraglider_button.clicked.connect(lambda: write_xbee("CMD," + TEAM_ID + ",MEC,EJECT"))
        self.activate_paraglider_button.clicked.connect(self.activate_paraglider_clicked)
        self.release_container_button.clicked.connect(self.release_container_clicked)
        self.telemetry_toggle_button.clicked.connect(self.toggle_telemetry)
        self.set_coordinates_button.clicked.connect(self.set_coordinates)
        self.set_north_button.clicked.connect(lambda: write_xbee("CMD," + TEAM_ID + ",SETN"))

        # Connect non-sim buttons to update sim button colors
        self.reset_state_button.clicked.connect(self.non_sim_button_clicked)
        self.set_time_gps_button.clicked.connect(self.non_sim_button_clicked)
        self.set_time_utc_button.clicked.connect(self.non_sim_button_clicked)
        self.calibrate_alt_button.clicked.connect(self.non_sim_button_clicked)
        self.release_payload_button.clicked.connect(self.non_sim_button_clicked)
        self.activate_paraglider_button.clicked.connect(self.non_sim_button_clicked)
        self.release_container_button.clicked.connect(self.non_sim_button_clicked)
        self.eject_paraglider_button.clicked.connect(self.non_sim_button_clicked)
        self.telemetry_toggle_button.clicked.connect(self.non_sim_button_clicked)
        self.set_coordinates_button.clicked.connect(self.non_sim_button_clicked)

    def update(self):
        '''
        Set telemetry fields to most recent data
        '''
        global telemetry, telemetry_on, lost_packet_count

        for field in TELEMETRY_FIELDS:
            if field != "TEAM_ID" and field != "PARAGLIDER_EJECTED" and field != "TILT":
                self.telemetry_labels[field].setText(telemetry[field])

        self.telemetry_labels["LOST_PACKET_COUNT"].setText(str(lost_packet_count))

        self.update_graphs()
        self.update_color_buttons()

    def make_button_green(self, button):
        '''
        Takes in a button object and makes the background green
        '''
        button.setStyleSheet("QPushButton{background-color: rgba(40, 167, 69, 1);} QPushButton:hover{background-color: rgba(36, 149, 62, 1);}")

    def make_button_red(self, button):
        '''
        Takes in a button object and makes the background red
        '''
        button.setStyleSheet("QPushButton{background-color: rgba(220, 53, 69, 1);} QPushButton:hover{background-color: rgba(200, 45, 59, 1);}")

    def make_button_blue(self, button):
        '''
        Takes in a button object and makes the background blue
        '''
        button.setStyleSheet("QPushButton{background-color: rgba(33,125,182,1);} QPushButton:hover{background-color: rgba(27, 100, 150, 1);}")

    def handle_simulation(self, cmd):
        '''
        Send a simulation command: ("ACTIVATE", "ENABLE", "DISABLE")
        '''
        global sim_enable, csv_indexer

        if cmd == "ACTIVATE" and sim_enable == False or cmd == "ENABLE" and sim_enable == True:
            return
        
        write_xbee("CMD," + TEAM_ID + ",SIM," + cmd)

        if cmd == "ENABLE":
            sim_enable = True
        elif cmd == "DISABLE":
            sim_enable = False
        elif cmd == "ACTIVATE":
            sim_enable = False
            csv_indexer = 0
        
        self.update_sim_button_colors()

        if cmd == "ACTIVATE":
            # Wait 1 second to let the Payload receive the command
            # before sending simp data
            time.sleep(1)

    def update_sim_button_colors(self):
        '''
        Set simulation buttons to the correct colors based off of
        if it is active, enabled, or disabled
        '''
        global sim, sim_enable

        if (sim):
            self.make_button_blue(self.sim_enable_button)
            self.make_button_blue(self.sim_activate_button)
            self.make_button_red(self.sim_disable_button)
        elif (sim_enable):
            self.make_button_blue(self.sim_enable_button)
            self.make_button_green(self.sim_activate_button)
            self.make_button_blue(self.sim_disable_button)
        else:
            self.make_button_green(self.sim_enable_button)
            self.make_button_blue(self.sim_activate_button)
            self.make_button_blue(self.sim_disable_button)

    def non_sim_button_clicked(self):
        '''
        Disable simulation when any button is pressed
        '''
        global sim_enable, sim
        sim_enable = False
        sim = False
        self.update_sim_button_colors()
    
    def reset_state(self):
        global packet_count, lost_packet_count, prev_mission_sec
        packet_count = 0
        lost_packet_count = 0
        prev_mission_sec = -1
        self.reset_graphs()
        write_xbee("CMD," + TEAM_ID + ",RST")

    def toggle_telemetry(self):
        global telemetry_on
        telemetry_on = not telemetry_on

        if telemetry_on:
            write_xbee("CMD,"+ TEAM_ID + ",CX,ON")
            self.telemetry_toggle_button.setText("Telemetry Toggle: On")
            self.make_button_green(self.telemetry_toggle_button)
            global just_sent_cxon
            just_sent_cxon = True
        else:
            write_xbee("CMD,"+ TEAM_ID + ",CX,OFF")
            self.telemetry_toggle_button.setText("Telemetry Toggle: Off")
            self.make_button_red(self.telemetry_toggle_button)

    def release_payload_clicked(self):
        global payload_released
        if payload_released:
            write_xbee("CMD," + TEAM_ID + ",MEC,PAYLOAD,OFF")
        else:
            write_xbee("CMD," + TEAM_ID + ",MEC,PAYLOAD,ON")
    
    def activate_paraglider_clicked(self):
        global paraglider_active
        if paraglider_active:
            write_xbee("CMD," + TEAM_ID + ",MEC,GLIDER,OFF")
        else:
            write_xbee("CMD," + TEAM_ID + ",MEC,GLIDER,ON")
    
    def release_container_clicked(self):
        global container_released
        if container_released:
            write_xbee("CMD," + TEAM_ID + ",MEC,CONTAINER,OFF")
        else:
            write_xbee("CMD," + TEAM_ID + ",MEC,CONTAINER,ON")

    def eject_paraglider_clicked(self):
        '''
        Send eject command
        '''
        global paraglider_ejected
        if not paraglider_ejected:
            write_xbee("CMD," + TEAM_ID + ",MEC,EJECT")

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            self.close()

    def update_color_buttons(self):
        global payload_released
        if payload_released:
            self.release_payload_button.setText("Reset Payload Release")
            self.make_button_green(self.release_payload_button)
        else:
            self.release_payload_button.setText("Release Payload")
            self.make_button_red(self.release_payload_button)

        global paraglider_active
        if paraglider_active:
            self.activate_paraglider_button.setText("Deactivate Paraglider")
            self.make_button_green(self.activate_paraglider_button)
        else:
            self.activate_paraglider_button.setText("Activate Paraglider")
            self.make_button_red(self.activate_paraglider_button)

        global container_released
        if container_released:
            self.release_container_button.setText("Reset Container Release")
            self.make_button_green(self.release_container_button)
        else:
            self.release_container_button.setText("Release Container")
            self.make_button_red(self.release_container_button)

        global paraglider_ejected
        if paraglider_ejected:
            self.eject_paraglider_button.setText("Reset Eject Paraglider")
            self.make_button_green(self.eject_paraglider_button)
        else:
            self.eject_paraglider_button.setText("Eject Paraglider")
            self.make_button_red(self.eject_paraglider_button)

    def init_graphs(self):
        self.x_data = []
        self.counter = 0

        self.altitude_figure = Figure()
        self.altitude_canvas = FigureCanvas(self.altitude_figure)
        self.altitude_graph.layout().addWidget(self.altitude_canvas)
        self.alt_subplot = self.altitude_figure.add_subplot(111)
        self.altitude_y_data = []

        self.accel_figure = Figure()
        self.accel_canvas = FigureCanvas(self.accel_figure)
        self.acceleration_graph.layout().addWidget(self.accel_canvas)
        self.accel_subplot = self.accel_figure.add_subplot(111)
        self.accel_r_y_data = []
        self.accel_p_y_data = []
        self.accel_y_y_data = []
        self.accel_r_line, = self.accel_subplot.plot([], [], label="R", color="blue")
        self.accel_p_line, = self.accel_subplot.plot([], [], label="P", color="orange")
        self.accel_y_line, = self.accel_subplot.plot([], [], label="Y", color="green")
        self.accel_figure.legend()

        self.rotation_figure = Figure()
        self.rotation_canvas = FigureCanvas(self.rotation_figure)
        self.rotation_graph.layout().addWidget(self.rotation_canvas)
        self.rotation_subplot = self.rotation_figure.add_subplot(111)
        self.rotation_r_y_data = []
        self.rotation_p_y_data = []
        self.rotation_y_y_data = []
        self.rotation_r_line, = self.rotation_subplot.plot([], [], label="R", color="blue")
        self.rotation_p_line, = self.rotation_subplot.plot([], [], label="P", color="orange")
        self.rotation_y_line, = self.rotation_subplot.plot([], [], label="Y", color="green")
        self.rotation_figure.legend()

        self.current_figure = Figure()
        self.current_canvas = FigureCanvas(self.current_figure)
        self.current_graph.layout().addWidget(self.current_canvas)
        self.current_subplot = self.current_figure.add_subplot(111)
        self.current_y_data = []

        self.voltage_figure = Figure()
        self.voltage_canvas = FigureCanvas(self.voltage_figure)
        self.voltage_graph.layout().addWidget(self.voltage_canvas)
        self.voltage_subplot = self.voltage_figure.add_subplot(111)
        self.voltage_y_data = []

        # Solar panel voltage graph. If no solar packet field is received, it stays at zero.
        self.solar_figure = Figure()
        self.solar_canvas = FigureCanvas(self.solar_figure)
        # UI now includes `solar_graph`, so add the canvas directly.
        self.solar_graph.layout().addWidget(self.solar_canvas)
        self.solar_subplot = self.solar_figure.add_subplot(111)
        self.solar_y_data = []

        # self.timer = QtCore.QTimer()
        # self.timer.setInterval(100)  # 100 ms update
        # self.timer.timeout.connect(self.update_graphs)
        # self.timer.start()

    def update_graphs(self):

        # Update data
        self.x_data.append(telemetry["PACKET_COUNT"])
        self.altitude_y_data.append(float(telemetry["ALTITUDE"]))
        self.accel_r_y_data.append(float(telemetry["ACCEL_R"]))
        self.accel_p_y_data.append(float(telemetry["ACCEL_P"]))
        self.accel_y_y_data.append(float(telemetry["ACCEL_Y"]))
        self.rotation_r_y_data.append(float(telemetry["GYRO_R"]))
        self.rotation_p_y_data.append(float(telemetry["GYRO_P"]))
        self.rotation_y_y_data.append(float(telemetry["GYRO_Y"]))
        self.current_y_data.append(float(telemetry["CURRENT"]))
        self.voltage_y_data.append(float(telemetry["VOLTAGE"]))

        # Solar voltage should only be plotted when the payload actually sends it.
        # If the field is absent, keep the graph at zero instead of matching the main battery voltage.
        try:
            solar_val = float(telemetry["SOLAR_VOLTAGE"])
        except (KeyError, ValueError, TypeError):
            solar_val = 0.0
        self.solar_y_data.append(solar_val)

        # self.x_data.append(self.counter)
        # self.altitude_y_data.append(random.randint(0,10))
        # self.accel_r_y_data.append(random.randint(0,10))
        # self.accel_p_y_data.append(random.randint(0,10))
        # self.accel_y_y_data.append(random.randint(0,10))
        # self.rotation_r_y_data.append(random.randint(0,10))
        # self.rotation_p_y_data.append(random.randint(0,10))
        # self.rotation_y_y_data.append(random.randint(0,10))
        # self.current_y_data.append(random.randint(0,10))
        # self.voltage_y_data.append(random.randint(0,10))
        # self.counter += 1

        # Only plot last 10 points
        if len(self.x_data) > 10:
            self.x_data.pop(0)
            self.altitude_y_data.pop(0)
            self.accel_r_y_data.pop(0)
            self.accel_p_y_data.pop(0)
            self.accel_y_y_data.pop(0)
            self.rotation_r_y_data.pop(0)
            self.rotation_p_y_data.pop(0)
            self.rotation_y_y_data.pop(0)
            self.current_y_data.pop(0)
            self.voltage_y_data.pop(0)
            self.solar_y_data.pop(0)

        # Plot
        self.alt_subplot.clear()
        self.alt_subplot.plot(self.x_data, self.altitude_y_data, color='blue')
        self.alt_subplot.set_title("Altitude (m)")
        self.altitude_canvas.draw()

        self.accel_subplot.clear()
        self.accel_subplot.plot(self.x_data, self.accel_r_y_data, color='blue')
        self.accel_subplot.plot(self.x_data, self.accel_p_y_data, color='orange')
        self.accel_subplot.plot(self.x_data, self.accel_y_y_data, color='green')
        self.accel_subplot.set_title("Acceleration (°/s^2)")
        self.accel_canvas.draw()

        self.rotation_subplot.clear()
        self.rotation_subplot.plot(self.x_data, self.rotation_r_y_data, color='blue')
        self.rotation_subplot.plot(self.x_data, self.rotation_p_y_data, color='orange')
        self.rotation_subplot.plot(self.x_data, self.rotation_y_y_data, color='green')
        self.rotation_subplot.set_title("Rotation Rate (°/s)")
        self.rotation_canvas.draw()

        self.current_subplot.clear()
        self.current_subplot.plot(self.x_data, self.current_y_data, color='blue')
        self.current_subplot.set_title("Current (A)")
        self.current_canvas.draw()

        self.voltage_subplot.clear()
        self.voltage_subplot.plot(self.x_data, self.voltage_y_data, color='blue')
        self.voltage_subplot.set_title("Voltage (V)")
        self.voltage_canvas.draw()

        # Solar panel voltage plot
        self.solar_subplot.clear()
        self.solar_subplot.plot(self.x_data, self.solar_y_data, color='gold')
        self.solar_subplot.set_title("Solar Panel Voltage (V)")
        self.solar_canvas.draw()

    def reset_graphs(self):
        self.x_data = []
        self.altitude_y_data = []
        self.accel_r_y_data = []
        self.accel_p_y_data = []
        self.accel_y_y_data = []
        self.rotation_r_y_data = []
        self.rotation_p_y_data = []
        self.rotation_y_y_data = []
        self.current_y_data = []
        self.voltage_y_data = []
        self.solar_y_data = []

    def set_coordinates(self):
        dialog = CoordinatesDiaglog()
        result = dialog.exec_()

        if result == QtWidgets.QDialog.Accepted:
            num1, num2 = dialog.get_values()
            write_xbee("CMD," + TEAM_ID + ",SC,{:.6f},{:.6f}".format(num1, num2))
        else:
            QtWidgets.QMessageBox.information(self, "Cancelled", "You pressed Cancel!")


class CoordinatesDiaglog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Enter Two Numbers")

        layout = QtWidgets.QFormLayout()

        # First number
        self.num1 = QtWidgets.QDoubleSpinBox()
        self.num1.setDecimals(6)       # allow up to 6 decimal places
        self.num1.setRange(-1e6, 1e6)  # set a large range
        self.num1.setSingleStep(0.0001) # step size
        self.num1.clear()
        layout.addRow("Target Latitude:", self.num1)

        # Second number
        self.num2 = QtWidgets.QDoubleSpinBox()
        self.num2.setDecimals(6)
        self.num2.setRange(-1e6, 1e6)
        self.num2.setSingleStep(0.0001)
        self.num2.clear()
        layout.addRow("Target Longitude:", self.num2)

        # OK / Cancel buttons
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.setLayout(layout)

    def get_values(self):
        return self.num1.value(), self.num2.value()

def calc_checksum(data):
    '''
    Calculate Checksum based off of packet data
    '''
    return sum(data.encode()) % 256

def verify_checksum(data, checksum):
    '''
    Return a boolean of if the checksum is correct
    '''
    return checksum == calc_checksum(data)



def parse_xbee(data):
    '''
    Parse the data from an incoming Xbee packet
    '''
    global telemetry, packet_count, lost_packet_count, prev_mission_sec, w #, last_recieved_packet

    # Ensure only recieving each packet once
    # sent_packet_count = int(data[TELEMETRY_FIELDS.index("PACKET_COUNT")])
    # if sent_packet_count == last_recieved_packet:
    #     return
    # last_recieved_packet = sent_packet_count

    packet_count += 1

    for i in range(len(data)):
        if TELEMETRY_FIELDS[i] == "PACKET_COUNT":
            telemetry["PACKET_COUNT"] = str(packet_count)
        else:
            telemetry[TELEMETRY_FIELDS[i]] = data[i]

    global payload_released
    if telemetry["PAYLOAD_RELEASED"] == "TRUE":
        payload_released = True
    else:
        payload_released = False

    global paraglider_active
    if telemetry["PARAGLIDER_ACTIVE"] == "TRUE":
        paraglider_active = True
    else:
        paraglider_active = False

    global container_released
    if telemetry["CONTAINER_RELEASED"] == "TRUE":
        container_released = True
    else:
        container_released = False

    global paraglider_ejected
    if telemetry["PARAGLIDER_EJECTED"] == "TRUE":
        paraglider_ejected = True
    else:
        paraglider_ejected = False

    global sim
    if telemetry['MODE'] == "S":
        sim = True
    else:
        sim = False

    # Update lost_packet_count
    global just_sent_cxon
    cur_mission_sec = int(telemetry["MISSION_TIME"][-2:])
    if just_sent_cxon:
        just_sent_cxon = False
        prev_mission_sec = cur_mission_sec
    else:
        if (prev_mission_sec != -1):
            missed_packets = cur_mission_sec - prev_mission_sec - 1
            if (missed_packets < 0):
                missed_packets += 60
            lost_packet_count += missed_packets
        prev_mission_sec = cur_mission_sec

    # Add data to csv file
    if MAKE_CSV_FILE:
        csv_dir = os.path.join(os.path.dirname(__file__), "flight-csv")
        file = os.path.join(csv_dir, "Flight_" + TEAM_ID + "_" + readable_time +'.csv')
        with open(file, 'a', newline='') as f_object:
            writer_object = writer(f_object)
            writer_object.writerow(list(telemetry.values()))

    w.update()

def read_xbee():
    '''
    Read packets from the Xbee module
    '''
    buffer = ""
    global serialConnected
    while True:     # Keep running as long as the serial connection is open
        if not serialConnected:
            connect_Serial()
            time.sleep(0.1)
            continue

        try:
            if ser.inWaiting() > 0:
                buffer += ser.read(ser.inWaiting()).decode(errors='replace')

                start_idx = buffer.find(START_DELIMITER)
                end_idx = buffer.find("\n", start_idx)
                next_start = buffer.find(START_DELIMITER, start_idx + 1)

                if next_start != -1 and (end_idx == -1 or next_start < end_idx):
                    frame_end = next_start
                elif end_idx != -1:
                    frame_end = end_idx
                else:
                    # Wait for a full packet
                    continue

                frame = buffer[start_idx + 1:frame_end].strip()
                buffer = buffer[frame_end + 1:]

                try:
                    data, checksum = frame.rsplit(",", 1)
                    if verify_checksum(data, float(checksum)):

                        split_data = data.split(",")
                        if len(split_data) == len(TELEMETRY_FIELDS):
                            parse_xbee(split_data)
                        else:
                            print("Incorrect number of fields in frame: ", frame)
                    else:
                        print("Failed to read frame:", frame)
                except Exception as e:
                    print(e)
                    print("Error reading frame: ", frame)

                # start_byte = ser.read(1)
                # if start_byte != START_DELIMITER:
                #     print(start_byte.decode())

                # if start_byte == START_DELIMITER:f
                #     time.sleep(0.1)
                #     frame = ser.read_until(b"\n").decode().strip()
                #     try:
                #         data, checksum = frame.rsplit(",", 1)
                #         if verify_checksum(data, float(checksum)):
                #             parse_xbee(data.split(","))
                #         else:
                #             print("Failed to read frame:", frame)
                #     except:
                #         print("Failed to read frame:", frame)

            serialConnected = True
        except serial.serialutil.SerialException as e:
            if serialConnected:
                print(f"Connection Lost: {e}")
            serialConnected = False
        except OSError as e:
            if (serialConnected):
                print(f"Connection Lost: {e}")
            serialConnected = False

        time.sleep(0.1)
            

def write_xbee(cmd):
    '''
    Write commands to the Xbee
    '''
    # Frame Format: ~<data>,<checksum>

    # Create Packet
    global packets_sent
    packets_sent += 1
    checksum = calc_checksum(f"{cmd}")
    frame = f"{START_DELIMITER}{cmd},{checksum:02X}"

    # Debug mode: simulate a local packet instead of sending over serial so the UI can still be tested
    if SER_DEBUG:
        print(f"[DEBUG] Packet queued locally: {cmd}")
        return

    # Send to XBee
    try:
        if (ser):
            ser.write(frame.encode())
            print(f"Packet Sent: {cmd}")
    except serial.serialutil.SerialException as e:
        print(f"Packet Not Sent: {e}")


def build_debug_telemetry_packet(sim_value):
    '''
    Create a synthetic packet matching the expected telemetry format for GUI testing.
    Use "F" (flight) when not in sim mode so disabling the local simulator stops the loop.
    '''
    global packet_count
    packet_count += 1

    mode = "S" if sim_enable else "F"

    return [
        TEAM_ID,
        "00:00:00",
        str(packet_count),
        mode,
        "FLIGHT",
        str(float(sim_value)),
        "22.5",
        str(float(sim_value) + 5.0),
        "12.5",
        str(float(sim_value) * 0.8),
        "0.55",
        "1.0",
        "2.0",
        "3.0",
        "4.0",
        "5.0",
        "6.0",
        "00:00:00",
        "120.0",
        "33.123456",
        "-117.123456",
        "8",
        "OK",
        "90.0",
        "300.0",
        "FALSE",
        "FALSE",
        "FALSE",
        "FALSE",
        "0.0",
        "0.0",
        "0.0",
    ]


def send_simp_data():
    '''
    Send simulated pressure data from the csv file at 1 Hz.
    '''
    global sim
    global sim_enable
    global csv_indexer

    sim_csv = os.path.join(os.path.dirname(__file__), "sim_data.csv")
    fallback_csv = os.path.join(os.path.dirname(__file__), "old_25_26_pres.csv")

    if os.path.exists(sim_csv):
        csv_path = sim_csv
    elif os.path.exists(fallback_csv):
        csv_path = fallback_csv
    else:
        print("No simulation CSV file found. Expected sim_data.csv or old_25_26_pres.csv")
        return

    with open(csv_path, 'r') as csv_file:
        csv_lines = csv_file.readlines()

    csv_indexer = 0
    while True:
        # When the GUI disables simulation, stop immediately instead of continuing to feed data.
        if not sim_enable and not sim:
            time.sleep(0.2)
            continue

        if (sim or sim_enable) and csv_indexer < len(csv_lines):
            csv_num = str(csv_lines[csv_indexer].strip())

            if SER_DEBUG:
                parse_xbee(build_debug_telemetry_packet(csv_num))
            else:
                write_xbee('CMD,' + TEAM_ID + ',SIMP,' + str(csv_num))

            csv_indexer += 1

        time.sleep(1)




def main():
    connect_Serial()

    # Create new csv file with header
    if MAKE_CSV_FILE:
        csv_dir = os.path.join(os.path.dirname(__file__), "flight-csv")
        os.makedirs(csv_dir, exist_ok=True)
        file = os.path.join(csv_dir, "Flight_" + TEAM_ID + "_" + readable_time + '.csv')
        with open(file, 'w', newline='') as f_object:
            writer_object = writer(f_object)
            writer_object.writerow(TELEMETRY_FIELDS)

    # Run the app
    app = QtWidgets.QApplication(sys.argv)
    global w
    w = GroundStationWindow()

    # simulate GUI clicks
    # QtCore.QTimer.singleShot(1000, lambda: w.telemetry_toggle_button.click())  # calls telemetry toggle button click   # run after 100 ms 
    
    if (not SER_DEBUG):
        threading.Thread(target=read_xbee, daemon=True).start()
    threading.Thread(target=send_simp_data, daemon=True).start()

    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
