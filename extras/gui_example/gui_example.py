"""
ADS131E08 Monitor GUI
Copyright (c) 2026 nawaf, AGL (Arok Gandring Lokajaya)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import sys
import struct
import numpy as np
import serial
import serial.tools.list_ports
import csv
import datetime
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QComboBox, QPushButton, QLabel, QTabWidget, 
                             QMessageBox, QTableWidget, QTableWidgetItem, QHeaderView, 
                             QCheckBox, QSlider, QGroupBox, QGridLayout, QFrame)
import pyqtgraph as pg
from scipy import signal

# --- System Configuration ---
# Match these settings with your ESP32 firmware configuration
SERIAL_BAUD = 921600      # Serial baud rate for high-speed data streaming
FRAME_HEADER = b'\xAA\xBB' # 2-byte frame synchronization header
PACKET_SIZE = 30          # Total packet size: Sync(2) + Status(3) + Data(24) + CRC(1)
NUM_CHANNELS = 8          # Number of ADC channels
FS = 2000                 # Sampling frequency in Hz
VREF = 2.4                # Reference voltage (Default 2.4V)
GAIN = 1.0                # Programmable Gain Amplifier setting

# Resolution calculation: 16-bit for high SPS, 24-bit for standard SPS
if FS >= 32000:
    LSB_SIZE = (VREF / GAIN) / (2**15)
else:
    LSB_SIZE = (VREF / GAIN) / (2**23) 

# CRC-8 Lookup Table for Polynomial 0x07 (matches hardware implementation)
CRC8_TABLE = [
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
]

class DataParser:
    """Handles serial byte stream buffering, frame synchronization, and CRC validation."""
    def __init__(self):
        self.buffer = b''
        self.loss_count = 0 
    
    def calculate_crc(self, data):
        """Calculates CRC-8 using the lookup table."""
        crc = 0x00
        for byte in data:
            crc = CRC8_TABLE[crc ^ byte]
        return crc

    def parse(self, new_data):
        """Finds valid packets in the byte stream and converts raw data to voltage."""
        self.buffer += new_data
        parsed_packets = []
        while len(self.buffer) >= PACKET_SIZE:
            idx = self.buffer.find(FRAME_HEADER)
            if idx == -1:
                # No header found, clear buffer but keep last byte for next check
                self.buffer = self.buffer[-1:] 
                break
            if len(self.buffer) - idx < PACKET_SIZE:
                # Header found but packet incomplete, wait for more data
                self.buffer = self.buffer[idx:] 
                break
            
            # Extract packet, payload (Status + ADC Data), and received CRC
            packet = self.buffer[idx : idx + PACKET_SIZE]
            payload = packet[2:29] 
            received_crc = packet[29]
            
            # Verify CRC-8 integrity
            if self.calculate_crc(payload) == received_crc:
                voltages = []
                raw_data = payload[3:] # Skip 3 bytes of Status Word
                for i in range(NUM_CHANNELS):
                    start_byte = i * 3
                    sample_bytes = raw_data[start_byte : start_byte + 3]

                    # Convert 24-bit Two's Complement bytes to integer
                    val_int = int.from_bytes(sample_bytes, byteorder='big', signed=False)
                    if val_int & 0x800000:
                        val_int -= 0x1000000
                    
                    # Convert raw value to actual voltage
                    voltages.append(val_int * LSB_SIZE)
                
                parsed_packets.append({'data': voltages})
                self.buffer = self.buffer[idx + PACKET_SIZE:] # Move buffer forward
            else:
                # CRC Mismatch: Skip current header and count as loss
                self.loss_count += 1
                self.buffer = self.buffer[idx + 1:]
        return parsed_packets

class SerialWorker(QThread):
    """Worker thread for non-blocking Serial communication."""
    data_received = pyqtSignal(list)
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True
        self.parser = DataParser() 

    def run(self):
        """Main serial loop running in a separate thread."""
        try:
            with serial.Serial(self.port, SERIAL_BAUD, timeout=0.1) as ser:
                ser.reset_input_buffer()
                while self.running:
                    if ser.in_waiting > 0:
                        raw = ser.read(ser.in_waiting)
                        packets = self.parser.parse(raw)
                        if packets: 
                            self.data_received.emit(packets)
                    else:
                        self.msleep(1) # CPU friendly delay
        except Exception as e: 
            print(f"Serial Error: {e}")

    def stop(self):
        """Signals the thread to stop."""
        self.running = False
        self.wait()

class MainWindow(QMainWindow):
    """Main Application Window containing the UI and signal processing logic."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ADS131E08 Monitor")
        self.resize(1200, 800)
        self.buffer_size = 2000 # Number of points to display on graph
        self.data_buffers = np.zeros((NUM_CHANNELS, self.buffer_size))
        self.is_recording = False
        self.init_ui()
        self.serial_worker = None

    def init_ui(self):
        """Initializes the Graphical User Interface."""
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # --- GLOBAL CONTROL PANEL ---
        global_ctrl = QHBoxLayout()
        
        # Connection Group
        grp_conn = QGroupBox("Connection")
        l_conn = QHBoxLayout()
        self.combo_ports = QComboBox()
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        
        l_conn.addWidget(self.combo_ports)
        l_conn.addWidget(self.btn_refresh)
        l_conn.addWidget(self.btn_connect)
        grp_conn.setLayout(l_conn)
        
        # Filter Settings Group
        grp_filter = QGroupBox("Global Filter")
        l_filter = QHBoxLayout()
        self.check_dc = QCheckBox("Remove DC"); self.check_dc.setChecked(True)
        self.check_notch50 = QCheckBox("Notch 50Hz")
        self.check_lpf = QCheckBox("LPF")
        
        self.slider_lpf = QSlider(Qt.Orientation.Horizontal)
        self.slider_lpf.setRange(50, FS//2 - 1)
        self.slider_lpf.setValue(FS//10)
        self.lbl_lpf_val = QLabel(f"{self.slider_lpf.value()} Hz")
        self.lbl_lpf_val.setFixedWidth(60)
        self.slider_lpf.valueChanged.connect(self.update_lpf_label)
        
        l_filter.addWidget(self.check_dc)
        l_filter.addWidget(self.check_notch50)
        l_filter.addWidget(self.check_lpf)
        l_filter.addWidget(self.slider_lpf)
        l_filter.addWidget(self.lbl_lpf_val)
        grp_filter.setLayout(l_filter)

        # System/Status Group
        grp_sys = QGroupBox("System")
        l_sys = QHBoxLayout()
        self.lbl_loss = QLabel("Loss: 0")
        self.btn_record = QPushButton("Record")
        self.btn_record.clicked.connect(self.toggle_recording); self.btn_record.setEnabled(False)
        l_sys.addWidget(self.lbl_loss); l_sys.addWidget(self.btn_record)
        grp_sys.setLayout(l_sys)

        global_ctrl.addWidget(grp_conn); global_ctrl.addWidget(grp_filter); global_ctrl.addWidget(grp_sys)
        layout.addLayout(global_ctrl)
        
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)
        self.channel_widgets = []

        # Create tabs for each channel
        for i in range(NUM_CHANNELS):
            tab = QWidget()
            t_layout = QVBoxLayout(tab)
            
            # Measurement Statistics Table
            table = QTableWidget(1, 5)
            table.setHorizontalHeaderLabels(["RMS (V)", "Vpeak (V)", "Max (V)", "Min (V)", "Freq (Hz)"])
            table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
            table.setFixedHeight(65)
            table.setStyleSheet("background-color: #111; color: #0f0; font-weight: bold;")
            t_layout.addWidget(table)

            # Plots: Time Domain, Linear FFT, and Log FFT
            glw = pg.GraphicsLayoutWidget()
            t_layout.addWidget(glw)
            c_t = glw.addPlot(title=f"Time Domain CH {i+1}").plot(pen=pg.intColor(i))
            glw.nextRow()
            c_fl = glw.addPlot(title="FFT (Linear)").plot(pen='c')
            glw.nextRow()
            p_log = glw.addPlot(title="FFT (dB)"); p_log.setLogMode(x=True, y=False)
            c_fg = p_log.plot(pen='m')
            
            self.channel_widgets.append({
                "table": table, "time": c_t, "fft_lin": c_fl, "fft_log": c_fg
            })
            self.tabs.addTab(tab, f"CH {i+1}")

        # Initialize port list and GUI update timer
        self.refresh_ports()
        self.timer = QTimer(); self.timer.timeout.connect(self.update_gui); self.timer.start(50) 

    def update_lpf_label(self, value):
        """Updates the LPF frequency label UI."""
        self.lbl_lpf_val.setText(f"{value} Hz")

    def apply_signal_processing(self, data):
        """Applies real-time DSP filters (DC removal, Notch, Butter LPF)."""
        processed_data = data.copy()
        if self.check_dc.isChecked():
            processed_data = processed_data - np.mean(processed_data)
        
        nyq = 0.5 * FS
        if self.check_notch50.isChecked():
            b, a = signal.iirnotch(50.0, 30.0, fs=FS)
            processed_data = signal.filtfilt(b, a, processed_data)
            
        if self.check_lpf.isChecked():
            cutoff = self.slider_lpf.value()
            norm_cutoff = cutoff / nyq
            b, a = signal.butter(4, norm_cutoff, btype='low', analog=False)
            processed_data = signal.filtfilt(b, a, processed_data)
        return processed_data

    def refresh_ports(self):
        """Scans and populates available COM ports."""
        self.combo_ports.clear()
        self.combo_ports.addItems([p.device for p in serial.tools.list_ports.comports()])

    def toggle_connection(self):
        """Handles serial connection/disconnection logic."""
        if self.serial_worker is None:
            port = self.combo_ports.currentText()
            if not port: return
            self.serial_worker = SerialWorker(port)
            self.serial_worker.data_received.connect(self.handle_data)
            self.serial_worker.start()
            self.btn_connect.setText("Disconnect"); self.btn_connect.setStyleSheet("background-color: #e74c3c; color: white;")
            self.btn_record.setEnabled(True)
        else:
            self.serial_worker.stop(); self.serial_worker = None
            self.btn_connect.setText("Connect"); self.btn_connect.setStyleSheet("")
            self.btn_record.setEnabled(False)

    def toggle_recording(self):
        """Starts/Stops data logging to a CSV file."""
        if not self.is_recording:
            fn = f"ads_log_{datetime.datetime.now().strftime('%H%M%S')}.csv"
            self.csv_file = open(fn, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["Timestamp"] + [f"CH{i+1}" for i in range(NUM_CHANNELS)])
            self.is_recording = True; self.btn_record.setText("Stop Recording")
        else:
            self.is_recording = False; self.csv_file.close(); self.btn_record.setText("Record")

    def handle_data(self, packets):
        """Main callback for data received from SerialWorker thread."""
        voltages = np.array([p['data'] for p in packets])
        # Update rolling data buffers
        self.data_buffers = np.roll(self.data_buffers, -len(packets), axis=1)
        self.data_buffers[:, -len(packets):] = voltages.T
        
        # Log to file if recording is enabled
        if self.is_recording:
            for p in packets: 
                self.csv_writer.writerow([datetime.datetime.now().isoformat()] + p['data'])

    def update_gui(self):
        """Timer callback for refreshing UI elements and calculating FFT/Stats."""
        if self.serial_worker is None: return
            
        self.lbl_loss.setText(f"Loss: {self.serial_worker.parser.loss_count}")
        curr_idx = self.tabs.currentIndex()
        raw_data = self.data_buffers[curr_idx]
        
        if np.all(raw_data == 0): return

        # Apply digital filters
        filtered = self.apply_signal_processing(raw_data)
        
        # Calculate Time Domain Statistics
        v_rms = np.sqrt(np.mean(filtered**2))
        v_max, v_min = np.max(filtered), np.min(filtered)
        v_peak = max(abs(v_max), abs(v_min))
        
        # Perform Fast Fourier Transform (FFT)
        windowed = filtered * np.hanning(len(filtered))
        fft_m = np.abs(np.fft.rfft(windowed)) / len(filtered)
        freqs = np.fft.rfftfreq(len(filtered), 1/FS)
        
        # Peak frequency detection
        if len(fft_m) > 1:
            if self.check_dc.isChecked():
                idx_offset = 1 # Skip DC bin
                peak_idx = np.argmax(fft_m[idx_offset:]) + idx_offset
            else:
                peak_idx = np.argmax(fft_m)
            peak_freq = freqs[peak_idx]
        else:
            peak_freq = 0

        # Update statistical table
        table = self.channel_widgets[curr_idx]["table"]
        table.setItem(0, 0, QTableWidgetItem(f"{v_rms:.5f}"))
        table.setItem(0, 1, QTableWidgetItem(f"{v_peak:.5f}"))
        table.setItem(0, 2, QTableWidgetItem(f"{v_max:.5f}"))
        table.setItem(0, 3, QTableWidgetItem(f"{v_min:.5f}"))
        table.setItem(0, 4, QTableWidgetItem(f"{peak_freq:.1f}"))

        # Update Plot Curves
        self.channel_widgets[curr_idx]["time"].setData(filtered)
        self.channel_widgets[curr_idx]["fft_lin"].setData(freqs, fft_m)
        self.channel_widgets[curr_idx]["fft_log"].setData(freqs[1:], 20*np.log10(fft_m[1:] + 1e-12))

if __name__ == "__main__":
    # Application Entry Point
    app = QApplication(sys.argv)
    pg.setConfigOption('background', 'k') # Dark background for plots
    win = MainWindow(); win.show()
    sys.exit(app.exec())