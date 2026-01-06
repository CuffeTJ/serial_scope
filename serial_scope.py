import sys
import serial
import serial.tools.list_ports
import numpy as np
import time
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QPushButton, QComboBox, QLabel, QMessageBox, 
                             QGroupBox, QLineEdit, QGridLayout, QScrollArea, 
                             QPlainTextEdit, QFileDialog)
from PyQt5.QtGui import QFont 
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt
import pyqtgraph as pg

FRAME_LEN = 40
BAUD_RATE = 1000000

FREQ_G_MAP = {
    "2KHz": 0,
    "1KHz": 1,
    "500Hz": 2,
    "250Hz": 3
}

def calculate_coeffs(g):
    nf = (21 * (2 ** (6 * g + 51))) / 125.0
    na = 2 ** (24 + 6 * g)
    return nf, na

class SerialThread(QThread):
    data_received_signal = pyqtSignal(list, bytes) 

    def __init__(self):
        super().__init__()
        self.serial_port = serial.Serial()
        self.is_running = False
        self.port_name = ""
        self.buffer = bytearray()
        nf, na = calculate_coeffs(0)
        self.current_coeffs = [nf, nf, na, na] 

    def open_port(self, port_name):
        self.port_name = port_name
        try:
            self.serial_port.port = self.port_name
            self.serial_port.baudrate = BAUD_RATE
            self.serial_port.timeout = 0.1
            self.serial_port.open()
            self.is_running = True
            self.start()
            return True
        except Exception as e:
            print(f"串口打开失败: {e}")
            return False

    def close_port(self):
        self.is_running = False
        if self.serial_port.is_open:
            self.serial_port.close()

    def send_data(self, data_bytes):
        if self.serial_port.is_open:
            try:
                self.serial_port.write(data_bytes)
                print(f"发送({len(data_bytes)}字节): {data_bytes.hex().upper()}")
            except Exception as e:
                print(f"发送失败: {e}")

    def update_frequency_gain(self, g):
        nf, na = calculate_coeffs(g)
        self.current_coeffs = [nf, nf, na, na]
        print(f"更新系数 g={g}: NF={nf:.2e}, NA={na:.2e}")

    def run(self):
        while self.is_running:
            try:
                if self.serial_port.in_waiting:
                    raw = self.serial_port.read(self.serial_port.in_waiting)
                    self.buffer.extend(raw)
                    while len(self.buffer) >= FRAME_LEN:
                        if self.buffer[0] == 0xAA:
                            frame = self.buffer[0:FRAME_LEN]
                            try:
                                val1 = int.from_bytes(frame[3:14], byteorder='big', signed=False)
                                val2 = int.from_bytes(frame[14:25], byteorder='big', signed=False)
                                val3 = int.from_bytes(frame[25:32], byteorder='big', signed=False)
                                val4 = int.from_bytes(frame[32:39], byteorder='big', signed=False)
                                processed_data = [
                                    val1 / self.current_coeffs[0],
                                    val2 / self.current_coeffs[1],
                                    val3 / self.current_coeffs[2],
                                    val4 / self.current_coeffs[3]
                                ]
                                self.data_received_signal.emit(processed_data, bytes(frame))
                            except Exception as e:
                                print(f"解析错误: {e}")
                            del self.buffer[0:FRAME_LEN]
                        else:
                            del self.buffer[0]
                else:
                    time.sleep(0.001)
            except Exception as e:
                print(f"接收线程错误: {e}")
                self.is_running = False

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serial_Scope_SJTU")
        self.resize(1280, 850)
        self.plot_len = 1000 
        self.data_ch1 = deque([0]*self.plot_len, maxlen=self.plot_len)
        self.data_ch2 = deque([0]*self.plot_len, maxlen=self.plot_len)
        self.data_ch3 = deque([0]*self.plot_len, maxlen=self.plot_len)
        self.data_ch4 = deque([0]*self.plot_len, maxlen=self.plot_len)
        self.temp_buffer = [] 
        self.save_file = None 
        self.is_saving = False
        self.serial_thread = SerialThread()
        self.serial_thread.data_received_signal.connect(self.on_data_received)
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50) 
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setMaximumWidth(380) 
        control_panel = QWidget()
        control_layout = QVBoxLayout()
        control_panel.setLayout(control_layout)
        scroll_area.setWidget(control_panel)
        gb_serial = QGroupBox("串口连接")
        serial_layout = QVBoxLayout()
        self.combo_ports = QComboBox()
        self.btn_refresh = QPushButton("刷新")
        self.btn_connect = QPushButton("打开串口")
        self.btn_connect.setCheckable(True)
        h_serial = QHBoxLayout()
        h_serial.addWidget(self.combo_ports)
        h_serial.addWidget(self.btn_refresh)
        serial_layout.addLayout(h_serial)
        serial_layout.addWidget(self.btn_connect)
        gb_serial.setLayout(serial_layout)
        gb_freq = QGroupBox("采样频率设置")
        freq_layout = QVBoxLayout()
        self.combo_freq = QComboBox()
        self.combo_freq.addItems(FREQ_G_MAP.keys())
        self.btn_set_freq = QPushButton("更新计算系数")
        freq_layout.addWidget(QLabel("选择频率:"))
        freq_layout.addWidget(self.combo_freq)
        freq_layout.addWidget(self.btn_set_freq)
        gb_freq.setLayout(freq_layout)
        gb_send = QGroupBox("参数配置 (HEX)")
        send_layout = QVBoxLayout()
        grid_inputs = QGridLayout()
        grid_inputs.setSpacing(8)
        self.user_inputs = [] 
        fixed_vals = {
            1: 0x24, 3: 0x25, 5: 0x26, 7: 0x27, 9: 0x28, 11: 0x29, 
            13: 0x2A, 15: 0x2B, 17: 0x2C, 19: 0x2D, 21: 0x2E
        }
        MAX_COLS = 5 
        self.send_structure = [] 
        style_fixed = """
            QLabel {
                background-color: #E0E0E0; 
                border: 1px solid #A0A0A0; 
                border-radius: 4px;
                color: #333333;
                font-family: Consolas, Monospace;
                font-weight: bold;
                font-size: 14px;
            }
        """
        style_input = """
            QLineEdit {
                background-color: #FFFFFF; 
                border: 1px solid #A0A0A0; 
                border-radius: 4px;
                color: #000000;
                font-family: Consolas, Monospace;
                font-size: 14px;
            }
        """
        for i in range(25):
            row = i // MAX_COLS
            col = i % MAX_COLS
            cell_layout = QVBoxLayout()
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.setSpacing(2)
            lbl_title = QLabel(f"B{i+1}")
            lbl_title.setAlignment(Qt.AlignCenter)
            lbl_title.setStyleSheet("color: #666666; font-size: 10px;")
            if i in fixed_vals:
                val = fixed_vals[i]
                widget = QLabel(f"{val:02X}")
                widget.setStyleSheet(style_fixed)
                self.send_structure.append(val)
            else:
                widget = QLineEdit("00")
                widget.setMaxLength(2)
                widget.setStyleSheet(style_input)
                self.send_structure.append(widget)
                self.user_inputs.append(widget)
            widget.setFixedSize(45, 30)
            widget.setAlignment(Qt.AlignCenter)
            cell_layout.addWidget(lbl_title)
            cell_layout.addWidget(widget)
            cell_layout.setAlignment(Qt.AlignCenter)
            grid_inputs.addLayout(cell_layout, row, col)
        send_layout.addLayout(grid_inputs)
        self.btn_send = QPushButton("发送配置")
        self.btn_send.setMinimumHeight(35)
        send_layout.addWidget(self.btn_send)
        gb_send.setLayout(send_layout)
        gb_save = QGroupBox("数据保存")
        save_layout = QHBoxLayout()
        self.btn_save_start = QPushButton("保存")
        self.btn_save_stop = QPushButton("停止保存")
        self.btn_save_stop.setEnabled(False) 
        save_layout.addWidget(self.btn_save_start)
        save_layout.addWidget(self.btn_save_stop)
        gb_save.setLayout(save_layout)
        gb_raw = QGroupBox("接收数据 (Hex, 40 Bytes)")
        raw_layout = QVBoxLayout()
        self.txt_raw_display = QPlainTextEdit()
        self.txt_raw_display.setReadOnly(True)
        self.txt_raw_display.setMaximumHeight(120) 
        self.txt_raw_display.setFont(QFont("Consolas", 9)) 
        raw_layout.addWidget(self.txt_raw_display)
        gb_raw.setLayout(raw_layout)
        control_layout.addWidget(gb_serial)
        control_layout.addWidget(gb_freq)
        control_layout.addWidget(gb_save)
        control_layout.addWidget(gb_raw)
        control_layout.addWidget(gb_send)
        control_layout.addStretch()
        self.graph_widget = pg.GraphicsLayoutWidget()
        self.graph_widget.setBackground('k') 
        self.plot1 = self.graph_widget.addPlot(title="CH1"); self.curve1 = self.plot1.plot(pen='y')
        self.graph_widget.nextRow()
        self.plot2 = self.graph_widget.addPlot(title="CH2"); self.curve2 = self.plot2.plot(pen='g')
        self.graph_widget.nextRow()
        self.plot3 = self.graph_widget.addPlot(title="CH3"); self.curve3 = self.plot3.plot(pen='c')
        self.graph_widget.nextRow()
        self.plot4 = self.graph_widget.addPlot(title="CH4"); self.curve4 = self.plot4.plot(pen='m')
        main_layout.addWidget(scroll_area)
        main_layout.addWidget(self.graph_widget)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect.clicked.connect(self.toggle_serial)
        self.btn_set_freq.clicked.connect(self.update_coeff_only)
        self.btn_send.clicked.connect(self.construct_and_send)
        self.btn_save_start.clicked.connect(self.start_save)
        self.btn_save_stop.clicked.connect(self.stop_save)
        self.refresh_ports()

    def refresh_ports(self):
        self.combo_ports.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_ports.addItem(p.device)

    def toggle_serial(self):
        if self.btn_connect.isChecked():
            port = self.combo_ports.currentText()
            if not port: return
            if self.serial_thread.open_port(port):
                self.btn_connect.setText("关闭")
                self.btn_connect.setStyleSheet("background-color: lightgreen")
                self.update_coeff_only()
            else:
                self.btn_connect.setChecked(False)
                QMessageBox.critical(self, "错误", "无法打开串口")
        else:
            self.stop_save()
            self.serial_thread.close_port()
            self.btn_connect.setText("打开")
            self.btn_connect.setStyleSheet("")

    def update_coeff_only(self):
        freq_name = self.combo_freq.currentText()
        g = FREQ_G_MAP[freq_name]
        self.serial_thread.update_frequency_gain(g)

    def construct_and_send(self):
        if not self.serial_thread.is_running:
            QMessageBox.warning(self, "警告", "请先打开串口")
            return
        packet = bytearray()
        try:
            for item in self.send_structure:
                if isinstance(item, int): packet.append(item)
                else:
                    text = item.text().strip()
                    val = int(text, 16) if text else 0
                    if not (0 <= val <= 255): raise ValueError
                    packet.append(val)
            self.serial_thread.send_data(packet)
            self.update_coeff_only()
        except Exception as e:
            QMessageBox.critical(self, "错误", f"发送出错: {e}")

    def on_data_received(self, data, raw_bytes):
        self.temp_buffer.append((data, raw_bytes))

    def update_plot(self):
        if not self.temp_buffer:
            return
        new_processed = [x[0] for x in self.temp_buffer] 
        new_raw = [x[1] for x in self.temp_buffer]       
        self.temp_buffer = []
        new_processed_np = np.array(new_processed)
        self.data_ch1.extend(new_processed_np[:, 0])
        self.data_ch2.extend(new_processed_np[:, 1])
        self.data_ch3.extend(new_processed_np[:, 2])
        self.data_ch4.extend(new_processed_np[:, 3])
        self.curve1.setData(np.array(self.data_ch1))
        self.curve2.setData(np.array(self.data_ch2))
        self.curve3.setData(np.array(self.data_ch3))
        self.curve4.setData(np.array(self.data_ch4))
        display_chunk = new_raw[-5:] 
        text_lines = []
        for frame in display_chunk:
            hex_str = " ".join([f"{b:02X}" for b in frame])
            text_lines.append(hex_str)
        self.txt_raw_display.setPlainText("\n".join(text_lines))
        if self.is_saving and self.save_file:
            try:
                for row_data in new_processed:
                    line = f"{row_data[0]:.8f}, {row_data[1]:.8f}, {row_data[2]:.8f}, {row_data[3]:.8f}\n"
                    self.save_file.write(line)
            except Exception as e:
                print(f"写入文件失败: {e}")
                self.stop_save()
    def start_save(self):
        filename, _ = QFileDialog.getSaveFileName(self, "保存数据", "", "Text Files (*.txt);;All Files (*)")
        if filename:
            try:
                self.save_file = open(filename, 'w')
                self.is_saving = True
                self.save_file.write("CH1,CH2,CH3,CH4\n")
                self.btn_save_start.setEnabled(False)
                self.btn_save_start.setText("保存中...")
                self.btn_save_stop.setEnabled(True)
                self.btn_save_stop.setStyleSheet("background-color: #ffcccc") 
            except Exception as e:
                QMessageBox.critical(self, "错误", f"无法创建文件:\n{e}")
    def stop_save(self):
        self.is_saving = False
        if self.save_file:
            try:
                self.save_file.close()
            except:
                pass
            self.save_file = None
        self.btn_save_start.setEnabled(True)
        self.btn_save_start.setText("保存")
        self.btn_save_stop.setEnabled(False)
        self.btn_save_stop.setStyleSheet("")

    def closeEvent(self, event):
        self.stop_save()
        super().closeEvent(event)
if __name__ == "__main__":
    try:
        try:
            from PyQt5.QtCore import Qt
            QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
            QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
        except:
            pass
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        import traceback
        traceback.print_exc()
        input("程序出错，按回车键退出...")