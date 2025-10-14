# fpga_integrated_tool_v0.2.py

import sys
import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QComboBox, QLabel, QLineEdit, QTextEdit, QFormLayout,
    QMessageBox, QSizePolicy, QTabWidget, QGroupBox, QCheckBox, QStackedWidget
)
from PyQt6.QtGui import QPainter, QPixmap, QPen
from PyQt6.QtCore import Qt, QPoint, QThread, pyqtSignal
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
import traceback
from collections import deque

# --- Constants ---
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 400
NUM_SAMPLES_SEND = 512
BAUD_RATE = 115200
RECEIVE_BLOCK_SIZE = 64
OSCILLOSCOPE_WIDTH = 512

# 设置中文字体以解决 UserWarning
try:
    import matplotlib.pyplot as plt
    plt.rcParams['font.sans-serif'] = ['SimHei'] # or 'Microsoft YaHei'
    plt.rcParams['axes.unicode_minus'] = False
except Exception as e:
    print(f"Warning: Could not set Chinese font. Matplotlib plots may not display Chinese characters correctly. Error: {e}")

# =============================================================================
# 复用的辅助类
# =============================================================================

class WaveformDrawer(QWidget):
    # ... (代码未改变，保持原样)
    """用于绘制波形的自定义控件"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(CANVAS_WIDTH, CANVAS_HEIGHT)
        self.canvas = QPixmap(self.size())
        self.canvas.fill(Qt.GlobalColor.white)
        self.points = []

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drawing = True
            self.last_point = event.pos()
            self.points.append(self.last_point)

    def mouseMoveEvent(self, event):
        if hasattr(self, 'drawing') and self.drawing:
            painter = QPainter(self.canvas)
            pen = QPen(Qt.GlobalColor.blue, 2, Qt.PenStyle.SolidLine)
            painter.setPen(pen)
            painter.drawLine(self.last_point, event.pos())
            self.last_point = event.pos()
            self.points.append(self.last_point)
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drawing = False

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawPixmap(QPoint(), self.canvas)

    def clear_canvas(self):
        self.canvas.fill(Qt.GlobalColor.white)
        self.points = []
        self.update()

    def get_waveform_data(self):
        if not self.points or len(self.points) < 2: return None
        sorted_points = sorted(self.points, key=lambda p: p.x())
        x_coords = [p.x() for p in sorted_points]
        y_coords = [p.y() for p in sorted_points]
        f = interp1d(x_coords, y_coords, kind='linear', bounds_error=False, fill_value=(y_coords[0], y_coords[-1]))
        x_new = np.linspace(min(x_coords), max(x_coords), num=NUM_SAMPLES_SEND)
        y_new = f(x_new)
        return (CANVAS_HEIGHT - y_new - CANVAS_HEIGHT / 2) / (CANVAS_HEIGHT / 2)

class MplCanvas(FigureCanvas):
    # ... (代码未改变，保持原样)
    """嵌入式matplotlib控件"""
    def __init__(self, parent=None, width=8, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)
        self.setParent(parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)


class ReceiverThread(QThread):
    # ... (代码未改变，保持原样)
    """后台串口数据接收线程"""
    data_received = pyqtSignal(list)
    error_occurred = pyqtSignal(str)

    def __init__(self, port, baudrate, parent=None):
        super().__init__(parent)
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None
        self._is_running = True

    def run(self):
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=1)
        except serial.SerialException as e:
            self.error_occurred.emit(f"无法打开串口 {self.port}: {e}")
            return

        while self._is_running:
            try:
                if self.serial_connection and self.serial_connection.in_waiting >= RECEIVE_BLOCK_SIZE:
                    data_bytes = self.serial_connection.read(RECEIVE_BLOCK_SIZE)
                    if data_bytes:
                        self.data_received.emit(list(data_bytes))
            except serial.SerialException as e:
                self.error_occurred.emit(f"串口读取错误: {e}")
                self._is_running = False
            except Exception as e:
                self.error_occurred.emit(f"处理数据时发生未知错误: {e}")
                self._is_running = False
        
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()

    def stop(self):
        self._is_running = False
        self.wait(2000)

# =============================================================================
# 集成化的主应用窗口
# =============================================================================

class FPGAIntegratedTool(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FPGA 集成通信工具 v1.1")
        self.setGeometry(100, 100, 900, 850)

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # 创建三个主功能选项卡
        self.waveform_sender_tab = QWidget()
        self.oscilloscope_tab = QWidget()
        self.protocol_tab = QWidget()
        
        self.tabs.addTab(self.waveform_sender_tab, "波形绘制与发送")
        self.tabs.addTab(self.oscilloscope_tab, "示波器")
        self.tabs.addTab(self.protocol_tab, "协议调试器")

        # 初始化每个选项卡的UI
        self.init_waveform_sender_tab()
        self.init_oscilloscope_tab()
        self.init_protocol_tab()

        self.receiver_thread = None
        self.refresh_all_ports()

    # --- 1. 波形绘制与发送选项卡 ---
    def init_waveform_sender_tab(self):
        main_layout = QVBoxLayout(self.waveform_sender_tab)
        sender_group = QGroupBox("波形绘制与发送")
        sender_layout = QVBoxLayout()

        sender_layout.addWidget(QLabel("1. 在下方白色区域绘制波形:"))
        self.drawer = WaveformDrawer()
        sender_layout.addWidget(self.drawer, alignment=Qt.AlignmentFlag.AlignCenter)
        
        sender_layout.addWidget(QLabel("2. 预览将要发送的波形:"))
        self.plot_canvas_sender = MplCanvas(self, width=8, height=3, dpi=100)
        sender_layout.addWidget(self.plot_canvas_sender, alignment=Qt.AlignmentFlag.AlignCenter)
        self.plot_sender_data(None)

        controls_layout = QHBoxLayout()
        controls_layout.addWidget(QLabel("发送串口:"))
        self.serial_ports_combo_sender_wf = QComboBox()
        self.refresh_button_wf1 = QPushButton("刷新")
        self.clear_button = QPushButton("清除画布")
        self.analyze_button = QPushButton("分析并发送")
        
        controls_layout.addWidget(self.serial_ports_combo_sender_wf, 1)
        controls_layout.addWidget(self.refresh_button_wf1)
        controls_layout.addStretch()
        controls_layout.addWidget(self.clear_button)
        controls_layout.addWidget(self.analyze_button)
        sender_layout.addLayout(controls_layout)
        sender_group.setLayout(sender_layout)

        main_layout.addWidget(sender_group)
        main_layout.addStretch() # 添加伸缩，使布局更紧凑

        # 连接信号
        self.refresh_button_wf1.clicked.connect(self.refresh_all_ports)
        self.clear_button.clicked.connect(self.clear_waveform_canvas)
        self.analyze_button.clicked.connect(self.analyze_and_send_waveform)

    # --- 2. 示波器选项卡 ---
    def init_oscilloscope_tab(self):
        main_layout = QVBoxLayout(self.oscilloscope_tab)
        receiver_group = QGroupBox("实时波形接收")
        receiver_layout = QVBoxLayout()
        
        self.plot_canvas_receiver = MplCanvas(self, width=8, height=6, dpi=100) # 增加了高度
        receiver_layout.addWidget(self.plot_canvas_receiver, alignment=Qt.AlignmentFlag.AlignCenter)
        self.data_buffer = deque([0] * OSCILLOSCOPE_WIDTH, maxlen=OSCILLOSCOPE_WIDTH)
        self.plot_receiver_data()

        self.status_label_receiver = QLabel("请选择串口并开始接收。")
        receiver_layout.addWidget(self.status_label_receiver)

        rx_controls_layout = QHBoxLayout()
        rx_controls_layout.addWidget(QLabel("接收串口:"))
        self.serial_ports_combo_receiver = QComboBox()
        self.refresh_button_wf2 = QPushButton("刷新")
        self.start_button = QPushButton("开始接收")
        self.stop_button = QPushButton("停止接收")
        self.stop_button.setEnabled(False)

        rx_controls_layout.addWidget(self.serial_ports_combo_receiver, 1)
        rx_controls_layout.addWidget(self.refresh_button_wf2)
        rx_controls_layout.addStretch()
        rx_controls_layout.addWidget(self.start_button)
        rx_controls_layout.addWidget(self.stop_button)
        receiver_layout.addLayout(rx_controls_layout)
        receiver_group.setLayout(receiver_layout)

        main_layout.addWidget(receiver_group)

        # 连接信号
        self.refresh_button_wf2.clicked.connect(self.refresh_all_ports)
        self.start_button.clicked.connect(self.start_receiving)
        self.stop_button.clicked.connect(self.stop_receiving)

    # --- 3. 协议调试器选项卡 ---
    def init_protocol_tab(self):
        # ... (代码未改变，保持原样)
        main_layout = QVBoxLayout(self.protocol_tab)
        main_layout.setSpacing(15)

        serial_group = QGroupBox("1. 串口配置")
        serial_layout = QHBoxLayout()
        serial_layout.addWidget(QLabel("串口选择:"))
        self.serial_ports_combo_protocol = QComboBox()
        serial_layout.addWidget(self.serial_ports_combo_protocol, 1)
        self.refresh_button_protocol = QPushButton("刷新列表")
        serial_layout.addWidget(self.refresh_button_protocol)
        serial_group.setLayout(serial_layout)
        
        protocol_group = QGroupBox("2. 协议与参数配置")
        protocol_group_layout = QVBoxLayout()
        protocol_layout = QHBoxLayout()
        protocol_layout.addWidget(QLabel("协议选择:"))
        self.protocol_combo = QComboBox()
        self.protocol_combo.addItems(["PWM", "I2C", "SPI", "UART", "CAN"])
        protocol_layout.addWidget(self.protocol_combo, 1)
        protocol_group_layout.addLayout(protocol_layout)
        self.stacked_widget = QStackedWidget()
        self.init_protocol_pages()
        protocol_group_layout.addWidget(self.stacked_widget)
        protocol_group.setLayout(protocol_group_layout)

        action_layout = QHBoxLayout()
        self.send_protocol_button = QPushButton("构建并发送命令")
        self.clear_logs_button = QPushButton("清空日志")
        action_layout.addWidget(self.send_protocol_button)
        action_layout.addWidget(self.clear_logs_button)
        
        display_group = QGroupBox("3. 数据监视")
        display_layout = QHBoxLayout()
        send_display_layout = QVBoxLayout()
        send_display_layout.addWidget(QLabel("待发送数据 (Hex):"))
        self.sent_data_display = QTextEdit()
        self.sent_data_display.setReadOnly(True); self.sent_data_display.setFontFamily("Courier New")
        send_display_layout.addWidget(self.sent_data_display)
        recv_display_layout = QVBoxLayout()
        recv_display_layout.addWidget(QLabel("接收数据 (Hex):"))
        self.received_data_display = QTextEdit()
        self.received_data_display.setReadOnly(True); self.received_data_display.setFontFamily("Courier New")
        recv_display_layout.addWidget(self.received_data_display)
        display_layout.addLayout(send_display_layout)
        display_layout.addLayout(recv_display_layout)
        display_group.setLayout(display_layout)
        
        main_layout.addWidget(serial_group)
        main_layout.addWidget(protocol_group)
        main_layout.addLayout(action_layout)
        main_layout.addWidget(display_group)

        # 连接协议调试器的信号
        self.refresh_button_protocol.clicked.connect(self.refresh_all_ports)
        self.protocol_combo.currentIndexChanged.connect(lambda: self.stacked_widget.setCurrentIndex(self.protocol_combo.currentIndex()))
        self.send_protocol_button.clicked.connect(self.construct_and_send_frame)
        self.clear_logs_button.clicked.connect(lambda: (self.sent_data_display.clear(), self.received_data_display.clear()))


    def init_protocol_pages(self):
        # ... (代码未改变，保持原样)
        page_pwm = QWidget(); layout = QFormLayout(page_pwm); self.pwm_inputs = []
        for i in range(1, 5):
            psc = QLineEdit("1000"); arr = QLineEdit("255"); duty = QLineEdit("128")
            layout.addRow(f"--- 通道 {i} ---", None)
            layout.addRow(f"PSC Ch{i} (8-byte):", psc); layout.addRow(f"ARR Ch{i} (8-byte):", arr); layout.addRow(f"Duty Ch{i} (4-byte):", duty)
            self.pwm_inputs.append({'psc': psc, 'arr': arr, 'duty': duty})
        self.stacked_widget.addWidget(page_pwm)

        page_i2c = QWidget(); layout = QFormLayout(page_i2c)
        self.i2c_addr_width = QComboBox(); self.i2c_addr_width.addItems(["8-bit", "16-bit"])
        self.i2c_device_addr = QLineEdit("68"); self.i2c_reg_addr = QLineEdit("01"); self.i2c_data_to_write = QLineEdit("AA BB CC"); self.i2c_read_len = QLineEdit("4")
        layout.addRow("寄存器地址宽度:", self.i2c_addr_width); layout.addRow("设备地址 (Hex):", self.i2c_device_addr); layout.addRow("寄存器地址 (Hex):", self.i2c_reg_addr); layout.addRow("写入数据 (Hex):", self.i2c_data_to_write); layout.addRow("期望读取长度 (Bytes):", self.i2c_read_len)
        self.stacked_widget.addWidget(page_i2c)

        page_spi = QWidget(); layout = QFormLayout(page_spi)
        self.spi_tx_only = QCheckBox("仅发送数据"); self.spi_data_to_write = QLineEdit("9F"); self.spi_read_len = QLineEdit("5")
        layout.addRow(self.spi_tx_only); layout.addRow("写入数据 (Hex):", self.spi_data_to_write); layout.addRow("期望读取长度 (Bytes):", self.spi_read_len)
        self.stacked_widget.addWidget(page_spi)
        
        page_uart = QWidget(); layout = QFormLayout(page_uart)
        self.uart_data_to_write = QLineEdit("DE AD BE EF"); layout.addRow("发送数据 (Hex):", self.uart_data_to_write)
        self.stacked_widget.addWidget(page_uart)

        page_can = QWidget(); layout = QFormLayout(page_can); layout.addRow("CAN 数据 (Hex):", QLineEdit("..."))
        self.stacked_widget.addWidget(page_can)


    # --- 4. 共享和核心逻辑 ---
    def refresh_all_ports(self):
        """刷新应用内所有的串口下拉列表"""
        ports = serial.tools.list_ports.comports()
        # **重要**: 确保所有串口下拉框都被包括
        combo_boxes = [self.serial_ports_combo_sender_wf, self.serial_ports_combo_receiver, self.serial_ports_combo_protocol]
        for combo in combo_boxes:
            current_data = combo.currentData() # 保存当前选择
            combo.clear()
        
        if not ports:
            for combo in combo_boxes: combo.addItem("未找到串口")
        else:
            for port in ports:
                for combo in combo_boxes: combo.addItem(f"{port.device} - {port.description}", port.device)
            # 尝试恢复之前的选择
            for combo in combo_boxes:
                index = combo.findData(current_data)
                if index != -1:
                    combo.setCurrentIndex(index)
        self.statusBar().showMessage("串口列表已刷新")

    def show_error(self, message):
        QMessageBox.critical(self, "错误", message)

    def closeEvent(self, event):
        self.stop_receiving()
        event.accept()

    # --- 5. 波形绘制与发送功能 ---
    def clear_waveform_canvas(self):
        # ... (代码未改变，保持原样)
        self.drawer.clear_canvas()
        self.plot_sender_data(None)
        self.statusBar().showMessage("画布已清除")

    def analyze_and_send_waveform(self):
        # ... (代码未改变，保持原样)
        try:
            waveform = self.drawer.get_waveform_data()
            if waveform is None:
                self.show_error("画布为空或数据不足，无法分析。"); return
            
            if len(waveform) > 10:
                ws = min(21, len(waveform) // 4); ws += 1 if ws % 2 == 0 else 0
                if ws > 3: waveform = savgol_filter(waveform, ws, 3)

            quantized = ((waveform - np.min(waveform)) / (np.max(waveform) - np.min(waveform))) * 255
            quantized_waveform = np.clip(quantized, 0, 255).astype(int)
            self.plot_sender_data(quantized_waveform, 'samples')
            
            data_to_send = "S," + ",".join(map(str, quantized_waveform)) + "\n"
            port_name = self.serial_ports_combo_sender_wf.currentData()
            if not port_name: self.show_error("请选择一个有效的发送串口！"); return
            
            with serial.Serial(port_name, BAUD_RATE, timeout=2, write_timeout=2) as ser:
                ser.write(data_to_send.encode('ascii'))
            self.statusBar().showMessage(f"波形数据已通过 {port_name} 成功发送！")
        except Exception as e:
            self.show_error(f"分析发送波形时出错: {e}"); traceback.print_exc()

    def plot_sender_data(self, data, plot_type='none'):
        # ... (代码未改变，保持原样)
        self.plot_canvas_sender.axes.clear()
        if plot_type == 'samples' and data is not None:
            self.plot_canvas_sender.axes.plot(data)
            self.plot_canvas_sender.axes.set_title("发送波形预览 (量化后)"); self.plot_canvas_sender.axes.set_xlabel("采样点"); self.plot_canvas_sender.axes.set_ylabel("数值 (0-255)"); self.plot_canvas_sender.axes.grid(True)
        else:
            self.plot_canvas_sender.axes.set_title("等待绘制..."); self.plot_canvas_sender.axes.text(0.5, 0.5, '此处显示预览', ha='center', va='center', transform=self.plot_canvas_sender.axes.transAxes)
        self.plot_canvas_sender.draw()

    # --- 6. 示波器功能 ---
    def start_receiving(self):
        # ... (代码未改变，保持原样)
        port_name = self.serial_ports_combo_receiver.currentData()
        if not port_name: self.show_error("请选择一个有效的接收串口！"); return
        if self.receiver_thread and self.receiver_thread.isRunning(): self.show_error("接收线程已在运行！"); return
        self.start_button.setEnabled(False); self.stop_button.setEnabled(True); self.serial_ports_combo_receiver.setEnabled(False)
        self.status_label_receiver.setText(f"正在从 {port_name} 接收数据...")
        self.receiver_thread = ReceiverThread(port=port_name, baudrate=BAUD_RATE)
        self.receiver_thread.data_received.connect(self.update_plot)
        self.receiver_thread.error_occurred.connect(self.on_receiver_error)
        self.receiver_thread.start()

    def stop_receiving(self):
        # ... (代码未改变，保持原样)
        if self.receiver_thread and self.receiver_thread.isRunning(): self.receiver_thread.stop(); self.receiver_thread = None
        self.start_button.setEnabled(True); self.stop_button.setEnabled(False); self.serial_ports_combo_receiver.setEnabled(True)
        self.status_label_receiver.setText("接收已停止。")

    def update_plot(self, data_list):
        # ... (代码未改变，保持原样)
        self.data_buffer.extend(data_list)
        self.plot_receiver_data()
    
    def on_receiver_error(self, message):
        # ... (代码未改变，保持原样)
        self.show_error(message)
        self.stop_receiving()

    def plot_receiver_data(self):
        # ... (代码未改变，保持原样)
        self.plot_canvas_receiver.axes.clear()
        self.plot_canvas_receiver.axes.plot(list(self.data_buffer))
        self.plot_canvas_receiver.axes.set_title("实时示波器"); self.plot_canvas_receiver.axes.set_ylim(0, 260); self.plot_canvas_receiver.axes.set_xlim(0, OSCILLOSCOPE_WIDTH); self.plot_canvas_receiver.axes.set_xlabel("时间 (采样点)"); self.plot_canvas_receiver.axes.set_ylabel("幅值 (0-255)"); self.plot_canvas_receiver.axes.grid(True)
        self.plot_canvas_receiver.draw()

    # --- 7. 协议调试器功能 ---
    def construct_and_send_frame(self):
        # ... (代码未改变，保持原样)
        try:
            protocol = self.protocol_combo.currentText()
            frame = self.build_protocol_frame(protocol)
            if frame:
                self.sent_data_display.setText(' '.join(f'{b:02X}' for b in frame))
                self.statusBar().showMessage(f"正在发送 {len(frame)} 字节...")
                response = self.send_and_receive_protocol_data(frame)
                if response:
                    self.received_data_display.setText(' '.join(f'{b:02X}' for b in response))
                    self.statusBar().showMessage(f"成功发送并接收到 {len(response)} 字节")
                else:
                    self.received_data_display.setText(""); self.statusBar().showMessage("发送成功，但未收到响应或响应超时")
        except ValueError as e: self.show_error(f"输入格式错误: {e}")
        except Exception as e: self.show_error(f"发生未知错误: {e}\n\n{traceback.format_exc()}")

    def build_protocol_frame(self, protocol):
        # ... (代码未改变，保持原样)
        frame = bytearray([0x20, 0x25]); selection_byte = 0; payload = bytearray()
        if protocol == "PWM":
            selection_byte |= 0b011
            psc_bytes, arr_bytes, duty_bytes = bytearray(), bytearray(), bytearray()
            for i in range(4):
                psc = int(self.pwm_inputs[i]['psc'].text().strip()); arr = int(self.pwm_inputs[i]['arr'].text().strip()); duty = int(self.pwm_inputs[i]['duty'].text().strip())
                psc_bytes.extend(psc.to_bytes(8, 'big')); arr_bytes.extend(arr.to_bytes(8, 'big')); duty_bytes.extend(duty.to_bytes(4, 'big'))
            payload.extend(psc_bytes); payload.extend(arr_bytes); payload.extend(duty_bytes)
        elif protocol == "I2C":
            selection_byte |= 0b001
            if self.i2c_addr_width.currentText() == "16-bit": selection_byte |= (1 << 4)
            dev_addr = int(self.i2c_device_addr.text().strip(), 16); payload.append(dev_addr)
            reg_addr = int(self.i2c_reg_addr.text().strip(), 16); addr_bytes = 2 if self.i2c_addr_width.currentText() == "16-bit" else 1
            payload.extend(reg_addr.to_bytes(addr_bytes, 'big'))
            data_str = self.i2c_data_to_write.text().strip()
            if data_str: payload.extend(bytes.fromhex(data_str.replace(' ', '')))
            read_len = int(self.i2c_read_len.text().strip()); payload.append(read_len)
        elif protocol == "SPI":
            selection_byte |= 0b010
            if self.spi_tx_only.isChecked(): selection_byte |= (1 << 3)
            data_str = self.spi_data_to_write.text().strip()
            if data_str: payload.extend(bytes.fromhex(data_str.replace(' ', '')))
            if not self.spi_tx_only.isChecked():
                read_len = int(self.spi_read_len.text().strip())
                if read_len > 0: payload.extend(b'\x00' * read_len)
        elif protocol == "UART":
            selection_byte |= 0b000
            data_str = self.uart_data_to_write.text().strip()
            if data_str: payload.extend(bytes.fromhex(data_str.replace(' ', '')))
        else: self.show_error(f"{protocol} 协议的帧格式未定义！"); return None
        frame.append(selection_byte); frame.extend(payload); frame.extend(b'\r\n')
        return frame

    def send_and_receive_protocol_data(self, data):
        # ... (代码未改变，保持原样)
        port_name = self.serial_ports_combo_protocol.currentData()
        if not port_name: self.show_error("请选择一个有效的串口！"); return None
        try:
            with serial.Serial(port_name, baudrate=BAUD_RATE, timeout=1) as ser:
                ser.write(data); return ser.read_until(b'\r\n')
        except serial.SerialException as e: self.show_error(f"串口通信错误: {e}"); return None

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = FPGAIntegratedTool()
    window.show()
    sys.exit(app.exec())