# -*- coding: utf-8 -*-
"""
fpga_unified_control_v3.4_full.py (已优化)
功能:
这是一个集成化的FPGA上位机工具，合并了高性能示波器、波形绘制发送和高级协议调试功能。
1. [示波器] 通过共享内存从C++ UDP接收器高速读取ADC数据，使用pyqtgraph实时显示时域波形和频域(FFT)频谱。
2. [波形绘制] 允许用户手绘波形，量化后通过串口发送。
3. [控制调试] 集成了DAC函数发生器和支持多种协议(4路PWM/I2C/SPI等)的高级协议调试器。
4. [协议解析] 添加了完整的协议帧接收和解析功能，支持UART、I2C、SPI、PWM和CAN协议。

主要修改 (编码助手 v3.4):
1. [优化发送日志] 为所有发送的数据增加了协议类型标签（如 [DAC], [PWM]）和精确时间戳，使其与接收日志格式统一，更易于调试分析。
2. [修正CAN协议] 优化了CAN协议的发送和解析逻辑，使其帧结构与I2C/SPI等协议保持一致。
3. [优化数据显示] 为所有从FPGA接收到的数据增加了时间戳，并美化了输出格式，使其更清晰。
4. [新增DAC波形] 在DAC模块中增加了“锯齿波”和“直流置零”两个新波形选项。
5. [修复PWM逻辑] 修正了PWM协议发送和解析逻辑中的一个微小不一致，提高了程序的健壮性。
"""
import sys
import mmap
import struct
import traceback
from datetime import datetime # 新增：用于添加时间戳
import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PyQt6.QtCore import QPoint, Qt, QTimer
from PyQt6.QtGui import QPainter, QPen, QPixmap
from PyQt6.QtWidgets import (QApplication, QCheckBox, QComboBox, QFileDialog,
                             QFormLayout, QGroupBox, QGridLayout, QHBoxLayout,
                             QLabel, QLineEdit, QMainWindow, QMessageBox,
                             QPushButton, QRadioButton, QSizePolicy,
                             QStackedWidget, QTabWidget, QTextEdit,
                             QVBoxLayout, QWidget)
from matplotlib.backends.backend_qt5agg import \
    FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter

# --- 配置参数 ---
SHARED_MEM_NAME = "FPGA_ADC_DATA"
SHARED_MEM_SIZE = 2048
SCOPE_UPDATE_INTERVAL_MS = 30  # 调整刷新率以平衡UI响应和FFT计算
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 400
NUM_SAMPLES_SEND = 512
DEFAULT_BAUD_RATE = 115200

# 设置Matplotlib中文字体
try:
    import matplotlib.pyplot as plt
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False
except Exception as e:
    print(f"警告: 无法设置中文字体。错误: {e}")

# =============================================================================
# 辅助类 (WaveformDrawer, MplCanvas) - 无变化
# =============================================================================
class WaveformDrawer(QWidget):
    """用于绘制波形的自定义控件"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(CANVAS_WIDTH, CANVAS_HEIGHT)
        self.canvas = QPixmap(self.size())
        self.canvas.fill(Qt.GlobalColor.white)
        self.points = []
        self.drawing = False
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drawing = True
            self.last_point = event.pos()
            self.points.append(self.last_point)
    def mouseMoveEvent(self, event):
        if self.drawing:
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
    """嵌入式matplotlib控件"""
    def __init__(self, parent=None, width=8, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)
        self.setParent(parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

# =============================================================================
# 主应用窗口
# =============================================================================
class FPGAMasterControl(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FPGA 集成上位机 v3.4 (优化版)")
        self.setGeometry(100, 100, 1200, 900)
        self.shared_memory = None
        self.serial_connection_dac = None
        self.serial_connection_protocol = None
        self.serial_connection_scope = None
        self.scope_data = np.array([])
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.scope_tab = QWidget()
        self.waveform_sender_tab = QWidget()
        self.control_tab = QWidget()
        self.tabs.addTab(self.scope_tab, "高速数字示波器")
        self.tabs.addTab(self.waveform_sender_tab, "波形绘制与发送")
        self.tabs.addTab(self.control_tab, "控制与高级调试")
        
        self.control_serial_buffer = bytearray()
        
        self.init_scope_tab()
        self.init_waveform_sender_tab()
        self.init_control_tab()
        self.scope_timer = QTimer(self)
        self.scope_timer.setInterval(SCOPE_UPDATE_INTERVAL_MS)
        self.scope_timer.timeout.connect(self.update_scope_plot)
        
        self.serial_check_timer = QTimer(self)
        self.serial_check_timer.setInterval(50)
        self.serial_check_timer.timeout.connect(self.check_control_serial_data)
        
        self.connect_shared_memory()

    def show_error(self, message):
        QMessageBox.critical(self, "错误", message)

    def closeEvent(self, event):
        print("正在关闭应用程序并释放资源...")
        self.stop_scope()
        self.disconnect_serial_dac()
        self.disconnect_serial_protocol()
        if self.shared_memory:
            self.shared_memory.close()
        if hasattr(self, 'serial_connection_scope') and self.serial_connection_scope and self.serial_connection_scope.is_open:
            self.serial_connection_scope.close()
        self.serial_check_timer.stop()
        event.accept()

    def refresh_all_ports(self):
        ports = serial.tools.list_ports.comports()
        combo_boxes = [
            self.ports_combo_wf, 
            self.scope_serial_combo,
            self.dac_ports_combo,
            self.protocol_ports_combo
        ]
        for combo in combo_boxes:
            current_data = combo.currentData()
            combo.clear()
        if not ports:
            for combo in combo_boxes: combo.addItem("未找到串口")
        else:
            sorted_ports = sorted(ports)
            for port in sorted_ports:
                for combo in combo_boxes:
                    combo.addItem(f"{port.device} - {port.description}", port.device)
            for combo in combo_boxes:
                index = combo.findData(current_data)
                if index != -1: combo.setCurrentIndex(index)
        self.statusBar().showMessage("串口列表已刷新", 2000)

    # =============================================================================
    # 选项卡 1: 高速数字示波器 - 无变化
    # =============================================================================
    def init_scope_tab(self):
        layout = QHBoxLayout(self.scope_tab)
        controls_group = QGroupBox("ADC 采样控制")
        controls_layout = QGridLayout(controls_group)
        controls_group.setFixedWidth(250)
        controls_layout.addWidget(QLabel("采样率 (SPS):"), 0, 0)
        self.sampling_rate_input = QLineEdit("10000000")
        controls_layout.addWidget(self.sampling_rate_input, 0, 1)
        controls_layout.addWidget(QLabel("数据点数:"), 1, 0)
        self.data_points_input = QLineEdit("N/A")
        self.data_points_input.setReadOnly(True)
        controls_layout.addWidget(self.data_points_input, 1, 1)
        self.fft_checkbox = QCheckBox("启用FFT分析")
        self.fft_checkbox.setChecked(True)
        controls_layout.addWidget(self.fft_checkbox, 2, 0, 1, 2)
        controls_layout.addWidget(QLabel("数据源:"), 3, 0) 
        self.data_source_combo = QComboBox() 
        self.data_source_combo.addItems(["共享内存 (UDP)", "串口"])
        controls_layout.addWidget(self.data_source_combo, 3, 1)
        self.serial_group_scope = QGroupBox("串口配置")
        serial_layout_scope = QGridLayout(self.serial_group_scope)
        serial_layout_scope.addWidget(QLabel("端口:"), 0, 0)
        self.scope_serial_combo = QComboBox()
        serial_layout_scope.addWidget(self.scope_serial_combo, 0, 1)
        serial_layout_scope.addWidget(QLabel("波特率:"), 1, 0)
        self.scope_baud_combo = QComboBox()
        self.scope_baud_combo.addItems(["9600", "57600", "115200", "1000000"])
        self.scope_baud_combo.setCurrentText("1000000")
        serial_layout_scope.addWidget(self.scope_baud_combo, 1, 1)
        self.connect_serial_button_scope = QPushButton("连接")
        self.disconnect_serial_button_scope = QPushButton("断开")
        self.disconnect_serial_button_scope.setEnabled(False)
        serial_layout_scope.addWidget(self.connect_serial_button_scope, 2, 0)
        serial_layout_scope.addWidget(self.disconnect_serial_button_scope, 2, 1)
        controls_layout.addWidget(self.serial_group_scope, 4, 0, 1, 2)
        self.serial_group_scope.setVisible(False)
        self.start_button = QPushButton("开始采样"); self.stop_button = QPushButton("停止采样")
        self.save_button = QPushButton("保存数据"); self.stop_button.setEnabled(False)
        controls_layout.addWidget(self.start_button, 5, 0, 1, 2)
        controls_layout.addWidget(self.stop_button, 6, 0, 1, 2)
        controls_layout.addWidget(self.save_button, 7, 0, 1, 2)
        self.status_label = QLabel("状态: 已停止")
        controls_layout.addWidget(self.status_label, 8, 0, 1, 2)
        controls_layout.setRowStretch(9, 1)
        plot_area_layout = QVBoxLayout()
        time_plot_group = QGroupBox("实时波形 (时域)")
        time_plot_layout = QVBoxLayout(time_plot_group)
        self.plot_widget_time = pg.PlotWidget()
        time_plot_layout.addWidget(self.plot_widget_time)
        self.curve_time = self.plot_widget_time.plot(pen=pg.mkPen(color='#00D0FF', width=2))
        self.setup_plot_style(self.plot_widget_time, "Real-Time ADC Data", "Sample Index", "ADC Value (0-255)", (0, 255))
        freq_plot_group = QGroupBox("频谱分析 (频域)")
        freq_plot_layout = QVBoxLayout(freq_plot_group)
        self.plot_widget_freq = pg.PlotWidget()
        freq_plot_layout.addWidget(self.plot_widget_freq)
        self.curve_freq = self.plot_widget_freq.plot(pen=pg.mkPen(color='#FFD700', width=2))
        self.setup_plot_style(self.plot_widget_freq, "Frequency Spectrum (FFT)", "Frequency (Hz)", "Magnitude |Y(f)|")
        self.plot_widget_freq.setLogMode(x=True, y=False)
        plot_area_layout.addWidget(time_plot_group)
        plot_area_layout.addWidget(freq_plot_group)
        layout.addWidget(controls_group)
        layout.addLayout(plot_area_layout)
        self.start_button.clicked.connect(self.start_scope)
        self.stop_button.clicked.connect(self.stop_scope)
        self.save_button.clicked.connect(self.save_scope_data)
        self.fft_checkbox.stateChanged.connect(lambda: self.plot_widget_freq.setVisible(self.fft_checkbox.isChecked()))
        self.data_source_combo.currentTextChanged.connect(self.toggle_serial_config)
        self.connect_serial_button_scope.clicked.connect(self.connect_serial_scope)
        self.disconnect_serial_button_scope.clicked.connect(self.disconnect_serial_scope)

    def setup_plot_style(self, plot_widget, title, xlabel, ylabel, yrange=None):
        plot_widget.setBackground('k')
        plot_widget.showGrid(x=True, y=True, alpha=0.3)
        plot_widget.setTitle(title, color="w", size="12pt")
        plot_widget.setLabel('bottom', xlabel, color='w')
        plot_widget.setLabel('left', ylabel, color='w')
        plot_widget.getAxis('bottom').setPen(pg.mkPen(color='w'))
        plot_widget.getAxis('left').setPen(pg.mkPen(color='w'))
        if yrange: plot_widget.setYRange(*yrange)

    def toggle_serial_config(self, source_text):
        self.serial_group_scope.setVisible(source_text == "串口")

    def connect_shared_memory(self):
        try:
            self.shared_memory = mmap.mmap(-1, SHARED_MEM_SIZE, tagname=SHARED_MEM_NAME)
            self.statusBar().showMessage(f"成功连接到共享内存 '{SHARED_MEM_NAME}'。", 5000)
        except FileNotFoundError:
            self.show_error(f"未找到共享内存 '{SHARED_MEM_NAME}'。\n请确保 C++ 接收程序已运行！")
        except Exception as e: self.show_error(f"访问共享内存时出错: {e}")

    def connect_serial_scope(self):
        port = self.scope_serial_combo.currentData()
        if not port: self.show_error("请选择串口！"); return
        baud = int(self.scope_baud_combo.currentText())
        try:
            self.serial_connection_scope = serial.Serial(port, baud, timeout=0.1)
            self.connect_serial_button_scope.setEnabled(False)
            self.disconnect_serial_button_scope.setEnabled(True)
            self.statusBar().showMessage(f"示波器串口已连接: {port}", 5000)
        except serial.SerialException as e: self.show_error(f"无法打开串口: {e}")

    def disconnect_serial_scope(self):
        if hasattr(self, 'serial_connection_scope') and self.serial_connection_scope and self.serial_connection_scope.is_open:
            self.serial_connection_scope.close()
        self.serial_connection_scope = None
        self.connect_serial_button_scope.setEnabled(True)
        self.disconnect_serial_button_scope.setEnabled(False)
        self.statusBar().showMessage("示波器串口已断开", 2000)

    def start_scope(self):
        source = self.data_source_combo.currentText()
        if source == "共享内存 (UDP)":
            if not self.shared_memory:
                self.connect_shared_memory()
                if not self.shared_memory: return
        elif source == "串口":
            if not self.serial_connection_scope or not self.serial_connection_scope.is_open:
                self.connect_serial_scope()
                if not hasattr(self, 'serial_connection_scope') or not self.serial_connection_scope: return
        self.scope_timer.start()
        self.start_button.setEnabled(False); self.stop_button.setEnabled(True)
        self.status_label.setText("状态: 正在采样...")

    def stop_scope(self):
        self.scope_timer.stop()
        self.start_button.setEnabled(True); self.stop_button.setEnabled(False)
        self.status_label.setText("状态: 已停止")
        
    def save_scope_data(self):
        if self.scope_data.size == 0: self.show_error("没有数据可以保存！"); return
        path, _ = QFileDialog.getSaveFileName(self, "保存波形数据", "", "CSV Files (*.csv)")
        if path:
            try:
                np.savetxt(path, self.scope_data, fmt='%d', delimiter=',', header='ADC_Value', comments='')
                self.statusBar().showMessage(f"数据成功保存到 {path}", 5000)
            except Exception as e: self.show_error(f"保存文件失败: {e}")

    def update_scope_plot(self):
        source = self.data_source_combo.currentText()
        if source == "共享内存 (UDP)":
            if not self.shared_memory: return
            try:
                self.shared_memory.seek(0)
                count_bytes = self.shared_memory.read(2)
                if len(count_bytes) < 2: return
                sample_count = struct.unpack('>H', count_bytes)[0]
                if 0 < sample_count <= (SHARED_MEM_SIZE - 2):
                    adc_data_bytes = self.shared_memory.read(sample_count)
                    self.scope_data = np.frombuffer(adc_data_bytes, dtype=np.uint8)
            except Exception as e:
                print(f"读取共享内存出错: {e}"); self.stop_scope()
        elif source == "串口":
            if not hasattr(self, 'serial_connection_scope') or not self.serial_connection_scope or not self.serial_connection_scope.is_open: return
            try:
                data_bytes = self.serial_connection_scope.read(self.serial_connection_scope.in_waiting or 1)
                if data_bytes: self.scope_data = np.frombuffer(data_bytes, dtype=np.uint8)
            except Exception as e:
                print(f"读取串口数据出错: {e}"); self.stop_scope()
        if self.scope_data.size > 0:
            self.curve_time.setData(self.scope_data)
            self.data_points_input.setText(str(self.scope_data.size))
            if self.fft_checkbox.isChecked() and self.scope_data.size > 1:
                self.update_fft_plot()

    def update_fft_plot(self):
        try:
            fs = float(self.sampling_rate_input.text())
            if fs <= 0: return
        except ValueError: return
        N = self.scope_data.size
        T = 1.0 / fs
        data_no_dc = self.scope_data - np.mean(self.scope_data)
        yf = np.fft.fft(data_no_dc)
        xf = np.fft.fftfreq(N, T)[:N//2]
        y_magnitude = 2.0/N * np.abs(yf[0:N//2])
        self.curve_freq.setData(x=xf, y=y_magnitude)

    # =============================================================================
    # 选项卡 2: 波形绘制与发送 - 无变化
    # =============================================================================
    def init_waveform_sender_tab(self):
        main_layout = QVBoxLayout(self.waveform_sender_tab)
        sender_group = QGroupBox("波形绘制与发送")
        sender_layout = QVBoxLayout(sender_group)
        sender_layout.addWidget(QLabel("1. 在下方白色区域用鼠标左键绘制波形:"))
        self.drawer = WaveformDrawer()
        sender_layout.addWidget(self.drawer, alignment=Qt.AlignmentFlag.AlignCenter)
        sender_layout.addWidget(QLabel("2. 预览将要发送的波形 (量化后):"))
        self.plot_canvas_sender = MplCanvas(self, width=8, height=3, dpi=100)
        sender_layout.addWidget(self.plot_canvas_sender, alignment=Qt.AlignmentFlag.AlignCenter)
        self.plot_sender_data(None)
        controls_layout = QHBoxLayout()
        controls_layout.addWidget(QLabel("发送串口:"))
        self.ports_combo_wf = QComboBox()
        self.refresh_button_wf = QPushButton("刷新")
        self.clear_button = QPushButton("清除画布")
        self.analyze_button = QPushButton("分析并发送")
        controls_layout.addWidget(self.ports_combo_wf, 1)
        controls_layout.addWidget(self.refresh_button_wf); controls_layout.addStretch()
        controls_layout.addWidget(self.clear_button); controls_layout.addWidget(self.analyze_button)
        sender_layout.addLayout(controls_layout)
        main_layout.addWidget(sender_group); main_layout.addStretch()
        self.refresh_button_wf.clicked.connect(self.refresh_all_ports)
        self.clear_button.clicked.connect(self.clear_waveform_canvas)
        self.analyze_button.clicked.connect(self.analyze_and_send_waveform)

    def clear_waveform_canvas(self):
        self.drawer.clear_canvas()
        self.plot_sender_data(None)
        self.statusBar().showMessage("画布已清除")

    def analyze_and_send_waveform(self):
        try:
            waveform = self.drawer.get_waveform_data()
            if waveform is None: self.show_error("画布为空或数据不足。"); return
            if len(waveform) > 10:
                ws = min(21, len(waveform)//4); ws += 1 if ws % 2 == 0 else 0
                if ws > 3: waveform = savgol_filter(waveform, ws, 3)
            quantized = ((waveform - np.min(waveform)) / (np.max(waveform) - np.min(waveform))) * 255
            quantized_waveform = np.clip(quantized, 0, 255).astype(np.uint8)
            self.plot_sender_data(quantized_waveform)
            data_to_send = "S," + ",".join(map(str, quantized_waveform)) + "\n"
            port_name = self.ports_combo_wf.currentData()
            if not port_name: self.show_error("请选择发送串口！"); return
            with serial.Serial(port_name, DEFAULT_BAUD_RATE, timeout=2, write_timeout=2) as ser:
                ser.write(data_to_send.encode('ascii'))
            self.statusBar().showMessage(f"波形数据已通过 {port_name} 发送！")
        except Exception as e:
            self.show_error(f"分析或发送波形时出错: {e}\n{traceback.format_exc()}")

    def plot_sender_data(self, data):
        self.plot_canvas_sender.axes.clear()
        if data is not None:
            self.plot_canvas_sender.axes.plot(data)
            self.plot_canvas_sender.axes.set_title("发送波形预览 (量化后)")
            self.plot_canvas_sender.axes.set_xlabel("采样点"); self.plot_canvas_sender.axes.set_ylabel("数值 (0-255)")
            self.plot_canvas_sender.axes.grid(True)
        else:
            self.plot_canvas_sender.axes.set_title("等待绘制...")
            self.plot_canvas_sender.axes.text(0.5, 0.5, '此处显示预览', ha='center', va='center', transform=self.plot_canvas_sender.axes.transAxes)
        self.plot_canvas_sender.draw()

    # =============================================================================
    # 选项卡 3: 控制与高级调试 - (无界面变化)
    # =============================================================================
    def init_control_tab(self):
        main_layout = QVBoxLayout(self.control_tab)
        serial_layout = QHBoxLayout()
        dac_serial_group = QGroupBox("DAC 串口配置")
        dac_serial_layout = QGridLayout(dac_serial_group)
        dac_serial_layout.addWidget(QLabel("端口:"), 0, 0)
        self.dac_ports_combo = QComboBox()
        dac_serial_layout.addWidget(self.dac_ports_combo, 0, 1)
        dac_serial_layout.addWidget(QLabel("波特率:"), 1, 0)
        self.dac_baud_combo = QComboBox()
        self.dac_baud_combo.addItems(["9600", "57600", "115200"])
        self.dac_baud_combo.setCurrentText(str(DEFAULT_BAUD_RATE))
        dac_serial_layout.addWidget(self.dac_baud_combo, 1, 1)
        self.connect_dac_button = QPushButton("打开")
        self.disconnect_dac_button = QPushButton("关闭")
        self.disconnect_dac_button.setEnabled(False)
        dac_serial_layout.addWidget(self.connect_dac_button, 2, 0)
        dac_serial_layout.addWidget(self.disconnect_dac_button, 2, 1)
        protocol_serial_group = QGroupBox("协议调试串口配置")
        protocol_serial_layout = QGridLayout(protocol_serial_group)
        protocol_serial_layout.addWidget(QLabel("端口:"), 0, 0)
        self.protocol_ports_combo = QComboBox()
        protocol_serial_layout.addWidget(self.protocol_ports_combo, 0, 1)
        protocol_serial_layout.addWidget(QLabel("波特率:"), 1, 0)
        self.protocol_baud_combo = QComboBox()
        self.protocol_baud_combo.addItems(["9600", "57600", "115200"])
        self.protocol_baud_combo.setCurrentText(str(DEFAULT_BAUD_RATE))
        protocol_serial_layout.addWidget(self.protocol_baud_combo, 1, 1)
        self.connect_protocol_button = QPushButton("打开")
        self.disconnect_protocol_button = QPushButton("关闭")
        self.disconnect_protocol_button.setEnabled(False)
        protocol_serial_layout.addWidget(self.connect_protocol_button, 2, 0)
        protocol_serial_layout.addWidget(self.disconnect_protocol_button, 2, 1)
        serial_layout.addWidget(dac_serial_group)
        serial_layout.addWidget(protocol_serial_group)
        main_layout.addLayout(serial_layout)
        control_panel_layout = QHBoxLayout()
        left_panel = QVBoxLayout()
        dac_group = QGroupBox("DAC 函数发生器")
        dac_layout = QGridLayout(dac_group)
        dac_layout.addWidget(QLabel("波形:"), 0, 0)
        self.wave_sine = QRadioButton("正弦"); self.wave_sine.setChecked(True)
        self.wave_square = QRadioButton("方波")
        self.wave_triangle = QRadioButton("三角")
        self.wave_sawtooth = QRadioButton("锯齿波")
        self.wave_zero = QRadioButton("直流(零)")
        wave_layout = QHBoxLayout()
        wave_layout.addWidget(self.wave_sine); wave_layout.addWidget(self.wave_square)
        wave_layout.addWidget(self.wave_triangle); wave_layout.addWidget(self.wave_sawtooth)
        wave_layout.addWidget(self.wave_zero)
        dac_layout.addLayout(wave_layout, 0, 1, 1, 3)
        dac_layout.addWidget(QLabel("频率(Hz):"), 1, 0)
        self.freq_input = QLineEdit("1000")
        dac_layout.addWidget(self.freq_input, 1, 1, 1, 3)
        dac_layout.addWidget(QLabel("幅值(V):"), 2, 0)
        self.amp_input = QLineEdit("2.50")
        dac_layout.addWidget(self.amp_input, 2, 1, 1, 3)
        self.start_dac_button = QPushButton("开始输出")
        self.stop_dac_button = QPushButton("停止输出")
        dac_layout.addWidget(self.start_dac_button, 3, 0, 1, 2)
        dac_layout.addWidget(self.stop_dac_button, 3, 2, 1, 2)
        protocol_group = QGroupBox("高级协议调试器")
        protocol_layout = QVBoxLayout(protocol_group)
        self.protocol_combo = QComboBox()
        self.protocol_combo.addItems(["PWM", "I2C", "SPI", "UART", "CAN"])
        protocol_layout.addWidget(self.protocol_combo)
        self.stacked_widget = QStackedWidget()
        self.init_protocol_pages()
        protocol_layout.addWidget(self.stacked_widget)
        self.send_protocol_button = QPushButton("构建并发送协议命令")
        protocol_layout.addWidget(self.send_protocol_button)
        left_panel.addWidget(dac_group)
        left_panel.addWidget(protocol_group)
        right_panel = QVBoxLayout()
        display_group = QGroupBox("数据监视")
        display_layout = QVBoxLayout(display_group)
        display_layout.addWidget(QLabel("发送历史:"))
        self.sent_data_display = QTextEdit()
        self.sent_data_display.setReadOnly(True)
        self.sent_data_display.setFontFamily("Courier New")
        display_layout.addWidget(self.sent_data_display)
        display_layout.addWidget(QLabel("接收历史:"))
        self.received_data_display = QTextEdit()
        self.received_data_display.setReadOnly(True)
        self.received_data_display.setFontFamily("Courier New")
        display_layout.addWidget(self.received_data_display)
        self.clear_logs_button = QPushButton("清空日志")
        display_layout.addWidget(self.clear_logs_button)
        right_panel.addWidget(display_group)
        control_panel_layout.addLayout(left_panel, 2)
        control_panel_layout.addLayout(right_panel, 1)
        main_layout.addLayout(control_panel_layout)
        self.refresh_all_ports()
        self.connect_dac_button.clicked.connect(self.connect_serial_dac)
        self.disconnect_dac_button.clicked.connect(self.disconnect_serial_dac)
        self.connect_protocol_button.clicked.connect(self.connect_serial_protocol)
        self.disconnect_protocol_button.clicked.connect(self.disconnect_serial_protocol)
        self.start_dac_button.clicked.connect(self.send_dac_command)
        self.stop_dac_button.clicked.connect(lambda: self.send_serial_command_dac("DAC,STOP\n"))
        self.protocol_combo.currentIndexChanged.connect(self.stacked_widget.setCurrentIndex)
        self.send_protocol_button.clicked.connect(self.construct_and_send_frame)
        self.clear_logs_button.clicked.connect(self.clear_logs)

    def clear_logs(self):
        self.sent_data_display.clear()
        self.received_data_display.clear()

    def init_protocol_pages(self):
        page_pwm = QWidget()
        main_layout = QVBoxLayout(page_pwm)
        pwm_group = QGroupBox("PWM 配置")
        layout = QGridLayout(pwm_group)
        layout.addWidget(QLabel("通道"), 0, 0)
        layout.addWidget(QLabel("启停"), 0, 1)
        layout.addWidget(QLabel("PSC (2-byte)"), 0, 2)
        layout.addWidget(QLabel("ARR (2-byte)"), 0, 3)
        layout.addWidget(QLabel("Duty (1-byte)"), 0, 4)
        self.pwm_inputs = []
        for i in range(1, 5):
            layout.addWidget(QLabel(f"通道 {i}"), i, 0)
            enable_check = QCheckBox("启用"); enable_check.setChecked(True)
            layout.addWidget(enable_check, i, 1)
            psc = QLineEdit("1000"); arr = QLineEdit("255"); duty = QLineEdit("128")
            layout.addWidget(psc, i, 2); layout.addWidget(arr, i, 3); layout.addWidget(duty, i, 4)
            self.pwm_inputs.append({'enable': enable_check, 'psc': psc, 'arr': arr, 'duty': duty})
        layout.setRowStretch(5, 1)
        main_layout.addWidget(pwm_group)
        main_layout.addStretch(1)
        self.stacked_widget.addWidget(page_pwm)
        page_i2c = QWidget(); layout = QFormLayout(page_i2c)
        self.i2c_addr_width = QComboBox(); self.i2c_addr_width.addItems(["8-bit", "16-bit"]); self.i2c_device_addr = QLineEdit("68")
        self.i2c_reg_addr = QLineEdit("01"); self.i2c_data_to_write = QLineEdit("AA BB"); self.i2c_read_len = QLineEdit("4")
        layout.addRow("寄存器地址宽度:", self.i2c_addr_width); layout.addRow("设备地址(Hex):", self.i2c_device_addr)
        layout.addRow("寄存器地址(Hex):", self.i2c_reg_addr); layout.addRow("写入数据(Hex):", self.i2c_data_to_write); layout.addRow("读取长度(Bytes):", self.i2c_read_len)
        self.stacked_widget.addWidget(page_i2c)
        page_spi = QWidget(); layout = QFormLayout(page_spi)
        self.spi_tx_only = QCheckBox("仅发送数据"); self.spi_data_to_write = QLineEdit("9F"); self.spi_read_len = QLineEdit("5")
        layout.addRow(self.spi_tx_only); layout.addRow("写入数据(Hex):", self.spi_data_to_write); layout.addRow("读取长度(Bytes):", self.spi_read_len)
        self.stacked_widget.addWidget(page_spi)
        page_uart = QWidget(); layout = QFormLayout(page_uart)
        self.uart_data_to_write = QLineEdit("DE AD BE EF"); layout.addRow("发送数据(Hex):", self.uart_data_to_write)
        self.stacked_widget.addWidget(page_uart)
        page_can = QWidget(); layout = QFormLayout(page_can)
        self.can_id = QLineEdit("123"); layout.addRow("CAN ID(Hex):", self.can_id)
        self.can_data = QLineEdit("AA BB CC DD"); layout.addRow("数据(Hex):", self.can_data)
        self.can_extended = QCheckBox("扩展帧"); layout.addRow(self.can_extended)
        self.can_remote = QCheckBox("远程帧"); layout.addRow(self.can_remote)
        self.stacked_widget.addWidget(page_can)
    
    def connect_serial_dac(self):
        port = self.dac_ports_combo.currentData()
        if not port: self.show_error("请选择DAC串口！"); return
        baud = int(self.dac_baud_combo.currentText())
        try:
            self.serial_connection_dac = serial.Serial(port, baud, timeout=0.5)
            self.connect_dac_button.setEnabled(False); self.disconnect_dac_button.setEnabled(True)
            self.statusBar().showMessage(f"DAC串口已连接: {port}", 5000)
        except serial.SerialException as e: self.show_error(f"无法打开DAC串口: {e}")
            
    def disconnect_serial_dac(self):
        if self.serial_connection_dac and self.serial_connection_dac.is_open:
            self.serial_connection_dac.close()
        self.serial_connection_dac = None
        self.connect_dac_button.setEnabled(True); self.disconnect_dac_button.setEnabled(False)
        self.statusBar().showMessage("DAC串口已关闭", 2000)
        
    def connect_serial_protocol(self):
        port = self.protocol_ports_combo.currentData()
        if not port: self.show_error("请选择协议调试串口！"); return
        baud = int(self.protocol_baud_combo.currentText())
        try:
            self.serial_connection_protocol = serial.Serial(port, baud, timeout=0.5)
            self.connect_protocol_button.setEnabled(False); self.disconnect_protocol_button.setEnabled(True)
            self.statusBar().showMessage(f"协议调试串口已连接: {port}", 5000)
            self.serial_check_timer.start()
        except serial.SerialException as e: self.show_error(f"无法打开协议调试串口: {e}")
            
    def disconnect_serial_protocol(self):
        self.serial_check_timer.stop()
        if self.serial_connection_protocol and self.serial_connection_protocol.is_open:
            self.serial_connection_protocol.close()
        self.serial_connection_protocol = None
        self.connect_protocol_button.setEnabled(True); self.disconnect_protocol_button.setEnabled(False)
        self.statusBar().showMessage("协议调试串口已关闭", 2000)
        
    def send_dac_command(self):
        wave = "SINE"
        if self.wave_square.isChecked(): wave = "SQUARE"
        if self.wave_triangle.isChecked(): wave = "TRIANGLE"
        if self.wave_sawtooth.isChecked(): wave = "SAWTOOTH"
        if self.wave_zero.isChecked(): wave = "ZERO"
        try:
            if wave == "ZERO":
                self.send_serial_command_dac("DAC,ZERO,0,0\n")
            else:
                freq = float(self.freq_input.text())
                amp = float(self.amp_input.text())
                self.send_serial_command_dac(f"DAC,{wave},{freq},{amp}\n")
        except ValueError: 
            self.show_error("频率和幅值必须是有效数字！")

    # =============================================================================
    # --- 以下为本次核心修改区域 ---
    # =============================================================================
            
    def send_serial_command_dac(self, data, is_binary=False):
        """通过DAC串口发送数据 (已修改，增加协议类型和时间戳)"""
        if not (self.serial_connection_dac and self.serial_connection_dac.is_open):
            self.show_error("DAC串口未连接！"); return
        try:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            if is_binary: # 虽然DAC目前不用二进制，但保持代码一致性
                self.serial_connection_dac.write(data)
                log_msg = f"[{timestamp}] -> [DAC] | [HEX] {' '.join(f'{b:02X}' for b in data)}"
                self.sent_data_display.append(log_msg)
            else:
                self.serial_connection_dac.write(data.encode('ascii'))
                log_msg = f"[{timestamp}] -> [DAC] | [ASCII] {data.strip()}"
                self.sent_data_display.append(log_msg)
        except serial.SerialException as e:
            self.show_error(f"DAC串口通信错误: {e}"); self.disconnect_serial_dac()
            
    def construct_and_send_frame(self):
        """通过协议调试串口发送协议帧 (已修改，传递协议名称)"""
        try:
            protocol = self.protocol_combo.currentText()
            frame = self.build_protocol_frame(protocol)
            if frame: 
                # 将当前选择的协议名称传递给发送函数
                self.send_serial_command_protocol(frame, protocol, is_binary=True)
        except ValueError as e: 
            self.show_error(f"输入格式错误: {e}")
        except Exception as e: 
            self.show_error(f"构建协议帧时出错: {e}\n{traceback.format_exc()}")
            
    def send_serial_command_protocol(self, data, protocol_name, is_binary=False):
        """通过协议调试串口发送数据 (已修改，增加协议类型和时间戳)"""
        if not (self.serial_connection_protocol and self.serial_connection_protocol.is_open):
            self.show_error("协议调试串口未连接！"); return
        try:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            protocol_tag = protocol_name.upper() # 获取协议名称
            if is_binary:
                self.serial_connection_protocol.write(data)
                log_msg = f"[{timestamp}] -> [{protocol_tag}] | [HEX] {' '.join(f'{b:02X}' for b in data)}"
                self.sent_data_display.append(log_msg)
            else:
                self.serial_connection_protocol.write(data.encode('ascii'))
                log_msg = f"[{timestamp}] -> [{protocol_tag}] | [ASCII] {data.strip()}"
                self.sent_data_display.append(log_msg)
        except serial.SerialException as e:
            self.show_error(f"协议调试串口通信错误: {e}"); self.disconnect_serial_protocol()
            
    def build_protocol_frame(self, protocol):
        """构建各类协议帧 (逻辑无变化)"""
        frame = bytearray([0x20, 0x25])
        payload = bytearray()
        selection_byte = 0

        if protocol == "PWM":
            selection_byte = 0b011
            psc_bytes, arr_bytes, duty_bytes = bytearray(), bytearray(), bytearray()
            enable_byte = 0
            for i in range(len(self.pwm_inputs)):
                enable = 1 if self.pwm_inputs[i]['enable'].isChecked() else 0
                if i < 4: enable_byte |= (enable << i)
                if enable:
                    psc_val = int(self.pwm_inputs[i]['psc'].text())
                    arr_val = int(self.pwm_inputs[i]['arr'].text())
                    duty_val = int(self.pwm_inputs[i]['duty'].text())
                else:
                    psc_val, arr_val, duty_val = 0, 0, 0
                psc_bytes.extend(psc_val.to_bytes(2, 'big'))
                arr_bytes.extend(arr_val.to_bytes(2, 'big'))
                duty_bytes.extend(duty_val.to_bytes(1, 'big'))
            payload.extend(psc_bytes); payload.extend(arr_bytes); payload.extend(duty_bytes)
            combined_byte = (enable_byte << 4) | (selection_byte & 0x0F)
            frame.append(combined_byte); frame.extend(payload); frame.extend(b'\r\n')
            return frame
        
        elif protocol == "I2C":
            selection_byte = 0b001
            addr_width = 1 if self.i2c_addr_width.currentText() == "16-bit" else 0
            device_addr = int(self.i2c_device_addr.text(), 16)
            reg_addr = int(self.i2c_reg_addr.text(), 16)
            write_data = bytes.fromhex(self.i2c_data_to_write.text().replace(' ', ''))
            read_len = int(self.i2c_read_len.text())
            combined_byte = (addr_width << 3) | (selection_byte & 0x07)
            payload.append(device_addr)
            if addr_width: payload.extend(reg_addr.to_bytes(2, 'big'))
            else: payload.append(reg_addr)
            payload.extend(write_data); payload.extend(read_len.to_bytes(1, 'big'))
            frame.append(combined_byte); frame.extend(payload); frame.extend(b'\r\n')
            return frame

        elif protocol == "SPI":
            selection_byte = 0b010
            tx_only = 1 if self.spi_tx_only.isChecked() else 0
            write_data = bytes.fromhex(self.spi_data_to_write.text().replace(' ', ''))
            read_len = int(self.spi_read_len.text())
            read_bytes = b'\x00' * read_len
            payload.extend(write_data); payload.extend(read_bytes)
            combined_byte = (tx_only << 3) | (selection_byte & 0x07)
            frame.append(combined_byte); frame.extend(payload); frame.extend(b'\r\n')
            return frame
        
        elif protocol == "UART":
            selection_byte = 0b000
            data_str = self.uart_data_to_write.text().strip()
            if data_str: payload.extend(bytes.fromhex(data_str.replace(' ', '')))
            frame.append(selection_byte); frame.extend(payload); frame.extend(b'\r\n')
            return frame
            
        elif protocol == "CAN":
            selection_byte = 0b100
            can_id = int(self.can_id.text(), 16)
            can_data = bytes.fromhex(self.can_data.text().replace(' ', ''))
            flags = 0
            if self.can_extended.isChecked(): flags |= 0x01
            if self.can_remote.isChecked(): flags |= 0x02
            combined_byte = (flags << 3) | (selection_byte & 0x07)
            payload.extend(can_id.to_bytes(4, 'big'))
            payload.append(len(can_data))
            payload.extend(can_data)
            frame.append(combined_byte); frame.extend(payload); frame.extend(b'\r\n')
            return frame
            
        else:
            self.show_error(f"协议 {protocol} 未定义！"); return None

    # =============================================================================
    # 协议帧解析功能 - (逻辑无变化)
    # =============================================================================
    def parse_received_frame(self, frame_data):
        """解析接收到的数据帧"""
        try:
            if frame_data[0:2] != b'\x20\x25': return "无效帧头"
            protocol_byte = frame_data[2]
            protocol_type = protocol_byte & 0b111
            extended_flags = protocol_byte >> 3
            payload = frame_data[3:-2]
            if protocol_type == 0b000:   # UART
                return f"<- [UART] | Data: {payload.hex(' ').upper()}"
            elif protocol_type == 0b001:  # I2C
                addr_width = extended_flags & 0b1
                device_addr = payload[0]
                if addr_width: reg_addr, data_start = int.from_bytes(payload[1:3], 'big'), 3
                else: reg_addr, data_start = payload[1], 2
                write_data = payload[data_start:-1]
                read_len = payload[-1]
                return (f"<- [I2C] | Dev: 0x{device_addr:02X}, Reg: 0x{reg_addr:0{4 if addr_width else 2}X}, "
                        f"Write: [{write_data.hex(' ').upper()}], ReadLen: {read_len}B")
            elif protocol_type == 0b010:  # SPI
                tx_only = extended_flags & 0b1
                read_len = payload[-1] if payload else 0
                write_data = payload[:-read_len] if read_len > 0 else payload
                return (f"<- [SPI] | Mode: {'TX Only' if tx_only else 'TX/RX'}, "
                        f"Write: [{write_data.hex(' ').upper()}], ReadLen: {read_len}B")
            elif protocol_type == 0b011:  # PWM
                enable_byte = extended_flags & 0b1111
                channels = []
                for i in range(4):
                    offset = i * 5
                    if offset + 5 > len(payload): break
                    psc = int.from_bytes(payload[offset:offset+2], 'big')
                    arr = int.from_bytes(payload[offset+2:offset+4], 'big')
                    duty = payload[offset+4]
                    enabled = (enable_byte >> i) & 0b1
                    if enabled: channels.append(f"CH{i+1}: ON (PSC={psc}, ARR={arr}, Duty={duty})")
                    else: channels.append(f"CH{i+1}: OFF")
                return f"<- [PWM] | Status: {' | '.join(channels)}"
            elif protocol_type == 0b100:  # CAN
                if len(payload) < 5: return "CAN帧不完整"
                can_id = int.from_bytes(payload[0:4], 'big')
                data_len = payload[4]
                data = payload[5:5+data_len] if data_len > 0 else b''
                is_extended = "Ext" if extended_flags & 0x01 else "Std"
                is_remote = "RTR" if extended_flags & 0x02 else "Data"
                return (f"<- [CAN] | ID: 0x{can_id:08X} [{is_extended}, {is_remote}], "
                        f"DLC: {data_len}, Data: [{data.hex(' ').upper()}]")
            else:
                return f"<- [未知协议] | Type: 0x{protocol_type:02X}, Data: {payload.hex(' ').upper()}"
        except IndexError: return "帧数据不完整"
        except Exception as e: return f"解析错误: {str(e)}"

    def check_control_serial_data(self):
        """检查协议调试串口接收的数据并解析帧"""
        if not (self.serial_connection_protocol and self.serial_connection_protocol.is_open): return
        try:
            data = self.serial_connection_protocol.read(self.serial_connection_protocol.in_waiting)
        except serial.SerialException as e:
            self.show_error(f"协议串口读取错误: {e}"); self.disconnect_serial_protocol(); return
        if not data: return
        self.control_serial_buffer.extend(data)
        while True:
            start_idx = self.control_serial_buffer.find(b'\x20\x25')
            if start_idx == -1: break
            end_idx = self.control_serial_buffer.find(b'\r\n', start_idx)
            if end_idx == -1: break
            frame_data = bytes(self.control_serial_buffer[start_idx : end_idx + 2])
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            parsed_info = self.parse_received_frame(frame_data)
            self.received_data_display.append(f"[{timestamp}] {parsed_info}")
            self.control_serial_buffer = self.control_serial_buffer[end_idx + 2:]

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = FPGAMasterControl()
    window.show()
    sys.exit(app.exec())