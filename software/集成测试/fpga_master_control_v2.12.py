# -*- coding: utf-8 -*-
"""
fpga_unified_control_v3.5_full.py (已修复和优化)
对应cpp_v1

功能:
这是一个集成化的FPGA上位机工具，合并了高性能示波器、波形绘制发送和高级协议调试功能。
1. [示波器] 通过共享内存从C++ UDP接收器高速读取ADC数据，使用pyqtgraph实时显示时域波形和频域(FFT)频谱。
2. [波形绘制] 允许用户手绘波形，量化后通过串口发送。
3. [控制调试] 集成了DAC函数发生器和支持多种协议(4路PWM/I2C/SPI等)的高级协议调试器。
4. [协议解析] 添加了完整的协议帧接收和解析功能，支持UART、I2C、SPI、PWM和CAN协议。

主要修改 (编码助手 v3.9 - DAC协议重构):
1. [重构-DAC协议] 根据新需求，修改了“控制与高级调试”选项卡中“DAC函数发生器”的发送逻辑。
2. [修改-发送帧] “开始输出”按钮现在发送符合新协议 `[0x20 0x25 FF*5 波形+频率 DDR3(4B) \r\n]` 的单个二进制命令帧。
3. [新增-DDR3容量] 在DAC函数发生器UI中，用“DDR3 容量”输入框替换了原有的“幅值”输入框。
4. [修改-UI标签] 将“频率(Hz)”标签改为“频率选择 (0-15)”，以反映新协议的要求。
5. [重构-停止命令] “停止输出”按钮现在会发送一个符合新协议的“直流零电平”命令，以实现一致的控制。
"""
import sys
import time
import mmap
import struct
import traceback
from datetime import datetime
import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PyQt6.QtCore import QPoint, Qt, QTimer
from PyQt6.QtGui import QPainter, QPen, QPixmap
from PyQt6.QtWidgets import (QApplication, QCheckBox, QComboBox, QFileDialog,
                             QFormLayout, QGroupBox, QGridLayout, QHBoxLayout,
                             QLabel, QLineEdit, QMainWindow, QMessageBox,
                             QPushButton, QRadioButton, QSizePolicy, QSlider,
                             QStackedWidget, QTabWidget, QTextEdit,
                             QVBoxLayout, QWidget)
from matplotlib.backends.backend_qt5agg import \
    FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter

# --- 配置参数 ---
SHARED_MEM_NAME = "FPGA_ADC_DATA"
SHARED_MEM_SIZE = 25 * 1024
SCOPE_UPDATE_INTERVAL_MS = 30
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 400
NUM_SAMPLES_SEND = 512
DEFAULT_BAUD_RATE = 115200
TRIGGER_LEVEL_RANGE = (0, 255)
TRIGGER_POSITION_RANGE = (0, 100)
DEFAULT_TRIGGER_LEVEL = 128
DEFAULT_TRIGGER_POSITION = 50
SCOPE_DISPLAY_POINTS = 2048
# 新增: 滚动缓冲区大小，用于处理低频信号。应足够大以容纳几个最低频率波形的周期。
SCOPE_ROLLING_BUFFER_SIZE = 50000

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
        self.setWindowTitle("FPGA 集成上位机 v2.12（最终稳定版）")
        self.setGeometry(100, 100, 1200, 900)
        self.shared_memory = None
        self.serial_connection_dac = None
        self.serial_connection_protocol = None
        self.serial_connection_scope = None
        # self.scope_data 现在是滚动缓冲区
        self.scope_data = np.array([], dtype=np.uint8)
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.scope_tab = QWidget()
        self.waveform_sender_tab = QWidget()
        self.control_tab = QWidget()
        self.tabs.addTab(self.scope_tab, "高速数字示波器")
        self.tabs.addTab(self.waveform_sender_tab, "波形绘制与发送")
        self.tabs.addTab(self.control_tab, "控制与高级调试")
        
        self.trigger_enabled = False
        self.trigger_level = DEFAULT_TRIGGER_LEVEL
        self.trigger_position = DEFAULT_TRIGGER_POSITION
        self.trigger_type = "上升沿"
        self.trigger_line = None
        
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
        controls_group.setFixedWidth(300)

        controls_layout.addWidget(QLabel("采样率 (SPS):"), 0, 0)
        self.sampling_rate_input = QLineEdit("10000000")
        controls_layout.addWidget(self.sampling_rate_input, 0, 1)
        
        controls_layout.addWidget(QLabel("缓冲区点数:"), 1, 0)
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
        
        self.trigger_group = QGroupBox("触发设置")
        trigger_layout = QGridLayout(self.trigger_group)
        
        self.trigger_enable_checkbox = QCheckBox("稳定波形")
        self.trigger_enable_checkbox.setChecked(False)
        trigger_layout.addWidget(self.trigger_enable_checkbox, 0, 0, 1, 2)
        
        trigger_layout.addWidget(QLabel("触发类型:"), 1, 0)
        self.trigger_type_combo = QComboBox()
        self.trigger_type_combo.addItems(["上升沿", "下降沿"])
        trigger_layout.addWidget(self.trigger_type_combo, 1, 1)
        
        trigger_layout.addWidget(QLabel("触发电平:"), 2, 0)
        self.trigger_level_slider = QSlider(Qt.Orientation.Horizontal)
        self.trigger_level_slider.setRange(TRIGGER_LEVEL_RANGE[0], TRIGGER_LEVEL_RANGE[1])
        self.trigger_level_slider.setValue(DEFAULT_TRIGGER_LEVEL)
        trigger_layout.addWidget(self.trigger_level_slider, 2, 1)
        
        self.trigger_level_label = QLabel(f"{DEFAULT_TRIGGER_LEVEL}")
        trigger_layout.addWidget(self.trigger_level_label, 2, 2)
        
        trigger_layout.addWidget(QLabel("触发位置:"), 3, 0)
        self.trigger_position_slider = QSlider(Qt.Orientation.Horizontal)
        self.trigger_position_slider.setRange(TRIGGER_POSITION_RANGE[0], TRIGGER_POSITION_RANGE[1])
        self.trigger_position_slider.setValue(DEFAULT_TRIGGER_POSITION)
        trigger_layout.addWidget(self.trigger_position_slider, 3, 1)
        
        self.trigger_position_label = QLabel(f"{DEFAULT_TRIGGER_POSITION}%")
        trigger_layout.addWidget(self.trigger_position_label, 3, 2)
        
        controls_layout.addWidget(self.trigger_group, 5, 0, 1, 2)
        
        self.start_button = QPushButton("开始采样")
        self.stop_button = QPushButton("停止采样")
        self.save_button = QPushButton("保存数据")
        self.stop_button.setEnabled(False)
        
        controls_layout.addWidget(self.start_button, 6, 0, 1, 2)
        controls_layout.addWidget(self.stop_button, 7, 0, 1, 2)
        controls_layout.addWidget(self.save_button, 8, 0, 1, 2)
        
        self.status_label = QLabel("状态: 已停止")
        controls_layout.addWidget(self.status_label, 9, 0, 1, 2)
        
        controls_layout.setRowStretch(10, 1)
        
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
        
        self.trigger_enable_checkbox.stateChanged.connect(self.toggle_trigger_settings)
        self.trigger_level_slider.valueChanged.connect(self.update_trigger_level)
        self.trigger_position_slider.valueChanged.connect(self.update_trigger_position)
        self.trigger_type_combo.currentIndexChanged.connect(self.update_trigger_type)
        
        self.trigger_level = DEFAULT_TRIGGER_LEVEL
        self.trigger_position = DEFAULT_TRIGGER_POSITION
        self.trigger_type = "上升沿"

    def setup_plot_style(self, plot_widget, title, xlabel, ylabel, yrange=None):
        plot_widget.setBackground('k')
        plot_widget.showGrid(x=True, y=True, alpha=0.3)
        plot_widget.setTitle(title, color="w", size="12pt")
        plot_widget.setLabel('bottom', xlabel, color='w')
        plot_widget.setLabel('left', ylabel, color='w')
        plot_widget.getAxis('bottom').setPen(pg.mkPen(color='w'))
        plot_widget.getAxis('left').setPen(pg.mkPen(color='w'))
        if yrange: plot_widget.setYRange(*yrange)

    def toggle_trigger_settings(self, state):
        self.trigger_enabled = state == Qt.CheckState.Checked.value

    def update_trigger_level(self, value):
        self.trigger_level = value
        self.trigger_level_label.setText(f"{value}")

    def update_trigger_position(self, value):
        self.trigger_position = value
        self.trigger_position_label.setText(f"{value}%")

    def update_trigger_type(self, index):
        self.trigger_type = self.trigger_type_combo.itemText(index)

    def toggle_serial_config(self, source_text):
        self.serial_group_scope.setVisible(source_text == "串口")

    def connect_shared_memory(self):
        try:
            self.shared_memory = mmap.mmap(-1, SHARED_MEM_SIZE, tagname=SHARED_MEM_NAME)
            self.statusBar().showMessage(f"成功连接到共享内存 '{SHARED_MEM_NAME}'。", 5000)
        except FileNotFoundError:
            self.show_error(f"未找到共享内存 '{SHARED_MEM_NAME}'。\n请确保 C++ 接收程序已运行！")
        except Exception as e: 
            self.show_error(f"访问共享内存时出错: {e}")

    def connect_serial_scope(self):
        port = self.scope_serial_combo.currentData()
        if not port: 
            self.show_error("请选择串口！")
            return
        baud = int(self.scope_baud_combo.currentText())
        try:
            self.serial_connection_scope = serial.Serial(port, baud, timeout=0.1)
            self.connect_serial_button_scope.setEnabled(False)
            self.disconnect_serial_button_scope.setEnabled(True)
            self.statusBar().showMessage(f"示波器串口已连接: {port}", 5000)
        except serial.SerialException as e: 
            self.show_error(f"无法打开串口: {e}")

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
                if not self.shared_memory: 
                    return
        elif source == "串口":
            if not self.serial_connection_scope or not self.serial_connection_scope.is_open:
                self.connect_serial_scope()
                if not hasattr(self, 'serial_connection_scope') or not self.serial_connection_scope: 
                    return
        # 清空旧的缓冲区数据
        self.scope_data = np.array([], dtype=np.uint8)
        self.scope_timer.start()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.status_label.setText("状态: 正在采样...")

    def stop_scope(self):
        self.scope_timer.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_label.setText("状态: 已停止")
        
    def save_scope_data(self):
        if self.scope_data.size == 0: 
            self.show_error("没有数据可以保存！")
            return
        path, _ = QFileDialog.getSaveFileName(self, "保存波形数据", "", "CSV Files (*.csv)")
        if path:
            try:
                # 保存当前滚动缓冲区中的数据
                np.savetxt(path, self.scope_data, fmt='%d', delimiter=',', header='ADC_Value', comments='')
                self.statusBar().showMessage(f"数据成功保存到 {path}", 5000)
            except Exception as e: 
                self.show_error(f"保存文件失败: {e}")
                
    def find_trigger_point(self, data):
        if self.trigger_type == "上升沿":
            condition = lambda i: (data[i-1] < self.trigger_level and 
                                   data[i] >= self.trigger_level)
        else:
            condition = lambda i: (data[i-1] > self.trigger_level and 
                                   data[i] <= self.trigger_level)
        
        for i in range(len(data) - 1, 0, -1):
            if condition(i):
                return i
                
        return -1

    def apply_trigger(self, data):
        if not self.trigger_enabled or len(data) < SCOPE_DISPLAY_POINTS:
            return data[-SCOPE_DISPLAY_POINTS:], None
        
        trigger_idx = self.find_trigger_point(data)
        
        if trigger_idx == -1:
            return data[-SCOPE_DISPLAY_POINTS:], None
            
        offset = int(SCOPE_DISPLAY_POINTS * (self.trigger_position / 100.0))
        start_idx = trigger_idx - offset
        
        if start_idx < 0:
            start_idx = 0
        
        end_idx = start_idx + SCOPE_DISPLAY_POINTS
        if end_idx > len(data):
            end_idx = len(data)
            start_idx = max(0, end_idx - SCOPE_DISPLAY_POINTS)
            
        display_data = data[start_idx:end_idx]
        trigger_pos_in_display = trigger_idx - start_idx
        
        return display_data, trigger_pos_in_display

    def update_scope_plot(self):
        new_data = None
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
                    new_data = np.frombuffer(adc_data_bytes, dtype=np.uint8)
            except Exception as e:
                print(f"读取共享内存出错: {e}")
                self.stop_scope()
        elif source == "串口":
            if not hasattr(self, 'serial_connection_scope') or not self.serial_connection_scope or not self.serial_connection_scope.is_open: return
            try:
                data_bytes = self.serial_connection_scope.read(self.serial_connection_scope.in_waiting or 1)
                if data_bytes: 
                    new_data = np.frombuffer(data_bytes, dtype=np.uint8)
            except Exception as e:
                print(f"读取串口数据出错: {e}")
                self.stop_scope()
        
        # [核心修复-滚动缓冲]
        if new_data is not None:
            # 将新数据附加到现有数据后面
            self.scope_data = np.concatenate((self.scope_data, new_data))
            
            # 如果缓冲区超过最大尺寸，则从开头丢弃旧数据，保持其滚动
            if self.scope_data.size > SCOPE_ROLLING_BUFFER_SIZE:
                self.scope_data = self.scope_data[-SCOPE_ROLLING_BUFFER_SIZE:]

        if self.scope_data.size > 0:
            # apply_trigger现在作用于更大的滚动缓冲区上
            display_data, trigger_pos = self.apply_trigger(self.scope_data)
            
            if display_data is not None and len(display_data) > 0:
                self.curve_time.setData(display_data)
            
            # 显示的是滚动缓冲区的总点数
            self.data_points_input.setText(str(self.scope_data.size))
            
            if self.trigger_line:
                self.plot_widget_time.removeItem(self.trigger_line)
                self.trigger_line = None
            
            if self.trigger_enabled and trigger_pos is not None:
                if 0 <= trigger_pos < len(display_data):
                    self.trigger_line = pg.InfiniteLine(
                        pos=trigger_pos, 
                        angle=90, 
                        pen=pg.mkPen(color='r', width=1.5),
                        movable=False
                    )
                    self.plot_widget_time.addItem(self.trigger_line)
            
            if self.fft_checkbox.isChecked() and self.scope_data.size > 1:
                self.update_fft_plot()

    def update_fft_plot(self):
        try:
            fs = float(self.sampling_rate_input.text())
            if fs <= 0: return
        except ValueError: return
        
        # FFT现在也作用于更大的滚动缓冲区，结果更精确
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
        controls_layout.addWidget(self.refresh_button_wf)
        controls_layout.addStretch()
        controls_layout.addWidget(self.clear_button)
        controls_layout.addWidget(self.analyze_button)
        sender_layout.addLayout(controls_layout)
        main_layout.addWidget(sender_group)
        main_layout.addStretch()
        
        self.refresh_button_wf.clicked.connect(self.refresh_all_ports)
        self.clear_button.clicked.connect(self.clear_waveform_canvas)
        self.analyze_button.clicked.connect(self.analyze_and_send_waveform)

    def clear_waveform_canvas(self):
        self.drawer.clear_canvas()
        self.plot_sender_data(None)
        self.statusBar().showMessage("画布已清除")

    def analyze_and_send_waveform(self):
        try:
            # 1. 获取并处理波形数据
            waveform = self.drawer.get_waveform_data()
            if waveform is None: 
                self.show_error("画布为空或数据不足。")
                return

            # 平滑滤波
            if len(waveform) > 10:
                ws = min(21, len(waveform)//4)
                ws += 1 if ws % 2 == 0 else 0
                if ws > 3: 
                    waveform = savgol_filter(waveform, ws, 3)
            
            # 量化为8位数据
            min_val, max_val = np.min(waveform), np.max(waveform)
            if min_val == max_val: # 避免除以零
                quantized_waveform = np.full(NUM_SAMPLES_SEND, 128, dtype=np.uint8)
            else:
                quantized = ((waveform - min_val) / (max_val - min_val)) * 255
                quantized_waveform = np.clip(quantized, 0, 255).astype(np.uint8)

            self.plot_sender_data(quantized_waveform)
            
            # 2. 获取串口信息
            port_name = self.ports_combo_wf.currentData()
            if not port_name: 
                self.show_error("请选择发送串口！")
                return
            
            # 3. 循环512次，为每个数据点构建并发送一个数据帧
            with serial.Serial(port_name, DEFAULT_BAUD_RATE, timeout=2, write_timeout=2) as ser:
                for address in range(NUM_SAMPLES_SEND):
                    data_point = quantized_waveform[address]

                    # 创建帧的字节数组
                    frame = bytearray()
                    
                    # 帧头: 0x20 0x25
                    frame.extend(b'\x20\x25')
                    
                    # 静态字节: 0xFF 0xFF
                    frame.extend(b'\xFF\xFF')

                    # 命令+地址最高位: F0或F1 (地址0-255用F0, 256-511用F1)
                    addr_high_bit = (address >> 8) & 0x01
                    command_word = 0xF0 + addr_high_bit
                    frame.extend(command_word.to_bytes(1, 'big'))

                    # 写入地址后8位
                    addr_low_byte = address & 0xFF
                    frame.append(addr_low_byte)

                    # 写入的8位波形数据
                    frame.append(data_point)
                    
                    # 波形选择(低4位)+频率选择(高4位)，若无可用数据则置0，自定义波形是5
                    frame.append(0x05)
                    
                    # DDR3容量(4字节)，若无可用数据则置0
                    frame.extend(b'\x00\x0F\xFF\xFF')

                    # 帧尾: \r\n
                    frame.extend(b'\r\n')
                    #hex_frame = ' '.join(f'{b:02X}' for b in frame) #test
                    #print(f"发送帧（地址{address}）: {hex_frame}") #test
                    # 通过串口发送当前帧
                    ser.write(frame)
                    #休眠
                    time.sleep(0.005)

            self.statusBar().showMessage(f"512个波形数据点已通过 {port_name} 发送！")
        except Exception as e:
            self.show_error(f"分析或发送波形时出错: {e}\n{traceback.format_exc()}")

    def plot_sender_data(self, data):
        self.plot_canvas_sender.axes.clear()
        if data is not None:
            self.plot_canvas_sender.axes.plot(data)
            self.plot_canvas_sender.axes.set_title("发送波形预览 (量化后)")
            self.plot_canvas_sender.axes.set_xlabel("采样点")
            self.plot_canvas_sender.axes.set_ylabel("数值 (0-255)")
            self.plot_canvas_sender.axes.grid(True)
        else:
            self.plot_canvas_sender.axes.set_title("等待绘制...")
            self.plot_canvas_sender.axes.text(0.5, 0.5, '此处显示预览', 
                                             ha='center', va='center', 
                                             transform=self.plot_canvas_sender.axes.transAxes)
        self.plot_canvas_sender.draw()

    # =============================================================================
    # 选项卡 3: 控制与高级调试 - 此部分已按要求修改
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
        self.dac_baud_combo.addItems(["9600", "57600", "115200", "1000000"])
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
        
        # --- DAC 函数发生器 UI 修改 ---
        dac_group = QGroupBox("DAC 函数发生器")
        dac_layout = QGridLayout(dac_group)
        dac_layout.addWidget(QLabel("波形:"), 0, 0)
        self.wave_sine = QRadioButton("正弦")
        self.wave_sine.setChecked(True)
        self.wave_square = QRadioButton("方波")
        self.wave_triangle = QRadioButton("三角")
        self.wave_sawtooth = QRadioButton("锯齿波") # 注意: 此波形在新协议中未定义
        self.wave_zero = QRadioButton("直流(零)")
        wave_layout = QHBoxLayout()
        wave_layout.addWidget(self.wave_sine)
        wave_layout.addWidget(self.wave_square)
        wave_layout.addWidget(self.wave_triangle)
        wave_layout.addWidget(self.wave_sawtooth)
        wave_layout.addWidget(self.wave_zero)
        dac_layout.addLayout(wave_layout, 0, 1, 1, 3)
        
        # 修改: 频率输入框
        dac_layout.addWidget(QLabel("频率选择 (0-15):"), 1, 0)
        self.freq_input = QLineEdit("1")
        dac_layout.addWidget(self.freq_input, 1, 1, 1, 3)
        
        # 修改: 幅值输入框改为DDR3容量输入框
        dac_layout.addWidget(QLabel("DDR3 容量:"), 2, 0)
        self.ddr_capacity_input = QLineEdit("268435455") # 0x0FFFFFFF
        dac_layout.addWidget(self.ddr_capacity_input, 2, 1, 1, 3)
        
        self.start_dac_button = QPushButton("开始输出")
        self.stop_dac_button = QPushButton("停止输出")
        dac_layout.addWidget(self.start_dac_button, 3, 0, 1, 2)
        dac_layout.addWidget(self.stop_dac_button, 3, 2, 1, 2)
        # --- UI 修改结束 ---

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
        
        # --- 信号连接修改 ---
        self.start_dac_button.clicked.connect(self.send_dac_command)
        self.stop_dac_button.clicked.connect(self.send_dac_stop_command)
        # --- 修改结束 ---
        
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
            enable_check = QCheckBox("启用")
            enable_check.setChecked(True)
            layout.addWidget(enable_check, i, 1)
            psc = QLineEdit("1000")
            arr = QLineEdit("255")
            duty = QLineEdit("128")
            layout.addWidget(psc, i, 2)
            layout.addWidget(arr, i, 3)
            layout.addWidget(duty, i, 4)
            self.pwm_inputs.append({'enable': enable_check, 'psc': psc, 'arr': arr, 'duty': duty})
        layout.setRowStretch(5, 1)
        main_layout.addWidget(pwm_group)
        main_layout.addStretch(1)
        self.stacked_widget.addWidget(page_pwm)
        
        page_i2c = QWidget()
        layout = QFormLayout(page_i2c)
        self.i2c_addr_width = QComboBox()
        self.i2c_addr_width.addItems(["8-bit", "16-bit"])
        self.i2c_device_addr = QLineEdit("68")
        self.i2c_reg_addr = QLineEdit("01")
        self.i2c_data_to_write = QLineEdit("AA BB")
        self.i2c_read_len = QLineEdit("4")
        layout.addRow("寄存器地址宽度:", self.i2c_addr_width)
        layout.addRow("设备地址(Hex):", self.i2c_device_addr)
        layout.addRow("寄存器地址(Hex):", self.i2c_reg_addr)
        layout.addRow("写入数据(Hex):", self.i2c_data_to_write)
        layout.addRow("读取长度(Bytes):", self.i2c_read_len)
        self.stacked_widget.addWidget(page_i2c)
        
        page_spi = QWidget()
        layout = QFormLayout(page_spi)
        self.spi_tx_only = QCheckBox("仅发送数据")
        self.spi_data_to_write = QLineEdit("9F")
        self.spi_read_len = QLineEdit("5")
        layout.addRow(self.spi_tx_only)
        layout.addRow("写入数据(Hex):", self.spi_data_to_write)
        layout.addRow("读取长度(Bytes):", self.spi_read_len)
        self.stacked_widget.addWidget(page_spi)
        
        page_uart = QWidget()
        layout = QFormLayout(page_uart)
        self.uart_data_to_write = QLineEdit("DE AD BE EF")
        layout.addRow("发送数据(Hex):", self.uart_data_to_write)
        self.stacked_widget.addWidget(page_uart)
        
        page_can = QWidget()
        layout = QFormLayout(page_can)
        self.can_id = QLineEdit("123")
        layout.addRow("CAN ID(Hex):", self.can_id)
        self.can_data = QLineEdit("AA BB CC DD")
        layout.addRow("数据(Hex):", self.can_data)
        self.can_extended = QCheckBox("扩展帧")
        layout.addRow(self.can_extended)
        self.can_remote = QCheckBox("远程帧")
        layout.addRow(self.can_remote)
        self.stacked_widget.addWidget(page_can)
    
    def connect_serial_dac(self):
        port = self.dac_ports_combo.currentData()
        if not port: 
            self.show_error("请选择DAC串口！")
            return
        baud = int(self.dac_baud_combo.currentText())
        try:
            self.serial_connection_dac = serial.Serial(port, baud, timeout=0.5)
            self.connect_dac_button.setEnabled(False)
            self.disconnect_dac_button.setEnabled(True)
            self.statusBar().showMessage(f"DAC串口已连接: {port}", 5000)
        except serial.SerialException as e: 
            self.show_error(f"无法打开DAC串口: {e}")
            
    def disconnect_serial_dac(self):
        if self.serial_connection_dac and self.serial_connection_dac.is_open:
            self.serial_connection_dac.close()
        self.serial_connection_dac = None
        self.connect_dac_button.setEnabled(True)
        self.disconnect_dac_button.setEnabled(False)
        self.statusBar().showMessage("DAC串口已关闭", 2000)
        
    def connect_serial_protocol(self):
        port = self.protocol_ports_combo.currentData()
        if not port: 
            self.show_error("请选择协议调试串口！")
            return
        baud = int(self.protocol_baud_combo.currentText())
        try:
            self.serial_connection_protocol = serial.Serial(port, baud, timeout=0.5)
            self.connect_protocol_button.setEnabled(False)
            self.disconnect_protocol_button.setEnabled(True)
            self.statusBar().showMessage(f"协议调试串口已连接: {port}", 5000)
            self.serial_check_timer.start()
        except serial.SerialException as e: 
            self.show_error(f"无法打开协议调试串口: {e}")
            
    def disconnect_serial_protocol(self):
        self.serial_check_timer.stop()
        if self.serial_connection_protocol and self.serial_connection_protocol.is_open:
            self.serial_connection_protocol.close()
        self.serial_connection_protocol = None
        self.connect_protocol_button.setEnabled(True)
        self.disconnect_protocol_button.setEnabled(False)
        self.statusBar().showMessage("协议调试串口已关闭", 2000)

    # --- DAC 命令发送逻辑重构 ---
    def send_dac_command(self):
        """
        根据新的协议构建并发送ADC波形控制命令。
        帧结构: [0x20 0x25 FF*5 波形+频率 DDR3(4B) \r\n]
        """
        try:
            # 1. 获取波形选择
            waveform_map = {
                self.wave_sine: 0,      # sine
                self.wave_square: 1,    # square
                self.wave_triangle: 2,  # triangular
                self.wave_sawtooth: 3,  # sawtooth
                self.wave_zero: 7       # DC zero
            }
            waveform_code = -1
            for radio_button, code in waveform_map.items():
                if radio_button.isChecked():
                    waveform_code = code
                    break
            
            if waveform_code == -1:
                if self.wave_sawtooth.isChecked():
                    self.show_error("当前协议不支持该波。")
                    return
                self.show_error("请选择一个波形类型。")
                return

            # 2. 获取并验证频率选择
            freq_code = int(self.freq_input.text())
            if not (0 <= freq_code <= 15):
                self.show_error("频率选择值必须在 0 到 15 之间！")
                return

            # 3. 获取并验证DDR3容量
            ddr_capacity = int(self.ddr_capacity_input.text())
            if not (0 <= ddr_capacity <= 0xFFFFFFFF):
                self.show_error("DDR3 容量值必须在 0 到 4294967295 (0xFFFFFFFF) 之间！")
                return

            # 4. 构建数据帧
            frame = bytearray()
            frame.extend(b'\x20\x25')
            frame.extend(b'\xFF\xFF\xFF\xFF\xFF') # 静态字节
            combined_byte = (freq_code << 4) | (waveform_code & 0x0F)
            frame.append(combined_byte)
            frame.extend(ddr_capacity.to_bytes(4, 'big')) # DDR3容量 (4字节, 大端)
            frame.extend(b'\r\n') # 帧尾

            # 5. 使用已有的二进制发送函数发送帧
            self.send_serial_command_dac(frame, is_binary=True)

        except ValueError as e:
            self.show_error(f"输入格式错误: {e}\n请确保频率和DDR3容量为有效整数。")
        except Exception as e:
            self.show_error(f"发送DAC命令时出错: {e}\n{traceback.format_exc()}")

    def send_dac_stop_command(self):
        """专门用于发送停止（直流零电平）命令的函数"""
        try:
            # 停止命令等效于发送一个波形为“直流零”，频率为0，DDR3容量为0的帧
            frame = bytearray()
            frame.extend(b'\x20\x25')
            frame.extend(b'\xFF\xFF\xFF\xFF\xFF')
            combined_byte = (7 << 4) | 0 # 波形=7 (直流零), 频率=0
            frame.append(combined_byte)
            frame.extend((0).to_bytes(4, 'big')) # DDR3 容量 = 0
            frame.extend(b'\r\n')
            self.send_serial_command_dac(frame, is_binary=True)
        except Exception as e:
            self.show_error(f"发送停止命令时出错: {e}\n{traceback.format_exc()}")
            
    def send_serial_command_dac(self, data, is_binary=False):
        if not (self.serial_connection_dac and self.serial_connection_dac.is_open):
            self.show_error("DAC串口未连接！")
            return
        try:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            if is_binary:
                self.serial_connection_dac.write(data)
                log_msg = f"[{timestamp}] -> [DAC] | [HEX] {' '.join(f'{b:02X}' for b in data)}"
                self.sent_data_display.append(log_msg)
            else:
                self.serial_connection_dac.write(data.encode('ascii'))
                log_msg = f"[{timestamp}] -> [DAC] | [ASCII] {data.strip()}"
                self.sent_data_display.append(log_msg)
        except serial.SerialException as e:
            self.show_error(f"DAC串口通信错误: {e}")
            self.disconnect_serial_dac()
    # --- 逻辑重构结束 ---
            
    def construct_and_send_frame(self):
        try:
            protocol = self.protocol_combo.currentText()
            frame = self.build_protocol_frame(protocol)
            if frame: 
                self.send_serial_command_protocol(frame, protocol, is_binary=True)
        except ValueError as e: 
            self.show_error(f"输入格式错误: {e}")
        except Exception as e: 
            self.show_error(f"构建协议帧时出错: {e}\n{traceback.format_exc()}")
            
    def send_serial_command_protocol(self, data, protocol_name, is_binary=False):
        if not (self.serial_connection_protocol and self.serial_connection_protocol.is_open):
            self.show_error("协议调试串口未连接！")
            return
        try:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            protocol_tag = protocol_name.upper()
            if is_binary:
                self.serial_connection_protocol.write(data)
                log_msg = f"[{timestamp}] -> [{protocol_tag}] | [HEX] {' '.join(f'{b:02X}' for b in data)}"
                self.sent_data_display.append(log_msg)
            else:
                self.serial_connection_protocol.write(data.encode('ascii'))
                log_msg = f"[{timestamp}] -> [{protocol_tag}] | [ASCII] {data.strip()}"
                self.sent_data_display.append(log_msg)
        except serial.SerialException as e:
            self.show_error(f"协议调试串口通信错误: {e}")
            self.disconnect_serial_protocol()
            
    def build_protocol_frame(self, protocol):
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
            payload.extend(psc_bytes)
            payload.extend(arr_bytes)
            payload.extend(duty_bytes)
            combined_byte = (enable_byte << 4) | (selection_byte & 0x0F)
            frame.append(combined_byte)
            frame.extend(payload)
            frame.extend(b'\r\n')
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
            if addr_width: 
                payload.extend(reg_addr.to_bytes(2, 'big'))
            else: 
                payload.append(reg_addr)
            payload.extend(write_data)
            payload.extend(read_len.to_bytes(1, 'big'))
            frame.append(combined_byte)
            frame.extend(payload)
            frame.extend(b'\r\n')
            return frame
            
        elif protocol == "SPI":
            selection_byte = 0b010
            tx_only = 1 if self.spi_tx_only.isChecked() else 0
            write_data = bytes.fromhex(self.spi_data_to_write.text().replace(' ', ''))
            read_len = int(self.spi_read_len.text())
            read_bytes = b'\x00' * read_len
            payload.extend(write_data)
            payload.extend(read_bytes)
            combined_byte = (tx_only << 3) | (selection_byte & 0x07)
            frame.append(combined_byte)
            frame.extend(payload)
            frame.extend(b'\r\n')
            return frame
        
        elif protocol == "UART":
            selection_byte = 0b000
            data_str = self.uart_data_to_write.text().strip()
            if data_str: 
                payload.extend(bytes.fromhex(data_str.replace(' ', '')))
            frame.append(selection_byte)
            frame.extend(payload)
            frame.extend(b'\r\n')
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
            frame.append(combined_byte)
            frame.extend(payload)
            frame.extend(b'\r\n')
            return frame
            
        else:
            self.show_error(f"协议 {protocol} 未定义！")
            return None
            
    # =============================================================================
    # 协议帧解析功能 - (逻辑无变化)
    # =============================================================================
    def parse_received_frame(self, frame_data):
        try:
            if frame_data[0:2] != b'\x20\x25': 
                return "无效帧头"
            protocol_byte = frame_data[2]
            protocol_type = protocol_byte & 0b111
            extended_flags = protocol_byte >> 3
            payload = frame_data[3:-2]
            if protocol_type == 0b000:
                return f"<- [UART] | Data: {payload.hex(' ').upper()}"
            elif protocol_type == 0b001:
                addr_width = extended_flags & 0b1
                device_addr = payload[0]
                if addr_width: 
                    reg_addr, data_start = int.from_bytes(payload[1:3], 'big'), 3
                else: 
                    reg_addr, data_start = payload[1], 2
                write_data = payload[data_start:-1]
                read_len = payload[-1]
                return (f"<- [I2C] | Dev: 0x{device_addr:02X}, Reg: 0x{reg_addr:0{4 if addr_width else 2}X}, "
                        f"Write: [{write_data.hex(' ').upper()}], ReadLen: {read_len}B")
            elif protocol_type == 0b010:
                tx_only = extended_flags & 0b1
                read_len = payload[-1] if payload else 0
                write_data = payload[:-read_len] if read_len > 0 else payload
                return (f"<- [SPI] | Mode: {'TX Only' if tx_only else 'TX/RX'}, "
                        f"Write: [{write_data.hex(' ').upper()}], ReadLen: {read_len}B")
            elif protocol_type == 0b011:
                enable_byte = extended_flags & 0b1111
                channels = []
                for i in range(4):
                    offset = i * 5
                    if offset + 5 > len(payload): break
                    psc = int.from_bytes(payload[offset:offset+2], 'big')
                    arr = int.from_bytes(payload[offset+2:offset+4], 'big')
                    duty = payload[offset+4]
                    enabled = (enable_byte >> i) & 0b1
                    if enabled: 
                        channels.append(f"CH{i+1}: ON (PSC={psc}, ARR={arr}, Duty={duty})")
                    else: 
                        channels.append(f"CH{i+1}: OFF")
                return f"<- [PWM] | Status: {' | '.join(channels)}"
            elif protocol_type == 0b100:
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
        except IndexError: 
            return "帧数据不完整"
        except Exception as e: 
            return f"解析错误: {str(e)}"
            
    def check_control_serial_data(self):
        if not (self.serial_connection_protocol and self.serial_connection_protocol.is_open): return
        try:
            data = self.serial_connection_protocol.read(self.serial_connection_protocol.in_waiting)
        except serial.SerialException as e:
            self.show_error(f"协议串口读取错误: {e}")
            self.disconnect_serial_protocol()
            return
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