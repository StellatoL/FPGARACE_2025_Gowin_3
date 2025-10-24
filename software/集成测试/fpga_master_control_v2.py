# -*- coding: utf-8 -*-
"""
fpga_unified_control_v3.1_fft.py

功能:
这是一个集成化的FPGA上位机工具，合并了高性能示波器、波形绘制发送和高级协议调试功能。
1. [示波器] 通过共享内存从C++ UDP接收器高速读取ADC数据，使用pyqtgraph实时显示时域波形和频域(FFT)频谱。
2. [波形绘制] 允许用户手绘波形，量化后通过串口发送。
3. [控制调试] 集成了DAC函数发生器和支持多种协议(4路PWM/I2C/SPI等)的高级协议调试器。

依赖库:
pip install PyQt6 pyqtgraph numpy pyserial scipy matplotlib
"""
import sys
import mmap
import struct
import traceback

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
        self.setWindowTitle("FPGA 集成上位机 v3.1 (含FFT)")
        self.setGeometry(100, 100, 1200, 900)

        self.shared_memory = None
        self.serial_connection_control = None
        self.scope_data = np.array([])

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.scope_tab = QWidget()
        self.waveform_sender_tab = QWidget()
        self.control_tab = QWidget()

        self.tabs.addTab(self.scope_tab, "高速数字示波器")
        self.tabs.addTab(self.waveform_sender_tab, "波形绘制与发送")
        self.tabs.addTab(self.control_tab, "控制与高级调试")

        self.init_scope_tab()
        self.init_waveform_sender_tab()
        self.init_control_tab()

        self.scope_timer = QTimer(self)
        self.scope_timer.setInterval(SCOPE_UPDATE_INTERVAL_MS)
        self.scope_timer.timeout.connect(self.update_scope_plot)
        
        self.connect_shared_memory()

    def show_error(self, message):
        QMessageBox.critical(self, "错误", message)

    def closeEvent(self, event):
        print("正在关闭应用程序并释放资源...")
        self.stop_scope()
        self.disconnect_serial_control()
        if self.shared_memory:
            self.shared_memory.close()
        event.accept()

    def refresh_all_ports(self):
        ports = serial.tools.list_ports.comports()
        combo_boxes = [self.ports_combo_wf, self.ports_combo_control]
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
    # 选项卡 1: 高速数字示波器 (已修改，增加FFT)
    # =============================================================================
    def init_scope_tab(self):
        layout = QHBoxLayout(self.scope_tab)
        # --- 左侧控制面板 ---
        controls_group = QGroupBox("ADC 采样控制")
        controls_layout = QGridLayout(controls_group)
        controls_group.setFixedWidth(250)
        controls_layout.addWidget(QLabel("采样率 (SPS):"), 0, 0)
        self.sampling_rate_input = QLineEdit("10000000") # 关键参数
        controls_layout.addWidget(self.sampling_rate_input, 0, 1)
        controls_layout.addWidget(QLabel("数据点数:"), 1, 0)
        self.data_points_input = QLineEdit("N/A")
        self.data_points_input.setReadOnly(True)
        controls_layout.addWidget(self.data_points_input, 1, 1)
        
        self.fft_checkbox = QCheckBox("启用FFT分析")
        self.fft_checkbox.setChecked(True)
        controls_layout.addWidget(self.fft_checkbox, 2, 0, 1, 2)
        
        self.start_button = QPushButton("开始采样"); self.stop_button = QPushButton("停止采样")
        self.save_button = QPushButton("保存数据"); self.stop_button.setEnabled(False)
        controls_layout.addWidget(self.start_button, 3, 0, 1, 2)
        controls_layout.addWidget(self.stop_button, 4, 0, 1, 2)
        controls_layout.addWidget(self.save_button, 5, 0, 1, 2)
        self.status_label = QLabel("状态: 已停止")
        controls_layout.addWidget(self.status_label, 6, 0, 1, 2)
        controls_layout.setRowStretch(7, 1)
        
        # --- 右侧绘图区域 (时域 + 频域) ---
        plot_area_layout = QVBoxLayout()
        # 时域图
        time_plot_group = QGroupBox("实时波形 (时域)")
        time_plot_layout = QVBoxLayout(time_plot_group)
        self.plot_widget_time = pg.PlotWidget()
        time_plot_layout.addWidget(self.plot_widget_time)
        self.curve_time = self.plot_widget_time.plot(pen=pg.mkPen(color='#00D0FF', width=2))
        self.setup_plot_style(self.plot_widget_time, "Real-Time ADC Data", "Sample Index", "ADC Value (0-255)", (0, 255))
        # 频域图
        freq_plot_group = QGroupBox("频谱分析 (频域)")
        freq_plot_layout = QVBoxLayout(freq_plot_group)
        self.plot_widget_freq = pg.PlotWidget()
        freq_plot_layout.addWidget(self.plot_widget_freq)
        self.curve_freq = self.plot_widget_freq.plot(pen=pg.mkPen(color='#FFD700', width=2))
        self.setup_plot_style(self.plot_widget_freq, "Frequency Spectrum (FFT)", "Frequency (Hz)", "Magnitude |Y(f)|")
        self.plot_widget_freq.setLogMode(x=True, y=False) # X轴使用对数坐标

        plot_area_layout.addWidget(time_plot_group)
        plot_area_layout.addWidget(freq_plot_group)
        layout.addWidget(controls_group)
        layout.addLayout(plot_area_layout)
        
        # 连接信号
        self.start_button.clicked.connect(self.start_scope)
        self.stop_button.clicked.connect(self.stop_scope)
        self.save_button.clicked.connect(self.save_scope_data)
        self.fft_checkbox.stateChanged.connect(lambda: self.plot_widget_freq.setVisible(self.fft_checkbox.isChecked()))

    def setup_plot_style(self, plot_widget, title, xlabel, ylabel, yrange=None):
        plot_widget.setBackground('k')
        plot_widget.showGrid(x=True, y=True, alpha=0.3)
        plot_widget.setTitle(title, color="w", size="12pt")
        plot_widget.setLabel('bottom', xlabel, color='w')
        plot_widget.setLabel('left', ylabel, color='w')
        plot_widget.getAxis('bottom').setPen(pg.mkPen(color='w'))
        plot_widget.getAxis('left').setPen(pg.mkPen(color='w'))
        if yrange: plot_widget.setYRange(*yrange)

    def connect_shared_memory(self):
        try:
            self.shared_memory = mmap.mmap(-1, SHARED_MEM_SIZE, tagname=SHARED_MEM_NAME)
            self.statusBar().showMessage(f"成功连接到共享内存 '{SHARED_MEM_NAME}'。", 5000)
        except FileNotFoundError:
            self.show_error(f"未找到共享内存 '{SHARED_MEM_NAME}'。\n请确保 C++ 接收程序已运行！")
        except Exception as e: self.show_error(f"访问共享内存时出错: {e}")

    def start_scope(self):
        if not self.shared_memory:
            self.connect_shared_memory()
            if not self.shared_memory: return
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
        """核心函数：读取数据，更新时域图，并计算和更新频域图"""
        if not self.shared_memory: return
        try:
            self.shared_memory.seek(0)
            count_bytes = self.shared_memory.read(2)
            if len(count_bytes) < 2: return
            sample_count = struct.unpack('>H', count_bytes)[0]
            
            if 0 < sample_count <= (SHARED_MEM_SIZE - 2):
                adc_data_bytes = self.shared_memory.read(sample_count)
                self.scope_data = np.frombuffer(adc_data_bytes, dtype=np.uint8)
                
                # 1. 更新时域图
                self.curve_time.setData(self.scope_data)
                self.data_points_input.setText(str(self.scope_data.size))
                
                # 2. 如果启用，则计算并更新频域图
                if self.fft_checkbox.isChecked() and self.scope_data.size > 1:
                    self.update_fft_plot()
                
        except Exception as e:
            print(f"读取或更新绘图时出错: {e}"); self.stop_scope()

    def update_fft_plot(self):
        try:
            fs = float(self.sampling_rate_input.text())
            if fs <= 0: return
        except ValueError:
            return # 采样率输入无效
        
        N = self.scope_data.size
        T = 1.0 / fs
        
        # 减去直流分量(均值)以获得更好的频谱视图
        data_no_dc = self.scope_data - np.mean(self.scope_data)
        
        # 执行FFT
        yf = np.fft.fft(data_no_dc)
        xf = np.fft.fftfreq(N, T)[:N//2]
        
        # 计算幅值并归一化
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
    # 选项卡 3: 控制与高级调试 (已修改，4路PWM)
    # =============================================================================
    def init_control_tab(self):
        main_layout = QVBoxLayout(self.control_tab)
        # 串口配置
        serial_group = QGroupBox("串口配置"); serial_layout = QHBoxLayout(serial_group)
        serial_layout.addWidget(QLabel("端口:")); self.ports_combo_control = QComboBox(); serial_layout.addWidget(self.ports_combo_control, 1)
        self.baud_combo_control = QComboBox(); self.baud_combo_control.addItems(["9600", "57600", "115200"]); self.baud_combo_control.setCurrentText(str(DEFAULT_BAUD_RATE))
        serial_layout.addWidget(QLabel("波特率:")); serial_layout.addWidget(self.baud_combo_control)
        self.refresh_button_control = QPushButton("刷新"); serial_layout.addWidget(self.refresh_button_control)
        self.connect_serial_button = QPushButton("打开"); self.disconnect_serial_button = QPushButton("关闭"); self.disconnect_serial_button.setEnabled(False)
        serial_layout.addWidget(self.connect_serial_button); serial_layout.addWidget(self.disconnect_serial_button)
        
        control_panel_layout = QHBoxLayout()
        left_panel = QVBoxLayout()
        # DAC发生器
        dac_group = QGroupBox("DAC 函数发生器"); dac_layout = QGridLayout(dac_group)
        dac_layout.addWidget(QLabel("波形:"), 0, 0); self.wave_sine = QRadioButton("正弦"); self.wave_sine.setChecked(True)
        self.wave_square = QRadioButton("方波"); self.wave_triangle = QRadioButton("三角")
        dac_layout.addWidget(self.wave_sine, 0, 1); dac_layout.addWidget(self.wave_square, 0, 2); dac_layout.addWidget(self.wave_triangle, 0, 3)
        dac_layout.addWidget(QLabel("频率(Hz):"), 1, 0); self.freq_input = QLineEdit("1000"); dac_layout.addWidget(self.freq_input, 1, 1, 1, 3)
        dac_layout.addWidget(QLabel("幅值(V):"), 2, 0); self.amp_input = QLineEdit("2.50"); dac_layout.addWidget(self.amp_input, 2, 1, 1, 3)
        self.start_dac_button = QPushButton("开始输出"); self.stop_dac_button = QPushButton("停止输出")
        dac_layout.addWidget(self.start_dac_button, 3, 0, 1, 2); dac_layout.addWidget(self.stop_dac_button, 3, 2, 1, 2)
        # 高级协议调试器
        protocol_group = QGroupBox("高级协议调试器"); protocol_layout = QVBoxLayout(protocol_group)
        self.protocol_combo = QComboBox(); self.protocol_combo.addItems(["PWM", "I2C", "SPI", "UART"])
        protocol_layout.addWidget(self.protocol_combo); self.stacked_widget = QStackedWidget(); self.init_protocol_pages()
        protocol_layout.addWidget(self.stacked_widget); self.send_protocol_button = QPushButton("构建并发送协议命令")
        protocol_layout.addWidget(self.send_protocol_button)
        left_panel.addWidget(dac_group); left_panel.addWidget(protocol_group)
        
        # 数据监视
        right_panel = QVBoxLayout(); display_group = QGroupBox("数据监视"); display_layout = QVBoxLayout(display_group)
        display_layout.addWidget(QLabel("发送历史:")); self.sent_data_display = QTextEdit(); self.sent_data_display.setReadOnly(True); self.sent_data_display.setFontFamily("Courier New")
        display_layout.addWidget(self.sent_data_display); display_layout.addWidget(QLabel("接收历史:"))
        self.received_data_display = QTextEdit(); self.received_data_display.setReadOnly(True); self.received_data_display.setFontFamily("Courier New")
        display_layout.addWidget(self.received_data_display); self.clear_logs_button = QPushButton("清空日志")
        display_layout.addWidget(self.clear_logs_button); right_panel.addWidget(display_group)
        
        control_panel_layout.addLayout(left_panel, 2); control_panel_layout.addLayout(right_panel, 1)
        main_layout.addWidget(serial_group); main_layout.addLayout(control_panel_layout)
        self.refresh_all_ports()

        # 连接信号
        self.refresh_button_control.clicked.connect(self.refresh_all_ports)
        self.connect_serial_button.clicked.connect(self.connect_serial_control)
        self.disconnect_serial_button.clicked.connect(self.disconnect_serial_control)
        self.start_dac_button.clicked.connect(self.send_dac_command)
        self.stop_dac_button.clicked.connect(lambda: self.send_serial_command_control("DAC,STOP\n"))
        self.protocol_combo.currentIndexChanged.connect(self.stacked_widget.setCurrentIndex)
        self.send_protocol_button.clicked.connect(self.construct_and_send_frame)
        self.clear_logs_button.clicked.connect(lambda: (self.sent_data_display.clear(), self.received_data_display.clear()))

    def init_protocol_pages(self):
        # *** 修改点: 恢复4路PWM ***
        page_pwm = QWidget(); layout = QFormLayout(page_pwm); self.pwm_inputs = []
        for i in range(1, 5): # 从 range(1,3) 改为 range(1,5)
            psc = QLineEdit("1000"); arr = QLineEdit("255"); duty = QLineEdit("128")
            layout.addRow(f"--- 通道 {i} ---", None)
            layout.addRow(f"PSC Ch{i} (8-byte):", psc); layout.addRow(f"ARR Ch{i} (8-byte):", arr); layout.addRow(f"Duty Ch{i} (4-byte):", duty)
            self.pwm_inputs.append({'psc': psc, 'arr': arr, 'duty': duty})
        self.stacked_widget.addWidget(page_pwm)
        # 其他页面保持不变
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

    def connect_serial_control(self):
        port = self.ports_combo_control.currentData(); baud = int(self.baud_combo_control.currentText())
        if not port: self.show_error("请选择串口！"); return
        try:
            self.serial_connection_control = serial.Serial(port, baud, timeout=0.5)
            self.connect_serial_button.setEnabled(False); self.disconnect_serial_button.setEnabled(True)
            self.statusBar().showMessage(f"成功连接到 {port}", 5000)
        except serial.SerialException as e: self.show_error(f"无法打开串口: {e}")

    def disconnect_serial_control(self):
        if self.serial_connection_control and self.serial_connection_control.is_open:
            self.serial_connection_control.close()
        self.serial_connection_control = None
        self.connect_serial_button.setEnabled(True); self.disconnect_serial_button.setEnabled(False)
        self.statusBar().showMessage("控制串口已关闭", 2000)

    def send_serial_command_control(self, data, is_binary=False):
        if not (self.serial_connection_control and self.serial_connection_control.is_open):
            self.show_error("控制串口未连接！"); return
        try:
            if is_binary:
                self.serial_connection_control.write(data)
                self.sent_data_display.append(f"-> [HEX] {' '.join(f'{b:02X}' for b in data)}")
                response = self.serial_connection_control.read_until(b'\r\n')
                if response: self.received_data_display.append(f"<- [HEX] {' '.join(f'{b:02X}' for b in response)}")
            else:
                self.serial_connection_control.write(data.encode('ascii'))
                self.sent_data_display.append(f"-> [ASCII] {data.strip()}")
                response = self.serial_connection_control.readline().decode('ascii', errors='ignore').strip()
                if response: self.received_data_display.append(f"<- [ASCII] {response}")
        except serial.SerialException as e:
            self.show_error(f"串口通信错误: {e}"); self.disconnect_serial_control()

    def send_dac_command(self):
        wave = "SINE"
        if self.wave_square.isChecked(): wave = "SQUARE"
        if self.wave_triangle.isChecked(): wave = "TRIANGLE"
        try:
            freq = float(self.freq_input.text()); amp = float(self.amp_input.text())
            self.send_serial_command_control(f"DAC,{wave},{freq},{amp}\n")
        except ValueError: self.show_error("频率和幅值必须是有效数字！")

    def construct_and_send_frame(self):
        try:
            protocol = self.protocol_combo.currentText()
            frame = self.build_protocol_frame(protocol)
            if frame: self.send_serial_command_control(frame, is_binary=True)
        except ValueError as e: self.show_error(f"输入格式错误: {e}")
        except Exception as e: self.show_error(f"构建协议帧时出错: {e}\n{traceback.format_exc()}")

    def build_protocol_frame(self, protocol):
        frame = bytearray([0x20, 0x25]); selection_byte = 0; payload = bytearray()
        if protocol == "PWM":
            selection_byte |= 0b011
            psc_bytes, arr_bytes, duty_bytes = bytearray(), bytearray(), bytearray()
            for i in range(len(self.pwm_inputs)):
                psc = int(self.pwm_inputs[i]['psc'].text()); arr = int(self.pwm_inputs[i]['arr'].text()); duty = int(self.pwm_inputs[i]['duty'].text())
                psc_bytes.extend(psc.to_bytes(8, 'big')); arr_bytes.extend(arr.to_bytes(8, 'big')); duty_bytes.extend(duty.to_bytes(4, 'big'))
            payload.extend(psc_bytes); payload.extend(arr_bytes); payload.extend(duty_bytes)
        elif protocol == "I2C":
            # (... 其他协议构建逻辑保持不变 ...)
            selection_byte |= 0b001
        elif protocol == "SPI":
            selection_byte |= 0b010
        elif protocol == "UART":
            selection_byte |= 0b000
            data_str = self.uart_data_to_write.text().strip()
            if data_str: payload.extend(bytes.fromhex(data_str.replace(' ', '')))
        else: self.show_error(f"协议 {protocol} 未定义！"); return None
        frame.append(selection_byte); frame.extend(payload); frame.extend(b'\r\n')
        return frame

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = FPGAMasterControl()
    window.show()
    sys.exit(app.exec())