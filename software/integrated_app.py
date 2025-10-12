# integrated_app.py

import sys
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QPushButton, QComboBox, QLabel, QHBoxLayout, 
                             QProgressBar, QMessageBox, QSizePolicy, QTabWidget)
from PyQt6.QtGui import QPainter, QPixmap, QPen, QColor
from PyQt6.QtCore import Qt, QPoint, QTimer, QThread, pyqtSignal
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
import traceback
from collections import deque

# --- Constants ---
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 400
NUM_SAMPLES_SEND = 512 # 发送时重采样的点数
BAUD_RATE = 115200
RECEIVE_BLOCK_SIZE = 64 # 每次从串口读取的数据块大小 (对应C++的BLOCK_SIZE)
OSCILLOSCOPE_WIDTH = 512 # 示波器显示的宽度

# 设置中文字体 (可选，如果系统支持)
try:
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False
except:
    print("Warning: Chinese font 'SimHei' not found. Using default.")

# =============================================================================
# 复用自 draw_wave.py 的组件
# =============================================================================

class WaveformDrawer(QWidget):
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
        """提取并重采样波形数据"""
        if not self.points:
            return None
        
        sorted_points = sorted(self.points, key=lambda p: p.x())
        
        # 使用x坐标作为索引，y坐标作为值
        x_coords = [p.x() for p in sorted_points]
        y_coords = [p.y() for p in sorted_points]

        if len(x_coords) < 2:
             return None # 点太少无法插值

        # 创建插值函数
        f = interp1d(x_coords, y_coords, kind='linear', bounds_error=False, 
                     fill_value=(y_coords[0], y_coords[-1]))

        # 在整个画布宽度上生成均匀间隔的点
        x_new = np.linspace(min(x_coords), max(x_coords), num=NUM_SAMPLES_SEND)
        y_new = f(x_new)

        # 归一化处理: y值反转并归一化到-1到1
        waveform = (CANVAS_HEIGHT - y_new - CANVAS_HEIGHT / 2) / (CANVAS_HEIGHT / 2)
        return waveform

class MplCanvas(FigureCanvas):
    """嵌入式matplotlib控件"""
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)
        self.setParent(parent)
        self.setMinimumWidth(CANVAS_WIDTH)
        self.setMaximumWidth(CANVAS_WIDTH)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)


# =============================================================================
# 新增的接收线程组件
# =============================================================================

class ReceiverThread(QThread):
    """后台串口数据接收线程"""
    # 定义一个信号，参数为Python的list类型
    data_received = pyqtSignal(list)
    error_occurred = pyqtSignal(str)

    def __init__(self, port, baudrate, parent=None):
        super().__init__(parent)
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None
        self._is_running = True

    def run(self):
        """线程主函数"""
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=1)
        except serial.SerialException as e:
            self.error_occurred.emit(f"无法打开串口 {self.port}: {e}")
            return

        while self._is_running:
            try:
                if self.serial_connection.in_waiting >= RECEIVE_BLOCK_SIZE:
                    # 读取一个完整的数据块
                    data_bytes = self.serial_connection.read(RECEIVE_BLOCK_SIZE)
                    if data_bytes:
                        # 将8位无符号字节转换为整数列表
                        data_list = list(data_bytes)
                        # 发射信号，将数据传递给主线程
                        self.data_received.emit(data_list)
            except serial.SerialException as e:
                self.error_occurred.emit(f"串口读取错误: {e}")
                self._is_running = False
            except Exception as e:
                self.error_occurred.emit(f"处理数据时发生未知错误: {e}")
                self._is_running = False
        
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()

    def stop(self):
        """停止线程"""
        self._is_running = False
        self.wait(2000) # 等待线程结束，最多2秒


# =============================================================================
# 集成化的主窗口
# =============================================================================

class IntegratedGUI(QMainWindow):
    """集成化的主应用窗口"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("集成波形工具 (绘制/发送/接收) v2.0")
        self.setGeometry(100, 100, 900, 800)

        # 创建选项卡控件
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # 创建两个选项卡
        self.sender_tab = QWidget()
        self.receiver_tab = QWidget()

        self.tabs.addTab(self.sender_tab, "波形绘制与发送")
        self.tabs.addTab(self.receiver_tab, "实时波形接收")

        # 初始化发送和接收选项卡的UI
        self.init_sender_ui()
        self.init_receiver_ui()

        self.refresh_serial_ports()

        # 初始化串口连接
        self.serial_connection = None
        self.receiver_thread = None

    # --- 发送选项卡 ---
    def init_sender_ui(self):
        layout = QVBoxLayout(self.sender_tab)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)

        # 绘制区域
        draw_label = QLabel("1. 在下方区域绘制波形:")
        self.drawer = WaveformDrawer()
        layout.addWidget(draw_label)
        layout.addWidget(self.drawer, alignment=Qt.AlignmentFlag.AlignCenter)

        # 预览图
        plot_label = QLabel("2. 预览将要发送的数据:")
        self.plot_canvas_sender = MplCanvas(self, width=CANVAS_WIDTH/100, height=3, dpi=100)
        layout.addWidget(plot_label)
        layout.addWidget(self.plot_canvas_sender, alignment=Qt.AlignmentFlag.AlignCenter)

        # 状态栏
        self.status_label_sender = QLabel("请绘制波形。")
        layout.addWidget(self.status_label_sender)

        # 控制区
        controls_layout = QHBoxLayout()
        self.serial_ports_combo_sender = QComboBox()
        self.refresh_button_sender = QPushButton("刷新串口")
        self.clear_button = QPushButton("清除画布")
        self.analyze_button = QPushButton("分析并发送")
        
        controls_layout.addWidget(QLabel("发送串口:"))
        controls_layout.addWidget(self.serial_ports_combo_sender, 1)
        controls_layout.addWidget(self.refresh_button_sender)
        controls_layout.addStretch()
        controls_layout.addWidget(self.clear_button)
        controls_layout.addWidget(self.analyze_button)
        layout.addLayout(controls_layout)

        # 连接信号
        self.refresh_button_sender.clicked.connect(self.refresh_serial_ports)
        self.clear_button.clicked.connect(self.clear_all)
        self.analyze_button.clicked.connect(self.analyze_and_send)
        
        #self.refresh_serial_ports()
        self.plot_sender_data(None)

    def refresh_serial_ports(self):
        """刷新两个选项卡中的串口列表"""
        ports = serial.tools.list_ports.comports()
        
        # 清空列表
        self.serial_ports_combo_sender.clear()
        self.serial_ports_combo_receiver.clear()
        
        if not ports:
            self.serial_ports_combo_sender.addItem("未找到串口")
            self.serial_ports_combo_receiver.addItem("未找到串口")
        else:
            for port in ports:
                self.serial_ports_combo_sender.addItem(f"{port.device} - {port.description}", port.device)
                self.serial_ports_combo_receiver.addItem(f"{port.device} - {port.description}", port.device)
                
        self.status_label_sender.setText("串口列表已刷新。")
        self.status_label_receiver.setText("串口列表已刷新，请选择端口并开始接收。")

    def clear_all(self):
        self.drawer.clear_canvas()
        self.plot_sender_data(None)
        self.status_label_sender.setText("画布已清除。")

    def analyze_and_send(self):
        try:
            waveform = self.drawer.get_waveform_data()
            if waveform is None:
                self.show_error("画布为空或数据不足，无法分析。")
                return
            
            # 滤波平滑
            if len(waveform) > 10:
                window_size = min(21, len(waveform) // 4)
                if window_size % 2 == 0: window_size += 1
                if window_size > 3:
                    waveform = savgol_filter(waveform, window_size, 3)

            # 量化到0-255
            quantized = ((waveform - np.min(waveform)) / (np.max(waveform) - np.min(waveform))) * 255
            quantized_waveform = np.clip(quantized, 0, 255).astype(int)

            self.status_label_sender.setText("分析完成，准备发送...")
            self.plot_sender_data(quantized_waveform, 'samples')
            
            # 格式化并发送数据: "S,val1,val2,...\n"
            data_to_send = "S," + ",".join(map(str, quantized_waveform)) + "\n"
            self.send_serial_data(data_to_send)

        except Exception as e:
            self.show_error(f"分析发送时出错: {e}")
            traceback.print_exc()

    def send_serial_data(self, data):
        port_name = self.serial_ports_combo_sender.currentData()
        if not port_name:
            self.show_error("请选择一个有效的发送串口！")
            return
        
        try:
            with serial.Serial(port_name, BAUD_RATE, timeout=2, write_timeout=2) as ser:
                ser.write(data.encode('ascii'))
                self.status_label_sender.setText(f"数据已通过 {port_name} 成功发送！")
        except serial.SerialException as e:
            self.show_error(f"发送串口错误: {e}")

    def plot_sender_data(self, data, plot_type='none'):
        self.plot_canvas_sender.axes.clear()
        if plot_type == 'samples' and data is not None:
            self.plot_canvas_sender.axes.plot(data)
            self.plot_canvas_sender.axes.set_title("发送波形预览 (量化后)")
            self.plot_canvas_sender.axes.set_xlabel("采样点")
            self.plot_canvas_sender.axes.set_ylabel("数值 (0-255)")
            self.plot_canvas_sender.axes.grid(True)
        else:
            self.plot_canvas_sender.axes.set_title("等待绘制...")
            self.plot_canvas_sender.axes.text(0.5, 0.5, '此处将显示发送波形的预览', 
                                             ha='center', va='center', transform=self.plot_canvas_sender.axes.transAxes)
        self.plot_canvas_sender.draw()

    # --- 接收选项卡 ---
    def init_receiver_ui(self):
        layout = QVBoxLayout(self.receiver_tab)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)

        # 示波器图
        plot_label = QLabel("实时波形显示:")
        self.plot_canvas_receiver = MplCanvas(self, width=CANVAS_WIDTH/100, height=4, dpi=100)
        layout.addWidget(plot_label)
        layout.addWidget(self.plot_canvas_receiver, alignment=Qt.AlignmentFlag.AlignCenter)

        # 状态栏
        self.status_label_receiver = QLabel("请选择串口并开始接收。")
        layout.addWidget(self.status_label_receiver)

        # 控制区
        controls_layout = QHBoxLayout()
        self.serial_ports_combo_receiver = QComboBox()
        self.refresh_button_receiver = QPushButton("刷新串口")
        self.start_button = QPushButton("开始接收")
        self.stop_button = QPushButton("停止接收")
        self.stop_button.setEnabled(False)

        controls_layout.addWidget(QLabel("接收串口:"))
        controls_layout.addWidget(self.serial_ports_combo_receiver, 1)
        controls_layout.addWidget(self.refresh_button_receiver)
        controls_layout.addStretch()
        controls_layout.addWidget(self.start_button)
        controls_layout.addWidget(self.stop_button)
        layout.addLayout(controls_layout)

        # 连接信号
        self.refresh_button_receiver.clicked.connect(self.refresh_serial_ports)
        self.start_button.clicked.connect(self.start_receiving)
        self.stop_button.clicked.connect(self.stop_receiving)

        # 初始化数据缓冲区
        self.data_buffer = deque([0] * OSCILLOSCOPE_WIDTH, maxlen=OSCILLOSCOPE_WIDTH)
        self.plot_receiver_data()

    def start_receiving(self):
        port_name = self.serial_ports_combo_receiver.currentData()
        if not port_name:
            self.show_error("请选择一个有效的接收串口！")
            return
            
        if self.receiver_thread and self.receiver_thread.isRunning():
            self.show_error("接收线程已在运行！")
            return

        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.serial_ports_combo_receiver.setEnabled(False)
        self.status_label_receiver.setText(f"正在从 {port_name} 接收数据...")

        self.receiver_thread = ReceiverThread(port=port_name, baudrate=BAUD_RATE)
        # 连接信号和槽
        self.receiver_thread.data_received.connect(self.update_plot)
        self.receiver_thread.error_occurred.connect(self.on_receiver_error)
        self.receiver_thread.start()

    def stop_receiving(self):
        if self.receiver_thread and self.receiver_thread.isRunning():
            self.receiver_thread.stop()
            self.receiver_thread = None

        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.serial_ports_combo_receiver.setEnabled(True)
        self.status_label_receiver.setText("接收已停止。")

    def update_plot(self, data_list):
        """槽函数：接收线程数据并更新图表"""
        # 将新数据块添加到缓冲区的末尾
        self.data_buffer.extend(data_list)
        self.plot_receiver_data()
    
    def on_receiver_error(self, message):
        self.show_error(message)
        self.stop_receiving()

    def plot_receiver_data(self):
        self.plot_canvas_receiver.axes.clear()
        self.plot_canvas_receiver.axes.plot(list(self.data_buffer))
        self.plot_canvas_receiver.axes.set_title("实时示波器")
        self.plot_canvas_receiver.axes.set_ylim(0, 260) # 8位数据范围 0-255
        self.plot_canvas_receiver.axes.set_xlim(0, OSCILLOSCOPE_WIDTH)
        self.plot_canvas_receiver.axes.set_xlabel("时间 (采样点)")
        self.plot_canvas_receiver.axes.set_ylabel("幅值 (0-255)")
        self.plot_canvas_receiver.axes.grid(True)
        self.plot_canvas_receiver.draw()

    # --- 通用函数 ---
    def show_error(self, message):
        QMessageBox.critical(self, "错误", message)

    def closeEvent(self, event):
        """关闭窗口时确保线程停止"""
        self.stop_receiving()
        event.accept()


# --- 主程序入口 ---
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = IntegratedGUI()
    window.show()
    sys.exit(app.exec())