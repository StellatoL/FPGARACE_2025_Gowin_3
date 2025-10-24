# -*- coding: utf-8 -*-
"""
fpga_master_control.py

功能:
1. 通过共享内存从 C++ UDP 接收器高速读取 ADC 数据。
2. 使用 pyqtgraph 实时显示高性能示波器波形。
3. 集成 DAC 函数发生器和通用指令发送功能，通过串口与FPGA通信。
4. 界面设计参考提供的上位机图片，使用选项卡布局。
"""
import sys
import mmap
import struct
import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QComboBox, QLabel, QLineEdit, QTextEdit, QGroupBox, QTabWidget,
    QMessageBox, QFileDialog, QRadioButton
)
from PyQt6.QtCore import QTimer
import serial
import serial.tools.list_ports

# --- 配置参数 ---
# 共享内存配置 (必须与 C++ 程序完全一致)
SHARED_MEM_NAME = "FPGA_ADC_DATA"
SHARED_MEM_SIZE = 2048
# 示波器刷新率 (毫秒)
SCOPE_UPDATE_INTERVAL_MS = 20 # 50Hz

class FPGAMasterControl(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FPGA 集成上位机 v2.0")
        self.setGeometry(100, 100, 1200, 800)

        # --- 初始化变量 ---
        self.shared_memory = None
        self.serial_connection = None
        self.scope_data = np.array([])

        # --- 创建主选项卡 ---
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # 创建各个功能页面
        self.scope_tab = QWidget()
        self.control_tab = QWidget()

        self.tabs.addTab(self.scope_tab, "高速数字示波器")
        self.tabs.addTab(self.control_tab, "信号发生与协议调试")

        # 初始化每个选项卡的UI
        self.init_scope_tab()
        self.init_control_tab()

        # 初始化并启动示波器更新定时器
        self.scope_timer = QTimer()
        self.scope_timer.setInterval(SCOPE_UPDATE_INTERVAL_MS)
        self.scope_timer.timeout.connect(self.update_scope_plot)
        
        # 尝试连接共享内存
        self.connect_shared_memory()

    def connect_shared_memory(self):
        """尝试连接到由C++程序创建的共享内存"""
        try:
            self.shared_memory = mmap.mmap(-1, SHARED_MEM_SIZE, tagname=SHARED_MEM_NAME)
            self.statusBar().showMessage(f"成功连接到共享内存 '{SHARED_MEM_NAME}'。", 5000)
        except FileNotFoundError:
            self.show_error(f"未找到共享内存 '{SHARED_MEM_NAME}'。\n\n请确保 C++ 接收程序 (udp_receiver_sm.exe) 已经运行！")
        except Exception as e:
            self.show_error(f"访问共享内存时发生未知错误: {e}")

    # =============================================================================
    # 选项卡 1: 高速数字示波器
    # =============================================================================
    def init_scope_tab(self):
        layout = QHBoxLayout(self.scope_tab)

        # --- 左侧控制面板 ---
        controls_group = QGroupBox("ADC 采样控制")
        controls_layout = QGridLayout()
        controls_group.setLayout(controls_layout)
        controls_group.setFixedWidth(250)

        controls_layout.addWidget(QLabel("采样率 (SPS):"), 0, 0)
        self.sampling_rate_input = QLineEdit("10000000") # 默认值参考图片
        controls_layout.addWidget(self.sampling_rate_input, 0, 1)

        controls_layout.addWidget(QLabel("数据点数:"), 1, 0)
        self.data_points_input = QLineEdit("2046") # 2048 - 2 bytes for count
        self.data_points_input.setReadOnly(True)
        controls_layout.addWidget(self.data_points_input, 1, 1)

        self.start_button = QPushButton("开始采样")
        self.stop_button = QPushButton("停止采样")
        self.save_button = QPushButton("保存数据")
        self.stop_button.setEnabled(False)

        controls_layout.addWidget(self.start_button, 2, 0, 1, 2)
        controls_layout.addWidget(self.stop_button, 3, 0, 1, 2)
        controls_layout.addWidget(self.save_button, 4, 0, 1, 2)
        
        # 状态显示
        self.status_label = QLabel("状态: 已停止")
        controls_layout.addWidget(self.status_label, 5, 0, 1, 2)
        
        # 添加一个伸缩项，让控件靠上
        controls_layout.setRowStretch(6, 1)
        
        # --- 右侧绘图区域 ---
        plot_group = QGroupBox("实时波形")
        plot_layout = QVBoxLayout()
        plot_group.setLayout(plot_layout)

        self.plot_widget = pg.PlotWidget()
        plot_layout.addWidget(self.plot_widget)

        # 设置图表样式
        self.plot_widget.setBackground('k')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', 'ADC Value (0-255)')
        self.plot_widget.setLabel('bottom', 'Sample Index')
        self.plot_widget.setTitle("Real-Time ADC Data", color="w", size="12pt")
        self.plot_widget.getAxis('left').setPen(pg.mkPen(color='w'))
        self.plot_widget.getAxis('bottom').setPen(pg.mkPen(color='w'))
        self.plot_widget.setYRange(0, 255) # 8位ADC数据范围

        # 创建数据曲线
        self.curve = self.plot_widget.plot(pen=pg.mkPen(color='#00D0FF', width=2))
        
        layout.addWidget(controls_group)
        layout.addWidget(plot_group)
        
        # --- 连接信号 ---
        self.start_button.clicked.connect(self.start_scope)
        self.stop_button.clicked.connect(self.stop_scope)
        self.save_button.clicked.connect(self.save_scope_data)

    def start_scope(self):
        if not self.shared_memory:
            self.connect_shared_memory() # 尝试重新连接
            if not self.shared_memory:
                return # 如果还是失败，则不启动
        
        self.scope_timer.start()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.status_label.setText("状态: 正在采样...")
        self.statusBar().showMessage("示波器已启动", 2000)

    def stop_scope(self):
        self.scope_timer.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_label.setText("状态: 已停止")
        self.statusBar().showMessage("示波器已停止", 2000)
        
    def save_scope_data(self):
        if self.scope_data.size == 0:
            self.show_error("没有数据可以保存！")
            return
        
        # 弹出文件保存对话框
        path, _ = QFileDialog.getSaveFileName(self, "保存波形数据", "", "CSV Files (*.csv);;Text Files (*.txt)")
        
        if path:
            try:
                np.savetxt(path, self.scope_data, fmt='%d', delimiter=',', header='ADC_Value', comments='')
                self.statusBar().showMessage(f"数据成功保存到 {path}", 5000)
            except Exception as e:
                self.show_error(f"保存文件失败: {e}")

    def update_scope_plot(self):
        """核心函数：从共享内存读取数据并更新图表"""
        if not self.shared_memory:
            return
        try:
            self.shared_memory.seek(0)
            
            # 读取前2个字节，这是网络字节序的采样点数
            count_bytes = self.shared_memory.read(2)
            if len(count_bytes) < 2: return # 数据不完整
            
            sample_count = struct.unpack('>H', count_bytes)[0]
            
            if 0 < sample_count <= (SHARED_MEM_SIZE - 2):
                adc_data_bytes = self.shared_memory.read(sample_count)
                # 将字节数据转换为 numpy 数组 (8位无符号整数)
                self.scope_data = np.frombuffer(adc_data_bytes, dtype=np.uint8)
                
                # 更新图表和点数显示
                self.curve.setData(self.scope_data)
                self.data_points_input.setText(str(self.scope_data.size))
                
        except Exception as e:
            print(f"Error reading shared memory or updating plot: {e}")
            self.stop_scope() # 发生严重错误时停止

    # =============================================================================
    # 选项卡 2: 信号发生与协议调试
    # =============================================================================
    def init_control_tab(self):
        main_layout = QVBoxLayout(self.control_tab)

        # --- 串口配置 ---
        serial_group = QGroupBox("串口配置")
        serial_layout = QHBoxLayout()
        serial_group.setLayout(serial_layout)
        
        serial_layout.addWidget(QLabel("端口:"))
        self.ports_combo = QComboBox()
        serial_layout.addWidget(self.ports_combo)
        
        serial_layout.addWidget(QLabel("波特率:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        serial_layout.addWidget(self.baud_combo)
        
        self.refresh_ports_button = QPushButton("刷新")
        serial_layout.addWidget(self.refresh_ports_button)
        
        self.connect_serial_button = QPushButton("打开串口")
        self.disconnect_serial_button = QPushButton("关闭串口")
        self.disconnect_serial_button.setEnabled(False)
        serial_layout.addWidget(self.connect_serial_button)
        serial_layout.addWidget(self.disconnect_serial_button)
        
        self.refresh_ports() # 初始化时刷新一次

        # --- DAC 函数发生器 ---
        dac_group = QGroupBox("DAC 函数发生器")
        dac_layout = QGridLayout()
        dac_group.setLayout(dac_layout)
        
        dac_layout.addWidget(QLabel("波形:"), 0, 0)
        self.wave_sine = QRadioButton("正弦")
        self.wave_square = QRadioButton("方波")
        self.wave_triangle = QRadioButton("三角")
        self.wave_sine.setChecked(True)
        dac_layout.addWidget(self.wave_sine, 0, 1)
        dac_layout.addWidget(self.wave_square, 0, 2)
        dac_layout.addWidget(self.wave_triangle, 0, 3)
        
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

        # --- 通用指令收发 ---
        cmd_group = QGroupBox("通用指令调试")
        cmd_layout = QVBoxLayout()
        cmd_group.setLayout(cmd_layout)

        cmd_layout.addWidget(QLabel("输入指令:"))
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("在此输入ASCII或HEX指令 (例如 'CMD:START' 或 'AA BB 01')")
        cmd_layout.addWidget(self.cmd_input)
        
        send_layout = QHBoxLayout()
        self.send_cmd_button = QPushButton("发送")
        send_layout.addWidget(self.send_cmd_button)
        cmd_layout.addLayout(send_layout)
        
        cmd_layout.addWidget(QLabel("接收数据:"))
        self.received_text = QTextEdit()
        self.received_text.setReadOnly(True)
        self.received_text.setFontFamily("Courier New")
        cmd_layout.addWidget(self.received_text)

        # --- 组合布局 ---
        main_layout.addWidget(serial_group)
        main_layout.addWidget(dac_group)
        main_layout.addWidget(cmd_group)

        # --- 连接信号 ---
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        self.connect_serial_button.clicked.connect(self.connect_serial)
        self.disconnect_serial_button.clicked.connect(self.disconnect_serial)
        self.start_dac_button.clicked.connect(self.send_dac_command)
        self.stop_dac_button.clicked.connect(lambda: self.send_serial_command("DAC,STOP\n"))
        self.send_cmd_button.clicked.connect(self.send_general_command)
        
    def refresh_ports(self):
        self.ports_combo.clear()
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.ports_combo.addItem("未找到串口")
        else:
            for port in sorted(ports):
                self.ports_combo.addItem(f"{port.device} - {port.description}", port.device)
        self.statusBar().showMessage("串口列表已刷新", 2000)
        
    def connect_serial(self):
        port_name = self.ports_combo.currentData()
        baud_rate = int(self.baud_combo.currentText())
        if not port_name:
            self.show_error("请选择一个有效的串口！")
            return
            
        try:
            self.serial_connection = serial.Serial(port_name, baud_rate, timeout=0.5)
            self.connect_serial_button.setEnabled(False)
            self.disconnect_serial_button.setEnabled(True)
            self.ports_combo.setEnabled(False)
            self.baud_combo.setEnabled(False)
            self.statusBar().showMessage(f"成功连接到 {port_name} @ {baud_rate} bps", 5000)
        except serial.SerialException as e:
            self.show_error(f"无法打开串口 {port_name}: {e}")

    def disconnect_serial(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        self.serial_connection = None
        self.connect_serial_button.setEnabled(True)
        self.disconnect_serial_button.setEnabled(False)
        self.ports_combo.setEnabled(True)
        self.baud_combo.setEnabled(True)
        self.statusBar().showMessage("串口已关闭", 2000)

    def send_serial_command(self, command):
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(command.encode('ascii'))
                self.received_text.append(f"-> {command.strip()}")
                # 简单的回读
                response = self.serial_connection.readline().decode('ascii', errors='ignore').strip()
                if response:
                    self.received_text.append(f"<- {response}")
            except serial.SerialException as e:
                self.show_error(f"串口通信错误: {e}")
                self.disconnect_serial()
        else:
            self.show_error("串口未连接！")
            
    def send_dac_command(self):
        if self.wave_sine.isChecked(): wave = "SINE"
        elif self.wave_square.isChecked(): wave = "SQUARE"
        else: wave = "TRIANGLE"
        
        try:
            freq = float(self.freq_input.text())
            amp = float(self.amp_input.text())
            command = f"DAC,{wave},{freq},{amp}\n"
            self.send_serial_command(command)
        except ValueError:
            self.show_error("频率和幅值必须是有效的数字！")
            
    def send_general_command(self):
        command = self.cmd_input.text()
        if not command: return
        self.send_serial_command(command + '\n') # 假设指令需要换行符结尾

    # =============================================================================
    # 通用辅助函数
    # =============================================================================
    def show_error(self, message):
        QMessageBox.critical(self, "错误", message)

    def closeEvent(self, event):
        """窗口关闭时，确保资源被正确释放"""
        print("正在关闭应用程序并释放资源...")
        self.stop_scope()
        self.disconnect_serial()
        if self.shared_memory:
            self.shared_memory.close()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = FPGAMasterControl()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()