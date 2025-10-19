# -*- coding: utf-8 -*-
"""
scope_plotter.py

功能:
1. 通过共享内存从 C++ UDP 接收器读取 ADC 数据。
2. 使用 pyqtgraph 实时显示数据波形。

使用方法:
1. 首先运行 C++ UDP 接收程序 (udp_receiver_sm.exe)。
2. 然后运行此 Python 脚本。
"""
import mmap
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import sys
import struct

# --- 配置参数 ---
SHARED_MEM_NAME = "FPGA_ADC_DATA" # 必须与 C++ 程序中的名称完全一致
SHARED_MEM_SIZE = 2048            # 必须与 C++ 程序中的大小一致
UPDATE_INTERVAL_MS = 20           # 更新图表的间隔（毫秒），50Hz刷新率

class RealTimeScope(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        
        # --- 共享内存初始化 ---
        self.shared_memory = None
        try:
            # 在 Windows 上，使用 tagname 连接到命名的共享内存
            # fileno=-1 表示我们不是在映射一个真实的文件
            self.shared_memory = mmap.mmap(-1, SHARED_MEM_SIZE, tagname=SHARED_MEM_NAME)
            print(f"Successfully connected to shared memory '{SHARED_MEM_NAME}'.")
        except FileNotFoundError:
            print(f"Error: Shared memory '{SHARED_MEM_NAME}' not found.")
            print("Please ensure the C++ receiver program is running first.")
            sys.exit(1)
        except Exception as e:
            print(f"An unexpected error occurred while accessing shared memory: {e}")
            sys.exit(1)

        # --- UI 界面设置 ---
        self.setWindowTitle("FPGA ADC Real-Time Scope")
        self.resize(1000, 600)
        
        # 创建一个绘图控件
        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)
        
        # 设置图表样式
        self.plot_widget.setBackground('k') # 黑色背景
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', 'ADC Value')
        self.plot_widget.setLabel('bottom', 'Sample Index')
        self.plot_widget.setTitle("Real-Time ADC Data", color="w", size="12pt")
        self.plot_widget.getAxis('left').setPen(pg.mkPen(color='w'))
        self.plot_widget.getAxis('bottom').setPen(pg.mkPen(color='w'))
        
        # 创建一个数据曲线
        self.curve = self.plot_widget.plot(pen=pg.mkPen(color='#00D0FF', width=2))
        
        # --- 定时器设置 ---
        # 创建一个定时器，用于周期性地更新图表
        self.timer = QtCore.QTimer()
        self.timer.setInterval(UPDATE_INTERVAL_MS)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def update_plot(self):
        """
        从共享内存读取数据并更新图表。
        """
        try:
            # --- 从共享内存读取数据 ---
            # 首先回到内存块的开头
            self.shared_memory.seek(0)
            
            # 读取前2个字节，这是网络字节序的采样点数
            count_bytes = self.shared_memory.read(2)
            # 使用'>H'格式解包：'>'表示大端(网络字节序)，'H'表示无符号短整型(2字节)
            sample_count = struct.unpack('>H', count_bytes)[0]
            
            if 0 < sample_count <= (SHARED_MEM_SIZE - 2):
                # 读取ADC数据
                adc_data_bytes = self.shared_memory.read(sample_count)
                # 将字节数据转换为 numpy 数组 (8位无符号整数)
                adc_samples = np.frombuffer(adc_data_bytes, dtype=np.uint8)
                
                # --- 更新图表 ---
                self.curve.setData(adc_samples)
                
        except Exception as e:
            print(f"Error during plot update or reading memory: {e}")
            self.timer.stop() # 发生错误时停止更新

    def closeEvent(self, event):
        """
        窗口关闭时，确保资源被正确释放。
        """
        print("Closing application and releasing resources.")
        self.timer.stop()
        if self.shared_memory:
            self.shared_memory.close()
        event.accept()

def main():
    # 创建PyQt应用程序实例
    app = QtWidgets.QApplication(sys.argv)
    # 创建并显示主窗口
    window = RealTimeScope()
    window.show()
    # 启动事件循环 (这里是修改点)
    sys.exit(app.exec())

if __name__ == '__main__':
    main()