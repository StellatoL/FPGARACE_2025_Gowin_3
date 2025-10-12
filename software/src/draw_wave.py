import sys
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QPushButton, QComboBox, QLabel, QHBoxLayout, 
                             QProgressBar, QMessageBox, QSizePolicy)
from PyQt6.QtGui import QPainter, QPixmap, QPen, QColor
from PyQt6.QtCore import Qt, QPoint, QTimer
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter, butter, filtfilt
from scipy.interpolate import interp1d, UnivariateSpline
import traceback

# --- Constants ---
CANVAS_WIDTH = 800  # 统一宽度
CANVAS_HEIGHT = 400
NUM_SAMPLES = 512
BAUD_RATE = 115200
FFT_DOMINANCE_THRESHOLD = 0.1
NUM_DOMINANT_FREQUENCIES = 8
SMOOTHING_WINDOW_RATIO = 0.05
MIN_FREQUENCY_THRESHOLD = 0.01
INTERPOLATION_KIND = 'cubic'
MAX_SERIAL_RETRIES = 3

def set_chinese_font():
    """设置Matplotlib中文字体支持"""
    try:
        plt.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.sans-serif'] = [
            'SimHei', 'Microsoft YaHei', 'KaiTi', 'SimSun', 
            'Arial Unicode MS', 'PingFang SC', 'Heiti SC', 
            'WenQuanYi Micro Hei'
        ]
        plt.rcParams['axes.unicode_minus'] = False
        return True
    except:
        return False

if not set_chinese_font():
    print("Warning: Chinese font setup failed")

class WaveformDrawer(QWidget):
    """Custom widget for drawing waveforms"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(CANVAS_WIDTH, CANVAS_HEIGHT)
        self.canvas = QPixmap(self.size())
        self.canvas.fill(Qt.GlobalColor.white)
        self.last_point = QPoint()
        self.drawing = False
        self.points = []
        self.min_x = float('inf')
        self.max_x = float('-inf')

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drawing = True
            self.last_point = event.pos()
            self._add_point(event.pos())

    def mouseMoveEvent(self, event):
        if self.drawing:
            painter = QPainter(self.canvas)
            pen = QPen(Qt.GlobalColor.blue, 2, Qt.PenStyle.SolidLine)
            painter.setPen(pen)
            painter.drawLine(self.last_point, event.pos())
            painter.end()
            self.last_point = event.pos()
            self._add_point(event.pos())
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drawing = False

    def _add_point(self, point):
        """Add point and update bounds"""
        x, y = point.x(), point.y()
        self.points.append((x, y))
        self.min_x = min(self.min_x, x)
        self.max_x = max(self.max_x, x)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawPixmap(QPoint(), self.canvas)

    def clear_canvas(self):
        self.canvas.fill(Qt.GlobalColor.white)
        self.points = []
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.update()

    def get_waveform_data(self):
        """Extract and resample waveform data"""
        if not self.points:
            return None

        # Handle single point case
        if len(self.points) == 1:
            x, y = self.points[0]
            waveform = np.full(NUM_SAMPLES, CANVAS_HEIGHT - y)
            return (waveform - CANVAS_HEIGHT/2) / (CANVAS_HEIGHT/2)

        # Create base waveform
        waveform = np.full(CANVAS_WIDTH, float(CANVAS_HEIGHT / 2))
        sorted_points = sorted(self.points, key=lambda p: p[0])
        
        # Only process points within drawn range
        valid_points = [p for p in sorted_points 
                       if self.min_x <= p[0] <= self.max_x]
        
        if not valid_points:
            return None
            
        # Add boundary points if needed
        if valid_points[0][0] > 0:
            valid_points.insert(0, (0, valid_points[0][1]))
        if valid_points[-1][0] < CANVAS_WIDTH - 1:
            valid_points.append((CANVAS_WIDTH - 1, valid_points[-1][1]))
        
        # Linear interpolation
        for i in range(len(valid_points) - 1):
            p1, p2 = valid_points[i], valid_points[i+1]
            x1, y1 = int(p1[0]), p1[1]
            x2, y2 = int(p2[0]), p2[1]
            
            # Ensure x2 >= x1
            if x2 < x1:
                x1, x2 = x2, x1
                y1, y2 = y2, y1
            
            # Calculate number of points
            num_points = x2 - x1 + 1
            
            # Handle adjacent points
            if num_points <= 1:
                waveform[x1] = y1
                continue
            
            # Vectorized interpolation
            x_vals = np.arange(x1, x2 + 1)
            y_vals = y1 + (y2 - y1) * ((x_vals - x1) / (x2 - x1))
            
            # Ensure slice length matches
            end_idx = min(x2 + 1, CANVAS_WIDTH)
            if end_idx - x1 != len(y_vals):
                # Adjust y_vals length to match target slice
                y_vals = y_vals[:end_idx - x1]
            
            # Assign waveform data
            waveform[x1:end_idx] = y_vals

        # Normalize and resample
        waveform = CANVAS_HEIGHT - waveform
        waveform = (waveform - CANVAS_HEIGHT / 2) / (CANVAS_HEIGHT / 2)
        
        # Only resample the drawn portion
        start_idx = max(0, int(self.min_x))
        end_idx = min(CANVAS_WIDTH, int(self.max_x) + 1)
        valid_waveform = waveform[start_idx:end_idx]
        
        if len(valid_waveform) < 2:
            return None
            
        # Create interpolation function for resampling
        x_original = np.linspace(0, 1, len(valid_waveform))
        interp_func = interp1d(
            x_original, 
            valid_waveform, 
            kind='linear', 
            fill_value='extrapolate'
        )
        x_new = np.linspace(0, 1, NUM_SAMPLES)
        return interp_func(x_new)

class MplCanvas(FigureCanvas):
    """Embedded matplotlib widget with fixed width"""
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)
        self.setParent(parent)
        
        # Set fixed width to match drawing canvas
        self.setMinimumWidth(CANVAS_WIDTH)
        self.setMaximumWidth(CANVAS_WIDTH)
        
        # Set size policy to fixed width, expanding height
        self.setSizePolicy(
            QSizePolicy.Policy.Fixed, 
            QSizePolicy.Policy.Expanding
        )

class MainWindow(QMainWindow):
    """Main application window with aligned components"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("手绘波形分析与发送工具 v1.3 (对齐优化版)")
        self.setMinimumSize(900, 700)
        
        # Main layout
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)
        
        # Drawing area
        draw_container = QWidget()
        draw_layout = QVBoxLayout()
        draw_layout.setContentsMargins(0, 0, 0, 0)
        
        draw_label = QLabel("绘制区域:")
        draw_layout.addWidget(draw_label)
        
        self.drawer = WaveformDrawer()
        draw_layout.addWidget(self.drawer, alignment=Qt.AlignmentFlag.AlignCenter)
        
        draw_container.setLayout(draw_layout)
        main_layout.addWidget(draw_container)
        
        # Preview area
        preview_container = QWidget()
        preview_layout = QVBoxLayout()
        preview_layout.setContentsMargins(0, 0, 0, 0)
        
        plot_label = QLabel("发送数据预览图:")
        preview_layout.addWidget(plot_label)
        
        self.plot_canvas = MplCanvas(self, width=CANVAS_WIDTH/100, height=3, dpi=100)
        preview_layout.addWidget(self.plot_canvas, alignment=Qt.AlignmentFlag.AlignCenter)
        
        preview_container.setLayout(preview_layout)
        main_layout.addWidget(preview_container)
        
        # Status area
        status_layout = QHBoxLayout()
        self.status_label = QLabel("请在上方区域绘制波形。")
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setVisible(False)
        status_layout.addWidget(self.status_label, 80)
        status_layout.addWidget(self.progress_bar, 20)
        main_layout.addLayout(status_layout)
        
        # Controls
        controls_layout = QHBoxLayout()
        self.serial_ports_combo = QComboBox()
        self.analyze_button = QPushButton("分析并发送")
        self.clear_button = QPushButton("清除画布")
        self.refresh_button = QPushButton("刷新串口")
        self.filter_send_button = QPushButton("滤波优化并发送")
        
        controls_layout.addWidget(QLabel("串口:"))
        controls_layout.addWidget(self.serial_ports_combo, 30)
        controls_layout.addWidget(self.refresh_button, 15)
        controls_layout.addStretch(10)
        controls_layout.addWidget(self.clear_button, 20)
        controls_layout.addWidget(self.analyze_button, 20)
        controls_layout.addWidget(self.filter_send_button, 25)
        
        main_layout.addLayout(controls_layout)
        
        # Set central widget
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
        # Initialize
        self.refresh_serial_ports()
        self.plot_data(None)
        self.serial_connection = None
        self.retry_count = 0
        
        # Connect signals
        self.analyze_button.clicked.connect(self.analyze_and_send)
        self.clear_button.clicked.connect(self.clear_all)
        self.refresh_button.clicked.connect(self.refresh_serial_ports)
        self.filter_send_button.clicked.connect(self.filter_optimize_and_send)

    def clear_all(self):
        """Clear drawing canvas and plots"""
        self.drawer.clear_canvas()
        self.plot_data(None)
        self.status_label.setText("画布已清除。")
        self.progress_bar.setVisible(False)

    def plot_data(self, data, plot_type='none', **kwargs):
        """Plot data on canvas"""
        self.plot_canvas.axes.clear()
        
        if plot_type == 'fft':
            frequencies, magnitudes = data
            self.plot_canvas.axes.stem(frequencies, magnitudes, basefmt=" ")
            self.plot_canvas.axes.set_title("FFT 频谱图")
            self.plot_canvas.axes.set_xlabel("频率 (Hz)")
            self.plot_canvas.axes.set_ylabel("振幅")
            self.plot_canvas.axes.set_xlim(0, NUM_SAMPLES // 2)
            
        elif plot_type == 'samples':
            self.plot_canvas.axes.plot(data, 'b-', label="量化波形")
            
            if 'original' in kwargs:
                orig = kwargs['original']
                self.plot_canvas.axes.plot(
                    orig, 
                    'r--', 
                    alpha=0.7, 
                    label="原始波形"
                )
                self.plot_canvas.axes.legend()
                
            self.plot_canvas.axes.set_title("采样量化波形")
            self.plot_canvas.axes.set_xlabel("采样点")
            self.plot_canvas.axes.set_ylabel("数值")
            self.plot_canvas.axes.grid(True, linestyle='--', alpha=0.6)
            
        else:
            self.plot_canvas.axes.set_title("等待数据...")
            self.plot_canvas.axes.text(
                0.5, 0.5, '此处将显示发送数据的预览图', 
                horizontalalignment='center', 
                verticalalignment='center',
                transform=self.plot_canvas.axes.transAxes
            )
            self.plot_canvas.axes.grid(False)
            
        self.plot_canvas.draw()

    def refresh_serial_ports(self):
        """Refresh available serial ports"""
        self.serial_ports_combo.clear()
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            self.serial_ports_combo.addItem("未找到串口")
        else:
            for port in ports:
                self.serial_ports_combo.addItem(
                    f"{port.device} - {port.description}", 
                    port.device
                )
                
        self.status_label.setText("串口列表已刷新。")

    def analyze_and_send(self):
        """Analyze and send without filtering"""
        try:
            self.progress_bar.setVisible(True)
            self.progress_bar.setValue(10)
            QApplication.processEvents()
            
            waveform = self.drawer.get_waveform_data()
            if waveform is None:
                self.status_label.setText("错误：画布为空，无法分析。")
                return
                
            # Simple quantization
            self.progress_bar.setValue(30)
            quantized_waveform = self._optimized_quantization(waveform)
            
            # Update UI
            self.progress_bar.setValue(70)
            self.status_label.setText("分析完成! 准备发送数据...")
            self.plot_data(quantized_waveform, plot_type='samples')
            
            # Format and send data
            data_parts = ["S"] + [str(int(v)) for v in quantized_waveform]
            data_to_send = ",".join(data_parts) + "\n"
            
            self.progress_bar.setValue(90)
            self.send_serial_data(data_to_send)
            self.progress_bar.setValue(100)
            
        except Exception as e:
            self.show_error(f"分析过程中出错: {str(e)}")
            traceback.print_exc()
        finally:
            QTimer.singleShot(1000, lambda: self.progress_bar.setVisible(False))

    def filter_optimize_and_send(self):
        """Filter, optimize, and send waveform"""
        try:
            self.progress_bar.setVisible(True)
            self.progress_bar.setValue(5)
            QApplication.processEvents()
            
            # Step 1: Get waveform
            waveform = self.drawer.get_waveform_data()
            if waveform is None or len(waveform) < 10:
                self.status_label.setText("错误：波形数据不足。")
                return
                
            # Step 2: Adaptive smoothing
            self.progress_bar.setValue(15)
            smoothed_waveform = self._adaptive_smoothing(waveform)
            
            # Step 3: FFT reconstruction
            self.progress_bar.setValue(35)
            reconstructed_waveform = self._intelligent_fft_reconstruction(smoothed_waveform)
            
            # Step 4: Interpolation
            self.progress_bar.setValue(55)
            interpolated_waveform = self._interpolation_optimization(reconstructed_waveform)
            
            # Step 5: Quantization
            self.progress_bar.setValue(75)
            quantized_waveform = self._optimized_quantization(interpolated_waveform)
            
            # Step 6: Get original for comparison
            original_quantized = self._optimized_quantization(smoothed_waveform)
            
            # Update UI
            self.progress_bar.setValue(90)
            self.status_label.setText(f"优化重构成功! 使用了{NUM_DOMINANT_FREQUENCIES}个主频率")
            self.plot_data(
                quantized_waveform, 
                plot_type='samples', 
                original=original_quantized
            )
            
            # Format and send data
            data_parts = ["S"] + [str(int(v)) for v in quantized_waveform]
            data_to_send = ",".join(data_parts) + "\n"
            
            self.send_serial_data(data_to_send)
            self.progress_bar.setValue(100)
            
        except Exception as e:
            self.show_error(f"滤波优化过程中出错: {str(e)}")
            traceback.print_exc()
        finally:
            QTimer.singleShot(1000, lambda: self.progress_bar.setVisible(False))

    def _adaptive_smoothing(self, waveform):
        """Adaptive smoothing with validation"""
        N = len(waveform)
        if N < 5:
            return waveform
            
        # Calculate window size
        window_size = max(5, min(21, int(N * SMOOTHING_WINDOW_RATIO)))
        if window_size % 2 == 0:
            window_size += 1
        window_size = min(window_size, N - 1)  # Ensure valid size
            
        polyorder = min(3, window_size - 1)
        
        try:
            # Savitzky-Golay filter
            smoothed = savgol_filter(waveform, window_size, polyorder)
            
            # Additional low-pass for longer signals
            if N > 50:
                nyquist = 0.5
                cutoff = min(0.3, nyquist * 0.9)  # Safe cutoff
                b, a = butter(4, cutoff / nyquist, btype='low')
                smoothed = filtfilt(b, a, smoothed)
                
            return smoothed
        except:
            return waveform

    def _intelligent_fft_reconstruction(self, waveform):
        """FFT reconstruction with proper handling"""
        N = len(waveform)
        
        # FFT processing
        fft_coeffs = np.fft.fft(waveform)
        magnitudes = np.abs(fft_coeffs)
        
        # Handle Nyquist component properly
        nyquist_index = N // 2
        positive_freqs = magnitudes[1:nyquist_index]
        
        # Calculate energy threshold
        total_energy = np.sum(positive_freqs)
        energy_threshold = total_energy * MIN_FREQUENCY_THRESHOLD
        
        # Find important frequencies
        important_indices = []
        for i in range(len(positive_freqs)):
            if positive_freqs[i] > energy_threshold:
                important_indices.append(i + 1)  # Offset for DC component
                
        # Ensure we have enough frequencies
        if len(important_indices) < NUM_DOMINANT_FREQUENCIES:
            num_to_take = min(NUM_DOMINANT_FREQUENCIES, len(positive_freqs))
            peak_indices = np.argsort(positive_freqs)[-num_to_take:] + 1
            important_indices = peak_indices.tolist()
            
        # Build filtered FFT coefficients
        filtered_fft_coeffs = np.zeros_like(fft_coeffs)
        filtered_fft_coeffs[0] = fft_coeffs[0]  # DC component
        
        # Handle Nyquist component
        if N % 2 == 0:
            filtered_fft_coeffs[nyquist_index] = fft_coeffs[nyquist_index]
        
        # Add important frequencies
        for i in important_indices:
            # Positive frequency
            filtered_fft_coeffs[i] = fft_coeffs[i]
            # Negative frequency (conjugate symmetric)
            if N - i < N:
                filtered_fft_coeffs[N - i] = fft_coeffs[N - i]
                
        # Inverse FFT
        reconstructed = np.fft.ifft(filtered_fft_coeffs).real
        return reconstructed

    def _interpolation_optimization(self, waveform):
        """Interpolation with validation"""
        N = len(waveform)
        if N < 4:
            return waveform
            
        x_original = np.linspace(0, 1, N)
        
        try:
            # Spline interpolation
            spline = UnivariateSpline(x_original, waveform, s=0.1, k=min(3, N-1))
            x_dense = np.linspace(0, 1, NUM_SAMPLES)
            return spline(x_dense)
        except:
            try:
                # Fallback to cubic interpolation
                interp_func = interp1d(
                    x_original, 
                    waveform, 
                    kind=INTERPOLATION_KIND,
                    fill_value="extrapolate"
                )
                x_dense = np.linspace(0, 1, NUM_SAMPLES)
                return interp_func(x_dense)
            except:
                return waveform

    def _optimized_quantization(self, waveform):
        """Safe quantization with validation"""
        if len(waveform) == 0:
            return np.array([], dtype=int)
            
        min_val = np.min(waveform)
        max_val = np.max(waveform)
        
        if np.isclose(min_val, max_val, atol=1e-6):
            return np.full(len(waveform), 128, dtype=int)
            
        # Normalize and quantize
        normalized = (waveform - min_val) / (max_val - min_val)
        quantized = (normalized * 255).astype(int)
        return np.clip(quantized, 0, 255)

    def send_serial_data(self, data_to_send):
        """Robust serial data transmission"""
        port_info = self.serial_ports_combo.currentData()
        if not port_info or port_info == "未找到串口":
            self.status_label.setText("发送失败：请选择有效串口")
            return
            
        port_name = port_info if isinstance(port_info, str) else port_info.device
        
        try:
            # Initialize or reconnect serial
            if (self.serial_connection is None or 
                not self.serial_connection.is_open or 
                self.serial_connection.port != port_name):
                
                if self.serial_connection and self.serial_connection.is_open:
                    self.serial_connection.close()
                    
                self.serial_connection = serial.Serial(
                    port=port_name,
                    baudrate=BAUD_RATE,
                    timeout=2,
                    write_timeout=2
                )
                
            # Send data
            self.serial_connection.write(data_to_send.encode('ascii'))
            self.status_label.setText(f"数据已通过 {port_name} 发送！")
            self.retry_count = 0  # Reset retry counter
            
        except serial.SerialException as e:
            self.retry_count += 1
            if self.retry_count <= MAX_SERIAL_RETRIES:
                self.status_label.setText(f"串口错误，重试 {self.retry_count}/{MAX_SERIAL_RETRIES}...")
                QTimer.singleShot(1000, lambda: self.send_serial_data(data_to_send))
            else:
                self.status_label.setText(f"串口错误: {str(e)}")
                self.retry_count = 0
                self.serial_connection = None

    def show_error(self, message):
        """Show error message dialog"""
        QMessageBox.critical(
            self, 
            "错误", 
            message, 
            QMessageBox.StandardButton.Ok
        )
        self.status_label.setText(message.split(':')[0])

    def closeEvent(self, event):
        """Cleanup on window close"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())