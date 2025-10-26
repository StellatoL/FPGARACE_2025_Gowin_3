import socket
import time
import numpy as np
import threading
import sys
import argparse
from datetime import datetime

# 默认配置参数
DEFAULT_PORT = 6102
DEFAULT_SAMPLE_RATE = 1000000  # 1Msps
DEFAULT_FREQUENCY = 1000  # 1kHz
DEFAULT_AMPLITUDE = 127  # 0-255范围
DEFAULT_PACKET_SIZE = 2048
DEFAULT_WAVEFORM = "sine"

class WaveformGenerator:
    def __init__(self, sample_rate, frequency, amplitude, waveform):
        self.sample_rate = sample_rate
        self.frequency = frequency
        self.amplitude = amplitude
        self.waveform_type = waveform
        self.phase = 0.0
        self.running = True
        self.last_time = time.time()
        self.samples_sent = 0
        self.data_rate = 0
        self.lock = threading.Lock()
        
        # 波形参数
        self.duty_cycle = 0.5  # 方波占空比
        
        # 统计信息
        self.start_time = time.time()
        self.total_packets = 0
        self.total_bytes = 0
    
    def generate_waveform(self, num_samples):
        """生成指定数量的波形样本"""
        t = np.arange(num_samples) / self.sample_rate + self.phase
        period = 1.0 / self.frequency if self.frequency > 0 else 1.0
        
        if self.waveform_type == "sine":
            # 正弦波: sin(2πft)
            samples = np.sin(2 * np.pi * self.frequency * t)
        elif self.waveform_type == "square":
            # 方波: 根据占空比生成
            phase = (t / period) % 1.0
            samples = np.where(phase < self.duty_cycle, 1.0, -1.0)
        elif self.waveform_type == "triangle":
            # 三角波: 使用绝对值函数生成
            phase = (t / period) % 1.0
            samples = 2 * np.abs(2 * phase - 1) - 1
        elif self.waveform_type == "sawtooth":
            # 锯齿波: 线性上升
            phase = (t / period) % 1.0
            samples = 2 * phase - 1
        else:
            # 默认正弦波
            samples = np.sin(2 * np.pi * self.frequency * t)
        
        # 更新相位
        self.phase += num_samples / self.sample_rate * self.frequency
        self.phase %= 1.0
        
        # 转换为8位ADC值 (0-255)
        adc_values = np.uint8((samples * self.amplitude) + 128)
        return adc_values.tobytes()
    
    def update_parameters(self, sample_rate=None, frequency=None, amplitude=None, waveform=None):
        """更新波形参数"""
        with self.lock:
            if sample_rate is not None:
                self.sample_rate = max(1000, sample_rate)  # 最小1ksps
            if frequency is not None:
                self.frequency = max(1, frequency)  # 最小1Hz
            if amplitude is not None:
                self.amplitude = max(1, min(127, amplitude))  # 限制在1-127范围
            if waveform is not None:
                self.waveform_type = waveform
    
    def get_stats(self):
        """获取统计信息"""
        elapsed = time.time() - self.start_time
        return {
            "total_packets": self.total_packets,
            "total_bytes": self.total_bytes,
            "data_rate_mbps": self.data_rate * 8 / 1e6,
            "samples_per_sec": self.samples_sent / elapsed,
            "elapsed_time": elapsed
        }
    
    def run(self, sock, dest_addr, packet_size):
        """主运行循环，生成并发送数据"""
        samples_per_packet = packet_size - 2  # 2字节用于长度信息
        interval = samples_per_packet / self.sample_rate
        
        while self.running:
            start_time = time.time()
            
            # 生成波形数据
            data = self.generate_waveform(samples_per_packet)
            
            # 添加长度信息 (大端序)
            length = len(data)
            header = length.to_bytes(2, 'big')
            packet = header + data
            
            # 发送数据
            try:
                sock.sendto(packet, dest_addr)
                with self.lock:
                    self.total_packets += 1
                    self.total_bytes += len(packet)
                    self.samples_sent += samples_per_packet
            except socket.error as e:
                print(f"发送错误: {e}")
                break
            
            # 计算数据率
            current_time = time.time()
            elapsed = current_time - self.last_time
            if elapsed >= 0.5:  # 每0.5秒更新一次数据率
                self.data_rate = self.total_bytes / elapsed
                self.last_time = current_time
                self.total_bytes = 0
            
            # 控制发送速率
            process_time = time.time() - start_time
            sleep_time = max(0, interval - process_time)
            time.sleep(sleep_time)

def control_thread(generator):
    """控制线程，允许用户交互式调整参数"""
    print("\n控制命令:")
    print("  f [频率] - 设置波形频率 (Hz)")
    print("  a [幅度] - 设置波形幅度 (1-127)")
    print("  s [采样率] - 设置采样率 (sps)")
    print("  w [波形类型] - 设置波形类型 (sine, square, triangle, sawtooth)")
    print("  d [占空比] - 设置方波占空比 (0.1-0.9)")
    print("  stats - 显示统计信息")
    print("  q - 退出程序")
    
    while True:
        try:
            cmd = input("> ").strip().split()
            if not cmd:
                continue
                
            if cmd[0] == 'f' and len(cmd) > 1:
                frequency = float(cmd[1])
                generator.update_parameters(frequency=frequency)
                print(f"频率设置为: {frequency} Hz")
                
            elif cmd[0] == 'a' and len(cmd) > 1:
                amplitude = int(cmd[1])
                generator.update_parameters(amplitude=amplitude)
                print(f"幅度设置为: {amplitude}")
                
            elif cmd[0] == 's' and len(cmd) > 1:
                sample_rate = int(cmd[1])
                generator.update_parameters(sample_rate=sample_rate)
                print(f"采样率设置为: {sample_rate} sps")
                
            elif cmd[0] == 'w' and len(cmd) > 1:
                waveform = cmd[1].lower()
                if waveform in ['sine', 'square', 'triangle', 'sawtooth']:
                    generator.update_parameters(waveform=waveform)
                    print(f"波形设置为: {waveform}")
                else:
                    print("无效波形类型 (sine, square, triangle, sawtooth)")
                    
            elif cmd[0] == 'd' and len(cmd) > 1:
                duty_cycle = float(cmd[1])
                if 0.1 <= duty_cycle <= 0.9:
                    with generator.lock:
                        generator.duty_cycle = duty_cycle
                    print(f"占空比设置为: {duty_cycle}")
                else:
                    print("占空比必须在0.1-0.9之间")
                    
            elif cmd[0] == 'stats':
                stats = generator.get_stats()
                print("\n--- 统计信息 ---")
                print(f"运行时间: {stats['elapsed_time']:.1f} 秒")
                print(f"总数据包: {stats['total_packets']}")
                print(f"总样本数: {stats['samples_per_sec'] * stats['elapsed_time']:.0f}")
                print(f"样本率: {stats['samples_per_sec']:.1f} sps")
                print(f"数据率: {stats['data_rate_mbps']:.2f} Mbps")
                
            elif cmd[0] == 'q':
                generator.running = False
                print("正在停止程序...")
                break
                
            else:
                print("未知命令")
                
        except Exception as e:
            print(f"命令错误: {e}")

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='ADC波形模拟器')
    parser.add_argument('--host', type=str, default='10.79.9.18', help='目标主机IP (默认: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help=f'目标端口 (默认: {DEFAULT_PORT})')
    parser.add_argument('--sample-rate', type=int, default=DEFAULT_SAMPLE_RATE, 
                        help=f'采样率 (sps) (默认: {DEFAULT_SAMPLE_RATE})')
    parser.add_argument('--frequency', type=int, default=DEFAULT_FREQUENCY, 
                        help=f'波形频率 (Hz) (默认: {DEFAULT_FREQUENCY})')
    parser.add_argument('--amplitude', type=int, default=DEFAULT_AMPLITUDE, 
                        help=f'波形幅度 (1-127) (默认: {DEFAULT_AMPLITUDE})')
    parser.add_argument('--waveform', type=str, default=DEFAULT_WAVEFORM, 
                        choices=['sine', 'square', 'triangle', 'sawtooth'],
                        help=f'波形类型 (默认: {DEFAULT_WAVEFORM})')
    parser.add_argument('--packet-size', type=int, default=DEFAULT_PACKET_SIZE, 
                        help=f'UDP包大小 (字节) (默认: {DEFAULT_PACKET_SIZE})')
    
    args = parser.parse_args()
    
    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest_addr = (args.host, args.port)
    
    # 创建波形生成器
    generator = WaveformGenerator(
        sample_rate=args.sample_rate,
        frequency=args.frequency,
        amplitude=args.amplitude,
        waveform=args.waveform
    )
    
    print(f"ADC波形模拟器 v1.0")
    print(f"目标: {args.host}:{args.port}")
    print(f"采样率: {args.sample_rate} sps")
    print(f"波形: {args.waveform}, 频率: {args.frequency} Hz, 幅度: {args.amplitude}")
    print(f"数据包大小: {args.packet_size} 字节")
    print("=" * 50)
    
    try:
        # 启动控制线程
        ctrl_thread = threading.Thread(target=control_thread, args=(generator,), daemon=True)
        ctrl_thread.start()
        
        # 主线程运行波形生成器
        generator.run(sock, dest_addr, args.packet_size)
        
    except KeyboardInterrupt:
        print("\n程序被用户中断")
        generator.running = False
    
    finally:
        sock.close()
        stats = generator.get_stats()
        print("\n程序结束")
        print(f"总运行时间: {stats['elapsed_time']:.1f} 秒")
        print(f"总发送数据包: {stats['total_packets']}")
        print(f"总发送样本数: {stats['samples_per_sec'] * stats['elapsed_time']:.0f}")
        print(f"平均样本率: {stats['samples_per_sec']:.1f} sps")
        print(f"平均数据率: {stats['data_rate_mbps']:.2f} Mbps")
        sys.exit(0)

if __name__ == "__main__":
    main()