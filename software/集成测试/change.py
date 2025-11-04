# =============================================================================
    # 协议帧解析功能 (已按新结构修改)
    # =============================================================================
    def parse_received_frame(self, frame_data):
        """
        解析接收到的数据帧。
        新帧结构: [数据部分(Payload), 协议选择字节(1B), 帧尾(b'\r\n')]
        """
        try:
            # 新的检查：帧至少需要包含 [协议字节, 帧尾] (3字节)
            if len(frame_data) < 3 or frame_data[-2:] != b'\r\n':
                return "无效或不完整帧 (结构错误或无帧尾)"
            
            # 协议字节现在位于帧尾 b'\r\n' 之前
            protocol_byte = frame_data[-3]
            protocol_type = protocol_byte & 0b111
            extended_flags = protocol_byte >> 3
            
            # 有效数据(Payload)是帧开头到协议字节(不含)的所有内容
            payload = frame_data[0:-3]

            if protocol_type == 0b000:
                # UART: 有效数据为整个负载
                return f"<- [UART] | Data: {payload.hex(' ').upper()}"
            
            elif protocol_type == 0b001:
                # I2C: 有效数据为寄存器地址、写入数据和请求读回的数据
                addr_width = extended_flags & 0b1
                device_addr = payload[0]
                if addr_width: 
                    reg_addr, data_start = int.from_bytes(payload[1:3], 'big'), 3
                    reg_addr_str = f"0x{reg_addr:04X}"
                else: 
                    reg_addr, data_start = payload[1], 2
                    reg_addr_str = f"0x{reg_addr:02X}"
                
                # 读取长度在帧的最后一个有效字节
                read_len = payload[-1] if len(payload) > 0 else 0
                
                # 假设返回帧的有效负载就是I2C读回的数据（如果有的话）
                # 注意：实际接收的I2C帧结构需要根据FPGA实际实现来确定
                # 此处假定 payload 就是I2C读回的字节
                if len(payload) > 0 and read_len > 0:
                    read_data = payload[:read_len] # 假设读回的数据是负载的前 read_len 字节
                    return (f"<- [I2C-Rx] | Dev: 0x{device_addr:02X}, Reg: {reg_addr_str}, "
                            f"Read Data: [{read_data.hex(' ').upper()}]")
                else:
                    return (f"<- [I2C-Tx] | Dev: 0x{device_addr:02X}, Reg: {reg_addr_str}, "
                            f"No read data in response frame.")

            elif protocol_type == 0b010:
                # SPI: 有效数据为读回的数据。
                # 假设发送帧结构中 read_len 是负载的最后一个字节。
                # 在接收帧中，有效数据应为MISO读回的字节。
                # 简化处理：假设接收帧的 payload 就是读回的 SPI 数据。
                if payload:
                    return f"<- [SPI-Rx] | Data: [{payload.hex(' ').upper()}]"
                else:
                    return "<- [SPI-Rx] | No read data received."

            elif protocol_type == 0b011:
                # PWM: 有效数据为各通道的配置参数
                enable_byte = extended_flags & 0b1111
                channels = []
                for i in range(4):
                    offset = i * 5
                    # 假设返回帧负载包含了4个通道的 (PSC, ARR, Duty) 共20字节
                    if len(payload) < 20:
                        channels.append("数据不完整")
                        break
                        
                    psc = int.from_bytes(payload[offset:offset+2], 'big')
                    arr = int.from_bytes(payload[offset+2:offset+4], 'big')
                    duty = payload[offset+4]
                    enabled = (enable_byte >> i) & 0b1
                    
                    status = "ON" if enabled else "OFF"
                    channels.append(f"CH{i+1}: {status} (PSC={psc}, ARR={arr}, Duty={duty})")
                    
                return f"<- [PWM] | Status: {' | '.join(channels)}"

            elif protocol_type == 0b100:  # CAN协议
                # CAN: 有效数据为 CAN ID、帧类型和数据负载             
                can_id = int.from_bytes(payload[0:5], 'big')
                can_data = payload[5:] # 8 bytes data
                # 将 can_id 格式化为 10 位十六进制数 ---
                return (f"<- [CAN-Rx] | ID: 0x{can_id:010X}, "
                        f"Data: [{can_data.hex(' ').upper()}]")

            elif protocol_type == 0b101:  #数字信号测量
                # 协议结构: 频率(4字节) + 占空比(2字节)
                if len(payload) < 6:
                    return "<- [DIGITAL] | 数据不完整 (需要6字节有效负载)"
                
                # 解析频率 (4字节大端序)
                freq_bytes = payload[0:4]
                frequency = int.from_bytes(freq_bytes, 'big')
                
                # 解析占空比 (2字节大端序)
                duty_bytes = payload[4:6]
                duty_value = int.from_bytes(duty_bytes, 'big')
                duty_percent = duty_value / 10.0  # 转换为百分比格式
                _duty_percent = (1000 - duty_value) / 10.0
                # 计算高电平时间
                if frequency > 0:
                    duty_time_us = (duty_percent / 100) * (1000 / frequency) * 1000
                    _duty_time_us = (_duty_percent / 100) * (1000 / frequency) * 1000
                else:
                    duty_time_us = 0

                return (f"<- [DIGITAL] | Freq: {frequency} Hz, Duty: {duty_percent:.1f}%, High Time: {duty_time_us:.3f} us, Low Time: {_duty_time_us:.3f} us")
            
            else: # 未知协议
                return f"<- [未知协议] | Type: 0x{protocol_type:02X}, Data: {payload.hex(' ').upper()}"
        
        except IndexError: 
            return "帧数据不完整（索引错误）"
        except Exception as e: 
            return f"解析错误: {str(e)}"
            
    def check_control_serial_data(self):
        """
        检查协议串口数据，按新结构 [Payload, Proto, Tail] 查找帧。
        """
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
            # 帧头(b'\x20\x25')已取消，我们现在只通过帧尾(b'\r\n')来切分数据包
            # 查找第一个出现的帧尾
            end_idx = self.control_serial_buffer.find(b'\r\n')
            
            # 如果没有找到帧尾，说明数据包不完整，退出循环等待更多数据
            if end_idx == -1: 
                break
            
            # 提取从缓冲区开始到第一个帧尾的完整帧
            # 'end_idx' 指向 '\r'，所以我们需要 +2 来包含 '\r\n'
            frame_data = bytes(self.control_serial_buffer[0 : end_idx + 2])
            
            # 解析这个帧
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            parsed_info = self.parse_received_frame(frame_data)
            
            # --- 修改: 根据协议类型，将数据显示到不同的窗口 ---
            if parsed_info.startswith("<- [DIGITAL]"):
                # 如果是数字信号测量结果，显示在专用窗口
                # 清除协议头，只显示数据
                display_text = parsed_info.replace("<- [DIGITAL] | ", "")
                self.digital_signal_display.append(f"[{timestamp}] {display_text}")
            else:
                # 其他协议信息显示在通用接收历史窗口
                self.received_data_display.append(f"[{timestamp}] {parsed_info}")
            
            # 从缓冲区移除已经处理过的帧
            self.control_serial_buffer = self.control_serial_buffer[end_idx + 2:]