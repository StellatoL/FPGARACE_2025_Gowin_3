    # =============================================================================
    # 选项卡 2: 波形绘制与发送 (已修改)
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
        
        # --- 修改 controls_layout ---
        controls_layout = QHBoxLayout()
        controls_layout.addWidget(QLabel("发送串口:"))
        self.ports_combo_wf = QComboBox()
        self.refresh_button_wf = QPushButton("刷新")
        
        controls_layout.addWidget(self.ports_combo_wf, 1)
        controls_layout.addWidget(self.refresh_button_wf)
        controls_layout.addStretch()
        
        # --- 新增频率控制 ---
        controls_layout.addWidget(QLabel("频率选择 (0-15):"))
        self.wf_freq_input = QLineEdit("1") # 新增: 波形频率输入框
        self.wf_freq_input.setFixedWidth(50) 
        controls_layout.addWidget(self.wf_freq_input)
        # --- 结束新增 ---
        
        self.clear_button = QPushButton("清除画布")
        self.analyze_button = QPushButton("分析并发送")
        controls_layout.addWidget(self.clear_button)
        controls_layout.addWidget(self.analyze_button)
        
        sender_layout.addLayout(controls_layout)
        main_layout.addWidget(sender_group)
        main_layout.addStretch()
        
        self.refresh_button_wf.clicked.connect(self.refresh_all_ports)
        self.clear_button.clicked.connect(self.clear_waveform_canvas)
        self.analyze_button.clicked.connect(self.analyze_and_send_waveform)

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
            
            # --- 新增: 1.5. 获取并验证频率 ---
            try:
                freq_code = int(self.wf_freq_input.text())
                if not (0 <= freq_code <= 15):
                    self.show_error("频率选择值必须在 0 到 15 之间！")
                    return
            except ValueError:
                self.show_error("频率选择值必须是一个有效的整数 (0-15)！")
                return
            # --- 结束新增 ---

            # 2. 获取串口信息
            port_name = self.ports_combo_wf.currentData()
            if not port_name: 
                self.show_error("请选择发送串口！")
                return
            
            # 3. 循环512次，为每个数据点构建并发送一个数据帧
            with serial.Serial(port_name, 1000000, timeout=2, write_timeout=2) as ser: #注意：这边波特率不是默认的，而是固定1000000
                
                # --- 新增: 组合波形和频率 ---
                # 波形选择(低4位) = 5 (自定义波形)
                # 频率选择(高4位) = freq_code
                combined_byte = (freq_code << 4) | 0x05
                # --- 结束新增 ---
                
                for address in range(NUM_SAMPLES_SEND):
                    data_point = quantized_waveform[address]                    
                    frame = bytearray() # 创建帧的字节数组                                      
                    frame.extend(b'\x20\x25') # 帧头: 0x20 0x25                                        
                    frame.extend(b'\xFF\xFF') # 静态字节: 0xFF 0xFF
                    # 命令+地址最高位: F0或F1 (地址0-255用F0, 256-511用F1)
                    addr_high_bit = (address >> 8) & 0x01
                    command_word = 0xF0 + addr_high_bit
                    frame.extend(command_word.to_bytes(1, 'big'))
                    # 写入地址后8位
                    addr_low_byte = address & 0xFF
                    frame.append(addr_low_byte)                    
                    frame.append(data_point) # 写入的8位波形数据       
                                     
                    # --- 修改: 使用组合字节 ---
                    frame.append(combined_byte) # 波形选择(5) + 频率选择(用户输入)
                    # --- 结束修改 ---
                                                    
                    frame.extend(b'\x00\x0F\xFF\xFF') # DDR3容量(4字节)                    
                    frame.extend(b'\r\n') # 帧尾: \r\n
                    #hex_frame = ' '.join(f'{b:02X}' for b in frame) #test
                    #print(f"发送帧（地址{address}）: {hex_frame}") #test                   
                    ser.write(frame) # 通过串口发送当前帧

            self.statusBar().showMessage(f"512个波形数据点已通过 {port_name} 发送！")
        except Exception as e:
            self.show_error(f"分析或发送波形时出错: {e}\n{traceback.format_exc()}")