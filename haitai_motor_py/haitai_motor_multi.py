import serial
import crcmod
import struct
import time
import termios
import fcntl
import select

class HaitaiMotorController:
    """
    优化版本的海泰电机控制器，提高实时性能
    减少通信延迟，提升控制响应速度
    """
    
    HEADER_HOST = 0x3E
    HEADER_SLAVE = 0x3C

    def __init__(self, port, baudrate=115200, motor_addresses=None, default_address=0x02, timeout=0.05):
        """
        初始化多电机控制器，默认超时时间减少到50ms
        """
        # 减小超时时间，提高响应速度
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        
        # 电机地址管理
        if motor_addresses is None:
            self.motor_addresses = {'default': default_address}
            self.default_motor_id = 'default'
        elif isinstance(motor_addresses, list):
            self.motor_addresses = {f'motor_{i}': addr for i, addr in enumerate(motor_addresses)}
            self.default_motor_id = 'motor_0'
        elif isinstance(motor_addresses, dict):
            self.motor_addresses = motor_addresses.copy()
            self.default_motor_id = list(self.motor_addresses.keys())[0]
        else:
            raise ValueError("motor_addresses应为None、列表或字典类型")
        
        # 为每个电机初始化独立的序列号
        self.packet_seq = {}
        for motor_id in self.motor_addresses.keys():
            self.packet_seq[motor_id] = 0
            
        self.crc16 = crcmod.predefined.mkCrcFun('modbus')
        
        # 优化串口配置
        self._optimize_serial_settings()
    
    def _optimize_serial_settings(self):
        """优化串口设置以减少延迟"""
        try:
            # 获取文件描述符
            fd = self.ser.fileno()
            
            # 设置为非阻塞模式，减少等待时间
            fcntl.fcntl(fd, fcntl.F_SETFL, fcntl.fcntl(fd, fcntl.F_GETFL) | fcntl.O_NONBLOCK)
            
            # 获取当前串口属性
            attrs = termios.tcgetattr(fd)
            
            # 优化读取行为：立即返回可用数据，不等待
            attrs[termios.CC][termios.VMIN] = 0   # 最小字符数为0
            attrs[termios.CC][termios.VTIME] = 1  # 超时时间0.1秒
            
            # 应用优化设置
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            
            # 刷新缓冲区，清除旧数据
            termios.tcflush(fd, termios.TCIOFLUSH)
            
        except Exception as e:
            print(f"串口优化警告: {e}")
    
    def _get_address(self, motor_id=None):
        """获取电机地址"""
        if motor_id is None:
            motor_id = self.default_motor_id
        if motor_id not in self.motor_addresses:
            raise ValueError(f"未找到电机ID: {motor_id}")
        return self.motor_addresses[motor_id]
    
    def _next_seq(self, motor_id):
        """获取电机的下一个序列号"""
        self.packet_seq[motor_id] = (self.packet_seq[motor_id] + 1) % 0x100
        return self.packet_seq[motor_id]
    
    def _build_packet(self, cmd, address, data=b'', motor_id='default'):
        """构建数据包"""
        seq = self._next_seq(motor_id)
        packet_len = len(data)
        header = struct.pack('<BBBBB', self.HEADER_HOST, seq, address, cmd, packet_len)
        full_packet = header + data
        crc = self.crc16(full_packet)
        return full_packet + struct.pack('<H', crc)
    
    def _send_and_receive_fast(self, cmd, address, data=b'', motor_id='default', expected_response_len=None):
        """
        优化的发送接收函数
        关键优化：减少等待时间，使用非阻塞读取
        """
        packet = self._build_packet(cmd, address, data, motor_id)
        
        # 清空输入缓冲区，避免旧数据干扰
        self.ser.reset_input_buffer()
        
        # 发送数据包
        self.ser.write(packet)
        self.ser.flush()  # 强制发送，不等待缓冲区
        
        # 优化的接收逻辑：使用更短的等待时间和非阻塞读取
        response = b''
        max_wait_cycles = 20  # 最多等待20个周期
        wait_cycle = 0
        
        # 先等待一个很短的时间让对方响应
        time.sleep(0.002)  # 2ms，比原来的10ms大幅减少
        
        while len(response) < 7 and wait_cycle < max_wait_cycles:
            try:
                chunk = self.ser.read(1024)
                if chunk:
                    response += chunk
                    # 如果收到了足够的包头数据，检查需要多少数据
                    if len(response) >= 5:
                        expected_total = 7 + response[4]  # 包头5字节 + 数据长度 + CRC2字节
                        
                        # 继续读取直到获得完整数据包
                        while len(response) < expected_total and wait_cycle < max_wait_cycles:
                            chunk = self.ser.read(expected_total - len(response))
                            if chunk:
                                response += chunk
                            else:
                                time.sleep(0.001)  # 1ms
                                wait_cycle += 1
                        break
                else:
                    time.sleep(0.001)  # 1ms，比原来的10ms大幅减少
                    wait_cycle += 1
            except:
                time.sleep(0.001)
                wait_cycle += 1
        
        if not response or len(response) < 7:
            raise ValueError(f"电机(地址:{hex(address)})无响应或响应不完整")
        
        # 解析响应
        header, seq, addr, rcmd, rlen = struct.unpack('<BBBBB', response[:5])
        
        if header != self.HEADER_SLAVE or addr != address or rcmd != cmd:
            raise ValueError(f"响应头部不匹配")
        
        data_end = 5 + rlen
        if len(response) < data_end + 2:
            raise ValueError("响应数据长度不足")
        
        rdata = response[5:data_end]
        crc_received = struct.unpack('<H', response[data_end:data_end+2])[0]
        crc_calculated = self.crc16(response[:data_end])
        
        if crc_received != crc_calculated:
            raise ValueError("CRC校验失败")
        
        return rdata
    
    # 系统命令
    def get_device_info(self, motor_id=None):
        """获取设备信息"""
        address = self._get_address(motor_id)
        data = self._send_and_receive_fast(0x0A, address, motor_id=motor_id or self.default_motor_id, expected_response_len=0x14)
        
        model = struct.unpack('<H', data[0:2])[0]
        hw_version = data[2]
        hw_config = data[3]
        sw_version = struct.unpack('<H', data[4:6])[0]
        mcu_id = data[6:18]
        rs485_ver = data[18]
        can_ver = data[19]
        
        return {
            'motor_id': motor_id or self.default_motor_id,
            'address': address,
            'model': model,
            'hw_version': hw_version,
            'hw_config': hw_config,
            'sw_version': sw_version,
            'mcu_id': mcu_id,
            'rs485_version': rs485_ver,
            'can_version': can_ver
        }
    
    def read_system_data(self, motor_id=None):
        """读取系统数据"""
        address = self._get_address(motor_id)
        data = self._send_and_receive_fast(0x0B, address, motor_id=motor_id or self.default_motor_id, expected_response_len=0x0D)
        
        single_turn_angle = struct.unpack('<H', data[0:2])[0] * (360 / 16384)
        multi_turn_angle = struct.unpack('<i', data[2:6])[0] * (360 / 16384)
        speed = struct.unpack('<h', data[6:8])[0] * 0.1
        voltage = data[8] * 0.2
        current = data[9] * 0.03
        temperature = data[10] * 0.4
        fault_code = data[11]
        run_state = data[12]
        
        return {
            'motor_id': motor_id or self.default_motor_id,
            'address': address,
            'single_turn_angle': single_turn_angle,
            'multi_turn_angle': multi_turn_angle,
            'speed': speed,
            'voltage': voltage,
            'current': current,
            'temperature': temperature,
            'fault_code': fault_code,
            'run_state': run_state
        }
    
    def read_system_params(self, motor_id=None):
        """读取系统参数"""
        address = self._get_address(motor_id)
        data = self._send_and_receive_fast(0x0C, address, motor_id=motor_id or self.default_motor_id, expected_response_len=0x1A)
        
        result = self._parse_system_params(data)
        result['motor_id'] = motor_id or self.default_motor_id
        result['address'] = address
        return result
    
    def write_system_params_temp(self, params, motor_id=None):
        """写入临时系统参数"""
        address = self._get_address(motor_id)
        data = self._pack_system_params(params)
        response = self._send_and_receive_fast(0x0D, address, data, motor_id or self.default_motor_id, expected_response_len=0x1A)
        
        result = self._parse_system_params(response)
        result['motor_id'] = motor_id or self.default_motor_id
        result['address'] = address
        return result
    
    def _pack_system_params(self, params):
        """打包系统参数"""
        dev_addr = params.get('device_address', 0x02)
        current_threshold = int(params['current_threshold'] / 0.03)
        voltage_threshold = int(params['voltage_threshold'] / 0.2)
        baudrates = params['baudrates']
        pos_kp = struct.pack('<f', params['position_kp'])
        pos_target_speed = struct.pack('<f', params['position_target_speed'] / 0.1)
        speed_kp = struct.pack('<f', params['speed_kp'])
        speed_ki = struct.pack('<f', params['speed_ki'])
        reserved = struct.pack('<f', params.get('reserved', 0.0))
        speed_filter = int(params['speed_filter'] * 100)
        power_percent = params['power_percent']
        
        return (struct.pack('<BBBB', dev_addr, current_threshold, voltage_threshold, baudrates) +
                pos_kp + pos_target_speed + speed_kp + speed_ki + reserved +
                struct.pack('<BB', speed_filter, power_percent))
    
    def _parse_system_params(self, data):
        """解析系统参数"""
        return {
            'device_address': data[0],
            'current_threshold': data[1] * 0.03,
            'voltage_threshold': data[2] * 0.2,
            'baudrates': data[3],
            'position_kp': struct.unpack('<f', data[4:8])[0],
            'position_target_speed': struct.unpack('<f', data[8:12])[0] * 0.1,
            'speed_kp': struct.unpack('<f', data[12:16])[0],
            'speed_ki': struct.unpack('<f', data[16:20])[0],
            'reserved': struct.unpack('<f', data[20:24])[0],
            'speed_filter': data[24] / 100,
            'power_percent': data[25]
        }
    
    def read_motor_status(self, motor_id=None):
        """读取电机状态"""
        address = self._get_address(motor_id)
        data = self._send_and_receive_fast(0x40, address, motor_id=motor_id or self.default_motor_id, expected_response_len=0x05)
        
        return {
            'motor_id': motor_id or self.default_motor_id,
            'address': address,
            'voltage': data[0] * 0.2,
            'current': data[1] * 0.03,
            'temperature': data[2] * 0.4,
            'fault_code': data[3],
            'run_state': data[4]
        }
    
    def clear_faults(self, motor_id=None):
        """清除故障"""
        address = self._get_address(motor_id)
        response = self._send_and_receive_fast(0x41, address, motor_id=motor_id or self.default_motor_id, expected_response_len=0x05)
        
        return {
            'motor_id': motor_id or self.default_motor_id,
            'address': address,
            'voltage': response[0] * 0.2,
            'current': response[1] * 0.03,
            'temperature': response[2] * 0.4,
            'fault_code': response[3],
            'run_state': response[4]
        }
    
    def set_current_position_as_origin(self, motor_id=None):
        """设置当前位置为原点"""
        address = self._get_address(motor_id)
        data = self._send_and_receive_fast(0x21, address, motor_id=motor_id or self.default_motor_id, expected_response_len=0x03)
        
        raw_angle = struct.unpack('<H', data[0:2])[0]
        success = data[2] == 0x01
        
        return {
            'motor_id': motor_id or self.default_motor_id,
            'address': address,
            'raw_angle': raw_angle,
            'success': success
        }
    
    def speed_closed_loop(self, target_speed, motor_id=None):
        """速度闭环控制"""
        address = self._get_address(motor_id)
        data = struct.pack('<h', target_speed)
        response = self._send_and_receive_fast(0x54, address, data, motor_id or self.default_motor_id, expected_response_len=0x08)
        
        single_turn = struct.unpack('<H', response[0:2])[0] * (360 / 16384)
        multi_turn = struct.unpack('<i', response[2:6])[0] * (360 / 16384)
        speed = struct.unpack('<h', response[6:8])[0] * 0.1
        
        return {
            'motor_id': motor_id or self.default_motor_id,
            'address': address,
            'single_turn_angle': single_turn,
            'multi_turn_angle': multi_turn,
            'speed': speed
        }
    
    def shutdown_all_motors(self):
        """关闭所有电机"""
        results = {}
        for motor_id in self.motor_addresses:
            try:
                address = self._get_address(motor_id)
                response = self._send_and_receive_fast(0x50, address, motor_id=motor_id, expected_response_len=0x08)
                results[motor_id] = {
                    'single_turn_angle': struct.unpack('<H', response[0:2])[0] * (360 / 16384),
                    'multi_turn_angle': struct.unpack('<i', response[2:6])[0] * (360 / 16384),
                    'speed': struct.unpack('<h', response[6:8])[0] * 0.1
                }
            except Exception as e:
                results[motor_id] = {'error': str(e)}
        return results
    
    def close(self):
        """关闭串口连接"""
        self.ser.close()
