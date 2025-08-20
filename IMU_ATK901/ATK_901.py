import serial
import time
import threading
import struct

class ATK_MS901M:
    """
    ATK-MS901M 十轴高性能角度传感器模块控制类
    实现了所有模块功能，包括：
    - 姿态角、四元数数据获取
    - 陀螺仪、加速度计、磁力计、气压计数据获取
    - 传感器校准（加速度计、磁力计、陀螺仪、气压计）
    - 扩展端口控制（模拟输入、数字输入/输出、PWM输出）
    - 模块参数配置
    """
    
    # 数据帧帧头
    DATA_FRAME_HEADER = bytes([0x55, 0x55])
    CMD_FRAME_HEADER = bytes([0x55, 0xAF])
    
    # 数据帧ID
    FRAME_ID_ATTITUDE = 0x01      # 姿态角数据
    FRAME_ID_QUATERNION = 0x02    # 四元数数据
    FRAME_ID_GYRO_ACC = 0x03      # 陀螺仪和加速度数据
    FRAME_ID_MAG = 0x04           # 磁力计数据
    FRAME_ID_BARO = 0x05          # 气压计数据
    FRAME_ID_PORT_STATUS = 0x06   # 端口状态数据
    
    # 指令帧ID
    CMD_SAVE = 0x00               # 保存配置
    CMD_SENCAL = 0x01             # 传感器校准
    CMD_SENSTA = 0x02             # 获取传感器校准状态
    CMD_GYROFSR = 0x03            # 陀螺仪量程设置
    CMD_ACCFSR = 0x04             # 加速度计量程设置
    CMD_GYROBW = 0x05             # 陀螺仪带宽设置
    CMD_ACCBW = 0x06              # 加速度计带宽设置
    CMD_BAUD = 0x07               # UART通讯波特率设置
    CMD_RETURNSET = 0x08          # 主动上报内容设置
    CMD_RETURNSET2 = 0x09         # 主动上报内容设置2(保留)
    CMD_RETURNRATE = 0x0A         # 主动上报速率设置
    CMD_ALG = 0x0B                # 算法选择设置
    CMD_ASM = 0x0C                # 安装方向设置
    CMD_GAUCAL = 0x0D             # 陀螺仪自校准设置
    CMD_BAUCAL = 0x0E             # 气压计自校准设置
    CMD_LEDOFF = 0x0F             # LED设置
    CMD_D0MODE = 0x10             # D0端口模式设置
    CMD_D1MODE = 0x11             # D1端口模式设置
    CMD_D2MODE = 0x12             # D2端口模式设置
    CMD_D3MODE = 0x13             # D3端口模式设置
    CMD_D1PULSE = 0x16            # D1端口PWM脉宽设置
    CMD_D3PULSE = 0x1A            # D3端口PWM脉宽设置
    CMD_D1PERIOD = 0x1F           # D1端口PWM周期设置
    CMD_D3PERIOD = 0x23           # D3端口PWM周期设置
    CMD_RESET = 0x7F              # 恢复出厂设置
    
    # 端口模式
    PORT_MODE_ANALOG_INPUT = 0x00         # 模拟输入模式
    PORT_MODE_DIGITAL_INPUT = 0x01        # 数字输入模式
    PORT_MODE_DIGITAL_OUTPUT_HIGH = 0x02  # 数字高电平输出模式
    PORT_MODE_DIGITAL_OUTPUT_LOW = 0x03   # 数字低电平输出模式
    PORT_MODE_PWM_OUTPUT = 0x04           # PWM输出模式(仅D1和D3支持)
    
    # 陀螺仪量程
    GYRO_FSR_250DPS = 0x00   # 250dps
    GYRO_FSR_500DPS = 0x01   # 500dps
    GYRO_FSR_1000DPS = 0x02  # 1000dps
    GYRO_FSR_2000DPS = 0x03  # 2000dps(默认)
    
    # 加速度计量程
    ACC_FSR_2G = 0x00    # 2G
    ACC_FSR_4G = 0x01    # 4G(默认)
    ACC_FSR_8G = 0x02    # 8G
    ACC_FSR_16G = 0x03   # 16G
    
    # 陀螺仪带宽
    GYRO_BW_176 = 0x00  # 176Hz
    GYRO_BW_92 = 0x01   # 92Hz(默认)
    GYRO_BW_41 = 0x02   # 41Hz
    GYRO_BW_20 = 0x03   # 20Hz
    GYRO_BW_10 = 0x04   # 10Hz
    GYRO_BW_5 = 0x05    # 5Hz
    
    # 加速度计带宽
    ACC_BW_218 = 0x00  # 218Hz
    ACC_BW_99 = 0x01   # 99Hz
    ACC_BW_45 = 0x02   # 45Hz(默认)
    ACC_BW_21 = 0x03   # 21Hz
    ACC_BW_10 = 0x04   # 10Hz
    ACC_BW_5 = 0x05    # 5Hz
    
    # 波特率设置
    BAUD_921600 = 0x00  # 921600bps
    BAUD_460800 = 0x01  # 460800bps
    BAUD_256000 = 0x02  # 256000bps
    BAUD_230400 = 0x03  # 230400bps
    BAUD_115200 = 0x04  # 115200bps(默认)
    BAUD_57600 = 0x05   # 57600bps
    BAUD_38400 = 0x06   # 38400bps
    BAUD_19200 = 0x07   # 19200bps
    BAUD_9600 = 0x08    # 9600bps
    BAUD_4800 = 0x09    # 4800bps
    BAUD_2400 = 0x0A    # 2400bps
    
    # 主动上报内容
    RETURN_ATTITUDE = 0x01     # 上报姿态角数据
    RETURN_QUATERNION = 0x02   # 上报四元数数据
    RETURN_GYRO_ACC = 0x04     # 上报陀螺仪和加速度数据
    RETURN_MAG = 0x08          # 上报磁力计数据
    RETURN_BARO = 0x10         # 上报气压计数据
    RETURN_PORT_STATUS = 0x20  # 上报端口状态数据
    RETURN_ANONYMOUS = 0x40    # 上报匿名上位机数据
    
    # 主动上报速率
    RATE_250HZ = 0x00  # 250Hz
    RATE_200HZ = 0x01  # 200Hz
    RATE_125HZ = 0x02  # 125Hz
    RATE_100HZ = 0x03  # 100Hz
    RATE_50HZ = 0x04   # 50Hz
    RATE_20HZ = 0x05   # 20Hz
    RATE_10HZ = 0x06   # 10Hz
    RATE_5HZ = 0x07    # 5Hz
    RATE_2HZ = 0x08    # 2Hz
    RATE_1HZ = 0x09    # 1Hz
    
    # 算法设置
    ALG_6AXIS = 0x00  # 六轴算法
    ALG_9AXIS = 0x01  # 九轴算法
    
    # 安装方向
    ASM_HORIZONTAL = 0x00  # 水平安装
    ASM_VERTICAL = 0x01    # 垂直安装
    
    # 陀螺仪自校准
    GAUCAL_OFF = 0x00  # 关闭自校准
    GAUCAL_ON = 0x01   # 开启自校准(默认)
    
    # 气压计自校准
    BAUCAL_OFF = 0x00  # 关闭自校准(默认)
    BAUCAL_ON = 0x01   # 开启自校准
    
    # LED设置
    LED_ON = 0x00   # LED开启(默认)
    LED_OFF = 0x01  # LED关闭

    def __init__(self, port, baudrate=115200):
        """
        初始化ATK-MS901M传感器
        
        参数:
            port (str): 串口号(例如'COM3', '/dev/ttyUSB0')
            baudrate (int): 通信波特率(默认: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.reading_thread = None
        
        # 数据存储
        self.attitude = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.quaternion = {"q0": 0.0, "q1": 0.0, "q2": 0.0, "q3": 0.0}
        self.gyro = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.acc = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.mag = {"x": 0, "y": 0, "z": 0, "temperature": 0.0}
        self.baro = {"pressure": 0, "altitude": 0, "temperature": 0.0}
        self.port_status = {"D0": 0, "D1": 0, "D2": 0, "D3": 0}
        
        # 配置设置
        self.gyro_fsr = 2000  # 默认: 2000dps
        self.acc_fsr = 4      # 默认: 4G
        
        # 数据帧处理回调
        self.callbacks = {
            self.FRAME_ID_ATTITUDE: self._process_attitude_data,
            self.FRAME_ID_QUATERNION: self._process_quaternion_data,
            self.FRAME_ID_GYRO_ACC: self._process_gyro_acc_data,
            self.FRAME_ID_MAG: self._process_mag_data,
            self.FRAME_ID_BARO: self._process_baro_data,
            self.FRAME_ID_PORT_STATUS: self._process_port_status_data,
        }
        
    def open(self):
        """打开串口并开始读取数据"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.running = True
            self.reading_thread = threading.Thread(target=self._read_data, daemon=True)
            self.reading_thread.start()
            return True
        except serial.SerialException as e:
            print(f"串口打开失败: {e}")
            return False
            
    def close(self):
        """关闭串口并停止读取数据"""
        self.running = False
        if self.reading_thread:
            self.reading_thread.join(timeout=1.0)
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def _read_data(self):
        """从传感器读取并处理数据帧"""
        buffer = bytearray()
        
        while self.running:
            if self.serial and self.serial.is_open:
                try:
                    # 读取数据
                    data = self.serial.read(self.serial.in_waiting or 1)
                    if data:
                        buffer.extend(data)
                        
                        # 处理缓冲区数据
                        while len(buffer) >= 2:
                            # 检查数据帧头
                            if (buffer[0] == 0x55 and buffer[1] == 0x55) and len(buffer) >= 4:
                                frame_id = buffer[2]
                                data_len = buffer[3]
                                
                                # 检查是否收到完整帧
                                if len(buffer) >= 4 + data_len + 1:
                                    frame_data = buffer[4:4+data_len]
                                    checksum = buffer[4+data_len]
                                    
                                    # 验证校验和
                                    calculated_checksum = (0x55 + 0x55 + frame_id + data_len + sum(frame_data)) & 0xFF
                                    
                                    if checksum == calculated_checksum:
                                        # 处理帧数据
                                        if frame_id in self.callbacks:
                                            self.callbacks[frame_id](frame_data)
                                    
                                    # 从缓冲区移除已处理的帧
                                    buffer = buffer[4+data_len+1:]
                                else:
                                    break
                            # 检查指令响应帧头
                            elif (buffer[0] == 0x55 and buffer[1] == 0xAF) and len(buffer) >= 4:
                                # 处理指令响应
                                frame_id = buffer[2]
                                data_len = buffer[3]
                                
                                if len(buffer) >= 4 + data_len + 1:
                                    # 从缓冲区移除已处理的帧
                                    buffer = buffer[4+data_len+1:]
                                else:
                                    break
                            else:
                                # 无效帧头，移除第一个字节
                                buffer.pop(0)
                except Exception as e:
                    print(f"读取数据错误: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.1)
    
    def _process_attitude_data(self, data):
        """处理姿态角数据"""
        if len(data) == 6:
            roll = self._bytes_to_float(data[0], data[1]) * 180
            pitch = self._bytes_to_float(data[2], data[3]) * 180
            yaw = self._bytes_to_float(data[4], data[5]) * 180
            
            self.attitude = {"roll": roll, "pitch": pitch, "yaw": yaw}
    
    def _process_quaternion_data(self, data):
        """处理四元数数据"""
        if len(data) == 8:
            q0 = self._bytes_to_float(data[0], data[1])
            q1 = self._bytes_to_float(data[2], data[3])
            q2 = self._bytes_to_float(data[4], data[5])
            q3 = self._bytes_to_float(data[6], data[7])
            
            self.quaternion = {"q0": q0, "q1": q1, "q2": q2, "q3": q3}
    
    def _process_gyro_acc_data(self, data):
        """处理陀螺仪和加速度计数据"""
        if len(data) == 12:
            ax = self._bytes_to_int16(data[0], data[1])
            ay = self._bytes_to_int16(data[2], data[3])
            az = self._bytes_to_int16(data[4], data[5])
            gx = self._bytes_to_int16(data[6], data[7])
            gy = self._bytes_to_int16(data[8], data[9])
            gz = self._bytes_to_int16(data[10], data[11])
            
            # 转换为物理单位
            self.acc = {
                "x": (ax / 32768.0) * self.acc_fsr,
                "y": (ay / 32768.0) * self.acc_fsr,
                "z": (az / 32768.0) * self.acc_fsr
            }
            
            self.gyro = {
                "x": (gx / 32768.0) * self.gyro_fsr,
                "y": (gy / 32768.0) * self.gyro_fsr,
                "z": (gz / 32768.0) * self.gyro_fsr
            }
    
    def _process_mag_data(self, data):
        """处理磁力计数据"""
        if len(data) == 8:
            mx = self._bytes_to_int16(data[0], data[1])
            my = self._bytes_to_int16(data[2], data[3])
            mz = self._bytes_to_int16(data[4], data[5])
            temp = self._bytes_to_int16(data[6], data[7]) / 100.0
            
            self.mag = {"x": mx, "y": my, "z": mz, "temperature": temp}
    
    def _process_baro_data(self, data):
        """处理气压计数据"""
        if len(data) == 10:
            pressure = struct.unpack("<I", bytes(data[0:4]))[0]
            altitude = struct.unpack("<I", bytes(data[4:8]))[0]
            temp = self._bytes_to_int16(data[8], data[9]) / 100.0
            
            self.baro = {"pressure": pressure, "altitude": altitude, "temperature": temp}
    
    def _process_port_status_data(self, data):
        """处理端口状态数据"""
        if len(data) == 8:
            d0 = self._bytes_to_uint16(data[0], data[1])
            d1 = self._bytes_to_uint16(data[2], data[3])
            d2 = self._bytes_to_uint16(data[4], data[5])
            d3 = self._bytes_to_uint16(data[6], data[7])
            
            self.port_status = {"D0": d0, "D1": d1, "D2": d2, "D3": d3}
    
    def _bytes_to_int16(self, low_byte, high_byte):
        """将两个字节转换为有符号16位整数"""
        value = (high_byte << 8) | low_byte
        # 转换为有符号数
        if value > 32767:
            value -= 65536
        return value
    
    def _bytes_to_uint16(self, low_byte, high_byte):
        """将两个字节转换为无符号16位整数"""
        return (high_byte << 8) | low_byte
    
    def _bytes_to_float(self, low_byte, high_byte):
        """将两个字节转换为浮点值(用于角度或四元数)"""
        return self._bytes_to_int16(low_byte, high_byte) / 32768.0
    
    def _calculate_checksum(self, data):
        """计算指令帧的校验和"""
        return sum(data) & 0xFF
    
    def _send_command(self, cmd_id, data=None, read_response=False):
        """
        发送指令到传感器并可选地读取响应
        
        参数:
            cmd_id: 指令ID
            data: 指令数据(默认[0x00])
            read_response: 是否读取响应
        """
        if not self.serial or not self.serial.is_open:
            print("串口未打开")
            return None
            
        # 准备指令数据
        if data is None:
            data = [0x00]
        data_len = len(data)
        
        # 创建指令帧
        frame = [0x55, 0xAF, cmd_id, data_len] + data
        checksum = self._calculate_checksum(frame)
        frame.append(checksum)
        
        # 发送指令
        self.serial.write(bytes(frame))
        
        # 读取响应(如果需要)
        if read_response:
            time.sleep(0.1)  # 等待响应
            response = bytearray()
            start_time = time.time()
            
            # 等待完整响应或超时
            while time.time() - start_time < 1.0:  # 1秒超时
                if self.serial.in_waiting:
                    new_data = self.serial.read(self.serial.in_waiting)
                    response.extend(new_data)
                    
                    # 检查是否收到完整响应
                    if len(response) >= 5 and response[0] == 0x55 and response[1] == 0xAF:
                        resp_cmd_id = response[2]
                        data_len = response[3]
                        
                        if len(response) >= 4 + data_len + 1:
                            # 收到完整响应
                            resp_data = response[4:4+data_len]
                            checksum = response[4+data_len]
                            
                            # 验证校验和
                            calculated_checksum = (0x55 + 0xAF + resp_cmd_id + data_len + sum(resp_data)) & 0xFF
                            
                            if checksum == calculated_checksum and resp_cmd_id == cmd_id:
                                return resp_data
                            else:
                                # 无效响应，继续等待
                                pass
                
                time.sleep(0.01)
            
            print("等待响应超时")
            return None
        
        return True
    
    def save_configuration(self):
        """保存当前配置到Flash"""
        return self._send_command(self.CMD_SAVE, [0x00])
    
    def calibrate_accelerometer(self):
        """
        校准加速度计
        校准过程中模块必须保持水平且静止
        """
        return self._send_command(self.CMD_SENCAL, [0x00])
    
    def calibrate_magnetometer(self):
        """
        校准磁力计
        校准后需绕X、Y、Z轴旋转模块
        """
        return self._send_command(self.CMD_SENCAL, [0x01])
    
    def calibrate_barometer(self):
        """
        校准气压计
        设置当前高度为0
        """
        return self._send_command(self.CMD_SENCAL, [0x02])
    
    def get_sensor_status(self):
        """获取传感器校准状态"""
        response = self._send_command(self.CMD_SENSTA | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            acc_calibrated = bool(response[0] & 0x01)
            mag_calibrated = bool(response[0] & 0x02)
            gyro_calibrated = bool(response[0] & 0x04)
            return {
                "accelerometer_calibrated": acc_calibrated, 
                "magnetometer_calibrated": mag_calibrated, 
                "gyroscope_calibrated": gyro_calibrated
            }
        return None
    
    def set_gyro_range(self, fsr):
        """
        设置陀螺仪满量程范围
        
        参数:
            fsr: 满量程值(GYRO_FSR_*)
        """
        if fsr in [self.GYRO_FSR_250DPS, self.GYRO_FSR_500DPS, 
                  self.GYRO_FSR_1000DPS, self.GYRO_FSR_2000DPS]:
            result = self._send_command(self.CMD_GYROFSR, [fsr])
            if result:
                # 更新存储的满量程值
                fsr_values = [250, 500, 1000, 2000]
                self.gyro_fsr = fsr_values[fsr]
            return result
        return False
    
    def get_gyro_range(self):
        """获取当前陀螺仪满量程范围"""
        response = self._send_command(self.CMD_GYROFSR | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            fsr_values = [250, 500, 1000, 2000]
            fsr = response[0]
            if 0 <= fsr < len(fsr_values):
                self.gyro_fsr = fsr_values[fsr]
                return fsr
        return None
    
    def set_acc_range(self, fsr):
        """
        设置加速度计满量程范围
        
        参数:
            fsr: 满量程值(ACC_FSR_*)
        """
        if fsr in [self.ACC_FSR_2G, self.ACC_FSR_4G, self.ACC_FSR_8G, self.ACC_FSR_16G]:
            result = self._send_command(self.CMD_ACCFSR, [fsr])
            if result:
                # 更新存储的满量程值
                fsr_values = [2, 4, 8, 16]
                self.acc_fsr = fsr_values[fsr]
            return result
        return False
    
    def get_acc_range(self):
        """获取当前加速度计满量程范围"""
        response = self._send_command(self.CMD_ACCFSR | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            fsr_values = [2, 4, 8, 16]
            fsr = response[0]
            if 0 <= fsr < len(fsr_values):
                self.acc_fsr = fsr_values[fsr]
                return fsr
        return None
    
    def set_gyro_bandwidth(self, bw):
        """
        设置陀螺仪带宽
        
        参数:
            bw: 带宽值(GYRO_BW_*)
        """
        if bw in [self.GYRO_BW_176, self.GYRO_BW_92, self.GYRO_BW_41,
                 self.GYRO_BW_20, self.GYRO_BW_10, self.GYRO_BW_5]:
            return self._send_command(self.CMD_GYROBW, [bw])
        return False
    
    def get_gyro_bandwidth(self):
        """获取当前陀螺仪带宽"""
        response = self._send_command(self.CMD_GYROBW | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            return response[0]
        return None
    
    def set_acc_bandwidth(self, bw):
        """
        设置加速度计带宽
        
        参数:
            bw: 带宽值(ACC_BW_*)
        """
        if bw in [self.ACC_BW_218, self.ACC_BW_99, self.ACC_BW_45,
                 self.ACC_BW_21, self.ACC_BW_10, self.ACC_BW_5]:
            return self._send_command(self.CMD_ACCBW, [bw])
        return False
    
    def get_acc_bandwidth(self):
        """获取当前加速度计带宽"""
        response = self._send_command(self.CMD_ACCBW | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            return response[0]
        return None
    
    def set_baudrate(self, baud):
        """
        设置UART通信波特率
        注意: 该设置需要重启模块后生效
        
        参数:
            baud: 波特率值(BAUD_*)
        """
        if baud in range(self.BAUD_921600, self.BAUD_2400 + 1):
            return self._send_command(self.CMD_BAUD, [baud])
        return False
    
    def get_baudrate(self):
        """获取当前UART波特率设置"""
        response = self._send_command(self.CMD_BAUD | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            return response[0]
        return None
    
    def set_return_data(self, data_flags):
        """
        设置模块应主动上报的数据类型
        
        参数:
            data_flags: 数据类型位掩码(RETURN_*)
        """
        return self._send_command(self.CMD_RETURNSET, [data_flags])
    
    def get_return_data(self):
        """获取当前主动上报数据设置"""
        response = self._send_command(self.CMD_RETURNSET | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            return response[0]
        return None
    
    def set_return_rate(self, rate):
        """
        设置数据上报速率
        
        参数:
            rate: 上报速率值(RATE_*)
        """
        if rate in range(self.RATE_250HZ, self.RATE_1HZ + 1):
            return self._send_command(self.CMD_RETURNRATE, [rate])
        return False
    
    def get_return_rate(self):
        """获取当前数据上报速率"""
        response = self._send_command(self.CMD_RETURNRATE | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            return response[0]
        return None
    
    def set_algorithm(self, alg):
        """
        设置姿态计算算法
        
        参数:
            alg: 算法值(ALG_*)
        """
        if alg in [self.ALG_6AXIS, self.ALG_9AXIS]:
            return self._send_command(self.CMD_ALG, [alg])
        return False
    
    def get_algorithm(self):
        """获取当前算法设置"""
        response = self._send_command(self.CMD_ALG | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            return response[0]
        return None
    
    def set_installation_direction(self, direction):
        """
        设置模块的安装方向
        注意: 该设置需要重启模块后生效
        
        参数:
            direction: 安装方向(ASM_*)
        """
        if direction in [self.ASM_HORIZONTAL, self.ASM_VERTICAL]:
            return self._send_command(self.CMD_ASM, [direction])
        return False
    
    def get_installation_direction(self):
        """获取当前安装方向设置"""
        response = self._send_command(self.CMD_ASM | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            return response[0]
        return None
    
    def set_gyro_auto_calibration(self, enabled):
        """
        启用或禁用陀螺仪上电自校准
        
        参数:
            enabled: True启用, False禁用
        """
        value = self.GAUCAL_ON if enabled else self.GAUCAL_OFF
        return self._send_command(self.CMD_GAUCAL, [value])
    
    def get_gyro_auto_calibration(self):
        """获取当前陀螺仪自校准设置"""
        response = self._send_command(self.CMD_GAUCAL | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            return response[0] == self.GAUCAL_ON
        return None
    
    def set_baro_auto_calibration(self, enabled):
        """
        启用或禁用气压计上电自校准
        
        参数:
            enabled: True启用, False禁用
        """
        value = self.BAUCAL_ON if enabled else self.BAUCAL_OFF
        return self._send_command(self.CMD_BAUCAL, [value])
    
    def get_baro_auto_calibration(self):
        """获取当前气压计自校准设置"""
        response = self._send_command(self.CMD_BAUCAL | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            return response[0] == self.BAUCAL_ON
        return None
    
    def set_led(self, enabled):
        """
        启用或禁用模块的LED
        
        参数:
            enabled: True启用, False禁用
        """
        value = self.LED_ON if enabled else self.LED_OFF
        return self._send_command(self.CMD_LEDOFF, [value])
    
    def get_led_state(self):
        """获取当前LED状态"""
        response = self._send_command(self.CMD_LEDOFF | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            return response[0] == self.LED_ON
        return None
    
    def set_port_mode(self, port, mode):
        """
        设置特定端口的模式
        
        参数:
            port: 端口号(0-3)
            mode: 端口模式(PORT_MODE_*)
        """
        if port < 0 or port > 3:
            return False
        
        # 检查PWM模式是否设置在不支持PWM的端口上
        if mode == self.PORT_MODE_PWM_OUTPUT and port not in [1, 3]:
            return False
            
        # 根据端口选择适当的命令
        cmd_id = [self.CMD_D0MODE, self.CMD_D1MODE, self.CMD_D2MODE, self.CMD_D3MODE][port]
        
        return self._send_command(cmd_id, [mode])
    
    def get_port_mode(self, port):
        """
        获取特定端口的当前模式
        
        参数:
            port: 端口号(0-3)
        """
        if port < 0 or port > 3:
            return None
            
        # 根据端口选择适当的命令
        cmd_id = [self.CMD_D0MODE, self.CMD_D1MODE, self.CMD_D2MODE, self.CMD_D3MODE][port]
        
        response = self._send_command(cmd_id | 0x80, [0x00], read_response=True)
        if response and len(response) == 1:
            return response[0]
        return None
    
    def set_pwm_pulse_width(self, port, width):
        """
        设置D1或D3端口的PWM脉宽
        
        参数:
            port: 端口号(1或3)
            width: 脉宽(微秒)(0-65535)
        """
        if port not in [1, 3]:
            return False
            
        # 转换宽度为字节
        width_low = width & 0xFF
        width_high = (width >> 8) & 0xFF
        
        # 根据端口选择适当的命令
        cmd_id = self.CMD_D1PULSE if port == 1 else self.CMD_D3PULSE
        
        return self._send_command(cmd_id, [width_low, width_high])
    
    def get_pwm_pulse_width(self, port):
        """
        获取D1或D3端口的当前PWM脉宽
        
        参数:
            port: 端口号(1或3)
        """
        if port not in [1, 3]:
            return None
            
        # 根据端口选择适当的命令
        cmd_id = self.CMD_D1PULSE if port == 1 else self.CMD_D3PULSE
        
        response = self._send_command(cmd_id | 0x80, [0x00], read_response=True)
        if response and len(response) == 2:
            return (response[1] << 8) | response[0]
        return None
    
    def set_pwm_period(self, port, period):
        """
        设置D1或D3端口的PWM周期
        
        参数:
            port: 端口号(1或3)
            period: 周期(微秒)(0-65535)
        """
        if port not in [1, 3]:
            return False
            
        # 转换周期为字节
        period_low = period & 0xFF
        period_high = (period >> 8) & 0xFF
        
        # 根据端口选择适当的命令
        cmd_id = self.CMD_D1PERIOD if port == 1 else self.CMD_D3PERIOD
        
        return self._send_command(cmd_id, [period_low, period_high])
    
    def get_pwm_period(self, port):
        """
        获取D1或D3端口的当前PWM周期
        
        参数:
            port: 端口号(1或3)
        """
        if port not in [1, 3]:
            return None
            
        # 根据端口选择适当的命令
        cmd_id = self.CMD_D1PERIOD if port == 1 else self.CMD_D3PERIOD
        
        response = self._send_command(cmd_id | 0x80, [0x00], read_response=True)
        if response and len(response) == 2:
            return (response[1] << 8) | response[0]
        return None
    
    def reset_to_defaults(self):
        """恢复模块出厂默认设置"""
        return self._send_command(self.CMD_RESET, [0x00])
    
    def get_attitude(self):
        """获取当前姿态角(横滚角、俯仰角、航向角)"""
        return self.attitude
    
    def get_quaternion(self):
        """获取当前四元数值"""
        return self.quaternion
    
    def get_gyro(self):
        """获取当前陀螺仪测量值"""
        return self.gyro
    
    def get_acc(self):
        """获取当前加速度计测量值"""
        return self.acc
    
    def get_mag(self):
        """获取当前磁力计测量值"""
        return self.mag
    
    def get_baro(self):
        """获取当前气压计测量值"""
        return self.baro
    
    def get_port_status(self):
        """获取当前端口状态"""
        return self.port_status
    
    def convert_port_value_to_voltage(self, value):
        """
        将端口ADC值转换为电压
        
        参数:
            value: 端口状态中的ADC值(0-4095)
        
        返回:
            电压值(0-3.3V)
        """
        return (value / 4095.0) * 3.3
