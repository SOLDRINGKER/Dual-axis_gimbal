import serial
import time
import struct

class WitIMU:
    """
    用于与维特科技IMU设备通信的类，使用私有协议通过串口通信
    """
    
    # 协议常量
    PROTOCOL_HEADER_READ = 0x55
    PROTOCOL_HEADER_WRITE = 0xFF
    PROTOCOL_HEADER_ACK = 0xAA
    
    # 数据类型标识符
    TYPE_TIME = 0x50
    TYPE_ACC = 0x51
    TYPE_GYRO = 0x52
    TYPE_ANGLE = 0x53
    TYPE_MAG = 0x54
    TYPE_PORT = 0x55
    TYPE_PRESS = 0x56
    TYPE_GPS = 0x57
    TYPE_VELOCITY = 0x58
    TYPE_QUATERNION = 0x59
    TYPE_GSA = 0x5A
    
    # 寄存器地址
    REG_SAVE = 0x00
    REG_CALSW = 0x01
    REG_RSW = 0x02
    REG_RRATE = 0x03
    REG_BAUD = 0x04
    REG_KEY = 0x69
    
    def __init__(self, port, baudrate=9600, timeout=0.1):
        """
        初始化WitIMU接口
        
        参数:
            port (str): 串口名称
            baudrate (int): 串口波特率
            timeout (float): 串口读取超时时间（秒）
        """
        self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        
        # 等待串口初始化
        time.sleep(2)
        
        # 清空现有数据
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()
    
    def close(self):
        """关闭串口"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
    
    def _calculate_checksum(self, data):
        """计算给定数据的校验和"""
        return sum(data) & 0xFF
    
    def _send_command(self, command):
        """发送命令到设备"""
        self.serial_port.write(command)
        time.sleep(0.01)  # 短暂延时确保命令被发送
    
    def unlock(self):
        """解锁设备以进行配置"""
        command = bytearray([0xFF, 0xAA, 0x69, 0x88, 0xB5])
        self._send_command(command)
        time.sleep(0.2)  # 按协议要求等待
        
    def save(self):
        """保存配置到设备"""
        command = bytearray([0xFF, 0xAA, 0x00, 0x00, 0x00])
        self._send_command(command)
        time.sleep(0.1)  # 短暂延时
    
    def write_register(self, register, value_low, value_high):
        """
        向寄存器写入值
        
        参数:
            register (int): 寄存器地址
            value_low (int): 值的低字节
            value_high (int): 值的高字节
        """
        command = bytearray([0xFF, 0xAA, register, value_low, value_high])
        self._send_command(command)
    
    def _combine_bytes(self, low_byte, high_byte):
        """将低字节和高字节组合成有符号短整型值"""
        value = (high_byte << 8) | low_byte
        # 如果需要，转换为有符号值
        if value > 32767:
            value -= 65536
        return value
    
    def _read_packet(self):
        """从设备读取数据包"""
        while self.serial_port.in_waiting:
            # 逐字节读取，直到找到帧头字节
            byte = self.serial_port.read(1)
            
            if byte and byte[0] == self.PROTOCOL_HEADER_READ:
                # 找到帧头，读取数据类型
                data_type_byte = self.serial_port.read(1)
                if data_type_byte:
                    data_type = data_type_byte[0]
                    
                    # 读取数据(8字节)和校验和(1字节)
                    data_and_checksum = self.serial_port.read(9)
                    if len(data_and_checksum) == 9:
                        data = data_and_checksum[:8]
                        checksum = data_and_checksum[8]
                        
                        # 验证校验和
                        calculated_checksum = self._calculate_checksum(bytearray([self.PROTOCOL_HEADER_READ, data_type]) + data)
                        if calculated_checksum == checksum:
                            # 有效数据包，返回类型和数据
                            return data_type, data
            
        return None, None
    
    def read_data(self):
        """
        从设备读取数据
        
        返回:
            tuple: (data_type, parsed_data) 或 (None, None)（如果没有有效数据）
        """
        data_type, data = self._read_packet()
        
        if data_type is None:
            return None, None
            
        # 根据数据类型解析数据
        if data_type == self.TYPE_TIME:
            # 时间数据: YY MM DD HH MN SS MSL MSH
            year, month, day, hour, minute, second, ms_low, ms_high = data
            millisecond = (ms_high << 8) | ms_low
            return data_type, {
                'year': year,
                'month': month,
                'day': day,
                'hour': hour,
                'minute': minute,
                'second': second,
                'millisecond': millisecond
            }
            
        elif data_type == self.TYPE_ACC:
            # 加速度数据: AxL AxH AyL AyH AzL AzH TL TH
            ax_l, ax_h, ay_l, ay_h, az_l, az_h, t_l, t_h = data
            ax = self._combine_bytes(ax_l, ax_h) / 32768.0 * 16  # g
            ay = self._combine_bytes(ay_l, ay_h) / 32768.0 * 16  # g
            az = self._combine_bytes(az_l, az_h) / 32768.0 * 16  # g
            temp = self._combine_bytes(t_l, t_h) / 100.0  # °C
            return data_type, {
                'ax': ax,
                'ay': ay,
                'az': az,
                'temperature': temp
            }
            
        elif data_type == self.TYPE_GYRO:
            # 角速度数据: WxL WxH WyL WyH WzL WzH VolL VolH
            wx_l, wx_h, wy_l, wy_h, wz_l, wz_h, vol_l, vol_h = data
            wx = self._combine_bytes(wx_l, wx_h) / 32768.0 * 2000  # °/s
            wy = self._combine_bytes(wy_l, wy_h) / 32768.0 * 2000  # °/s
            wz = self._combine_bytes(wz_l, wz_h) / 32768.0 * 2000  # °/s
            voltage = self._combine_bytes(vol_l, vol_h) / 100.0  # V (非蓝牙产品可能无效)
            return data_type, {
                'wx': wx,
                'wy': wy,
                'wz': wz,
                'voltage': voltage
            }
            
        elif data_type == self.TYPE_ANGLE:
            # 角度数据: RollL RollH PitchL PitchH YawL YawH VL VH
            roll_l, roll_h, pitch_l, pitch_h, yaw_l, yaw_h, v_l, v_h = data
            roll = self._combine_bytes(roll_l, roll_h) / 32768.0 * 180  # °
            pitch = self._combine_bytes(pitch_l, pitch_h) / 32768.0 * 180  # °
            yaw = self._combine_bytes(yaw_l, yaw_h) / 32768.0 * 180  # °
            version = (v_h << 8) | v_l
            return data_type, {
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'version': version
            }
            
        elif data_type == self.TYPE_MAG:
            # 磁场数据: HxL HxH HyL HyH HzL HzH TL TH
            hx_l, hx_h, hy_l, hy_h, hz_l, hz_h, t_l, t_h = data
            hx = self._combine_bytes(hx_l, hx_h)
            hy = self._combine_bytes(hy_l, hy_h)
            hz = self._combine_bytes(hz_l, hz_h)
            temp = self._combine_bytes(t_l, t_h) / 100.0  # °C
            return data_type, {
                'hx': hx,
                'hy': hy,
                'hz': hz,
                'temperature': temp
            }
            
        elif data_type == self.TYPE_PORT:
            # 端口状态数据: D0L D0H D1L D1H D2L D2H D3L D3H
            d0_l, d0_h, d1_l, d1_h, d2_l, d2_h, d3_l, d3_h = data
            d0 = (d0_h << 8) | d0_l
            d1 = (d1_h << 8) | d1_l
            d2 = (d2_h << 8) | d2_l
            d3 = (d3_h << 8) | d3_l
            return data_type, {
                'd0': d0,
                'd1': d1,
                'd2': d2,
                'd3': d3
            }
            
        elif data_type == self.TYPE_PRESS:
            # 气压和高度数据: P0 P1 P2 P3 H0 H1 H2 H3
            p0, p1, p2, p3, h0, h1, h2, h3 = data
            pressure = (p3 << 24) | (p2 << 16) | (p1 << 8) | p0  # Pa
            height = (h3 << 24) | (h2 << 16) | (h1 << 8) | h0  # cm
            return data_type, {
                'pressure': pressure,
                'height': height
            }
            
        elif data_type == self.TYPE_GPS:
            # GPS数据(经纬度): Lon0 Lon1 Lon2 Lon3 Lat0 Lat1 Lat2 Lat3
            lon0, lon1, lon2, lon3, lat0, lat1, lat2, lat3 = data
            longitude = (lon3 << 24) | (lon2 << 16) | (lon1 << 8) | lon0
            latitude = (lat3 << 24) | (lat2 << 16) | (lat1 << 8) | lat0
            
            # 如果需要，转换为有符号整数
            if longitude & 0x80000000:
                longitude -= 0x100000000
            if latitude & 0x80000000:
                latitude -= 0x100000000
                
            # 计算度和分
            lon_degrees = longitude // 10000000
            lon_minutes = (longitude % 10000000) / 100000
            lat_degrees = latitude // 10000000
            lat_minutes = (latitude % 10000000) / 100000
            
            return data_type, {
                'longitude_raw': longitude,
                'latitude_raw': latitude,
                'lon_degrees': lon_degrees,
                'lon_minutes': lon_minutes,
                'lat_degrees': lat_degrees,
                'lat_minutes': lat_minutes
            }
            
        elif data_type == self.TYPE_VELOCITY:
            # GPS速度数据: GPSHeightL GPSHeightH GPSYawL GPSYawH GPSV0 GPSV1 GPSV2 GPSV3
            height_l, height_h, yaw_l, yaw_h, v0, v1, v2, v3 = data
            gps_height = self._combine_bytes(height_l, height_h) / 10.0  # m
            gps_yaw = self._combine_bytes(yaw_l, yaw_h) / 100.0  # °
            gps_velocity = ((v3 << 24) | (v2 << 16) | (v1 << 8) | v0) / 1000.0  # km/h
            return data_type, {
                'gps_height': gps_height,
                'gps_yaw': gps_yaw,
                'gps_velocity': gps_velocity
            }
            
        elif data_type == self.TYPE_QUATERNION:
            # 四元数数据: Q0L Q0H Q1L Q1H Q2L Q2H Q3L Q3H
            q0_l, q0_h, q1_l, q1_h, q2_l, q2_h, q3_l, q3_h = data
            q0 = self._combine_bytes(q0_l, q0_h) / 32768.0
            q1 = self._combine_bytes(q1_l, q1_h) / 32768.0
            q2 = self._combine_bytes(q2_l, q2_h) / 32768.0
            q3 = self._combine_bytes(q3_l, q3_h) / 32768.0
            return data_type, {
                'q0': q0,
                'q1': q1,
                'q2': q2,
                'q3': q3
            }
            
        elif data_type == self.TYPE_GSA:
            # GPS精度数据: SNL SNH PDOPL PDOPH HDOPL HDOPH VDOPL VDOPH
            sn_l, sn_h, pdop_l, pdop_h, hdop_l, hdop_h, vdop_l, vdop_h = data
            satellite_num = (sn_h << 8) | sn_l
            pdop = self._combine_bytes(pdop_l, pdop_h) / 100.0
            hdop = self._combine_bytes(hdop_l, hdop_h) / 100.0
            vdop = self._combine_bytes(vdop_l, vdop_h) / 100.0
            return data_type, {
                'satellite_num': satellite_num,
                'pdop': pdop,
                'hdop': hdop,
                'vdop': vdop
            }
            
        # 未知数据类型
        return data_type, data
    
    def set_output_content(self, output_time=False, acc=False, gyro=False, angle=False, mag=False, port=False,
                          pressure=False, gps=False, velocity=False, quaternion=False, gsa=False):
        """
        设置输出内容
        
        参数:
            output_time (bool): 输出时间
            acc (bool): 输出加速度
            gyro (bool): 输出角速度
            angle (bool): 输出角度
            mag (bool): 输出磁场
            port (bool): 输出端口状态
            pressure (bool): 输出气压
            gps (bool): 输出GPS
            velocity (bool): 输出速度
            quaternion (bool): 输出四元数
            gsa (bool): 输出GSA
        """
        value = 0
        if output_time:  # 修改这里，使用output_time而不是time
            value |= 0x01
        if acc:
            value |= 0x02
        if gyro:
            value |= 0x04
        if angle:
            value |= 0x08
        if mag:
            value |= 0x10
        if port:
            value |= 0x20
        if pressure:
            value |= 0x40
        if gps:
            value |= 0x80
        if velocity:
            value |= 0x100
        if quaternion:
            value |= 0x200
        if gsa:
            value |= 0x400
        
        self.unlock()
        self.write_register(self.REG_RSW, value & 0xFF, (value >> 8) & 0xFF)
        time.sleep(0.1)
        self.save()
    
    def set_output_rate(self, rate_code):
        """
        设置输出速率
        
        参数:
            rate_code (int): 速率代码
                0x01: 0.2Hz
                0x02: 0.5Hz
                0x03: 1Hz
                0x04: 2Hz
                0x05: 5Hz
                0x06: 10Hz
                0x07: 20Hz
                0x08: 50Hz
                0x09: 100Hz
                0x0B: 200Hz
                0x0C: 单次回传
                0x0D: 不回传
        """
        self.unlock()
        self.write_register(self.REG_RRATE, rate_code, 0x00)
        time.sleep(0.1)
        self.save()
    
    def set_baudrate(self, baudrate_code):
        """
        设置串口波特率
        
        参数:
            baudrate_code (int): 波特率代码
                0x01: 4800bps
                0x02: 9600bps
                0x03: 19200bps
                0x04: 38400bps
                0x05: 57600bps
                0x06: 115200bps
                0x07: 230400bps
                0x08: 460800bps (仅WT931/JY931/HWT606/HWT906)
                0x09: 921600bps (仅WT931/JY931/HWT606/HWT906)
        """
        self.unlock()
        self.write_register(self.REG_BAUD, baudrate_code, 0x00)
        
        # 更改波特率后，需要用新波特率重新解锁和保存
        time.sleep(0.2)
        
        # 更新串口波特率
        baudrates = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
        if 1 <= baudrate_code <= len(baudrates):
            new_baudrate = baudrates[baudrate_code - 1]
            self.serial_port.baudrate = new_baudrate
        
        self.unlock()
        time.sleep(0.2)
        self.save()
    
    def request_single_data(self):
        """请求单个数据包（如果输出速率设置为单次响应模式）"""
        command = bytearray([0xFF, 0xAA, 0x27, 0x00, 0x00])
        self._send_command(command)
