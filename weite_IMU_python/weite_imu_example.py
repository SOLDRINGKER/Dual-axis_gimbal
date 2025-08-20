import time
from weite_imu import WitIMU
def simple_example():
    """简单的WitIMU使用示例"""
    
    # 初始化IMU
    # 在Linux下使用实际的串口设备，通常是'/dev/ttyUSB0'或'/dev/ttyACM0'
    imu = WitIMU(port='/dev/ttyCH341USB0', baudrate=9600)
    print("IMU初始化完成")
    
    try:
        # 解锁设备
        imu.unlock()
        print("设备已解锁")
        
        # 设置输出内容: 加速度、角速度和角度
        imu.set_output_content(acc=True, gyro=True, angle=True)
        print("已设置输出内容")
        
        # 设置输出速率为10Hz
        imu.set_output_rate(0x06)  # 0x06 = 10Hz
        print("已设置输出速率: 10Hz")
        
        # 等待配置生效
        time.sleep(0.5)
        
        # 读取5秒数据
        print("开始读取数据，5秒...")
        start_time = time.time()
        
        while time.time() - start_time < 5:
            data_type, data = imu.read_data()
            
            if data_type == imu.TYPE_ANGLE:
                print(f"角度: Roll={data['roll']:.2f}°, Pitch={data['pitch']:.2f}°, Yaw={data['yaw']:.2f}°")
            
            elif data_type == imu.TYPE_ACC:
                print(f"加速度: X={data['ax']:.2f}g, Y={data['ay']:.2f}g, Z={data['az']:.2f}g")
                
            elif data_type == imu.TYPE_GYRO:
                print(f"角速度: X={data['wx']:.2f}°/s, Y={data['wy']:.2f}°/s, Z={data['wz']:.2f}°/s")
                
            time.sleep(0.01)  # 短暂延时，避免CPU占用过高
        
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 关闭串口连接
        imu.close()
        print("IMU连接已关闭")

if __name__ == "__main__":
    simple_example()
