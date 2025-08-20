import time
from  ATK_901 import ATK_MS901M

# 初始化传感器
sensor = ATK_MS901M('/dev/ttyCH341USB0')  
if sensor.open():
    print("传感器连接成功")
    
    try:
        # 设置主动上报内容
        sensor.set_return_data(
            sensor.RETURN_ATTITUDE | 
            sensor.RETURN_GYRO_ACC | 
            sensor.RETURN_MAG |
            sensor.RETURN_BARO |
            sensor.RETURN_PORT_STATUS
        )
        
        # 设置上报速率为50Hz
        sensor.set_return_rate(sensor.RATE_50HZ)
        
        # 保存配置
        sensor.save_configuration()
        
        # 读取一些数据
        for _ in range(20):
            # 获取数据
            attitude = sensor.get_attitude()
            gyro = sensor.get_gyro()
            acc = sensor.get_acc()
            mag = sensor.get_mag()
            baro = sensor.get_baro()
            
            # 打印数据
            print(f"姿态角: Roll={attitude['roll']:.2f}°, Pitch={attitude['pitch']:.2f}°, Yaw={attitude['yaw']:.2f}°")
            print(f"陀螺仪: X={gyro['x']:.2f}°/s, Y={gyro['y']:.2f}°/s, Z={gyro['z']:.2f}°/s")
            print(f"加速度: X={acc['x']:.2f}g, Y={acc['y']:.2f}g, Z={acc['z']:.2f}g")
            print(f"磁力计: X={mag['x']}, Y={mag['y']}, Z={mag['z']}, 温度={mag['temperature']:.2f}°C")
            print(f"气压计: 气压={baro['pressure']}Pa, 海拔={baro['altitude']}cm, 温度={baro['temperature']:.2f}°C")
            print("-" * 40)
            
            time.sleep(0.1)
            
    finally:
        # 关闭连接
        sensor.close()
        print("传感器连接已关闭")
else:
    print("无法连接传感器")
