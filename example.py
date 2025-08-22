import time
import math
import numpy as np
from haitai_motor_py.haitai_motor_multi import HaitaiMotorController
import matplotlib.pyplot as plt

# 配置matplotlib
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 12

# PD控制参数
MOTOR_PD_PARAMS = {
    'motor1': {'KP': 0.7, 'KD': 0.05},  # 提高Kp，降低Kd以获得更快响应
    'motor2': {'KP': 0.7, 'KD': 0.05}
}

MAX_SPEED = 3000  # 最大速度限制

# 优化后的速度环PID参数
SPEED_KP = 0.8    # 提高速度环增益
SPEED_KI = 0.2
SPEED_FILTER = 0.9

# 正弦轨迹参数
TRAJECTORY_PARAMS = {
    'motor1': {
        'amplitude': 100.0,
        'frequency': 1,
        'phase': 0.0
    },
    'motor2': {
        'amplitude': 100.0,
        'frequency': 1,
        'phase': 0.0
    }
}

# 简化的安全参数
MAX_POSITION_ERROR = 25.0    # 最大位置误差
ERROR_TIMEOUT = 2.0          # 减少超时时间

# 控制参数
CONTROL_FREQUENCY = 100      # 保持100Hz控制频率
DURATION = 15.0
PORT = '/dev/ttyCH341USB0'

# 电机地址
MOTOR_ADDRESSES = {
    'motor1': 0x01,
    'motor2': 0x02
}

def pd_control(target_position, current_position, current_speed, motor_id):
    """优化后的PD控制器"""
    params = MOTOR_PD_PARAMS[motor_id]
    KP = params['KP']
    KD = params['KD']
    
    position_error = target_position - current_position
    speed_command = KP * position_error - KD * current_speed
    speed_command_int = int(speed_command * 10)
    
    # 速度限幅
    speed_command_int = max(min(speed_command_int, MAX_SPEED), -MAX_SPEED)
    
    return speed_command_int

def generate_sine_target(elapsed_time, motor_id):
    """生成正弦轨迹"""
    params = TRAJECTORY_PARAMS[motor_id]
    amplitude = params['amplitude']
    frequency = params['frequency']
    phase = params['phase']
    return amplitude * math.sin(2 * math.pi * frequency * elapsed_time + phase)

def emergency_stop_motors(controller, motor_ids):
    """紧急停止电机"""
    print("\n🚨 紧急停止...")
    try:
        for motor_id in motor_ids:
            try:
                controller.speed_closed_loop(0, motor_id)
            except:
                pass
        time.sleep(0.05)  # 减少等待时间
        controller.shutdown_all_motors()
        print("✅ 电机已停止")
    except Exception as e:
        print(f"停止错误: {e}")

# 主程序
if __name__ == "__main__":
    print("\n" + "="*55)
    print("🎯 海泰双电机PD正弦跟随控制 (性能优化版)")
    print("="*55)
    print(f"串口: {PORT}")
    print(f"电机配置:")
    for motor_id, address in MOTOR_ADDRESSES.items():
        params = MOTOR_PD_PARAMS[motor_id]
        traj = TRAJECTORY_PARAMS[motor_id]
        print(f"  {motor_id.upper()}: 地址=0x{address:02X}, Kp={params['KP']}, Kd={params['KD']}")
        print(f"           轨迹: 振幅={traj['amplitude']}°, 频率={traj['frequency']}Hz")
    print(f"控制频率: {CONTROL_FREQUENCY}Hz (周期: {1000/CONTROL_FREQUENCY:.1f}ms)")
    print(f"速度环优化: Kp={SPEED_KP}, Ki={SPEED_KI}")
    print("="*55)
    
    # 记录大误差开始时间
    large_error_start_time = {}
    
    try:
        print("\n🔌 连接优化电机控制器...")
        motor_controller = HaitaiMotorController(
            port=PORT,
            baudrate=115200,
            motor_addresses=MOTOR_ADDRESSES,
            timeout=0.05  # 使用更短的超时时间
        )
        print("✅ 控制器连接成功")
        
        # 快速检查电机连接
        print("\n🔍 检查电机...")
        for motor_id in MOTOR_ADDRESSES.keys():
            try:
                info = motor_controller.get_device_info(motor_id)
                print(f"✅ {motor_id.upper()}: 地址=0x{info['address']:02X}")
                
                status = motor_controller.read_motor_status(motor_id)
                if status['fault_code'] != 0:
                    motor_controller.clear_faults(motor_id)
                    print(f"   故障已清除")
            except Exception as e:
                print(f"❌ {motor_id.upper()} 连接失败: {e}")
                motor_controller.close()
                exit(1)
        
        # 快速设置PID参数
        print("\n⚙️  优化PID参数...")
        for motor_id in MOTOR_ADDRESSES.keys():
            try:
                original_params = motor_controller.read_system_params(motor_id)
                new_params = original_params.copy()
                new_params['speed_kp'] = SPEED_KP
                new_params['speed_ki'] = SPEED_KI
                new_params['speed_filter'] = SPEED_FILTER
                motor_controller.write_system_params_temp(new_params, motor_id)
                print(f"✅ {motor_id.upper()} PID已优化")
            except Exception as e:
                print(f"⚠️  {motor_id.upper()} PID设置失败: {e}")
        
        # 设置零点
        print("\n📍 设置零点...")
        for motor_id in MOTOR_ADDRESSES.keys():
            try:
                result = motor_controller.set_current_position_as_origin(motor_id)
                if result['success']:
                    print(f"✅ {motor_id.upper()} 零点已设置")
            except Exception as e:
                print(f"⚠️  {motor_id.upper()} 零点设置失败: {e}")
        
        time.sleep(0.1)  # 减少等待时间
        
        # 数据存储
        data_storage = {}
        for motor_id in MOTOR_ADDRESSES.keys():
            data_storage[motor_id] = {
                'time_data': [],
                'target_data': [],
                'position_data': [],
                'error_data': [],
                'speed_cmd_data': [],
                'actual_speed_data': []
            }
        
        # 控制循环
        start_time = time.time()
        loop_period = 1.0 / CONTROL_FREQUENCY
        control_active = True
        loop_count = 0
        
        print("\n🚀 开始优化控制循环...")
        print("按 Ctrl+C 停止")
        print("="*70)
        
        # 预热循环，建立稳定的时序
        print("⏳ 控制系统预热中...")
        for _ in range(10):
            for motor_id in MOTOR_ADDRESSES.keys():
                try:
                    motor_controller.read_system_data(motor_id)
                except:
                    pass
            time.sleep(loop_period)
        
        print("🎮 正式开始控制...")
        
        try:
            while (time.time() - start_time) < DURATION and control_active:
                loop_start_time = time.time()
                elapsed_time = time.time() - start_time
                current_time = time.time()
                loop_count += 1
                
                motor_data_current = {}
                motor_targets = {}
                motor_errors = {}
                
                # 控制每个电机
                for motor_id in MOTOR_ADDRESSES.keys():
                    try:
                        # 生成目标
                        target_position = generate_sine_target(elapsed_time, motor_id)
                        motor_targets[motor_id] = target_position
                        
                        # 读取当前状态
                        motor_data = motor_controller.read_system_data(motor_id)
                        motor_data_current[motor_id] = motor_data
                        current_position = motor_data['multi_turn_angle']
                        current_speed = motor_data['speed']
                        position_error = target_position - current_position
                        motor_errors[motor_id] = position_error
                        
                        # 简化的安全检查
                        abs_error = abs(position_error)
                        if abs_error > MAX_POSITION_ERROR:
                            if motor_id not in large_error_start_time:
                                large_error_start_time[motor_id] = current_time
                            elif current_time - large_error_start_time[motor_id] > ERROR_TIMEOUT:
                                print(f"\n🚨 {motor_id.upper()} 误差过大，停止控制!")
                                control_active = False
                                break
                        else:
                            if motor_id in large_error_start_time:
                                del large_error_start_time[motor_id]
                        
                        # PD控制和发送命令
                        speed_command = pd_control(target_position, current_position, current_speed, motor_id)
                        motor_controller.speed_closed_loop(speed_command, motor_id)
                        
                        # 记录数据
                        data = data_storage[motor_id]
                        data['time_data'].append(elapsed_time)
                        data['target_data'].append(target_position)
                        data['position_data'].append(current_position)
                        data['error_data'].append(position_error)
                        data['speed_cmd_data'].append(speed_command / 10)
                        data['actual_speed_data'].append(current_speed)
                        
                    except Exception as e:
                        print(f"\n❌ {motor_id.upper()} 控制错误: {e}")
                        control_active = False
                        break
                
                if not control_active:
                    break
                
                # 显示状态 (每秒约5次)
                if loop_count % 20 == 0:
                    status_line = f"\r⏱️  {elapsed_time:.1f}s "
                    for motor_id in MOTOR_ADDRESSES.keys():
                        if motor_id in motor_targets and motor_id in motor_data_current:
                            target = motor_targets[motor_id]
                            current = motor_data_current[motor_id]['multi_turn_angle']
                            error = motor_errors[motor_id]
                            speed = motor_data_current[motor_id]['speed']
                            status_line += f"| {motor_id.upper()}: {target:.1f}°→{current:.1f}° "
                            status_line += f"(误差{error:.1f}° 速度{speed:.0f}) "
                    print(status_line, end="", flush=True)
                
                # 精确的频率控制
                loop_duration = time.time() - loop_start_time
                sleep_time = loop_period - loop_duration
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -loop_period * 0.1:  # 超时超过10%才警告
                    if loop_count % 100 == 0:  # 每100次警告一次
                        print(f"\n⚠️  控制延迟: {-sleep_time*1000:.1f}ms")
        
        except KeyboardInterrupt:
            print("\n\n⚠️  用户中断")
            control_active = False
        
        # 停止电机
        emergency_stop_motors(motor_controller, MOTOR_ADDRESSES.keys())
        
        print(f"\n✅ 控制完成，执行了 {loop_count} 次循环")
        
        # 性能分析
        if loop_count > 0:
            actual_freq = loop_count / DURATION
            print(f"📊 实际控制频率: {actual_freq:.1f}Hz (目标: {CONTROL_FREQUENCY}Hz)")
            efficiency = (actual_freq / CONTROL_FREQUENCY) * 100
            print(f"📊 控制效率: {efficiency:.1f}%")
        
        # 绘制结果
        try:
            print("\n📈 生成性能分析图表...")
            
            fig = plt.figure(figsize=(15, 10))
            colors = {'motor1': 'blue', 'motor2': 'red'}
            
            for i, motor_id in enumerate(MOTOR_ADDRESSES.keys()):
                data = data_storage[motor_id]
                if not data['time_data']:
                    continue
                
                color = colors.get(motor_id, f'C{i}')
                
                # 位置跟踪
                ax1 = fig.add_subplot(3, 2, i+1)
                ax1.plot(data['time_data'], data['target_data'], f'{color[0]}-', 
                        linewidth=2, alpha=0.8, label='目标')
                ax1.plot(data['time_data'], data['position_data'], f'{color[0]}--', 
                        linewidth=2, label='实际')
                ax1.set_xlabel('时间 (s)')
                ax1.set_ylabel('位置 (°)')
                ax1.set_title(f'{motor_id.upper()} - 位置跟踪 (优化版)')
                ax1.legend()
                ax1.grid(True, alpha=0.3)
                
                # 跟踪误差
                ax2 = fig.add_subplot(3, 2, i+3)
                ax2.plot(data['time_data'], data['error_data'], f'{color[0]}-', linewidth=2)
                ax2.axhline(y=MAX_POSITION_ERROR, color='red', linestyle=':', 
                           alpha=0.7, label=f'限制(±{MAX_POSITION_ERROR}°)')
                ax2.axhline(y=-MAX_POSITION_ERROR, color='red', linestyle=':', alpha=0.7)
                ax2.set_xlabel('时间 (s)')
                ax2.set_ylabel('误差 (°)')
                ax2.set_title(f'{motor_id.upper()} - 跟踪误差')
                ax2.legend()
                ax2.grid(True, alpha=0.3)
                
                # 速度响应
                ax3 = fig.add_subplot(3, 2, i+5)
                ax3.plot(data['time_data'], data['speed_cmd_data'], f'{color[0]}-', 
                        linewidth=2, alpha=0.8, label='命令')
                ax3.plot(data['time_data'], data['actual_speed_data'], f'{color[0]}--', 
                        linewidth=2, label='实际')
                ax3.set_xlabel('时间 (s)')
                ax3.set_ylabel('速度 (RPM)')
                ax3.set_title(f'{motor_id.upper()} - 速度响应')
                ax3.legend()
                ax3.grid(True, alpha=0.3)
                
                # 计算并显示性能指标
                if data['error_data']:
                    max_error = max([abs(e) for e in data['error_data']])
                    rms_error = math.sqrt(sum([e**2 for e in data['error_data']]) / len(data['error_data']))
                    print(f"📊 {motor_id.upper()}: 最大误差={max_error:.2f}°, RMS误差={rms_error:.2f}°")
            
            plt.tight_layout(pad=2.5)
            fig.suptitle('海泰双电机PD控制 - 性能优化版本', fontsize=16, y=0.98)
            plt.show()
            
            print("📊 图表显示完成")
            
        except ImportError:
            print("⚠️  matplotlib未安装，无法显示图表")
        except Exception as e:
            print(f"❌ 绘图错误: {e}")
    
    except Exception as e:
        print(f"❌ 程序错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        try:
            if 'motor_controller' in locals():
                emergency_stop_motors(motor_controller, MOTOR_ADDRESSES.keys())
                motor_controller.close()
                print("✅ 资源已清理")
        except:
            pass
        
        print("\n程序结束")
        print("=" * 55)
