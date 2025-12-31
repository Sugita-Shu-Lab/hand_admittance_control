import serial
import time
import struct
import math
import sys
import csv  
import matplotlib.pyplot as plt

# ==========================================
# 0. 配置开关
# ==========================================
# 绘图开关
ENABLE_PLOTTING = True 

# 外置传感器开关
ENABLE_EXTERNAL_SENSOR = False
EXTERNAL_SENSOR_PORT = 'COM7' 

# ==========================================
# 1. 传感器与驱动类
# ==========================================

# 外置力传感器驱动 
class ForceSensor:
    def __init__(self, port='COM7', baudrate=921600, device_addr=0x03):
        self.port = port
        self.baudrate = baudrate
        self.device_addr = device_addr
        self.ser = None
        self.start_addr = 1008
        self.read_len = 3

    def calculate_lrc(self, data: bytes) -> int:
        total_sum = sum(data)
        modulo = total_sum % 256
        if modulo == 0:
            lrc = 0
        else:
            lrc = 256 - modulo
        return lrc

    def build_read_request(self) -> bytes:
        header1, header2 = 0x55, 0xAA
        frame_len = 9
        func_code = 0xFB
        reserved = 0x00

        header_bytes = struct.pack('BB', header1, header2)
        data_len_bytes = struct.pack('<H', frame_len)
        ctrl_bytes = struct.pack('BBB', self.device_addr, reserved, func_code)
        param_bytes = struct.pack('<IH', self.start_addr, self.read_len)
        
        preceding_bytes = header_bytes + data_len_bytes + ctrl_bytes + param_bytes
        lrc = self.calculate_lrc(preceding_bytes)
        
        return preceding_bytes + struct.pack('B', lrc)

    def parse_sensor_data(self, payload: bytes):
        if len(payload) < 3:
            return None

        fx_raw = struct.unpack('b', payload[0:1])[0]
        fy_raw = struct.unpack('b', payload[1:2])[0]
        fz_raw = struct.unpack('B', payload[2:3])[0]

        # 转换为牛顿 N
        fx_n = fx_raw * 0.1
        fy_n = fy_raw * 0.1
        fz_n = fz_raw * 0.1

        return {
            'Fx': fx_n,
            'Fy': fy_n,
            'Fz': fz_n,  # 使用 Fz
            'Raw': payload.hex().upper()
        }

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.02 
            )
            print(f">>> 外置传感器已连接: {self.port}")
            return True
        except Exception as e:
            print(f"!!! 无法打开外置传感器串口: {e}")
            return False

    def read_data(self):
        if not self.ser:
            return None
        try:
            cmd = self.build_read_request()
            self.ser.reset_input_buffer()
            self.ser.write(cmd)
            expected_fixed_len = 18 
            response_head = self.ser.read(expected_fixed_len)
            
            if len(response_head) < expected_fixed_len:
                return None
            return self.parse_sensor_data(response_head[-(self.read_len+1):-1])
        except Exception:
            return None

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None

class SimpleLowPassFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.last_val = None

    def filter(self, val):
        if self.last_val is None:
            self.last_val = val
            return val
        filtered = self.alpha * val + (1 - self.alpha) * self.last_val
        self.last_val = filtered
        return filtered

class RH56_Driver:
    def __init__(self, port, baudrate=115200, hand_id=1):
        self.ser = serial.Serial(port, baudrate, timeout=0.05, write_timeout=0.05)
        self.hand_id = hand_id

    def _calc_checksum(self, data):
        return sum(data) & 0xFF

    def _clear_buffer(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def write_register(self, address, value):
        value = max(0, min(1000, int(value)))
        data_bytes = list(struct.pack('<H', value))
        frame_body = [self.hand_id, 0x05, 0x12, address & 0xFF, (address >> 8) & 0xFF, data_bytes[0], data_bytes[1]]
        checksum = self._calc_checksum(frame_body)
        full_frame = [0xEB, 0x90] + frame_body + [checksum]
        self._clear_buffer()
        self.ser.write(bytearray(full_frame))
        time.sleep(0.002) 

    def read_register(self, address):
        frame_body = [self.hand_id, 0x04, 0x11, address & 0xFF, (address >> 8) & 0xFF, 0x02]
        checksum = self._calc_checksum(frame_body)
        full_frame = [0xEB, 0x90] + frame_body + [checksum]
        self._clear_buffer()
        self.ser.write(bytearray(full_frame))
        start_time = time.time()
        while (time.time() - start_time) < 0.05: 
            if self.ser.in_waiting > 0:
                if self.ser.read(1) == b'\x90':
                    if self.ser.read(1) == b'\xEB':
                        rem = self.ser.read(8)
                        if len(rem) == 8:
                            return rem[5] | (rem[6] << 8)
                        return 0
        return 0 

# ==========================================
# 2. PD 导纳控制器
# ==========================================
class PD_AdmittanceController:
    def __init__(self, kp, kd, max_step, deadzone=5):
        self.kp = kp                                        
        self.kd = kd                                         
        self.max_step = max_step
        self.deadzone = deadzone
        self.last_force = None                               
        self.filter = SimpleLowPassFilter(alpha=0.4) 

    def compute(self, raw_force, target_force):
        current_force = self.filter.filter(raw_force)
        
        if self.last_force is None:
            self.last_force = current_force
            return 0

        error = target_force - current_force
        dF = current_force - self.last_force 
        self.last_force = current_force 

        if abs(error) < self.deadzone and abs(dF) < 5:
            return 0
        
        p_term = -(error * self.kp)       
        d_term = (dF * self.kd)           
        
        raw_delta = p_term + d_term
        
        delta = raw_delta
        if delta > self.max_step: delta = self.max_step
        if delta < -self.max_step: delta = -self.max_step
        
        return int(delta)

# ==========================================
# 3. 参数配置
# ==========================================
# --- 传感器反馈寄存器 (ACT) ---
REG_FORCE_ACT_RING = 1584        # 无名指实际受力值
REG_ANGLE_ACT_RING = 1548        

# --- 角度控制寄存器 (SET) ---
REG_ANGLE_SET_RING = 1488        
REG_ANGLE_SET_THUMB_BEND = 1494      
REG_ANGLE_SET_THUMB_ROT = 1496       

# --- 速度设置寄存器 (SPEED) ---
REG_SPEED_SET_RING = 1524            
REG_SPEED_SET_THUMB_BEND = 1530      
REG_SPEED_SET_THUMB_ROT = 1532       

# GESTURE_PREPARE = {"ring_angle": 800, "thumb_bend_angle": 600, "thumb_rot_angle": 90}
GESTURE_PREPARE = {"ring_angle": 700, "thumb_bend_angle": 1000, "thumb_rot_angle": 1000}
GESTURE_RESET = {"ring_angle": 1000, "thumb_bend_angle": 1000, "thumb_rot_angle": 1000}

def set_hand_speed(hand: RH56_Driver, speed_val):
    speed_val = int(max(0, min(1000, speed_val)))
    hand.write_register(REG_SPEED_SET_RING, speed_val)
    hand.write_register(REG_SPEED_SET_THUMB_BEND, speed_val)
    hand.write_register(REG_SPEED_SET_THUMB_ROT, speed_val)

def reset_hand_status(hand: RH56_Driver):
    print("\n>>> Resetting...")
    set_hand_speed(hand, 1000)
    hand.write_register(REG_ANGLE_SET_RING, GESTURE_RESET["ring_angle"])
    time.sleep(0.1)
    hand.write_register(REG_ANGLE_SET_THUMB_BEND, GESTURE_RESET["thumb_bend_angle"])
    time.sleep(0.1)
    hand.write_register(REG_ANGLE_SET_THUMB_ROT, GESTURE_RESET["thumb_rot_angle"])
    print(">>> Done.")

# 参数生成函数
def get_pd_params_auto(measured_stiffness, user_target_force):
    """
    根据实测刚度自动生成控制参数
    """
    K = max(0.1, min(200.0, float(measured_stiffness)))
    if(0<K<33):
        kp = 1.0 / K + 10
        kd = 0.0275 + (K * 0.01)
    elif(33<=K<90):
        kp = 1.0 / (K - 10) + 8    # 0.018 
        kd = 0.03 + (K * 0.012)    # 1.158 
    else:    
        kp = 1.0 / (K - 20)
        kd = 0.035 + (K * 0.015)
    # 步长限制
    if user_target_force > 450:
        max_step = 1
    else:
        max_step = 2

    # 4. 容差 (Tolerance)
    base_tol = 6 + (K * 0.25)
    force_tol = user_target_force * 0.02 
    tolerance = 15 #base_tol + force_tol

    print(f"[Auto-Param] Calc_K={K:.1f} | User_Target={int(user_target_force)}g")
    print(f"            Kp={kp:.3f} | Kd={kd:.3f} | Tol={int(tolerance)}")
    
    return {
        "kp": kp,
        "kd": kd,
        "max_step": max_step,
        "tolerance": int(tolerance)
    }

# ==========================================
# 4. 数据保存与绘图功能
# ==========================================
def save_csv_data(data_log, target_force_g):
    """
    将记录的数据保存为 CSV 文件
    """
    if not data_log['time']:
        print("无数据，跳过 CSV 保存")
        return

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"grasp_data_{timestamp}.csv"
    
    # 准备数据列
    times = data_log['time']
    int_forces = data_log['force']
    angles = data_log['angle']
    ext_forces = data_log.get('ext_force', [0.0] * len(times))
    min_len = min(len(times), len(int_forces), len(angles), len(ext_forces))
    
    try:
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # 写入表头
            writer.writerow(['Time_s', 'Internal_Force_Raw', 'Angle', 'External_Force_N', 'Target_Force_Raw'])
            
            # 写入数据
            for i in range(min_len):
                writer.writerow([
                    f"{times[i]:.4f}", 
                    int_forces[i], 
                    angles[i], 
                    f"{ext_forces[i]:.4f}",
                    target_force_g
                ])
        print(f"\n[CSV] 数据已保存至: {filename}")
    except Exception as e:
        print(f"!!! CSV 保存失败: {e}")

def save_plot(data_log, target_force_g, events):
    if not data_log['time']:
        print("无数据可绘图")
        return

    # 转换内部传感器单位：g -> N (1N ≈ 100g)
    force_N = [f / 100.0 for f in data_log['force']]
    target_N = target_force_g / 100.0
    
    plt.figure(figsize=(12, 10))
    
    # --- 子图1: 力 vs 时间 ---
    plt.subplot(2, 1, 1)
    plt.plot(data_log['time'], force_N, label='Internal Force (N)', color='blue', linewidth=1.5)
    
    if 'ext_force' in data_log and len(data_log['ext_force']) > 0:
        ext_forces = [f if f is not None else 0 for f in data_log['ext_force']]
        plt.plot(data_log['time'], ext_forces, label='External Force (N)', color='magenta', linestyle='-.', linewidth=1.5)

    plt.axhline(y=target_N, color='red', linestyle='--', label=f'Target ({target_N:.2f}N)')
    
    if events['contact_time']:
        plt.axvline(x=events['contact_time'], color='red', linestyle=':', linewidth=2, label='Contact Detected')

    if events['stiff_start'] and events['stiff_end']:
        plt.axvspan(events['stiff_start'], events['stiff_end'], color='gray', alpha=0.3, label='Stiffness Calc')
        
    if events['stable_time']:
        plt.axvline(x=events['stable_time'], color='green', linestyle='-', linewidth=2, label='Stable')

    plt.title('Force Tracking Comparison')
    plt.ylabel('Force (N)')
    plt.ylim(0, 10) 
    plt.grid(True)
    plt.legend(loc='upper left')
    
    # --- 子图2: 角度 vs 时间 ---
    plt.subplot(2, 1, 2)
    plt.plot(data_log['time'], data_log['angle'], label='Finger Angle', color='orange', linewidth=1.5)
    
    if events['contact_time']:
        plt.axvline(x=events['contact_time'], color='red', linestyle=':', linewidth=2, label='Contact')
    if events['stiff_start'] and events['stiff_end']:
        plt.axvspan(events['stiff_start'], events['stiff_end'], color='gray', alpha=0.3, label='Stiffness Calc')
    if events['stable_time']:
        plt.axvline(x=events['stable_time'], color='green', linestyle='-', linewidth=2, label='Stable')
        
    plt.title('Finger Position Trajectory')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (0-1000)')
    plt.grid(True)
    plt.legend(loc='upper right')
    
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"grasp_result_{timestamp}.png"
    plt.tight_layout()
    plt.savefig(filename)
    print(f"[Plot] 图像已保存至: {filename}")
    plt.close()

# ==========================================
# 5. 主逻辑 (自适应刚度辨识 + 导纳控制)
# ==========================================
def run_grasp_task(hand: RH56_Driver, user_target_force, ext_sensor=None):
    
    # 初始化数据记录
    data_log = {'time': [], 'force': [], 'angle': []}
    if ext_sensor:
        data_log['ext_force'] = []
    
    events = {'contact_time': None, 'stiff_start': None, 'stiff_end': None, 'stable_time': None}
    start_time = time.time()
    
    # 辅助函数：记录一次数据
    def log_step():
        elapsed = time.time() - start_time
        f_g = hand.read_register(REG_FORCE_ACT_RING)
        if f_g > 2000: f_g = 0 # 简单滤波
        ang = hand.read_register(REG_ANGLE_ACT_RING)
        
        data_log['time'].append(elapsed)
        data_log['force'].append(f_g)
        data_log['angle'].append(ang)
        
        if ext_sensor:
            ext_data = ext_sensor.read_data()
            if ext_data and 'Fz' in ext_data:
                data_log['ext_force'].append(ext_data['Fz']) 
            else:
                if data_log['ext_force']:
                    data_log['ext_force'].append(data_log['ext_force'][-1])
                else:
                    data_log['ext_force'].append(0.0)
                    
        return f_g, ang, elapsed

    # --- Phase A: 预动作 ---
    print(">>> Phase A: 预动作准备")
    set_hand_speed(hand, 1000)    
    hand.write_register(REG_ANGLE_SET_THUMB_ROT, GESTURE_PREPARE["thumb_rot_angle"])
    hand.write_register(REG_ANGLE_SET_THUMB_BEND, GESTURE_PREPARE["thumb_bend_angle"])
    hand.write_register(REG_ANGLE_SET_RING, GESTURE_PREPARE["ring_angle"])
    time.sleep(0.8)
    
    current_angle = hand.read_register(REG_ANGLE_ACT_RING)
    log_step()

    # --- Phase B: 试探接近 ---
    print(">>> Phase B: 试探接近，正在检测接触 ")
    set_hand_speed(hand, 500) 
    
    CONTACT_THRESHOLD = 30 
    PROBE_STEP = 4           
    
    try:
        while True:
            # 记录数据
            f_val, _, t_now = log_step()

            if f_val >= CONTACT_THRESHOLD:
                print(f"!!! Contact Detected: {f_val}g")
                events['contact_time'] = t_now 
                break
            
            if current_angle <= 50:
                print("Range End (No Object)")
                # 退出前保存
                if ENABLE_PLOTTING: save_plot(data_log, user_target_force, events)
                save_csv_data(data_log, user_target_force) # [新增]
                return
            
            current_angle -= PROBE_STEP
            hand.write_register(REG_ANGLE_SET_RING, current_angle)
            time.sleep(0.01) 

        # =======================================================
        # Phase B.6: 刚度辨识
        # =======================================================
        print(">>> Phase B.6: Estimating Stiffness...")
        _, _, t_stiff_start = log_step()
        events['stiff_start'] = t_stiff_start
        
        time.sleep(0.2) 
        log_step()
        angle_start = hand.read_register(REG_ANGLE_ACT_RING)
        force_start = hand.read_register(REG_FORCE_ACT_RING)
        if force_start > 2000: force_start = CONTACT_THRESHOLD

        PRESS_DEPTH = 4
        target_press_angle = max(0, current_angle - PRESS_DEPTH)
        hand.write_register(REG_ANGLE_SET_RING, target_press_angle)
        
        for _ in range(3):
            time.sleep(0.05)
            log_step()
        
        angle_end = hand.read_register(REG_ANGLE_ACT_RING)
        force_end = hand.read_register(REG_FORCE_ACT_RING)
        if force_end > 2000: force_end = force_start 

        d_angle = abs(angle_start - angle_end) 
        d_force = force_end - force_start
        
        if d_angle < 1: d_angle = 1 
        
        raw_stiffness = d_force / d_angle
        calc_k = raw_stiffness * 4.0 
        
        print(f"    dAngle: {d_angle}, dForce: {d_force}")
        print(f"    Raw Slope: {raw_stiffness:.2f} -> Mapped K: {calc_k:.1f}")
        
        params = get_pd_params_auto(calc_k, user_target_force)
        
        current_angle = target_press_angle
        
        _, _, t_stiff_end = log_step()
        events['stiff_end'] = t_stiff_end

        # =======================================================
        # Phase C: 导纳控制
        # =======================================================
        controller = PD_AdmittanceController(
            kp=params['kp'], 
            kd=params['kd'], 
            max_step=params['max_step'], 
            deadzone=3
        )
        
        target_f = user_target_force
        tolerance = params['tolerance']
        print(f">>> Phase C: PD Control (Target: {target_f}g)")
        
        stable_count = 0        
        STABLE_TARGET = 2
        
        while True:
            # 记录数据
            f_val, _, t_now = log_step()
            real_force = f_val
            
            if real_force > 2000: real_force = target_f 
            
            # 2. 达标判断
            force_diff = abs(real_force - target_f)
            
            if force_diff < tolerance:
                stable_count += 1
            else:
                stable_count = 0
            
            # 达到稳定后，停止控制但继续记录1秒数据
            if stable_count >= STABLE_TARGET:
                print(f"\n========================")
                print(f"抓稳了! 停止控制，继续记录数据 1s ...")
                print(f"Force: {real_force}g / Target: {target_f}g")
                print(f"========================")
                events['stable_time'] = t_now
                
                hold_start_t = time.time()
                while (time.time() - hold_start_t) < 1.0:
                    # 仅记录数据，不计算PD，不写入电机
                    log_step()
                    time.sleep(0.01) # 保持采样率
                
                print(">>> 记录完成，退出任务。")
                break # 彻底退出
            
            # 3. PD 计算 (仅在未稳定时执行)
            delta = controller.compute(real_force, target_f)
            
            # 4. 执行
            if delta != 0:
                current_angle += delta
                current_angle = max(0, min(1000, current_angle))
                hand.write_register(REG_ANGLE_SET_RING, current_angle)
            
            time.sleep(0.08)

    except KeyboardInterrupt:
        print("Stopped")
    finally:
        # 任务结束，保存 CSV 和 绘图
        save_csv_data(data_log, user_target_force) # [新增] 保存 CSV
        if ENABLE_PLOTTING: 
            save_plot(data_log, user_target_force, events)

if __name__ == "__main__":
    # 初始化外置传感器 (如果启用)
    ext_sensor_driver = None
    if ENABLE_EXTERNAL_SENSOR:
        ext_sensor_driver = ForceSensor(port=EXTERNAL_SENSOR_PORT, baudrate=921600)
        if not ext_sensor_driver.connect():
            print("警告: 无法连接外置传感器，将仅使用内置传感器运行。")
            ext_sensor_driver = None
        else:
            print(">>> 外置传感器就绪，将在绘图中显示对比数据。")

    try:
        driver = RH56_Driver(port='COM6') 
        print(">>> Smart Grasp Ready (Auto-Stiffness Detection)")
        
        while True:
            s = input("\n请输入目标抓取力 (单位:g, 推荐 200-800) 或 'q' 退出: ")
            if s.lower() == 'q': 
                reset_hand_status(driver)
                break
            try:
                target_f = float(s)
                # 传入外置传感器对象
                run_grasp_task(driver, target_f, ext_sensor=ext_sensor_driver)
            except ValueError:
                print("请输入有效的数字")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ext_sensor_driver:
            ext_sensor_driver.close()