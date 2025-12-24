import serial
import time
import struct
import math


# 1. 工具类: 滤波器、驱动

class SimpleLowPassFilter:
    """
    简单指数移动平均滤波器 (EMA)
    用于平滑力传感器数据，防止 PD 控制中的 D 项因噪声产生剧烈抖动
    """
    def __init__(self, alpha=0.3):
        # alpha 越小，滤波越强(平滑)，但滞后越大
        # 0.3 是一个平衡点，既能滤除尖峰噪声，延迟也在 3-4 个周期内
        self.alpha = alpha
        self.last_val = None

    def filter(self, val):
        if self.last_val is None:
            self.last_val = val
            return val
        
        # EMA 公式: New = Alpha * Raw + (1-Alpha) * Last
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


# 2. 导纳PD控制器 

class PD_AdmittanceController:
    def __init__(self, kp, kd, max_step, deadzone=5):
        self.kp = kp                                 # 比例增益————动力
        self.kd = kd                                 # 微分增益————阻尼
        self.max_step = max_step
        self.deadzone = deadzone
        self.last_force = None                       # 上一次的力
        self.filter = SimpleLowPassFilter(alpha=0.4) # 初始化滤波器

    def compute(self, raw_force, target_force):
        # 1. 滤波处理 (关键步骤)
        current_force = self.filter.filter(raw_force)
        
        # 初始化 last_force
        if self.last_force is None:
            self.last_force = current_force
            return 0

        # 2. 计算误差 (P项输入)
        error = target_force - current_force
        
        # 3. 计算力变化率 (D项输入)
        dF = current_force - self.last_force 

        # 4.更新力
        self.last_force = current_force 

        # 5.死区判断 最大设为5
        if abs(error) < self.deadzone and abs(dF) < 5:
            return 0
        
        
        p_term = -(error * self.kp)       #  逻辑: 缺力(error>0) -> 需要负Delta(捏紧)
        d_term = (dF * self.kd)           #  逻辑: 力激增(dF>0) -> 需要正Delta(松开) 
        
        raw_delta = p_term + d_term
        
        # 严格限幅 (Gentle Motion)
        delta = raw_delta
        if delta > self.max_step: delta = self.max_step
        if delta < -self.max_step: delta = -self.max_step
        
        return int(delta)


# 3. 参数配置 （因为内部换了电机，此处调取的是无名指和拇指的地址，实际看到的现象是食指和拇指对捏）

REG_ANGLE_SET_RING = 1488   
REG_FORCE_ACT_RING = 1584   
REG_ANGLE_ACT_RING = 1548   
REG_ANGLE_SET_THUMB_BEND = 1494 
REG_ANGLE_SET_THUMB_ROT = 1496  
REG_SPEED_SET_RING = 1524       
REG_SPEED_SET_THUMB_BEND = 1530 
REG_SPEED_SET_THUMB_ROT = 1532  

GESTURE_PREPARE = {"ring_angle": 1000, "thumb_bend_angle": 600, "thumb_rot_angle": 90}
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

def get_pd_params_continuous(stiffness_val):
    """
    基于刚度的连续参数映射
    输入: stiffness_val (0.1 - 100.0)
    输出: target_force, kp, kd, max_step, tolerance
    """
    # 1. 刚度范围 【0.1指的塑料一次性杯（很软） 100指的硬塑料块（很硬）】
    K = max(0.1, min(100.0, float(stiffness_val)))
    
    # 2. 映射逻辑

    # A. 目标力 
    # 线性映射: 0->180g, 100->800g
    target_force = 180 + (K * 6.2)  
    target_force = min(800, target_force)

    # B. 比例增益 Kp 
    # 原则: 物体越硬，Kp 必须越小，否则一碰力就飞
    kp = 1 / (K + 5) + 0.02  
    
    # C. 微分增益 Kd 
    # 原则: 物体越硬，回弹越快，需要大kd刹车 
    kd = 0.02 + (K * 0.0035)   

    # D. 步长限制 
    # 原则来讲硬物体应该给小步长，软物体应该给大步长，但是机械手本身位移的分辨率已经已经是0.8mm了，因此都给最小步长1
    if K > 60:
        max_step = 1
    elif K > 30:
        max_step = 1
    else:
        max_step = 1

    # E. 容差 软物体容差小，硬物体容差大
    tolerance = 12 + (K * 0.40)

    print(f"[Auto-Tune] K={K:.1f} | Tgt={int(target_force)}g | Kp={kp:.3f} | Kd={kd:.3f} | Step={max_step}")
    
    return {
        "target_force": int(target_force),
        "kp": kp,
        "kd": kd,
        "max_step": max_step,
        "tolerance": int(tolerance)
    }


# 4. 主逻辑 

def run_grasp_task(hand: RH56_Driver, object_stiffness_value):
    # 使用新的参数生成函数
    params = get_pd_params_continuous(object_stiffness_value)
    
    # 初始化 PD 控制器
    controller = PD_AdmittanceController(
        kp=params['kp'], 
        kd=params['kd'], 
        max_step=params['max_step'], 
        deadzone=5
    )
    
    # --- Phase A: 预动作  --- 此处直接给的食指拇指对捏
    print(">>> Phase A: Prepare")
    set_hand_speed(hand, 1000)    
    hand.write_register(REG_ANGLE_SET_THUMB_ROT, GESTURE_PREPARE["thumb_rot_angle"])
    hand.write_register(REG_ANGLE_SET_THUMB_BEND, GESTURE_PREPARE["thumb_bend_angle"])
    hand.write_register(REG_ANGLE_SET_RING, GESTURE_PREPARE["ring_angle"])
    time.sleep(0.8)
    
    current_angle = hand.read_register(REG_ANGLE_ACT_RING)

    # --- Phase B: 试探接近阶段  ---
    # 动态调整接近速度 (硬物体接近慢，软物体接近快)
    approach_speed = int(300 - params['kd'] * 1000) # 简单映射，Kd越大说明越硬，速度越慢
    approach_speed = max(150, min(500, approach_speed))
    print(f">>> Slowing down (Speed {approach_speed})")
    set_hand_speed(hand, approach_speed)    

    print(">>> Phase B: Probe")
    CONTACT_THRESHOLD = 100
    PROBE_STEP = 4 
    
    while True:
        f_val = hand.read_register(REG_FORCE_ACT_RING)
        if f_val > 2000: f_val = 0 
        
        if f_val >= CONTACT_THRESHOLD:
            print(f"!!! Contact: {f_val}g")
            break
        
        if current_angle <= 50:
            print("Range End")
            break
            
        current_angle -= PROBE_STEP
        hand.write_register(REG_ANGLE_SET_RING, current_angle)
        time.sleep(0.01) 

    # --- Phase B.5: 接触后走一小步试探 ---
    print(">>> Phase B.5: Tentative")
    TENTATIVE_STEP = 1           
    current_angle -= TENTATIVE_STEP
    current_angle = max(0, current_angle)
    hand.write_register(REG_ANGLE_SET_RING, current_angle)
    time.sleep(0.1) 
    
    # --- Phase C: 导纳PD控制阶段 ---
    target_f = params['target_force']
    tolerance = params['tolerance']
    print(f">>> Phase C: PD Control (Tgt: {target_f}g)")
    
    stable_count = 0        
    STABLE_TARGET = 4 # 稍微提高稳定要求
    
    try:
        while True:
            # 1. 读传感器
            real_force = hand.read_register(REG_FORCE_ACT_RING)
            if real_force > 2000: real_force = target_f # 异常值过滤
            
            # 2. 达标判断 (使用原始值判断，因为控制器内部有滤波)
            force_diff = abs(real_force - target_f)
            
            if force_diff < tolerance:
                stable_count += 1
            else:
                stable_count = 0
            
            if stable_count >= STABLE_TARGET:
                print(f"\n========================")
                print(f"已捏紧!")
                print(f"Force: {real_force}g / Target: {target_f}g")
                print(f"========================\n")
                break 
            
            # 3. PD 计算 (内部含滤波)
            delta = controller.compute(real_force, target_f)
            
            # 4. 执行
            if delta != 0:
                current_angle += delta
                current_angle = max(0, min(1000, current_angle))
                hand.write_register(REG_ANGLE_SET_RING, current_angle)
            
            time.sleep(0.01) 
            
    except KeyboardInterrupt:
        print("Stopped")

if __name__ == "__main__":
    try:
        driver = RH56_Driver(port='COM6') 
        print(">>> PD Control Ready")
        
        while True:
            s = input("\n请输入准确刚度值 (0.1 - 100.0) 或 'q' 退出: ")
            if s.lower() == 'q': 
                reset_hand_status(driver)
                break
            try:
                run_grasp_task(driver, float(s))
            except ValueError:
                print("Invalid number")
            
    except Exception as e:
        print(f"Error: {e}")