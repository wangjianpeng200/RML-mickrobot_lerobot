import base64
import json
import threading
import time
import math
from pathlib import Path
import cv2
import zmq
import sys
import struct
from lerobot.common.robot_devices.robots.mobile_manipulator import LeKiwi
from Robotic_Arm.rm_robot_interface import *
from serial.serialutil import SerialException
import serial
import time

class Chassis:
    def __init__(self):
        self.available = 0
        self.vx = self.vy = self.wz = 0.0
        self.px = self.py = self.pz = 0.0
        self.chassis_type = 0  # 0:X4  1:M4  2:Ackermann  3:4WS4WD
        self.motor_type = 0   

class MotorMeasure:
    def __init__(self):
        self.speed_rpm = 0
        self.round_cnt = 0
        self.angle = 0

mickv3_chassis = Chassis()
arm_lock = threading.Lock()  

class X4ChassisController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        self.ser = self.init_serial(port, baudrate, timeout)
        self.speed_offset = 10  # 速度偏移值，用于将负数转换为正数
        self.moto_chassis = [MotorMeasure() for _ in range(4)]
        self.chassis= Chassis()

    def init_serial(self, port, baudrate, timeout):
        try:
            ser = serial.Serial(
                port=port,          # 串口路径（Windows 为 COMx）
                baudrate=baudrate,  # 波特率（与硬件一致）s
                parity=serial.PARITY_NONE,  # 校验位（无）
                stopbits=serial.STOPBITS_ONE,  # 停止位（1位）
                bytesize=serial.EIGHTBITS,    # 数据位（8位）
                timeout=timeout       # 读取超时时间（秒）
            )
            if ser.is_open:
                print(f"串口 {port} 已打开，波特率 {baudrate}")
                return ser
        except serial.SerialException as e:
            print(f"串口打开失败：{e}")
            return None

    def read_serial(self,port):
        # serial_data = b''
        serial_data= port.read(port.in_waiting or 0)
        return serial_data

    def send_speed_to_X4chassis(self, x: float, y: float, w: float):
        if not self.ser or not self.ser.is_open:
            print("串口未连接")
            return

        speed_offset = 10  
        data_tem = bytearray()

        data_tem.extend([0xAE, 0xEA, 0x00, 0xF3])  

        def pack_speed(v: float) -> list:
            scaled = int((v + speed_offset) * 100)
            return [scaled >> 8 & 0xFF, scaled & 0xFF]

        data_tem.extend(pack_speed(x))  # X速度
        data_tem.extend(pack_speed(y))  # Y速度
        data_tem.extend(pack_speed(w))  # 角速度
        data_tem.extend([0x00, 0x00])  # 保留位
        checksum = sum(data_tem) & 0xFF
        data_tem.append(0xFF - checksum)
        data_tem[2] = len(data_tem) - 2  # 排除起始的AE EA两个字节
        data_tem.extend([0xEF, 0xFE])
        try:
            self.ser.write(data_tem)
        except Exception as e:
            print(f"发送失败: {str(e)}")

    def cmd_vel_callback(self, linear_x: float, angular_z: float, chassis_type: int):
        speed_x = linear_x
        speed_y = 0.0 
        speed_w = angular_z
        self.send_speed_to_X4chassis(speed_x, speed_y, speed_w)

    def updata(self,mick_lock,stop_event):
        while not stop_event.is_set():
            if not self.ser or not self.ser.is_open:
                return False
            data = self.read_serial(self.ser)
            if data:
                uart_recive_flag=self.analy_uart_recive_data(data)
                if uart_recive_flag:
                    with mick_lock:
                        mickv3_chassis.vx = self.chassis.vx
                        mickv3_chassis.vy = self.chassis.vy
                        mickv3_chassis.wz = self.chassis.wz
                        mickv3_chassis.available = self.chassis.available
                        # print("接受",mickv3_chassis.vx,mickv3_chassis.vy,mickv3_chassis.wz)
            time.sleep(0.06)

    def analy_uart_recive_data(self, serial_data: bytes) -> bool:
        reviced_tem = []
        rec_flag = False
        tem_last = 0
        tem_curr = 0
        header_count = 0
        step = 0
        # 帧头检测
        for i in range(len(serial_data)):
            tem_last = tem_curr
            tem_curr = serial_data[i]
            if not rec_flag:
                if tem_last == 0xAE and tem_curr == 0xEA:
                    rec_flag = True
                    reviced_tem.extend([tem_last, tem_curr])
            else:
                reviced_tem.append(tem_curr)
                if tem_last == 0xEF and tem_curr == 0xFE:
                    header_count += 1
                    rec_flag = False

        for k in range(header_count):
            if step + 2 >= len(reviced_tem):
                break
            LL = reviced_tem[step + 2]
            frame_len = LL + 4  

            if any([
                reviced_tem[step] != 0xAE,
                reviced_tem[step + 1] != 0xEA,
                reviced_tem[step + frame_len - 2] != 0xEF,
                reviced_tem[step + frame_len - 1] != 0xFE
            ]):
                step += 1
                continue

            if reviced_tem[step + 3] == 0xA7:
                offset = step + 4
                try:
                    vx = int.from_bytes(reviced_tem[offset:offset+2], 'big', signed=True) / 1000.0
                    vy = int.from_bytes(reviced_tem[offset+2:offset+4], 'big', signed=True) / 1000.0
                    wz = int.from_bytes(reviced_tem[offset+4:offset+6], 'big', signed=True) / 1000.0
                    self.chassis.vx = vx
                    self.chassis.vy = vy
                    self.chassis.wz = wz
                    self.chassis.available = True
                    return True     
                except Exception as e:
                    pass
            step += frame_len
        return False

class Gen72:
    def __init__(self):
        
        self.arm= RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        robot=self.arm.rm_create_robot_arm("192.168.1.18", 8080)

        self.arm.rm_set_arm_run_mode(2)
        self.arm.rm_set_tool_voltage(3) 
        self.arm.rm_set_modbus_mode(1,115200,2)   
        self.write_params = rm_peripheral_read_write_params_t(1, 40000, 1)
        self.arm.rm_write_single_register(self.write_params,100) 
        self.arm.rm_change_tool_frame("lebai2")
        #-------------------
        self.gipflag=1
        self.gipflag_send=1
        self.leader_pos = {}
        self.joint_teleop_write = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_teleop_read = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_obs_read=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_send=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gip_obs=0
        joint_base = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.arm.rm_movej(joint_base, 20, 0, 0, 1)

    def joint_teleop(self, joint_teleop_read):
        for i in range(7):
            self.joint_teleop_write[i] = joint_teleop_read[i]
        # self.joint_teleop_write[1] = 0-self.joint_teleop_write[1]
        self.joint_teleop_write[3] = 0-self.joint_teleop_write[3]
        self.joint_teleop_write[5] = 0-self.joint_teleop_write[5]
        self.arm.rm_movej_canfd(self.joint_teleop_write, True, 0, 2,200)
        giper_trans = ((math.fabs(joint_teleop_read[7]-98))/(98-33))*100
        
        giper_trans=round(giper_trans, 2)
        if (giper_trans < 21) and (self.gipflag == 1):
            self.arm.rm_write_single_register(self.write_params,10)
            self.gipflag=0
        #状态为闭合，且需要张开夹爪
        if (giper_trans > 79) and (self.gipflag == 0):
            self.arm.rm_write_single_register(self.write_params,100)
            self.gipflag=1
        self.gip_obs=giper_trans

    def arm_state_func(self,data):
        joint_status = data.joint_status.to_dict()
        # print("joint_position: ", joint_status["joint_position"])
        with arm_lock:
            self.joint_obs_read = joint_status["joint_position"]
        
    def update(self,robot_configs,even_stop):
        ip=robot_configs.ip_rml
        custom = rm_udp_custom_config_t()
        custom.joint_speed = 0
        custom.lift_state = 0
        custom.expand_state = 0
        custom.arm_current_status=1
        config = rm_realtime_push_config_t(4, True, 8089, 0, ip, custom)
        print(self.arm.rm_set_realtime_push(config))
        print(self.arm.rm_get_realtime_push())
        arm_state_callback = rm_realtime_arm_state_callback_ptr(self.arm_state_func)
        self.arm.rm_realtime_arm_state_call_back(arm_state_callback)    
        while not even_stop.is_set():
            time.sleep(0.1)

def setup_zmq_sockets(config):
    context = zmq.Context()
    cmd_socket = context.socket(zmq.PULL)
    cmd_socket.setsockopt(zmq.CONFLATE, 1)
    cmd_socket.bind(f"tcp://*:{config.port}")

    video_socket = context.socket(zmq.PUSH)
    video_socket.setsockopt(zmq.CONFLATE, 1)
    video_socket.bind(f"tcp://*:{config.video_port}")

    return context, cmd_socket, video_socket

def run_camera_capture(cameras, images_lock, latest_images_dict, stop_event):
    while not stop_event.is_set():
        local_dict = {}
        for name, cam in cameras.items():
            frame = cam.async_read()
            ret, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if ret:
                local_dict[name] = base64.b64encode(buffer).decode("utf-8")
            else:
                local_dict[name] = ""
        with images_lock:
            latest_images_dict.update(local_dict)
        time.sleep(0.01)

def run_lekiwi(robot_config):
    from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs
    # Initialize cameras from the robot configuration.
    cameras = make_cameras_from_configs(robot_config.cameras)
    for cam in cameras.values():
        cam.connect()

    arm_motor_ids = [
            "shoulder_pan",    # 肩部水平旋转关节
            "shoulder_lift",   # 肩部垂直抬升关节
            "elbow_flex",      # 肘部弯曲关节
            "wrist_flex",      # 腕部弯曲关节
            "wrist_roll",      # 腕部转动关节
            "wrist_1",         # 腕部第一轴
            "wrist_2",         # 腕部第二轴
            "gripper"          # 夹具控制
        ]

    context, cmd_socket, video_socket = setup_zmq_sockets(robot_config)

    # Start the camera capture thread.
    # 存储最新相机帧的字典（线程共享资源）
    gen72=Gen72()
    mick=X4ChassisController(port="/dev/ttyUSB0", baudrate=115200)
    latest_images_dict = {}  
    images_lock = threading.Lock() 
    mick_lock = threading.Lock()  
    stop_event = threading.Event()  
    cam_thread = threading.Thread(target=run_camera_capture, args=(cameras, images_lock, latest_images_dict, stop_event),daemon=True)
    arm_thread = threading.Thread(target=gen72.update,args=(robot_config,stop_event),daemon=True)
    mick_thread= threading.Thread(target=mick.updata,args=(mick_lock,stop_event),daemon=True)
    cam_thread.start()  
    arm_thread.start()
    mick_thread.start()
    last_cmd_time = time.time()
    time.sleep(1)
    print("LeKiwi robot server started. Waiting for commands...")
    try:
        while True:
            loop_start_time = time.time()
            while True:
                try:
                    msg = cmd_socket.recv_string(zmq.NOBLOCK)
                except zmq.Again:
                    break
                try:
                    data = json.loads(msg)
                    if "arm_positions" in data:
                        arm_positions = data["arm_positions"]
                        if not isinstance(arm_positions, list):
                            print(f"[ERROR] Invalid arm_positions: {arm_positions}")
                        elif len(arm_positions) < len(arm_motor_ids):
                            print(
                                f"[WARNING] Received {len(arm_positions)} arm positions, expected {len(arm_motor_ids)}"
                            )
                        else:
                            joint_state = [0.0] * 8
                            for index, (motor, pos) in enumerate(zip(arm_motor_ids, arm_positions, strict=False)):
                                joint_state[index] = pos
                            gen72.joint_teleop(joint_state)
                    if "raw_velocity" in data:
                        raw_command = data["raw_velocity"]
                        command_speeds = [
                            int(raw_command.get("left_wheel", 0)),
                            int(raw_command.get("back_wheel", 0)),
                            int(raw_command.get("right_wheel", 0)),
                        ]
                        # print(f"[INFO] Setting velocity to {command_speeds}")
                        mick.cmd_vel_callback(command_speeds[1],command_speeds[0],command_speeds[2])
                        
                        last_cmd_time = time.time()
                except Exception as e:
                    print(f"[ERROR] Parsing message failed: {e}")
            # Watchdog: stop the robot if no command is received for over 0.5 seconds.
            now = time.time()
            if now - last_cmd_time > 0.5:
                last_cmd_time = now

            current_velocity ={}
            with arm_lock:
                follower_arm_state = gen72.joint_obs_read+[gen72.gip_obs]
            with images_lock:
                images_dict_copy = dict(latest_images_dict)
            with mick_lock:
                current_velocity ={"left_wheel": int(mickv3_chassis.vx), "back_wheel": int(mickv3_chassis.vy), "right_wheel": int(mickv3_chassis.wz)}
                # print("发送速度",mickv3_chassis.vx,mickv3_chassis.vy,mickv3_chassis.wz)
                # print(follower_arm_state)

            observation = {
                "images": images_dict_copy,
                "present_speed": current_velocity,
                "follower_arm_state": follower_arm_state,
            }
            # Send the observation over the video socket.
            video_socket.send_string(json.dumps(observation))

            elapsed = time.time() - loop_start_time
            time.sleep(
                max(0.033 - elapsed, 0)
            ) 
    except KeyboardInterrupt:
        print("Shutting down LeKiwi server.")
    finally:
        stop_event.set()
        cam_thread.join()
        arm_thread.join()
        gen72.arm.rm_delete_robot_arm()
        cmd_socket.close()
        video_socket.close()
        context.term()
