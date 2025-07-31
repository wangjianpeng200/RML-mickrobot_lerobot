# import base64
# import json
# import os
# import sys
# from pathlib import Path

# import cv2
# import numpy as np
# import torch
# import zmq

# from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs
# from lerobot.common.robot_devices.motors.dynamixel import TorqueMode
# from lerobot.common.robot_devices.motors.utils import MotorsBus, make_motors_buses_from_configs
# from lerobot.common.robot_devices.robots.configs import LeKiwiRobotConfig
# # from lerobot.common.robot_devices.robots.feetech_calibration import run_arm_manual_calibration
# from lerobot.common.robot_devices.robots.dynamixel_calibration import run_arm_calibration
# from lerobot.common.robot_devices.robots.utils import get_arm_id
# from lerobot.common.robot_devices.utils import RobotDeviceNotConnectedError

# PYNPUT_AVAILABLE = True
# try:
#     # Only import if there's a valid X server or if we're not on a Pi
#     if ("DISPLAY" not in os.environ) and ("linux" in sys.platform):
#         print("No DISPLAY set. Skipping pynput import.")
#         raise ImportError("pynput blocked intentionally due to no display.")

#     from pynput import keyboard
# except ImportError:
#     keyboard = None
#     PYNPUT_AVAILABLE = False
# except Exception as e:
#     keyboard = None
#     PYNPUT_AVAILABLE = False
#     print(f"Could not import pynput: {e}")


# class MobileManipulator:
#     """
#     MobileManipulator is a class for connecting to and controlling a remote mobile manipulator robot.
#     The robot includes a three omniwheel mobile base and a remote follower arm.
#     The leader arm is connected locally (on the laptop) and its joint positions are recorded and then
#     forwarded to the remote follower arm (after applying a safety clamp).
#     In parallel, keyboard teleoperation is used to generate raw velocity commands for the wheels.
#     """

#     def __init__(self, config: LeKiwiRobotConfig):
#         """
#         移动操作机器人初始化方法
        
#         参数:
#             config: 包含机器人配置的LeKiwiRobotConfig对象
#               主要配置项包括:
#               - ip/port/video_port: 远程机器人的网络连接配置
#               - calibration_dir: 校准文件目录
#               - leader_arms/follower_arms: 本地/远程机械臂配置
#               - teleop_keys: 遥控按键映射
#         """
#         # 基础配置初始化
#         self.robot_type = config.type          # 机器人类型标识
#         self.config = config                   # 保存完整配置对象
#         self.remote_ip = config.ip             # 远程机器人IP地址
#         self.remote_port = config.port         # 命令控制端口
#         self.remote_port_video = config.video_port  # 视频流端口
#         self.calibration_dir = Path(self.config.calibration_dir)  # 校准文件路径
#         self.logs = {}                         # 日志存储字典

#         # 遥控按键配置
#         self.teleop_keys = self.config.teleop_keys  # 从配置获取按键映射
#         self.leader_arms = make_motors_buses_from_configs(self.config.leader_arms)
#         # 初始化摄像头设备
#         # self.cameras = make_cameras_from_configs(self.config.cameras)
#         self.cameras = self.config.cameras 
        
#         # 连接状态标志
#         self.is_connected = False

#         # 状态缓存初始化
#         self.last_frames = {}                   # 最新视频帧缓存
#         self.last_present_speed = {}            # 最新速度状态缓存
#         self.last_remote_arm_state = torch.zeros(6, dtype=torch.float32)  # 远程机械臂状态

#         # 速度档位配置（慢/中/快三档）
#         self.speed_levels = [
#             {"xy": 0.1, "theta": 30},  # 慢速档：线速度0.1m/s，角速度30度/s
#             {"xy": 0.2, "theta": 60},  # 中速档：线速度0.2m/s，角速度60度/s
#             {"xy": 0.3, "theta": 90},  # 快速档：线速度0.3m/s，角速度90度/s
#         ]
#         self.speed_index = 0  # 当前速度档位索引（默认从慢速开始）

#         # ZeroMQ网络通信初始化
#         self.context = None      # ZeroMQ上下文
#         self.cmd_socket = None  # 命令发送套接字
#         self.video_socket = None  # 视频接收套接字

#         # 键盘状态跟踪
#         self.running = True  # 控制主循环的运行标志
#         self.pressed_keys = {  # 记录各方向键的按下状态
#             "forward": False,
#             "backward": False,
#             "left": False,
#             "right": False,
#             "rotate_left": False,
#             "rotate_right": False,
#         }

#         # 初始化键盘监听器（如果可用）
#         if PYNPUT_AVAILABLE:
#             print("pynput可用 - 启用本地键盘监听")
#             self.listener = keyboard.Listener(
#                 on_press=self.on_press,    # 按键按下回调
#                 on_release=self.on_release # 按键释放回调
#             )
#             self.listener.start()
#         else:
#             print("pynput不可用 - 跳过键盘监听")
#             self.listener = None

#     def get_motor_names(self, arms: dict[str, MotorsBus]) -> list:
#         return [f"{arm}_{motor}" for arm, bus in arms.items() for motor in bus.motors]

#     @property
#     def camera_features(self) -> dict:
#         cam_ft = {}
#         for cam_key, cam in self.cameras.items():
#             key = f"observation.images.{cam_key}"
#             cam_ft[key] = {
#                 "shape": (cam.height, cam.width, cam.channels),
#                 "names": ["height", "width", "channels"],
#                 "info": None,
#             }
#         return cam_ft

#     @property
#     #---------------------------------------------------------------------------------------------------机械臂轴数的更改，以及底盘移动只有两个自由度-----------------------------------------
#     def motor_features(self) -> dict:
#         # 定义远程机械臂的6个关节名称（肩部平移/抬升，肘部弯曲，腕部弯曲/旋转，夹具）
#         follower_arm_names = [
#                     "shoulder_pan",    # 肩部水平旋转关节
#                     "shoulder_lift",   # 肩部垂直抬升关节
#                     "elbow_flex",      # 肘部弯曲关节
#                     "wrist_flex",      # 腕部弯曲关节
#                     "wrist_roll",      # 腕部转动关节
#                     "wrist_1",         # 腕部第一轴
#                     "wrist_2",         # 腕部第二轴
#                     "gripper"          # 夹具控制
#                 ]
#         # 定义移动底座的观测维度（毫米坐标和角度）
#         observations = ["x_mm", "y_mm", "theta"]  # x/y轴位移（毫米单位），旋转角度（度）
        
#         # 合并机械臂关节和底座观测的特征名称
#         combined_names = follower_arm_names + observations
        
#         return {
#             # 动作空间定义：包含机械臂目标位置和底座速度指令
#             "action": {
#                 "dtype": "float32",                    # 单精度浮点类型
#                 "shape": (len(combined_names),),       # 9维向量（6关节 + 3底座）
#                 "names": combined_names,               # 特征名称列表
#             },
#             # 状态观测定义：包含机械臂当前位置和底座实时状态
#             "observation.state": {
#                 "dtype": "float32",                    # 单精度浮点类型
#                 "shape": (len(combined_names),),       # 与动作空间维度一致
#                 "names": combined_names,               # 相同特征名称保证数据对齐
#             },
#         }

#     @property
#     def features(self):
#         return {**self.motor_features, **self.camera_features}

#     @property
#     def has_camera(self):
#         return len(self.cameras) > 0

#     @property
#     def num_cameras(self):
#         return len(self.cameras)

#     @property
#     def available_arms(self):

#         available = []
#         # 遍历所有本地机械臂，生成leader类型的ID
#         for name in self.leader_arms:
#             available.append(get_arm_id(name, "leader"))
#         # 遍历所有远程机械臂，生成follower类型的ID
#         # for name in self.follower_arms:
#         #     available.append(get_arm_id(name, "follower"))
#         return available


# #-------------------------------------------------------------------------------------------------------去掉一个键，或者保留作为升降机的数据位-----------------------------------------------------
#     def on_press(self, key):
#         """处理键盘按下事件，更新控制状态和速度档位"""
#         try:
#             # 运动控制按键处理
#             if key.char == self.teleop_keys["forward"]:   # 前进键
#                 self.pressed_keys["forward"] = True
#             elif key.char == self.teleop_keys["backward"]: # 后退键
#                 self.pressed_keys["backward"] = True
#             elif key.char == self.teleop_keys["left"]:    # 左移键
#                 self.pressed_keys["left"] = True
#             elif key.char == self.teleop_keys["right"]:    # 右移键
#                 self.pressed_keys["right"] = True
#             elif key.char == self.teleop_keys["rotate_left"]:  # 左旋转键
#                 self.pressed_keys["rotate_left"] = True
#             elif key.char == self.teleop_keys["rotate_right"]: # 右旋转键
#                 self.pressed_keys["rotate_right"] = True

#             # 退出遥控模式（q键）
#             elif key.char == self.teleop_keys["quit"]:
#                 self.running = False  # 停止主循环
#                 return False          # 结束监听

#             # 速度控制逻辑
#             elif key.char == self.teleop_keys["speed_up"]:    # 加速键
#                 self.speed_index = min(self.speed_index + 1, 2)  # 限制最大档位为2
#                 print(f"Speed index increased to {self.speed_index}")
#             elif key.char == self.teleop_keys["speed_down"]:  # 减速键
#                 self.speed_index = max(self.speed_index - 1, 0)  # 限制最小档位为0
#                 print(f"Speed index decreased to {self.speed_index}")

#         except AttributeError:
#             # 处理没有char属性的特殊按键（如ESC）
#             if key == keyboard.Key.esc:  # ESC键也触发退出
#                 self.running = False
#                 return False

#     def on_release(self, key):
#         """处理键盘按键释放事件，更新控制状态"""
#         try:
#             # 检查按键是否有char属性（普通字符键）
#             if hasattr(key, "char"):
#                 # 根据释放的按键更新对应方向的状态
#                 if key.char == self.teleop_keys["forward"]:    # 前进键释放
#                     self.pressed_keys["forward"] = False
#                 elif key.char == self.teleop_keys["backward"]: # 后退键释放
#                     self.pressed_keys["backward"] = False
#                 elif key.char == self.teleop_keys["left"]:     # 左移键释放
#                     self.pressed_keys["left"] = False
#                 elif key.char == self.teleop_keys["right"]:    # 右移键释放
#                     self.pressed_keys["right"] = False
#                 elif key.char == self.teleop_keys["rotate_left"]:  # 左旋转键释放
#                     self.pressed_keys["rotate_left"] = False
#                 elif key.char == self.teleop_keys["rotate_right"]: # 右旋转键释放
#                     self.pressed_keys["rotate_right"] = False
#         except AttributeError:
#             # 忽略没有char属性的特殊按键（如功能键）
#             pass

#     def connect(self):
#         """建立与远程机器人的连接，包括初始化本地机械臂和网络通信"""
#         # 检查是否存在本地机械臂
#         if not self.leader_arms:
#             raise ValueError("MobileManipulator has no leader arm to connect.")
        
#         # 初始化并校准每个本地机械臂
#         for name in self.leader_arms:
#             print(f"Connecting {name} leader arm.")
#             self.calibrate_leader()
#         # 初始化ZeroMQ上下文和套接字
#         self.context = zmq.Context()
        
#         # 创建命令发送套接字（PUSH模式）
#         self.cmd_socket = self.context.socket(zmq.PUSH)
#         connection_string = f"tcp://{self.remote_ip}:{self.remote_port}"
#         self.cmd_socket.connect(connection_string)
#         self.cmd_socket.setsockopt(zmq.CONFLATE, 1)  # 只保留最新消息
        
#         # 创建视频接收套接字（PULL模式）
#         self.video_socket = self.context.socket(zmq.PULL)
#         video_connection = f"tcp://{self.remote_ip}:{self.remote_port_video}"
#         self.video_socket.connect(video_connection)
#         self.video_socket.setsockopt(zmq.CONFLATE, 1)  # 只保留最新消息
#         print(
#             f"[INFO] Connected to remote robot at {connection_string} and video stream at {video_connection}."
#         )
        
#         self.is_connected = True

#     def load_or_run_calibration_(self, name, arm, arm_type):
#         arm_id = get_arm_id(name, arm_type)
#         arm_calib_path = self.calibration_dir / f"{arm_id}.json"

#         if arm_calib_path.exists():
#             with open(arm_calib_path) as f:
#                 calibration = json.load(f)
#         else:
#             print(f"Missing calibration file '{arm_calib_path}'")
#             calibration = run_arm_calibration(arm, self.robot_type, name, arm_type)
#             print(f"Calibration is done! Saving calibration file '{arm_calib_path}'")
#             arm_calib_path.parent.mkdir(parents=True, exist_ok=True)
#             with open(arm_calib_path, "w") as f:
#                 json.dump(calibration, f)

#         return calibration
#     def calibrate_leader(self):
#         for name, arm in self.leader_arms.items():
#             # Connect the bus
#             arm.connect()

#             # Disable torque on all motors
#             for motor_id in arm.motors:
#                 arm.write("Torque_Enable", TorqueMode.DISABLED.value, motor_id)

#             # Now run calibration
#             calibration = self.load_or_run_calibration_(name, arm, "leader")
#             arm.set_calibration(calibration)

#     def _get_data(self):  

#         frames = {}
#         present_speed = {}
#         remote_arm_state_tensor = torch.zeros(6, dtype=torch.float32)

#         # 设置15ms的轮询时间
#         poller = zmq.Poller()
#         poller.register(self.video_socket, zmq.POLLIN)
#         socks = dict(poller.poll(15))
        
#         # 如果没有新数据到达，返回上次缓存的数据
#         if self.video_socket not in socks or socks[self.video_socket] != zmq.POLLIN:
#             return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)

#         # 获取最新消息，丢弃之前的消息
#         last_msg = None
#         while True:
#             try:
#                 obs_string = self.video_socket.recv_string(zmq.NOBLOCK)
#                 last_msg = obs_string
#             except zmq.Again:
#                 break

#         # 如果没有获取到消息，返回上次缓存的数据
#         if not last_msg:
#             return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)

#         # 解码最后一条消息
#         try:
#             observation = json.loads(last_msg)

#             # 获取图像数据
#             images_dict = observation.get("images", {})
#             # 获取当前速度
#             new_speed = observation.get("present_speed", {})
#             # 获取机械臂状态
#             new_arm_state = observation.get("follower_arm_state", None)

#             # 转换图像数据
#             for cam_name, image_b64 in images_dict.items():
#                 if image_b64:
#                     jpg_data = base64.b64decode(image_b64)
#                     np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
#                     frame_candidate = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#                     if frame_candidate is not None:
#                         frames[cam_name] = frame_candidate

#             # 如果有新的机械臂状态和图像数据，更新缓存
#             if new_arm_state is not None and frames is not None:
#                 self.last_frames = frames
#                 remote_arm_state_tensor = torch.tensor(new_arm_state, dtype=torch.float32)
#                 self.last_remote_arm_state = remote_arm_state_tensor
#                 present_speed = new_speed
#                 self.last_present_speed = new_speed
#             else:
#                 # 否则使用上次缓存的数据
#                 frames = self.last_frames
#                 remote_arm_state_tensor = self.last_remote_arm_state
#                 present_speed = self.last_present_speed

#         except Exception as e:
#             # 如果解码失败，返回上次缓存的数据
#             print(f"[DEBUG] Error decoding video message: {e}")
#             return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)

#         return frames, present_speed, remote_arm_state_tensor


# #-------------------------------------------------------------------------------------------------------控制最主要修改的地方-----------------------------------------------------
#     def teleop_step(
#         self, record_data: bool = False
#     ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:

#         if not self.is_connected:
#             raise RobotDeviceNotConnectedError("MobileManipulator is not connected. Run `connect()` first.")

#         speed_setting = self.speed_levels[self.speed_index] #通过索引获取当前速度档位
#         xy_speed = speed_setting["xy"]  # 线速度，例如0.1, 0.25, 或 0.4 m/s
#         theta_speed = speed_setting["theta"]  # 角速度，例如30, 60, 或 90 度/s
#         # 读取本地机械臂的当前位置，机械臂的控制
#         arm_positions = []
#         for name in self.leader_arms:
#             pos = self.leader_arms[name].read("Present_Position")
#             pos_tensor = torch.from_numpy(pos).float()
#             arm_positions.extend(pos_tensor.tolist())
#         # 初始化速度命令
#         y_cmd = 0.0  # 前进/后退速度 (m/s)
#         x_cmd = 0.0  # 横向移动速度 (m/s)
#         theta_cmd = 0.0  # 旋转速度 (度/s)

#         # 根据按键状态更新速度命令
#         if self.pressed_keys["forward"]:
#             y_cmd += xy_speed
#         if self.pressed_keys["backward"]:
#             y_cmd -= xy_speed
#         if self.pressed_keys["left"]:
#             x_cmd += xy_speed
#         if self.pressed_keys["right"]:
#             x_cmd -= xy_speed
#         if self.pressed_keys["rotate_left"]:
#             theta_cmd += theta_speed
#         if self.pressed_keys["rotate_right"]:
#             theta_cmd -= theta_speed

#         wheel_commands={"left_wheel": x_cmd*10, "back_wheel": y_cmd*10, "right_wheel": theta_cmd}
#         # 构建并发送控制消息
#         message = {"raw_velocity": wheel_commands, "arm_positions": arm_positions}
#         # print(arm_positions)
#         self.cmd_socket.send_string(json.dumps(message))

#         # 如果不记录数据，直接返回
#         if not record_data:
#             return

#         # 获取当前观测数据
#         obs_dict = self.capture_observation()
#         # 将机械臂位置转换为张量
#         arm_state_tensor = torch.tensor(arm_positions, dtype=torch.float32)
#         # 将速度单位转换为mm/s
#         wheel_velocity_mm = (x_cmd*10, y_cmd*10, theta_cmd)
#         # 创建速度张量
#         wheel_tensor = torch.tensor(wheel_velocity_mm, dtype=torch.float32)
#         # 合并机械臂和速度张量
#         action_tensor = torch.cat([arm_state_tensor, wheel_tensor])
#         action_dict = {"action": action_tensor}

#         # 返回观测和动作数据
#         return obs_dict, action_dict

#     def capture_observation(self) -> dict:

#         # 检查是否已连接
#         if not self.is_connected:
#             raise RobotDeviceNotConnectedError("Not connected. Run `connect()` first.")

#         frames, present_speed, remote_arm_state_tensor = self._get_data()

#         body_state_mm = (present_speed["left_wheel"], present_speed["back_wheel"], present_speed["right_wheel"])
#         # body_state_mm = (body_state[0] * 1000.0, body_state[1] * 1000.0, body_state[2])
#         wheel_state_tensor = torch.tensor(body_state_mm, dtype=torch.float32)
#         combined_state_tensor = torch.cat((remote_arm_state_tensor, wheel_state_tensor), dim=0)

#         # 初始化观测字典
#         obs_dict = {"observation.state": combined_state_tensor}
#         # 遍历每个配置的摄像头
#         for cam_name, cam in self.cameras.items():
#             # 获取当前摄像头帧
#             frame = frames.get(cam_name, None)
#             if frame is None:
#                 # 如果没有获取到帧，创建黑色图像
#                 frame = np.zeros((cam.height, cam.width, cam.channels), dtype=np.uint8)
#             # 将图像转换为张量并存入观测字典
#             obs_dict[f"observation.images.{cam_name}"] = torch.from_numpy(frame)
#         return obs_dict

#     def send_action(self, action: torch.Tensor) -> torch.Tensor:
#         if not self.is_connected:
#             raise RobotDeviceNotConnectedError("Not connected. Run `connect()` first.")

#         if action.numel() < 11:
#             # Pad with zeros if there are not enough elements.
#             padded = torch.zeros(11, dtype=action.dtype)
#             padded[: action.numel()] = action
#             action = padded

#         # Extract arm and base actions.
#         arm_actions = action[:8].flatten()
#         base_actions = action[8:].flatten()

#         x_cmd_mm = base_actions[0].item()  # mm/s
#         y_cmd_mm = base_actions[1].item()  # mm/s
#         theta_cmd = base_actions[2].item()  # deg/s

#         # Convert mm/s to m/s for the kinematics calculations.
#         x_cmd = x_cmd_mm
#         y_cmd = y_cmd_mm  

#         # Compute wheel commands from body commands.
#         wheel_commands = {"left_wheel": x_cmd, "back_wheel":y_cmd, "right_wheel": theta_cmd}

#         arm_positions_list = arm_actions.tolist()

#         message = {"raw_velocity": wheel_commands, "arm_positions": arm_positions_list}
#         self.cmd_socket.send_string(json.dumps(message))
#         return action

#     def print_logs(self):
#         pass

#     def disconnect(self):
#         if not self.is_connected:
#             raise RobotDeviceNotConnectedError("Not connected.")
#         if self.cmd_socket:
#             stop_cmd = {
#                 "raw_velocity": {"left_wheel": 0, "back_wheel": 0, "right_wheel": 0},
#                 "arm_positions": {},
#             }
#             self.cmd_socket.send_string(json.dumps(stop_cmd))
#             self.cmd_socket.close()
#         if self.video_socket:
#             self.video_socket.close()
#         if self.context:
#             self.context.term()
#         if PYNPUT_AVAILABLE:
#             self.listener.stop()
#         self.is_connected = False
#         print("[INFO] Disconnected from remote robot.")

#     def __del__(self):
#         if getattr(self, "is_connected", False):
#             self.disconnect()
#         if PYNPUT_AVAILABLE:
#             self.listener.stop()

#     @staticmethod
#     def degps_to_raw(degps: float) -> int:
#         steps_per_deg = 4096.0 / 360.0
#         speed_in_steps = abs(degps) * steps_per_deg
#         speed_int = int(round(speed_in_steps))
#         if speed_int > 0x7FFF:
#             speed_int = 0x7FFF
#         if degps < 0:
#             return speed_int | 0x8000
#         else:
#             return speed_int & 0x7FFF

#     @staticmethod
#     def raw_to_degps(raw_speed: int) -> float:
#         steps_per_deg = 4096.0 / 360.0
#         magnitude = raw_speed & 0x7FFF
#         degps = magnitude / steps_per_deg
#         if raw_speed & 0x8000:
#             degps = -degps
#         return degps

#     def body_to_wheel_raw(
#         self,
#         x_cmd: float,
#         y_cmd: float,
#         theta_cmd: float,
#         wheel_radius: float = 0.05,
#         base_radius: float = 0.125,
#         max_raw: int = 3000,
#     ) -> dict:
#         """
#         将身体坐标系的速度命令转换为轮子的原始控制命令
        
#         参数:
#             x_cmd: x轴方向线速度 (m/s)
#             y_cmd: y轴方向线速度 (m/s)
#             theta_cmd: 旋转角速度 (度/s)
#             wheel_radius: 轮子半径 (米)
#             base_radius: 旋转中心到轮子的距离 (米)
#             max_raw: 每个轮子的最大原始命令值 (ticks)
            
#         返回:
#             包含轮子原始命令的字典:
#                 {"left_wheel": 值, "back_wheel": 值, "right_wheel": 值}
#         """
#         # 将旋转速度从度/秒转换为弧度/秒
#         theta_rad = theta_cmd * (np.pi / 180.0)
#         # 创建身体速度向量 [x, y, theta_rad]
#         velocity_vector = np.array([x_cmd, y_cmd, theta_rad])

#         # 定义轮子安装角度（从y轴顺时针定义）
#         angles = np.radians(np.array([300, 180, 60]))
#         # 构建运动学矩阵：每行将身体速度映射到轮子的线速度
#         # 第三列（base_radius）考虑了旋转的影响
#         m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

#         # 计算每个轮子的线速度（m/s）然后转换为角速度（rad/s）
#         wheel_linear_speeds = m.dot(velocity_vector)
#         wheel_angular_speeds = wheel_linear_speeds / wheel_radius

#         # 将轮子角速度从弧度/秒转换为度/秒
#         wheel_degps = wheel_angular_speeds * (180.0 / np.pi)

#         # 速度缩放处理
#         steps_per_deg = 4096.0 / 360.0
#         raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
#         max_raw_computed = max(raw_floats)
#         if max_raw_computed > max_raw:
#             # 如果计算值超过最大值，按比例缩放
#             scale = max_raw / max_raw_computed
#             wheel_degps = wheel_degps * scale

#         # 将每个轮子的角速度（度/秒）转换为原始整数值
#         wheel_raw = [MobileManipulator.degps_to_raw(deg) for deg in wheel_degps]

#         return {"left_wheel": wheel_raw[0], "back_wheel": wheel_raw[1], "right_wheel": wheel_raw[2]}

#     def wheel_raw_to_body(
#         self, wheel_raw: dict, wheel_radius: float = 0.05, base_radius: float = 0.125
#     ) -> tuple:
#         """
#         将轮子的原始控制命令转换回身体坐标系的速度
        
#         参数:
#             wheel_raw: 包含轮子原始命令的字典，键为"left_wheel", "back_wheel", "right_wheel"
#             wheel_radius: 轮子半径 (米)
#             base_radius: 机器人中心到轮子的距离 (米)
            
#         返回:
#             包含身体坐标系速度的元组 (x_cmd, y_cmd, theta_cmd):
#                 x_cmd: x轴方向线速度 (m/s)
#                 y_cmd: y轴方向线速度 (m/s)
#                 theta_cmd: 旋转角速度 (度/s)
#         """
#         # 按顺序提取原始命令值
#         raw_list = [
#             int(wheel_raw.get("left_wheel", 0)),
#             int(wheel_raw.get("back_wheel", 0)),
#             int(wheel_raw.get("right_wheel", 0)),
#         ]

#         # 将每个原始命令转换为角速度（度/秒）
#         wheel_degps = np.array([MobileManipulator.raw_to_degps(r) for r in raw_list])
#         # 将角速度从度/秒转换为弧度/秒
#         wheel_radps = wheel_degps * (np.pi / 180.0)
#         # 根据轮子半径计算每个轮子的线速度（m/s）
#         wheel_linear_speeds = wheel_radps * wheel_radius

#         # 定义轮子安装角度（从y轴顺时针定义）
#         angles = np.radians(np.array([300, 180, 60]))
#         # 构建运动学矩阵
#         m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

#         # 求解逆运动学：身体速度 = M⁻¹ · 轮子线速度
#         m_inv = np.linalg.inv(m)
#         velocity_vector = m_inv.dot(wheel_linear_speeds)
#         # 提取x,y速度和旋转速度（弧度/秒）
#         x_cmd, y_cmd, theta_rad = velocity_vector
#         # 将旋转速度转换为度/秒
#         theta_cmd = theta_rad * (180.0 / np.pi)
#         return (x_cmd, y_cmd, theta_cmd)


# class LeKiwi:
#     def __init__(self, motor_bus):
#         """
#         Initializes the LeKiwi with Feetech motors bus.
#         """
#         self.motor_bus = motor_bus
#         self.motor_ids = ["left_wheel", "back_wheel", "right_wheel"]

#         # Initialize motors in velocity mode.
#         self.motor_bus.write("Lock", 0)
#         self.motor_bus.write("Mode", [1, 1, 1], self.motor_ids)
#         self.motor_bus.write("Lock", 1)
#         print("Motors set to velocity mode.")

#     def read_velocity(self):
#         """
#         Reads the raw speeds for all wheels. Returns a dictionary with motor names:
#         """
#         raw_speeds = self.motor_bus.read("Present_Speed", self.motor_ids)
#         return {
#             "left_wheel": int(raw_speeds[0]),
#             "back_wheel": int(raw_speeds[1]),
#             "right_wheel": int(raw_speeds[2]),
#         }

#     def set_velocity(self, command_speeds):
#         """
#         Sends raw velocity commands (16-bit encoded values) directly to the motor bus.
#         The order of speeds must correspond to self.motor_ids.
#         """
#         self.motor_bus.write("Goal_Speed", command_speeds, self.motor_ids)

#     def stop(self):
#         """Stops the robot by setting all motor speeds to zero."""
#         self.motor_bus.write("Goal_Speed", [0, 0, 0], self.motor_ids)
#         print("Motors stopped.")



import base64
import json
import os
import sys
from pathlib import Path
import cv2
import numpy as np
import torch
import zmq
from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs
from lerobot.common.robot_devices.motors.dynamixel import TorqueMode
from lerobot.common.robot_devices.motors.utils import MotorsBus, make_motors_buses_from_configs
from lerobot.common.robot_devices.robots.configs import LeKiwiRobotConfig
from lerobot.common.robot_devices.robots.dynamixel_calibration import run_arm_calibration
from lerobot.common.robot_devices.robots.utils import get_arm_id
from lerobot.common.robot_devices.utils import RobotDeviceNotConnectedError

PYNPUT_AVAILABLE = True
try:
    if ("DISPLAY" not in os.environ) and ("linux" in sys.platform):
        raise ImportError("pynput blocked intentionally due to no display.")
    from pynput import keyboard
except Exception:
    keyboard = None
    PYNPUT_AVAILABLE = False

class MobileManipulator:
    def __init__(self, config: LeKiwiRobotConfig):
        self.robot_type = config.type
        self.config = config
        self.remote_ip = config.ip
        self.remote_port = config.port
        self.remote_port_video = config.video_port
        self.calibration_dir = Path(self.config.calibration_dir)
        self.logs = {}
        self.teleop_keys = self.config.teleop_keys
        self.leader_arms = make_motors_buses_from_configs(self.config.leader_arms)
        self.cameras = self.config.cameras 
        self.is_connected = False
        self.last_frames = {}
        self.last_present_speed = {}
        self.last_remote_arm_state = torch.zeros(6, dtype=torch.float32)
        self.speed_levels = [
            {"xy": 0.1, "theta": 30},
            {"xy": 0.2, "theta": 60},
            {"xy": 0.3, "theta": 90},
        ]
        self.speed_index = 0
        self.context = None
        self.cmd_socket = None
        self.video_socket = None
        self.running = True
        self.pressed_keys = {
            "forward": False,
            "backward": False,
            "left": False,
            "right": False,
            "rotate_left": False,
            "rotate_right": False,
        }
        if PYNPUT_AVAILABLE:
            self.listener = keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release
            )
            self.listener.start()
        else:
            self.listener = None

    def get_motor_names(self, arms: dict[str, MotorsBus]) -> list:
        return [f"{arm}_{motor}" for arm, bus in arms.items() for motor in bus.motors]

    @property
    def camera_features(self) -> dict:
        cam_ft = {}
        for cam_key, cam in self.cameras.items():
            key = f"observation.images.{cam_key}"
            cam_ft[key] = {
                "shape": (cam.height, cam.width, cam.channels),
                "names": ["height", "width", "channels"],
                "info": None,
            }
        return cam_ft

    @property
    def motor_features(self) -> dict:
        follower_arm_names = [
            "shoulder_pan", "shoulder_lift", "elbow_flex",
            "wrist_flex", "wrist_roll", "wrist_1", "wrist_2", "gripper"
        ]
        observations = ["x_mm", "y_mm", "theta"]
        combined_names = follower_arm_names + observations
        return {
            "action": {
                "dtype": "float32",
                "shape": (len(combined_names),),
                "names": combined_names,
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (len(combined_names),),
                "names": combined_names,
            },
        }

    @property
    def features(self):
        return {**self.motor_features, **self.camera_features}

    @property
    def has_camera(self):
        return len(self.cameras) > 0

    @property
    def num_cameras(self):
        return len(self.cameras)

    @property
    def available_arms(self):
        available = []
        for name in self.leader_arms:
            available.append(get_arm_id(name, "leader"))
        return available

    def on_press(self, key):
        try:
            if key.char == self.teleop_keys["forward"]:
                self.pressed_keys["forward"] = True
            elif key.char == self.teleop_keys["backward"]:
                self.pressed_keys["backward"] = True
            elif key.char == self.teleop_keys["left"]:
                self.pressed_keys["left"] = True
            elif key.char == self.teleop_keys["right"]:
                self.pressed_keys["right"] = True
            elif key.char == self.teleop_keys["rotate_left"]:
                self.pressed_keys["rotate_left"] = True
            elif key.char == self.teleop_keys["rotate_right"]:
                self.pressed_keys["rotate_right"] = True
            elif key.char == self.teleop_keys["quit"]:
                self.running = False
                return False
            elif key.char == self.teleop_keys["speed_up"]:
                self.speed_index = min(self.speed_index + 1, 2)
            elif key.char == self.teleop_keys["speed_down"]:
                self.speed_index = max(self.speed_index - 1, 0)
        except AttributeError:
            if key == keyboard.Key.esc:
                self.running = False
                return False

    def on_release(self, key):
        try:
            if hasattr(key, "char"):
                if key.char == self.teleop_keys["forward"]:
                    self.pressed_keys["forward"] = False
                elif key.char == self.teleop_keys["backward"]:
                    self.pressed_keys["backward"] = False
                elif key.char == self.teleop_keys["left"]:
                    self.pressed_keys["left"] = False
                elif key.char == self.teleop_keys["right"]:
                    self.pressed_keys["right"] = False
                elif key.char == self.teleop_keys["rotate_left"]:
                    self.pressed_keys["rotate_left"] = False
                elif key.char == self.teleop_keys["rotate_right"]:
                    self.pressed_keys["rotate_right"] = False
        except AttributeError:
            pass

    def connect(self):
        if not self.leader_arms:
            raise ValueError("MobileManipulator has no leader arm to connect.")
        for name in self.leader_arms:
            print(f"Connecting {name} leader arm.")
            self.calibrate_leader()
        self.context = zmq.Context()
        self.cmd_socket = self.context.socket(zmq.PUSH)
        connection_string = f"tcp://{self.remote_ip}:{self.remote_port}"
        self.cmd_socket.connect(connection_string)
        self.cmd_socket.setsockopt(zmq.CONFLATE, 1)
        self.video_socket = self.context.socket(zmq.PULL)
        video_connection = f"tcp://{self.remote_ip}:{self.remote_port_video}"
        self.video_socket.connect(video_connection)
        self.video_socket.setsockopt(zmq.CONFLATE, 1)
        print(f"[INFO] Connected to remote robot at {connection_string} and video stream at {video_connection}.")
        self.is_connected = True

    def load_or_run_calibration_(self, name, arm, arm_type):
        arm_id = get_arm_id(name, arm_type)
        arm_calib_path = self.calibration_dir / f"{arm_id}.json"
        if arm_calib_path.exists():
            with open(arm_calib_path) as f:
                calibration = json.load(f)
        else:
            print(f"Missing calibration file '{arm_calib_path}'")
            calibration = run_arm_calibration(arm, self.robot_type, name, arm_type)
            print(f"Calibration is done! Saving calibration file '{arm_calib_path}'")
            arm_calib_path.parent.mkdir(parents=True, exist_ok=True)
            with open(arm_calib_path, "w") as f:
                json.dump(calibration, f)
        return calibration

    def calibrate_leader(self):
        for name, arm in self.leader_arms.items():
            arm.connect()
            for motor_id in arm.motors:
                arm.write("Torque_Enable", TorqueMode.DISABLED.value, motor_id)
            calibration = self.load_or_run_calibration_(name, arm, "leader")
            arm.set_calibration(calibration)

    def _get_data(self):  
        frames = {}
        present_speed = {}
        remote_arm_state_tensor = torch.zeros(6, dtype=torch.float32)
        poller = zmq.Poller()
        poller.register(self.video_socket, zmq.POLLIN)
        socks = dict(poller.poll(15))
        if self.video_socket not in socks or socks[self.video_socket] != zmq.POLLIN:
            return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)
        last_msg = None
        while True:
            try:
                obs_string = self.video_socket.recv_string(zmq.NOBLOCK)
                last_msg = obs_string
            except zmq.Again:
                break
        if not last_msg:
            return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)
        try:
            observation = json.loads(last_msg)
            images_dict = observation.get("images", {})
            new_speed = observation.get("present_speed", {})
            new_arm_state = observation.get("follower_arm_state", None)
            for cam_name, image_b64 in images_dict.items():
                if image_b64:
                    jpg_data = base64.b64decode(image_b64)
                    np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
                    frame_candidate = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if frame_candidate is not None:
                        frames[cam_name] = frame_candidate
            if new_arm_state is not None and frames is not None:
                self.last_frames = frames
                remote_arm_state_tensor = torch.tensor(new_arm_state, dtype=torch.float32)
                self.last_remote_arm_state = remote_arm_state_tensor
                present_speed = new_speed
                self.last_present_speed = new_speed
            else:
                frames = self.last_frames
                remote_arm_state_tensor = self.last_remote_arm_state
                present_speed = self.last_present_speed
        except Exception as e:
            print(f"[DEBUG] Error decoding video message: {e}")
            return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)
        return frames, present_speed, remote_arm_state_tensor

    def teleop_step(self, record_data: bool = False) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("MobileManipulator is not connected. Run `connect()` first.")
        speed_setting = self.speed_levels[self.speed_index]
        xy_speed = speed_setting["xy"]
        theta_speed = speed_setting["theta"]
        arm_positions = []
        for name in self.leader_arms:
            pos = self.leader_arms[name].read("Present_Position")
            pos_tensor = torch.from_numpy(pos).float()
            arm_positions.extend(pos_tensor.tolist())
        y_cmd = 0.0
        x_cmd = 0.0
        theta_cmd = 0.0
        if self.pressed_keys["forward"]:
            y_cmd += xy_speed
        if self.pressed_keys["backward"]:
            y_cmd -= xy_speed
        if self.pressed_keys["left"]:
            x_cmd += xy_speed
        if self.pressed_keys["right"]:
            x_cmd -= xy_speed
        if self.pressed_keys["rotate_left"]:
            theta_cmd += theta_speed
        if self.pressed_keys["rotate_right"]:
            theta_cmd -= theta_speed
        wheel_commands={"left_wheel": x_cmd*10, "back_wheel": y_cmd*10, "right_wheel": theta_cmd}
        message = {"raw_velocity": wheel_commands, "arm_positions": arm_positions}
        self.cmd_socket.send_string(json.dumps(message))
        if not record_data:
            return
        obs_dict = self.capture_observation()
        arm_state_tensor = torch.tensor(arm_positions, dtype=torch.float32)
        wheel_velocity_mm = (x_cmd*10, y_cmd*10, theta_cmd)
        wheel_tensor = torch.tensor(wheel_velocity_mm, dtype=torch.float32)
        action_tensor = torch.cat([arm_state_tensor, wheel_tensor])
        action_dict = {"action": action_tensor}
        return obs_dict, action_dict

    def capture_observation(self) -> dict:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("Not connected. Run `connect()` first.")
        frames, present_speed, remote_arm_state_tensor = self._get_data()
        body_state_mm = (present_speed["left_wheel"], present_speed["back_wheel"], present_speed["right_wheel"])
        wheel_state_tensor = torch.tensor(body_state_mm, dtype=torch.float32)
        combined_state_tensor = torch.cat((remote_arm_state_tensor, wheel_state_tensor), dim=0)
        obs_dict = {"observation.state": combined_state_tensor}
        for cam_name, cam in self.cameras.items():
            frame = frames.get(cam_name, None)
            if frame is None:
                frame = np.zeros((cam.height, cam.width, cam.channels), dtype=np.uint8)
            obs_dict[f"observation.images.{cam_name}"] = torch.from_numpy(frame)
        return obs_dict

    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("Not connected. Run `connect()` first.")
        if action.numel() < 11:
            padded = torch.zeros(11, dtype=action.dtype)
            padded[: action.numel()] = action
            action = padded
        arm_actions = action[:8].flatten()
        base_actions = action[8:].flatten()
        x_cmd_mm = base_actions[0].item()
        y_cmd_mm = base_actions[1].item()
        theta_cmd = base_actions[2].item()
        x_cmd = x_cmd_mm
        y_cmd = y_cmd_mm  
        wheel_commands = {"left_wheel": x_cmd, "back_wheel":y_cmd, "right_wheel": theta_cmd}
        arm_positions_list = arm_actions.tolist()
        message = {"raw_velocity": wheel_commands, "arm_positions": arm_positions_list}
        self.cmd_socket.send_string(json.dumps(message))
        return action

    def print_logs(self):
        pass

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("Not connected.")
        if self.cmd_socket:
            stop_cmd = {
                "raw_velocity": {"left_wheel": 0, "back_wheel": 0, "right_wheel": 0},
                "arm_positions": {},
            }
            self.cmd_socket.send_string(json.dumps(stop_cmd))
            self.cmd_socket.close()
        if self.video_socket:
            self.video_socket.close()
        if self.context:
            self.context.term()
        if PYNPUT_AVAILABLE:
            self.listener.stop()
        self.is_connected = False
        print("[INFO] Disconnected from remote robot.")

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()
        if PYNPUT_AVAILABLE:
            self.listener.stop()





