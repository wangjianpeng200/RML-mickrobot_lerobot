# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Logic to calibrate a robot arm built with dynamixel motors"""
# TODO(rcadene, aliberts): move this logic into the robot code when refactoring

import numpy as np

from lerobot.common.robot_devices.motors.dynamixel import (
    CalibrationMode,
    TorqueMode,
    convert_degrees_to_steps,
)
from lerobot.common.robot_devices.motors.utils import MotorsBus

URL_TEMPLATE = (
    "https://raw.githubusercontent.com/huggingface/lerobot/main/media/{robot}/{arm}_{position}.webp"
)

# The following positions are provided in nominal degree range ]-180, +180[
# For more info on these constants, see comments in the code where they get used.
ZERO_POSITION_DEGREE = 0
ROTATED_POSITION_DEGREE = 90


def assert_drive_mode(drive_mode):
    # `drive_mode` is in [0,1] with 0 means original rotation direction for the motor, and 1 means inverted.
    if not np.all(np.isin(drive_mode, [0, 1])):
        raise ValueError(f"`drive_mode` contains values other than 0 or 1: ({drive_mode})")


def apply_drive_mode(position, drive_mode):
    assert_drive_mode(drive_mode)
    # Convert `drive_mode` from [0, 1] with 0 indicates original rotation direction and 1 inverted,
    # to [-1, 1] with 1 indicates original rotation direction and -1 inverted.
    signed_drive_mode = -(drive_mode * 2 - 1)
    position *= signed_drive_mode
    return position


def compute_nearest_rounded_position(position, models):
    delta_turn = convert_degrees_to_steps(ROTATED_POSITION_DEGREE, models)
    nearest_pos = np.round(position.astype(float) / delta_turn) * delta_turn
    return nearest_pos.astype(position.dtype)


def run_arm_calibration(arm: MotorsBus, robot_type: str, arm_name: str, arm_type: str):
    """运行机械臂校准
    
    该方法用于确保在不同机器人上训练的神经网络可以通用。通过计算每个电机的homing offset和drive mode，
    使得在不同机器人上设置相同的目标位置时，机械臂会移动到相同的位置。
    
    参数:
        arm: 要校准的机械臂对象
        robot_type: 机器人类型（如"koch"）
        arm_name: 机械臂名称（如"left"）
        arm_type: 机械臂类型（"leader"或"follower"）
        
    返回:
        包含校准数据的字典
    """
    # 检查所有电机的扭矩是否已禁用
    if (arm.read("Torque_Enable") != TorqueMode.DISABLED.value).any():
        raise ValueError("To run calibration, the torque must be disabled on all motors.")

    # 打印校准开始信息
    print(f"\nRunning calibration of {robot_type} {arm_name} {arm_type}...")

    # 提示用户将机械臂移动到零位
    print("\nMove arm to zero position")
    print("See: " + URL_TEMPLATE.format(robot=robot_type, arm=arm_type, position="zero"))
    input("Press Enter to continue...")

    # 计算零位目标位置（将角度转换为步数）
    zero_target_pos = convert_degrees_to_steps(ZERO_POSITION_DEGREE, arm.motor_models)

    # 读取当前零位位置并计算homing offset
    zero_pos = arm.read("Present_Position")
    zero_nearest_pos = compute_nearest_rounded_position(zero_pos, arm.motor_models)
    homing_offset = zero_target_pos - zero_nearest_pos

    # 提示用户将机械臂旋转90度
    print("\nMove arm to rotated target position")
    print("See: " + URL_TEMPLATE.format(robot=robot_type, arm=arm_type, position="zero"))
    input("Press Enter to continue...")

    # 计算旋转90度后的目标位置
    rotated_target_pos = convert_degrees_to_steps(ROTATED_POSITION_DEGREE, arm.motor_models)

    # 读取旋转后的位置并计算drive mode
    rotated_pos = arm.read("Present_Position")
    drive_mode = (rotated_pos < zero_pos).astype(np.int32)

    # 根据drive mode重新计算homing offset
    rotated_drived_pos = apply_drive_mode(rotated_pos, drive_mode)
    rotated_nearest_pos = compute_nearest_rounded_position(rotated_drived_pos, arm.motor_models)
    homing_offset = rotated_target_pos - rotated_nearest_pos

    # 提示用户将机械臂移动到休息位置
    print("\nMove arm to rest position")
    print("See: " + URL_TEMPLATE.format(robot=robot_type, arm=arm_type, position="rest"))
    input("Press Enter to continue...")
    print()

    # 设置校准模式（旋转关节使用角度，夹具使用线性）
    calib_mode = [CalibrationMode.DEGREE.name] * len(arm.motor_names)

    # 对于Aloha机器人的夹具，使用线性校准模式
    if robot_type in ["aloha"] and "gripper" in arm.motor_names:
        calib_idx = arm.motor_names.index("gripper")
        calib_mode[calib_idx] = CalibrationMode.LINEAR.name

    # 返回包含所有校准数据的字典
    calib_data = {
        "homing_offset": homing_offset.tolist(),
        "drive_mode": drive_mode.tolist(),
        "start_pos": zero_pos.tolist(),
        "end_pos": rotated_pos.tolist(),
        "calib_mode": calib_mode,
        "motor_names": arm.motor_names,
    }
    return calib_data
