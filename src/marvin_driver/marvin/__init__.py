# -*- coding: utf-8 -*-
"""
MARVIN Robot SDK
机器人控制SDK核心模块
"""

from .robot import Marvin_Robot
from .kinematics import Marvin_Kine
from .structure import DCSS
from .gripper import GripperController
from .modbus import (
    calculate_modbus_crc,
    verify_modbus_crc,
    build_read_holding_registers_command,
    build_write_single_register_command,
    parse_read_registers_response,
    parse_write_single_register_response,
    READ_HOLDING_REGISTERS,
    WRITE_SINGLE_REGISTER,
    WRITE_MULTIPLE_REGISTERS,
)

__all__ = [
    "Marvin_Robot",
    "Marvin_Kine",
    "DCSS",
    "GripperController",
    "calculate_modbus_crc",
    "verify_modbus_crc",
    "build_read_holding_registers_command",
    "build_write_single_register_command",
    "parse_read_registers_response",
    "parse_write_single_register_response",
    "READ_HOLDING_REGISTERS",
    "WRITE_SINGLE_REGISTER",
    "WRITE_MULTIPLE_REGISTERS",
]

__version__ = "1.0.0"
