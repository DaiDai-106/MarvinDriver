"""
Modbus-RTU 协议封装模块

只负责协议的打包和解析，不包含发送功能
支持：
- 功能码 03: 读保持寄存器
- 功能码 04: 读输入寄存器
- 功能码 06: 写单个寄存器
- 功能码 10: 写多个寄存器
"""

from typing import Optional, List


def calculate_modbus_crc(data: bytes) -> bytes:
    """
    计算Modbus CRC校验码

    参数:
        data: 要计算CRC的数据字节

    返回:
        bytes: CRC校验码(2字节,低字节在前)
    """
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def verify_modbus_crc(data: bytes) -> bool:
    """
    验证Modbus CRC校验码

    参数:
        data: 包含CRC校验码的数据（最后2字节为CRC）

    返回:
        bool: True表示校验通过，False表示校验失败
    """
    if len(data) < 2:
        return False
    recv_crc = data[-2:]
    calc_crc = calculate_modbus_crc(data[:-2])
    return recv_crc == calc_crc


# Modbus 功能码常量
READ_HOLDING_REGISTERS = 0x03  # 读保持寄存器
READ_INPUT_REGISTERS = 0x04  # 读输入寄存器
WRITE_SINGLE_REGISTER = 0x06  # 写单个寄存器
WRITE_MULTIPLE_REGISTERS = 0x10  # 写多个寄存器


def build_read_holding_registers_command(
    device_id: int, register_address: int, register_count: int = 1
) -> bytes:
    """
    构建读保持寄存器命令帧

    参数:
        device_id: Modbus设备ID
        register_address: 起始寄存器地址
        register_count: 读取的寄存器数量，默认1

    返回:
        bytes: 完整的Modbus-RTU命令帧（包含CRC）
    """
    cmd = bytes([device_id])
    cmd += bytes([READ_HOLDING_REGISTERS])
    cmd += bytes([(register_address >> 8) & 0xFF, register_address & 0xFF])
    cmd += bytes([(register_count >> 8) & 0xFF, register_count & 0xFF])
    cmd += calculate_modbus_crc(cmd)
    return cmd


def build_read_input_registers_command(
    device_id: int, register_address: int, register_count: int = 1
) -> bytes:
    """
    构建读输入寄存器命令帧

    参数:
        device_id: Modbus设备ID
        register_address: 起始寄存器地址
        register_count: 读取的寄存器数量，默认1

    返回:
        bytes: 完整的Modbus-RTU命令帧（包含CRC）
    """
    cmd = bytes([device_id])
    cmd += bytes([READ_INPUT_REGISTERS])
    cmd += bytes([(register_address >> 8) & 0xFF, register_address & 0xFF])
    cmd += bytes([(register_count >> 8) & 0xFF, register_count & 0xFF])
    cmd += calculate_modbus_crc(cmd)
    return cmd


def build_write_single_register_command(
    device_id: int, register_address: int, value: int
) -> bytes:
    """
    构建写单个寄存器命令帧

    参数:
        device_id: Modbus设备ID
        register_address: 寄存器地址
        value: 要写入的值

    返回:
        bytes: 完整的Modbus-RTU命令帧（包含CRC）
    """
    cmd = bytes([device_id])
    cmd += bytes([WRITE_SINGLE_REGISTER])
    cmd += bytes([(register_address >> 8) & 0xFF, register_address & 0xFF])
    cmd += bytes([(value >> 8) & 0xFF, value & 0xFF])
    cmd += calculate_modbus_crc(cmd)
    return cmd


def build_write_multiple_registers_command(
    device_id: int, register_address: int, values: List[int]
) -> bytes:
    """
    构建写多个寄存器命令帧

    参数:
        device_id: Modbus设备ID
        register_address: 起始寄存器地址
        values: 要写入的值列表

    返回:
        bytes: 完整的Modbus-RTU命令帧（包含CRC）
    """
    cmd = bytes([device_id])
    cmd += bytes([WRITE_MULTIPLE_REGISTERS])
    cmd += bytes([(register_address >> 8) & 0xFF, register_address & 0xFF])
    register_count = len(values)
    byte_count = register_count * 2
    cmd += bytes([(register_count >> 8) & 0xFF, register_count & 0xFF])
    cmd += bytes([byte_count])
    for value in values:
        cmd += bytes([(value >> 8) & 0xFF, value & 0xFF])
    cmd += calculate_modbus_crc(cmd)
    return cmd


def parse_read_registers_response(
    response: bytes, expected_device_id: int, expected_function_code: int
) -> Optional[List[int]]:
    """
    解析读寄存器响应

    参数:
        response: 响应数据（包含CRC）
        expected_device_id: 期望的设备ID
        expected_function_code: 期望的功能码

    返回:
        List[int]: 读取的寄存器值列表，失败返回None
    """
    if response is None or len(response) < 5:
        return None

    # 验证CRC
    if not verify_modbus_crc(response):
        return None

    # 验证设备ID和功能码
    if response[0] != expected_device_id or response[1] != expected_function_code:
        return None

    # 解析响应：地址码(1) + 功能码(1) + 字节数(1) + 数据(N*2) + CRC(2)
    byte_count = response[2]
    if len(response) < 3 + byte_count + 2:
        return None

    register_count = byte_count // 2
    values = []
    for i in range(register_count):
        idx = 3 + i * 2
        if idx + 1 < len(response) - 2:  # 排除CRC
            value = (response[idx] << 8) | response[idx + 1]
            values.append(value)

    return values if len(values) == register_count else None


def parse_write_single_register_response(
    response: bytes,
    expected_device_id: int,
    expected_register_address: int,
    expected_value: int,
) -> bool:
    """
    解析写单个寄存器响应

    参数:
        response: 响应数据（包含CRC）
        expected_device_id: 期望的设备ID
        expected_register_address: 期望的寄存器地址
        expected_value: 期望的值

    返回:
        bool: True表示成功，False表示失败
    """
    if response is None or len(response) < 6:
        return False

    # 验证CRC
    if not verify_modbus_crc(response):
        return False

    # 验证响应格式：地址码 + 功能码 + 寄存器地址 + 数据 + CRC
    if (
        response[0] == expected_device_id
        and response[1] == WRITE_SINGLE_REGISTER
        and response[2] == (expected_register_address >> 8) & 0xFF
        and response[3] == expected_register_address & 0xFF
        and response[4] == (expected_value >> 8) & 0xFF
        and response[5] == expected_value & 0xFF
    ):
        return True

    return False


def parse_write_multiple_registers_response(
    response: bytes,
    expected_device_id: int,
    expected_register_address: int,
    expected_register_count: int,
) -> bool:
    """
    解析写多个寄存器响应

    参数:
        response: 响应数据（包含CRC）
        expected_device_id: 期望的设备ID
        expected_register_address: 期望的起始寄存器地址
        expected_register_count: 期望的寄存器数量

    返回:
        bool: True表示成功，False表示失败
    """
    if response is None or len(response) < 6:
        return False

    # 验证CRC
    if not verify_modbus_crc(response):
        return False

    # 验证响应格式：地址码 + 功能码 + 起始地址 + 寄存器数量 + CRC
    if (
        response[0] == expected_device_id
        and response[1] == WRITE_MULTIPLE_REGISTERS
        and response[2] == (expected_register_address >> 8) & 0xFF
        and response[3] == expected_register_address & 0xFF
        and response[4] == (expected_register_count >> 8) & 0xFF
        and response[5] == expected_register_count & 0xFF
    ):
        return True

    return False
