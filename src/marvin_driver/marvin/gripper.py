"""
夹爪控制模块

基于 Modbus-RTU 协议实现夹爪控制功能
使用 fx_robot.py 中的发送接口
"""

from .modbus import (
    build_read_holding_registers_command,
    build_write_single_register_command,
    parse_read_registers_response,
    parse_write_single_register_response,
    READ_HOLDING_REGISTERS,
)
from typing import Optional, Callable


class GripperController:
    """
    夹爪控制器类

    基于 Modbus-RTU 协议控制夹爪设备
    """

    # ========== Modbus 寄存器地址定义 ==========
    # 0x01xx - 基础控制寄存器
    REG_INIT = 0x0100  # 初始化夹爪
    REG_FORCE = 0x0101  # 力值
    REG_POSITION = 0x0103  # 位置

    # 0x02xx - 状态反馈寄存器
    REG_INIT_STATE = 0x0200  # 初始化状态反馈
    REG_HOLD_STATE = 0x0201  # 夹持状态反馈
    REG_POSITION_FB = 0x0202  # 位置反馈
    REG_CURRENT = 0x0204  # 电流反馈

    # 0x03xx - 参数配置寄存器
    REG_SAVE_PARA = 0x0300  # 写入保存
    REG_INIT_MODE = 0x0301  # 初始化方向

    def __init__(
        self, send_callback: Callable[[bytes], Optional[bytes]], device_id: int = 1
    ):
        """
        初始化夹爪控制器

        参数:
            send_callback: 发送回调函数，接收命令字节，返回响应字节或None
            device_id: Modbus设备ID，默认1
        """
        self.send_callback = send_callback
        self.device_id = device_id

    def _send_command(self, command: bytes) -> Optional[bytes]:
        """
        通过回调函数发送命令并接收响应

        参数:
            command: Modbus命令帧

        返回:
            bytes: 响应数据，失败返回None
        """
        return self.send_callback(command)

    # ========== 初始化相关方法 (REG_INIT: 0x0100) ==========

    def init(self, is_full: bool = False) -> bool:
        """
        夹爪初始化

        参数:
            is_full: False表示单方向初始化（寻找最大位置或最小位置），
                    True表示进行一次张开闭合初始化
                    默认False

        返回:
            bool: True表示成功，False表示失败
        """
        # 根据is_full选择初始化值：False=0x01（单方向），True=0xA5（张开闭合）
        init_value = 0xA5 if is_full else 0x01

        cmd = build_write_single_register_command(
            self.device_id, self.REG_INIT, init_value
        )
        response = self._send_command(cmd)

        if response is None:
            print("failed:夹爪初始化失败!")
            return False

        result = parse_write_single_register_response(
            response, self.device_id, self.REG_INIT, init_value
        )
        if result:
            print("success:夹爪初始化成功!")
        else:
            print("failed:夹爪初始化失败!")
        return result

    def get_init_state(self) -> int:
        """
        获取初始化状态 (REG_INIT_STATE: 0x0200)

        返回:
            int: 0：未初始化 1：初始化完成 2：初始化中，失败返回-1
        """
        cmd = build_read_holding_registers_command(
            self.device_id, self.REG_INIT_STATE, 1
        )
        response = self._send_command(cmd)

        if response is None:
            print("failed:夹爪初始化状态获取失败")
            return -1

        values = parse_read_registers_response(
            response, self.device_id, READ_HOLDING_REGISTERS
        )
        if values is None or len(values) == 0:
            print("failed:夹爪初始化状态获取失败")
            return -1
        return values[0]

    def set_init_mode(self, value: str) -> bool:
        """
        设置初始化方向 (REG_INIT_MODE: 0x0301)
        初始化结束后停留方向
        已添加写入保存，不要重复操作

        参数:
            value: "open"：张开，"close"：闭合

        返回:
            bool: True表示成功，False表示失败
        """
        if value == "open":
            mode_value = 0x00
        elif value == "close":
            mode_value = 0x01
        else:
            print("failed:夹爪初始化方向设置错误，只能是'open'或'close'!")
            return False

        cmd = build_write_single_register_command(
            self.device_id, self.REG_INIT_MODE, mode_value
        )
        response = self._send_command(cmd)

        if response is None:
            print("failed:夹爪初始化方向设置失败!")
            return False

        result = parse_write_single_register_response(
            response, self.device_id, self.REG_INIT_MODE, mode_value
        )
        if not result:
            print("failed:夹爪初始化方向设置失败!")
            return False

        # 自动保存参数
        return self.save_para()

    # ========== 力值相关方法 (REG_FORCE: 0x0101) ==========

    def set_force(self, value: int) -> bool:
        """
        设置夹爪力值

        参数:
            value: 力值 20%-100%

        返回:
            bool: True表示成功，False表示失败
        """
        if value < 20 or value > 100:
            print("failed:夹爪力值设置错误，范围20-100!")
            return False

        cmd = build_write_single_register_command(self.device_id, self.REG_FORCE, value)
        response = self._send_command(cmd)

        if response is None:
            print("failed:夹爪力值设置失败!")
            return False

        result = parse_write_single_register_response(
            response, self.device_id, self.REG_FORCE, value
        )
        if result:
            print("success:夹爪力值设置成功!")
        else:
            print("failed:夹爪力值设置失败!")
        return result

    def get_force(self) -> int:
        """
        获取力值

        返回:
            int: 力值，失败返回-1
        """
        cmd = build_read_holding_registers_command(self.device_id, self.REG_FORCE, 1)
        response = self._send_command(cmd)

        if response is None:
            print("failed:夹爪力值获取失败")
            return -1

        values = parse_read_registers_response(
            response, self.device_id, READ_HOLDING_REGISTERS
        )
        if values is None or len(values) == 0:
            print("failed:夹爪力值获取失败")
            return -1
        return values[0]

    # ========== 位置相关方法 (REG_POSITION: 0x0103, REG_POSITION_FB: 0x0202) ==========

    def set_target_position(self, value: int) -> bool:
        """
        设置目标位置

        参数:
            value: 目标位置值 0-1000

        返回:
            bool: True表示成功，False表示失败
        """
        if value < 0 or value > 1000:
            print("failed:夹爪位置设置错误，范围0-1000!")
            return False

        cmd = build_write_single_register_command(
            self.device_id, self.REG_POSITION, value
        )
        response = self._send_command(cmd)

        if response is None:
            print("failed:夹爪目标位置设置失败!")
            return False

        result = parse_write_single_register_response(
            response, self.device_id, self.REG_POSITION, value
        )
        if result:
            print("success:夹爪目标位置设置成功!")
        else:
            print("failed:夹爪目标位置设置失败!")
        return result

    def get_target_position(self) -> int:
        """
        获取目标位置

        返回:
            int: 目标位置，失败返回-1
        """
        cmd = build_read_holding_registers_command(self.device_id, self.REG_POSITION, 1)
        response = self._send_command(cmd)

        if response is None:
            print("failed:夹爪目标位置获取失败")
            return -1

        values = parse_read_registers_response(
            response, self.device_id, READ_HOLDING_REGISTERS
        )
        if values is None or len(values) == 0:
            print("failed:夹爪目标位置获取失败")
            return -1
        return values[0]

    def get_current_position(self) -> int:
        """
        获取当前位置 (REG_POSITION_FB: 0x0202)

        返回:
            int: 当前位置，失败返回-1
        """
        cmd = build_read_holding_registers_command(
            self.device_id, self.REG_POSITION_FB, 1
        )
        response = self._send_command(cmd)

        if response is None:
            print("failed:夹爪当前位置获取失败")
            return -1

        values = parse_read_registers_response(
            response, self.device_id, READ_HOLDING_REGISTERS
        )
        if values is None or len(values) == 0:
            print("failed:夹爪当前位置获取失败")
            return -1
        return values[0]

    # ========== 状态相关方法 (REG_HOLD_STATE: 0x0201) ==========

    def get_hold_state(self) -> int:
        """
        获取夹持状态

        返回:
            int: 0：运动中 1：到达位置 2：夹住物体 3：物体掉落，失败返回-1
        """
        cmd = build_read_holding_registers_command(
            self.device_id, self.REG_HOLD_STATE, 1
        )
        response = self._send_command(cmd)

        if response is None:
            print("failed:夹爪夹持状态获取失败")
            return -1

        values = parse_read_registers_response(
            response, self.device_id, READ_HOLDING_REGISTERS
        )
        if values is None or len(values) == 0:
            print("failed:夹爪夹持状态获取失败")
            return -1
        return values[0]

    # ========== 电流相关方法 (REG_CURRENT: 0x0204) ==========

    def get_current(self) -> int:
        """
        获取电流

        返回:
            int: 电流值，失败返回-1
        """
        cmd = build_read_holding_registers_command(self.device_id, self.REG_CURRENT, 1)
        response = self._send_command(cmd)

        if response is None:
            print("failed:夹爪当前电流获取失败")
            return -1

        values = parse_read_registers_response(
            response, self.device_id, READ_HOLDING_REGISTERS
        )
        if values is None or len(values) == 0:
            print("failed:夹爪当前电流获取失败")
            return -1
        return values[0]

    # ========== 参数配置相关方法 (REG_SAVE_PARA: 0x0300) ==========

    def save_para(self) -> bool:
        """
        写入保存参数
        若对夹爪进行过 IO 配置以及 RS485 的参数配置。必须要在此命令下对参数进行FLASH写入保存。
        写入操作会持续 1-2 秒，期间不会响应其他命令，因此建议不要在实时控制中使用此命令

        返回:
            bool: True表示成功，False表示失败
        """
        cmd = build_write_single_register_command(
            self.device_id, self.REG_SAVE_PARA, 0x01
        )
        response = self._send_command(cmd)

        if response is None:
            print("failed:参数保存失败!")
            return False

        result = parse_write_single_register_response(
            response, self.device_id, self.REG_SAVE_PARA, 0x01
        )
        if result:
            print("success:参数保存成功!")
        else:
            print("failed:参数保存失败!")
        return result
