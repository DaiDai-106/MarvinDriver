import logging
import threading
import time
import numpy as np
import os
import glob
import re
from pathlib import Path
from pydantic import BaseModel
from Chin.core.HttpServerUtil import DummyServer
from marvin_driver.marvin import Marvin_Robot, Marvin_Kine
from typing import Optional, List
from fastapi.responses import JSONResponse
from marvin_driver.marvin.structure import DCSS

# 获取日志器（日志配置已在main.py中完成）
logger = logging.getLogger(__name__)

_MOVL_LINE_PATTERN = re.compile(
        r'X\s+([-+]?\d+\.?\d*)\$Y\s+([-+]?\d+\.?\d*)\$Z\s+([-+]?\d+\.?\d*)\$A\s+([-+]?\d+\.?\d*)\$B\s+([-+]?\d+\.?\d*)\$C\s+([-+]?\d+\.?\d*)\$U\s+([-+]?\d+\.?\d*)'
    )

"""一些序列化的参数"""
class ToolParams( BaseModel ):
    robot_id: str
    kine_params: list[float]
    dynamic_params: list[float]

class DynamicParams( BaseModel ):
    robot_id: str
    k: list[float]
    d: list[float]

class VelAccParams( BaseModel ):
    robot_id: str
    vel: float
    acc: float

class JointTrajectoryParams( BaseModel ):
    robot_id: str
    joints: list[float] 
    vel_ratio: float
    acc_ratio: float

class CartTrajectoryParams( BaseModel ):
    robot_id: str
    start_joints: list
    end_joints: list
    vel: float
    save_path: str

class CartMoveParams( BaseModel ):
    robot_id: str
    path: list
    vel: float
    acc: float
    prep: bool # 是否提前设置位置跟随模式

class IkRequest( BaseModel ):
    robot_id: str
    pose: list
    ref_joints: list

"""
Marvin双臂驱动服务, 以及夹抓的服务
"""
class MarvinServer( object ):
    def __init__( self ):
        logger.info("初始化MarvinServer...")
        self.server = DummyServer()
        self.server.register_service("DummyMarvinServer", "A service to test marvin robot...", "1.0.2", {
            "ping": {"method": "GET", "description": "ping the marvin server"},
            "connect": {"method": "GET", "description": "connect to marvin robot"},
            "disconnect": {"method": "GET", "description": "disconnect from marvin robot"},
            "clear_error": {"method": "GET", "description": "clear the error of marvin robot"},
            "soft_stop": {"method": "GET", "description": "soft stop the marvin robot"},
            "left_gripper_close": {"method": "GET", "description": "open the left gripper of marvin robot"},
            "left_gripper_release": {"method": "GET", "description": "close the left gripper of marvin robot"},
            "state": {"method": "GET", "description": "get the state of marvin robot"},
            "joint_pos": {"method": "GET", "description": "get the joint position of marvin robot"},
            "cart_pos": {"method": "GET", "description": "get the cartesian position of marvin robot"},
            "reset": {"method": "GET", "description": "reset the marvin robot"},
            "set_pvt_mode": {"method": "GET", "description": "set the pvt mode of marvin robot"},
            "set_position_mode": {"method": "GET", "description": "set the position mode of marvin robot"},
            "set_impedance_mode": {"method": "GET", "description": "set the impedance mode of marvin robot"},
            "set_tool": {"method": "POST", "description": "set the dynamic and kinematics tool of marvin robot"},
            "set_joint_kd_params": {"method": "POST", "description": "set the joint kd params of marvin robot"},
            "set_cart_kd_params": {"method": "POST", "description": "set the cart kd params of marvin robot"},
            "set_vel_acc": {"method": "POST", "description": "set the vel and acc of marvin robot"},
            "set_drag_space": {"method": "GET", "description": "set the drag space of marvin robot"},
            "ik": {"method": "POST", "description": "calculate the inverse kinematics of marvin robot"},
            "/move/joint": {"method": "POST", "description": "move the robot along a joint trajectory"},
            "/calculate/cart": {"method": "POST", "description": "calculate the cartesian trajectory"},
            "/move/cart": {"method": "POST", "description": "move the robot along a cartesian trajectory"},
        })

        self.app = self.server.create_app("DummyMarvinServer", "1.0.2")
        self.register_routes()
        logger.info("服务接口注册完成")

        self.marvin_robot = Marvin_Robot()
        self.kk1 = Marvin_Kine()
        self.kk2 = Marvin_Kine()
        self.dcss = DCSS()
        self.tools_txt = "tool_dyn_kine.txt"
        self._connected = False

        # 后台订阅线程：持续读取机器人状态
        self._latest_data = None  # 机器人的状态读取
        self._subscriber_thread = None  # 后台订阅线程
        self._subscriber_stop_event = threading.Event()  # 后台订阅线程停止事件

        # 加载配置文件并初始化运动学
        project_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        )
        config_dir = os.path.join(project_root, "src/marvin_driver/config")
        config_paths = glob.glob(os.path.join(config_dir, "*.MvKDCfg"))
        if config_paths:
            ini_result = self.kk1.load_config(config_path=config_paths[0])
            if ini_result:
                self.kk1.initial_kine(
                    robot_serial=0,
                    robot_type=ini_result["TYPE"][0],
                    dh=ini_result["DH"][0],
                    pnva=ini_result["PNVA"][0],
                    j67=ini_result["BD"][0],
                )
                ini_result2 = self.kk2.load_config(config_path=config_paths[0])
                self.kk2.initial_kine(
                    robot_serial=1,
                    robot_type=ini_result2["TYPE"][0],
                    dh=ini_result2["DH"][0],
                    pnva=ini_result2["PNVA"][0],
                    j67=ini_result2["BD"][0],
                )
        print("初始化运动学 MarvinServer initialized successfully")

        '''开启日志以便检查'''
        # self.marvin_robot.log_switch('1') #全局日志开关
        # self.marvin_robot.local_log_switch('1') # 主要日志
        # print("初始化日志打印 MarvinServer initialized successfully")

    def register_routes(self):

        """
        ping the marvin server
        """
        @self.app.get("/ping")
        def ping():
            return JSONResponse(content="pong")

        """
        connect to the marvin robot
        """
        @self.app.get("/connect")
        def connect():
            if self._connected:
                return JSONResponse(status_code=400, content="Robot is already connected")

            code = self.marvin_robot.connect('172.31.1.68')
            if code == 0:
                return JSONResponse(status_code=400, content="Failed to connect to the robot, port is used")
            
            elif code == 1:
                time.sleep(0.1)
                self.marvin_robot.clear_set()
                self.marvin_robot.clear_error('A')
                self.marvin_robot.clear_error('B')
                self.marvin_robot.send_cmd()
                time.sleep(0.1)

                motion_tag = 0
                frame_update = None
                for i in range(5):
                    sub_data = self.marvin_robot.subscribe(self.dcss)
                    logger.info(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
                    if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0]['frame_serial']:
                        motion_tag += 1
                        frame_update = sub_data['outputs'][0]['frame_serial']
                    time.sleep(0.1)
                if motion_tag > 0:
                    logger.info('success:robot connection successful!')
                    self._connected = True
                    self._start_subscriber_thread()
                else:
                    logger.error('failed:robot connection failed!')
                    return JSONResponse(status_code=400, content="Failed to connect to the robot")
            return JSONResponse(status_code=200, content="Robot connected successfully")

        """
        disconnect from the marvin robot
        """
        @self.app.get("/disconnect")
        def disconnect():
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            self._stop_subscriber_thread()
            code = self.marvin_robot.release_robot()
            if code == 0:
                return JSONResponse(status_code=400, content="Failed to disconnect the robot")
            else:
                self._connected = False
                return JSONResponse(status_code=200, content="Robot disconnected successfully")

        """
        clear the error of the marvin robot
        """
        @self.app.get("/clear_error")
        def clear_error():
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")
            self.marvin_robot.clear_set()
            self.marvin_robot.clear_error('A')
            self.marvin_robot.clear_error('B')
            self.marvin_robot.send_cmd()
            return JSONResponse(status_code=200, content="Error cleared successfully")
        
        """
        soft stop the marvin robot
        """
        @self.app.get("/soft_stop")
        def soft_stop():
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")
            self.marvin_robot.soft_stop("AB")
            time.sleep(0.5)
            return JSONResponse(status_code=200, content="Robot soft stopped successfully")
        

        """
        close the left gripper of the marvin robot
        """
        @self.app.get("/left_gripper_close")
        def left_gripper_close():
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")
            self.marvin_robot.set_gripper_force('A', 100, channel=2)
            self.marvin_robot.set_gripper_position('A', 0, channel=2)
            return JSONResponse(status_code=200, content="Left gripper closed successfully")

        """
        release the left gripper of the marvin robot
        """
        @self.app.get("/left_gripper_release")
        def left_gripper_release():
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")
            self.marvin_robot.set_gripper_force('A', 0, channel=2)
            self.marvin_robot.set_gripper_position('A', 1000, channel=2)
            return JSONResponse(status_code=200, content="Left gripper released successfully")  
        #--------------------------------- 获取机械臂状态等相关方法 ---------------------------------
        """
        get the state of the marvin robot
        """
        @self.app.get("/state")
        def get_state( robot_id: str):
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")
            
            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")

            sub_data = self._get_robot_data()
            if sub_data is None:
                return JSONResponse(status_code=503, content="Robot data not ready, please retry")
            arm_index = 0 if robot_id == "A" else 1
            if arm_index < len(sub_data["outputs"]):
                output = sub_data["outputs"][arm_index]
                joints = output.get("fb_joint_pos", [0.0] * 7)
                try:
                    kine_obj = self.kk1 if arm_index == 0 else self.kk2
                    list_joints = [float(j) for j in joints]
                    fk_mat = kine_obj.fk(robot_serial=arm_index, joints=list_joints)
                    if not fk_mat:
                        content = {
                            "code": 1,
                            "status": "FAILURE",
                            "message": "Failed to calculate cartesian position"
                        }
                        return JSONResponse(status_code=400, content=content)
                    fk_mat[0][3] *=0.001
                    fk_mat[1][3] *=0.001
                    fk_mat[2][3] *=0.001
                    content = {
                        "code": 0,
                        "status": "SUCCESS",
                        "cartesian": fk_mat
                    }
                    return JSONResponse(status_code=200, content=content)
                except Exception as e:
                    content = {
                        "code": 1,
                        "status": "FAILURE",
                        "message": f"Failed to calculate cartesian position: {str(e)}"
                    }
                    return JSONResponse(status_code=400, content=content)
            else:
                content = {
                    "code": 1,
                    "status": "FAILURE",
                    "message": "Invalid robot index"
                }
                return JSONResponse(status_code=400, content=content)

        """
        get the joint position of the marvin robot
        """
        @self.app.get("/joint_pos")
        def get_joint_pos( robot_id: str):
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")

            sub_data = self._get_robot_data()
            if sub_data is None:
                return JSONResponse(status_code=503, content="Robot data not ready, please retry")
            arm_index = 0 if robot_id == "A" else 1
            if arm_index < len(sub_data["outputs"]):
                output = sub_data["outputs"][arm_index]
                joints_pos = output.get("fb_joint_pos", [0.0] * 7)
                return JSONResponse(status_code=200, content=joints_pos)

        """
        get the cartesian position of the marvin robot
        """
        @self.app.get("/cart_pos")
        def get_cart_pos( robot_id: str):
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")

            sub_data = self._get_robot_data()
            if sub_data is None:
                return JSONResponse(status_code=503, content="Robot data not ready, please retry")
            arm_index = 0 if robot_id == "A" else 1
            if arm_index < len(sub_data["outputs"]):
                output = sub_data["outputs"][arm_index]
                joints = output.get("fb_joint_pos", [0.0] * 7)
            else:
                return JSONResponse(status_code=400, content="Invalid robot index")

            try:
                kine_obj = self.kk1 if arm_index == 0 else self.kk2
                list_joints = [float(j) for j in joints]
                fk_mat = kine_obj.fk(robot_serial=arm_index, joints=list_joints)

                if not fk_mat:
                    return JSONResponse(status_code=400, content="Failed to calculate cartesian position")

                # 4x4矩阵 -> XYZABC格式
                pose_6d = kine_obj.mat4x4_to_xyzabc(pose_mat=fk_mat)

                if not pose_6d:
                    return JSONResponse(status_code=400, content="Failed to calculate cartesian position")

                return JSONResponse(status_code=200, content=pose_6d)
            except Exception as e:
                return JSONResponse(status_code=400, content=f"Failed to calculate cartesian position: {str(e)}")


        #--------------------------------- 机械臂状态设置等相关方法 ---------------------------------
        """
        reset the marvin robot
        """
        @self.app.get("/reset")
        def reset( robot_id: str):
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")

            code = self._set_robot_state(robot_id, 0)
            if code !=0 : return JSONResponse(status_code=400, content="Failed to reset the robot")
            else:
                return JSONResponse(status_code=200, content="Robot reset successfully")

        """让机械臂进入PVT模式"""
        @self.app.get("/set_pvt_mode")
        def set_pvt_mode( robot_id: str):
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")
            self.marvin_robot.clear_set()
            self.marvin_robot.set_state(arm=robot_id, state=2)
            self.marvin_robot.send_cmd()
            time.sleep( 0.1 )
            code = self._get_robot_error_code(robot_id)
            if code != 0:
                return JSONResponse(status_code=400, content="Failed to set PVT mode")
            return JSONResponse(status_code=200, content="PVT mode set successfully")

        """让机械臂进入位置跟随模式"""
        @self.app.get("/set_position_mode")
        def set_position_mode( robot_id: str):
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")
            self.marvin_robot.clear_set()
            self.marvin_robot.set_state(arm=robot_id, state=1)
            self.marvin_robot.send_cmd()
            time.sleep( 0.1 )
            code = self._get_robot_error_code(robot_id)
            if code != 0:
                return JSONResponse(status_code=400, content="Failed to set the position mode")
            else:
                return JSONResponse(status_code=200, content="Position mode set successfully")

        """让机械臂进入阻抗模式"""
        @self.app.get("/set_impedance_mode")
        def set_impedance_mode( robot_id: str, type: int):
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")
            self.marvin_robot.clear_set()
            self.marvin_robot.set_state(arm=robot_id, state=3)
            self.marvin_robot.send_cmd()
            time.sleep( 0.1 )
            self.marvin_robot.clear_set()
            self.marvin_robot.set_impedance_type(arm=robot_id, type=type)
            self.marvin_robot.send_cmd()
            time.sleep( 0.1 )
            code = self._get_robot_error_code(robot_id)
            if code != 0:
                return JSONResponse(status_code=400, content="Failed to set the impedance mode")
            else:
                return JSONResponse(status_code=200, content="Impedance mode set successfully")

        """设置拖拽空间"""
        @self.app.get("/set_drag_space")
        def set_drag_space( robot_id: str, type: int):
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")
            self.marvin_robot.clear_set()
            self.marvin_robot.set_drag_space(arm=robot_id, dgType=type)
            self.marvin_robot.send_cmd()
            time.sleep( 0.1 )
            code = self._get_robot_error_code(robot_id)
            if code != 0:
                return JSONResponse(status_code=400, content="Failed to set the drag space")
            else:
                return JSONResponse(status_code=200, content="Drag space set successfully")
        #--------------------------------- 参数设置等相关方法 ---------------------------------
        """设置工具参数"""
        @self.app.post("/set_tool")
        def set_tool(tool_params: ToolParams):
            robot_id = tool_params.robot_id
            logger.info(f"set_tool: {robot_id}, {tool_params}")
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")
            
            kine_p = tool_params.kine_params
            dyn_p = tool_params.dynamic_params

            if len(kine_p) != 6:
                return JSONResponse(status_code=400, content="工具运动学参数必须为6个")
            if len(dyn_p) != 10:
                return JSONResponse(status_code=400, content="工具动力学参数必须为10个")

            kine_p = [float(item) for item in kine_p]
            dyn_p = [float(item) for item in dyn_p]

            self.marvin_robot.clear_set()
            result = self.marvin_robot.set_tool(
                arm=robot_id, kineParams=kine_p, dynamicParams=dyn_p
            )
            self.marvin_robot.send_cmd()

            if result == 1:
                # 更新运动学模型
                tool_mat = (
                    self.kk1.xyzabc_to_mat4x4(xyzabc=kine_p)
                    if robot_id == "A"
                    else self.kk2.xyzabc_to_mat4x4(xyzabc=kine_p)
                )
                if robot_id == "A":
                    self.kk1.set_tool_kine(robot_serial=0, tool_mat=tool_mat)
                elif robot_id == "B":
                    self.kk2.set_tool_kine(robot_serial=1, tool_mat=tool_mat)
                return JSONResponse(status_code=200, content="Tool parameters set successfully")
            else:
                return JSONResponse(status_code=400, content="Failed to set tool parameters")

        """设置关节阻抗参数"""
        @self.app.post("/set_joint_kd_params")
        def set_joint_kd_params(kd_params: DynamicParams):
            robot_id = kd_params.robot_id
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")

            try:
                k = kd_params.k
                d = kd_params.d

                if len(k) != 7 or len(d) != 7:
                    return JSONResponse(status_code=400, content="K和D参数必须各为7个")

                k = [float(item) for item in k]
                d = [float(item) for item in d]

                self.marvin_robot.clear_set()
                result = self.marvin_robot.set_joint_kd_params(arm=robot_id, K=k, D=d)
                self.marvin_robot.send_cmd()

                if result == 1:
                    return JSONResponse(status_code=200, content="Joint kd params set successfully")
                else:
                    return JSONResponse(status_code=400, content="Failed to set joint kd params")
            except Exception as e:
                return JSONResponse(status_code=400, content=f"Failed to set joint kd params: {str(e)}")
    
        """设置笛卡尔阻抗参数"""
        @self.app.post("/set_cart_kd_params")
        def set_cart_kd_params(kd_params: DynamicParams):
            robot_id = kd_params.robot_id
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")

            try:
                k = kd_params.k
                d = kd_params.d

                if len(k) != 7 or len(d) != 7:
                    return JSONResponse(status_code=400, content="K和D参数必须各为7个")

                k = [float(item) for item in k]
                d = [float(item) for item in d]

                self.marvin_robot.clear_set()
                result = self.marvin_robot.set_cart_kd_params(arm=robot_id, K=k, D=d, type=1)
                self.marvin_robot.send_cmd()

                if result == 1:
                    return JSONResponse(status_code=200, content="Cart kd params set successfully")
                else:
                    return JSONResponse(status_code=400, content="Failed to set cart kd params")
            except Exception as e:
                return JSONResponse(status_code=400, content=f"Failed to set cart kd params: {str(e)}")
    
        """设置速度和加速度"""
        @self.app.post("/set_vel_acc")
        def set_vel_acc(vel_acc_params: VelAccParams):
            robot_id = vel_acc_params.robot_id
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")

            try:
                vel = vel_acc_params.vel
                acc = vel_acc_params.acc

                if vel < 0 or vel > 100:
                    return JSONResponse(status_code=400, content="Vel must be between 0 and 100")
                if acc < 0 or acc > 100:
                    return JSONResponse(status_code=400, content="Acc must be between 0 and 100")

                self.marvin_robot.clear_set()
                result = self.marvin_robot.set_vel_acc(arm=robot_id, velRatio=vel, AccRatio=acc)
                self.marvin_robot.send_cmd()

                if result == 1:
                    return JSONResponse(status_code=200, content="Vel and acc set successfully")
                else:
                    return JSONResponse(status_code=400, content="Failed to set vel and acc")
            except Exception as e:
                return JSONResponse(status_code=400, content=f"Failed to set vel and acc: {str(e)}")
    
        #--------------------------------- 机器人运动相关方法 ---------------------------------

        """逆运动学求解"""
        @self.app.post("/ik")
        def ik(ik_request: IkRequest):
            robot_id = ik_request.robot_id
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")

            pose = ik_request.pose
            ref_joints = ik_request.ref_joints

            try:
                pose_arr = np.array(pose, dtype=float)
                if pose_arr.size != 16:
                    return JSONResponse(status_code=400, content="Pose must have 16 elements (4x4 matrix)")
                pose_mat = pose_arr.reshape(4, 4).tolist()

                ref_joints_arr = np.array(ref_joints, dtype=float).flatten()
                print("pose_mat", pose_mat)
                print("ref_joints_arr", ref_joints_arr)
                

                if ref_joints_arr.size != 7:
                    return JSONResponse(status_code=400, content="Ref joints must be 7")
                ref_joints_list = ref_joints_arr.tolist()
            except (ValueError, TypeError) as e:
                return JSONResponse(status_code=400, content=f"Invalid pose or ref_joints format: {e}")

            end_ik = self.kk1.ik(robot_serial=0 if robot_id == "A" else 1, pose_mat=pose_mat, ref_joints=ref_joints_list)
            if end_ik is False:
                print("ik 求解失败")
                return JSONResponse(status_code=400, content="Failed to solve ik")
                
            joint_data = end_ik.m_Output_RetJoint.to_list()
            return JSONResponse(status_code=200, content=joint_data)

        """沿关节轨迹运动"""
        @self.app.post("/move/joint")
        def move_joint(trajectory_params: JointTrajectoryParams):
            robot_id = trajectory_params.robot_id
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")
                
            joints = trajectory_params.joints
            if len(joints) != 7:
                return JSONResponse(status_code=400, content="Joints must be 7")

            joints = [float(item) for item in joints]

            
            self.marvin_robot.clear_set()
            self.marvin_robot.set_state(arm=robot_id, state=1)
            self.marvin_robot.set_vel_acc(arm=robot_id, velRatio=trajectory_params.vel_ratio, AccRatio=trajectory_params.acc_ratio)
            self.marvin_robot.send_cmd()
            time.sleep(0.02)
            self.marvin_robot.clear_set()
            self.marvin_robot.set_joint_cmd_pose(arm=robot_id, joints=joints)
            self.marvin_robot.send_cmd()
            time.sleep( 0.1 )

            self._wait_for_low_speed_flag(robot_id)
            code = self._get_robot_error_code(robot_id)
            if code != 0:
                return JSONResponse(status_code=400, content="Failed to move the robot along a joint trajectory")
            else:
                return JSONResponse(status_code=200, content="Robot moved along a joint trajectory successfully")

        @self.app.post("/calculate/cart")
        def calculate_cart(trajectory_params: CartTrajectoryParams):
            robot_id = trajectory_params.robot_id
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")
                
            save_path = Path(__file__).parent / "path_cache" / trajectory_params.save_path
            success = self.kk1.movL_KeepJ( robot_serial=0 if robot_id == "A" else 1, start_joints=trajectory_params.start_joints, end_joints=trajectory_params.end_joints, vel=trajectory_params.vel, save_path=str(save_path) )
            if success:
                # 这里要进行数据解析
                parsed_data = self._parse_movl_path_file(save_path)
                return JSONResponse(status_code=200, content=parsed_data)
            else:
                return JSONResponse(status_code=400, content="Failed to calculate the cartesian trajectory")

        @self.app.post("/move/cart")
        def move_cart(move_params: CartMoveParams):
            robot_id = move_params.robot_id
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")
                
            path = move_params.path
            vel = move_params.vel
            acc = move_params.acc
            prep = move_params.prep

            if len(path) == 0:
                return JSONResponse(status_code=400, content="Path must be not empty")

            if prep:
                self.marvin_robot.clear_set()
                self.marvin_robot.set_state(arm=robot_id, state=1)
                self.marvin_robot.set_vel_acc(arm=robot_id, velRatio=vel, AccRatio=acc)
                self.marvin_robot.send_cmd()
                time.sleep(0.5)
            for joint_config in path:
                self.marvin_robot.clear_set()
                self.marvin_robot.set_joint_cmd_pose(arm=robot_id, joints=joint_config)
                self.marvin_robot.send_cmd()
                time.sleep(0.02)

            time.sleep(0.02)
            self._wait_for_low_speed_flag(robot_id)
            code = self._get_robot_error_code(robot_id)
            if code != 0:
                return JSONResponse(status_code=400, content="Failed to move the robot along a cartesian trajectory")
            else:
                return JSONResponse(status_code=200, content="Robot moved along a cartesian trajectory successfully")   
                
    """
    set the state of the marvin robot, 设置机械臂的运动模式
    """    
    def _set_robot_state(self, robot_id: str, state: int):
        if self._connected:
            self.marvin_robot.clear_set()
            self.marvin_robot.set_state(arm=robot_id, state=state)
            self.marvin_robot.send_cmd()
            time.sleep( 0.1 )
            return self._get_robot_error_code(robot_id)
        else:
            return 0

    def _get_servo_error_code(self, robot_id: str):
        code = self.marvin_robot.get_servo_error_code(robot_id)
        return code

    def _get_robot_error_code(self, robot_id: str):
        sub_data = self._get_robot_data()
        if sub_data is None:
            return 0
        arm_index = 0 if robot_id == "A" else 1
        if arm_index < len(sub_data.get("outputs", [])):
            output = sub_data["outputs"][arm_index]
            return output.get("m_ERRCode", 0)
        return 0

    def _get_robot_data(self):
        """返回后台线程缓存的机器人状态数据"""
        return self._latest_data

    def _subscriber_loop(self):
        """后台线程：持续 subscribe 并更新 _latest_data"""
        while not self._subscriber_stop_event.is_set():
            if self._connected:
                try:
                    data = self.marvin_robot.subscribe(self.dcss)
                    self._latest_data = data
                except Exception as e:
                    logger.exception(f"Subscribe error: {e}")
            self._subscriber_stop_event.wait(timeout=0.002)

    def _start_subscriber_thread(self):
        """连接成功后启动后台订阅线程"""
        self._subscriber_stop_event.clear()
        self._subscriber_thread = threading.Thread(
            target=self._subscriber_loop,
            daemon=True,
            name="MarvinSubscriber",
        )
        self._subscriber_thread.start()
        logger.info("后台订阅线程已启动")

    def _stop_subscriber_thread(self):
        """断开连接前停止后台订阅线程"""
        self._subscriber_stop_event.set()
        if self._subscriber_thread and self._subscriber_thread.is_alive():
            self._subscriber_thread.join(timeout=2.0)
        self._subscriber_thread = None
        self._latest_data = None
        logger.info("后台订阅线程已停止")

    def _wait_for_low_speed_flag(self,robot_id: str):
        """
        等待低速标志，当 low_speed_flag == b'\\x01' 时返回
        使用后台线程缓存的 _latest_data 轮询
        """
        arm_index = 0 if robot_id == "A" else 1
        while True:
            sub_data = self._latest_data
            if sub_data is None or arm_index >= len(sub_data.get("outputs", [])):
                time.sleep(0.01)
                continue

            output = sub_data["outputs"][arm_index]
            low_speed_flag = output.get("low_speed_flag", False)
            # print("low_speed_flag", low_speed_flag)
            
            # 如果 low_speed_flag == b'\x01'，则返回
            if low_speed_flag == b'\x01':
                print("检测到低速标志，函数返回")
                return
            
            time.sleep(0.01)

    def _parse_movl_path_file(self, file_path: Path) -> List[List[float]]:
        """解析 PLN_MOVL 格式的 txt 路径文件，返回 [X,Y,Z,A,B,C,U] 列表"""
        parsed_data: List[List[float]] = []
        with open(file_path, 'r') as f:
            for line in f.readlines()[1:]:
                line = line.strip()
                if not line:
                    continue
                match = _MOVL_LINE_PATTERN.search(line)
                if match:
                    parsed_data.append([float(match.group(i)) for i in range(1, 8)])
        return parsed_data

    def run(self):
        logger.info("启动MarvinServer服务...")
        return self.app
