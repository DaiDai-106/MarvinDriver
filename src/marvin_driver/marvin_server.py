import logging
import time
import os
import glob
from pydantic import BaseModel
from Chin.core.HttpServerUtil import DummyServer
from marvin_driver.marvin import Marvin_Robot, Marvin_Kine
from typing import Optional
from fastapi.responses import JSONResponse
from marvin_driver.marvin.structure import DCSS

# 获取日志器（日志配置已在main.py中完成）
logger = logging.getLogger(__name__)

"""一些序列化的参数"""
class ToolParams( BaseModel ):
    kine_params: list[float]
    dynamic_params: list[float]

class DynamicParams( BaseModel ):
    k: list[float]
    d: list[float]

class VelAccParams( BaseModel ):
    vel: float
    acc: float

class JointTrajectoryParams( BaseModel ):
    joints: list[float] 
    time: float  # 这个参数目前未作用

class CartTrajectoryParams( BaseModel ):
    pose: list[list[float]]
    time: float  # 这个参数目前未作用

"""
Marvin双臂驱动服务, 以及夹抓的服务
"""
class MarvinServer( object ):
    def __init__( self ):
        logger.info("初始化MarvinServer...")
        self.server = DummyServer()
        self.server.register_service("DummyMarvinServer", "A service to test marvin robot...", "1.0.0", {
            "ping": {"method": "GET", "description": "ping the marvin server"},
            "connect": {"method": "GET", "description": "connect to marvin robot"},
            "disconnect": {"method": "GET", "description": "disconnect from marvin robot"},
            "clear_error": {"method": "GET", "description": "clear the error of marvin robot"},
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
            "/move/joint": {"method": "POST", "description": "move the robot along a joint trajectory"},
            "/move/trajectory/cart": {"method": "POST", "description": "move the robot along a cartesian trajectory"},
        })

        self.app = self.server.create_app("DummyMarvinServer", "1.0.0")
        self.register_routes()
        logger.info("服务接口注册完成")

        self.marvin_robot = Marvin_Robot()
        self.kk1 = Marvin_Kine()
        self.kk2 = Marvin_Kine()

        self.dcss = DCSS()
        
        self.tools_txt = "tool_dyn_kine.txt"
        self._connected = False
        self._currentData = None

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
        def connect(ip: str):
            if self._connected:
                return JSONResponse(status_code=400, content="Robot is already connected")

            self._currentData = None # 清空当前数据

             # 验证IP格式
            try:
                ip_parts = ip.split(".")
                if len(ip_parts) != 4 or not all(0 <= int(p) <= 255 for p in ip_parts):
                    return JSONResponse(status_code=400, content="Invalid IP format")
            except (ValueError, AttributeError):
                return JSONResponse(status_code=400, content=f"Invalid IP format: {ip}")
    
            code = self.marvin_robot.connect(ip)
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

            code = self.marvin_robot.release_robot()
            if code == 0:
                self._connected = False
                self._currentData = None
                return JSONResponse(status_code=400, content="Failed to disconnect the robot")
            else:
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

            self._subscribe_robot_data()
            arm_index = 0 if robot_id == "A" else 1
            if arm_index < len(self._currentData["states"]):
                state = self._currentData["states"][arm_index]
                return JSONResponse(status_code=200, content=state)
            else:
                return JSONResponse(status_code=400, content="Invalid robot index")

        """
        get the joint position of the marvin robot
        """
        @self.app.get("/joint_pos")
        def get_joint_pos( robot_id: str):
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")

            self._subscribe_robot_data()
            """获取关节数据"""
            arm_index = 0 if robot_id == "A" else 1
            if arm_index < len(self._currentData["outputs"]):
                output = self._currentData["outputs"][arm_index]
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

                
            self._subscribe_robot_data()
            arm_index = 0 if robot_id == "A" else 1
            if arm_index < len(self._currentData["outputs"]):
                output = self._currentData["outputs"][arm_index]
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
            return self._get_robot_error_code(robot_id)

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
        def set_tool( robot_id: str, tool_params: ToolParams):
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
        def set_joint_kd_params( robot_id: str, kd_params: DynamicParams):
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
        def set_cart_kd_params( robot_id: str, kd_params: DynamicParams):
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
        def set_vel_acc( robot_id: str, vel_acc_params: VelAccParams):
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
        """沿关节轨迹运动"""
        @self.app.post("/move/joint")
        def move_joint( robot_id: str, trajectory_params: JointTrajectoryParams):
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")
                
            joints = trajectory_params.joints
            if len(joints) != 7:
                return JSONResponse(status_code=400, content="Joints must be 7")

            joints = [float(item) for item in joints]
            self.marvin_robot.clear_set()
            self.marvin_robot.set_joint_cmd_pose(arm=robot_id, joints=joints)
            self.marvin_robot.send_cmd()
            time.sleep( 0.1 )
            code = self._get_robot_error_code(robot_id)
            if code != 0:
                return JSONResponse(status_code=400, content="Failed to move the robot along a joint trajectory")
            else:
                return JSONResponse(status_code=200, content="Robot moved along a joint trajectory successfully")

        @self.app.post("/move/trajectory/cart")
        def move_trajectory_cart( robot_id: str, trajectory_params: CartTrajectoryParams):
            if not self._connected:
                return JSONResponse(status_code=400, content="Robot is not connected")

            if robot_id not in ["A", "B"]:
                return JSONResponse(status_code=400, content="Invalid robot ID")
                
            pass
        
    
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
        self._subscribe_robot_data()    
        arm_index = 0 if robot_id == "A" else 1
        if arm_index < len(self._currentData["outputs"]):
            output = self._currentData["outputs"][arm_index]
            return output.get("m_ERRCode", 0)
        else:
            return 0

    def _subscribe_robot_data(self):
        sub_data = self.marvin_robot.subscribe(self.dcss)
        self._currentData = sub_data

    def run(self):
        logger.info("启动MarvinServer服务...")
        return self.app
