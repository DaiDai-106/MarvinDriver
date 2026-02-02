import logging
from Chin.core.HttpServerUtil import DummyServer
from marvin_driver.marvin import Marvin_Robot, Marvin_Kine

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

logger = logging.getLogger(__name__)


"""
Marvin双臂驱动服务, 以及夹抓的服务
"""
class MarvinServer( object ):
    def __init__( self ):
        logger.info("初始化MarvinServer...")
        self.marvin_robot = Marvin_Robot()
        self.marvin_kine = Marvin_Kine()
        self.server = DummyServer()
        self.server.register_service("DummtMarvinServer", "A service to test marvin robot...", "1.0.0", {
            "connect": {"method": "GET", "description": "connect to marvin robot"},
            "disconnect": {"method": "GET", "description": "disconnect from marvin robot"},
            "state": {"method": "GET", "description": "get the state of marvin robot, including current joint and cartesian position"},
            "reset": {"method": "GET", "description": "reset the marvin robot"},
        })

        self.app = self.server.create_app("DummtMarvinServer", "1.0.0")
        logger.info("服务接口注册完成")

    def run(self):
        logger.info("启动MarvinServer服务...")
        return self.app
