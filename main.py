import logging
from dotenv import load_dotenv
import uvicorn

# 先配置日志，必须在导入其他模块之前
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    force=True  # Python 3.8+ 支持强制重新配置
)

load_dotenv()

from src.marvin_driver.marvin_server import MarvinServer

logger = logging.getLogger(__name__)

def main():
    logger.info("=" * 50)
    logger.info("启动Marvin驱动服务")
    logger.info("=" * 50)
    marvin_server = MarvinServer()
    app = marvin_server.run()
    logger.info("服务运行在 http://0.0.0.0:8010")
    uvicorn.run(app, host="0.0.0.0", port=8010)

if __name__ == "__main__":
    main()