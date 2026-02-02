from dotenv import load_dotenv
import uvicorn
load_dotenv()

from src.marvin_driver.marvin_server import MarvinServer


def main():
    marvin_server = MarvinServer()
    app = marvin_server.run()
    uvicorn.run(app, host="0.0.0.0", port=8010)

if __name__ == "__main__":
    main()