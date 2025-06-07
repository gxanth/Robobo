from loguru import logger
from pathlib import Path

# Set log file path using pathlib
log_dir = Path(__file__).resolve().parents[3] / "logs"
if not log_dir.exists():
    log_dir.mkdir(parents=True)
log_file = log_dir / "my_robot.log"

# Configure loguru
logger.remove()  # Remove default stderr logger
logger.add(log_file, level="DEBUG", format="{time} | {level} | {message}", rotation="1 MB")

def get_logger_func(level: str = "INFO"):
    """
    Returns a function that logs messages using Loguru at the given level.
    Compatible with SimulationRobobo and HardwareRobobo logger interface.
    """
    def _logger_func(msg: str):
        logger.log(level.upper(), msg)
    return _logger_func
