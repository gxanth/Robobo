from pathlib import Path

# Root = project root
ROOT = Path(__file__).resolve().parents[3]

# Base directories
LOG_DIR = ROOT / "logs"
RESULTS_DIR = ROOT / "results"
MODELS_DIR = ROOT / "models"

# Subdirectories
SIM_RESULTS_DIR = RESULTS_DIR / "simulations"
HARD_RESULTS_DIR = RESULTS_DIR / "hardware"

# Ensure all folders exist
for path in [LOG_DIR, RESULTS_DIR, MODELS_DIR, SIM_RESULTS_DIR, HARD_RESULTS_DIR]:
    path.mkdir(parents=True, exist_ok=True)

# Export paths for convenience
RESULTS_SUBDIRS = {
    "simulation": SIM_RESULTS_DIR,
    "hardware": HARD_RESULTS_DIR,
}
