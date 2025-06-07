from datetime import datetime
from typing import Any, Dict

import pandas as pd

from robobo_interface.utils.paths import RESULTS_DIR


class SensorStash:
    def __init__(self, mode: str = "simulation"):
        """
        mode: 'simulation' or 'hardware'
        """
        if mode not in {"simulation", "hardware"}:
            raise ValueError(f"Invalid mode: {mode}. Use 'simulation' or 'hardware'.")

        self._data = []

        # Choose correct subfolder
        subdir = RESULTS_DIR / ("simulations" if mode == "simulation" else "hardware")
        subdir.mkdir(parents=True, exist_ok=True)

        # Filename only handled here
        filename = self._format_csv_name(mode)
        self._stash_path = subdir / filename

    def update(self, reading: Dict[str, Any]) -> None:
        """Add a new reading (one timestep)."""
        flat = self._flatten_dict(reading)
        self._data.append(flat)

    def reset(self) -> None:
        """Clear all stored readings."""
        self._data.clear()

    def to_dataframe(self) -> pd.DataFrame:
        """Export all readings as a Pandas DataFrame."""
        return pd.DataFrame(self._data)

    def save_csv(self) -> None:
        """Save readings to a CSV file. Index will be named 'timesteps'."""
        df = self.to_dataframe()
        df.index.name = "timesteps"
        df.to_csv(self._stash_path)

    @staticmethod
    def _format_csv_name(mode: str, ext: str = "csv") -> str:
        """
        Generate a timestamped filename like 'session_2025-06-07_19-12-01.csv'
        """
        prefix = "session" if mode == "simulation" else "run"
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        return f"{prefix}_{timestamp}.{ext}"

    @staticmethod
    def _flatten_dict(
        d: Dict[str, Any], parent_key: str = "", sep: str = "_"
    ) -> Dict[str, Any]:
        """Recursively flatten nested dictionaries."""
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(SensorStash._flatten_dict(v, new_key, sep=sep).items())
            elif isinstance(v, (list, tuple)):
                for i, val in enumerate(v):
                    items.append((f"{new_key}{sep}{i}", val))
            else:
                items.append((new_key, v))
        return dict(items)
