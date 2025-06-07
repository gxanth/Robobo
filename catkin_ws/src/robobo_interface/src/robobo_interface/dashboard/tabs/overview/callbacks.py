# callbacks.py
"""
Overview tab callbacks for the dashboard.
"""

import pandas as pd
import plotly.express as px
from dash import Output
from robobo_interface.paths import RESULTS_DIR


def register(app):
    @app.callback(
        Output("sensor-timeseries-plot", "figure"),
        [],
        [],
    )
    def update_sensor_plot():
        # Load sensor data from CSV (if available)
        try:
            df = pd.read_csv(RESULTS_DIR / "sensor_stash.csv", index_col=0)
            fig = px.line(df, title="Sensor Time Series")
        except Exception:
            fig = px.line(title="No data available")
        return fig
