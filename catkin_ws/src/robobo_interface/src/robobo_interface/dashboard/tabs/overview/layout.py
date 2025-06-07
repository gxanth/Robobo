# layout.py
"""
Overview tab layout for the dashboard.
"""

import dash_bootstrap_components as dbc
from dash import dcc, html

overview_layout = dbc.Container(
    [
        html.H2("Robobo Dashboard Overview"),
        html.P("Welcome to the Robobo Dashboard. Use the sidebar to navigate."),
        dcc.Graph(id="sensor-timeseries-plot"),
        # Add more overview components here
    ],
    fluid=True,
)
