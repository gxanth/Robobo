# core.py
"""
Core Dash app setup and layout management.
"""

import dash_bootstrap_components as dbc
from dash import Dash

app = Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])
app.title = "Robobo Dashboard"

# Expose app for use in other modules
__all__ = ["app"]
