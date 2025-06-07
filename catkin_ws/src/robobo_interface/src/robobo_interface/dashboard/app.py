# app.py
"""
Main entry point for the modular Robobo Dashboard.
"""

import dash
import dash_bootstrap_components as dbc
from dash import dcc, html

from .core import app
from .register_callbacks import register_callbacks
from .sidebar import sidebar

# Import layouts for tabs
from .tabs.overview.layout import overview_layout

# Register all callbacks
register_callbacks(app)

app.layout = dbc.Container(
    [
        dbc.Row(
            [
                dbc.Col(sidebar, width=2),
                dbc.Col(dcc.Location(id="url"), width=10),
            ],
            className="g-0",
        ),
        dbc.Row(
            [dbc.Col(html.Div(id="page-content"), width={"size": 10, "offset": 2})]
        ),
    ],
    fluid=True,
)


# Callback for page routing
@app.callback(
    dash.dependencies.Output("page-content", "children"),
    [dash.dependencies.Input("url", "pathname")],
)
def display_page(pathname):
    if pathname == "/" or pathname is None:
        return overview_layout
    # Add more routes here
    return html.H3("404: Page not found")


if __name__ == "__main__":
    app.run_server(debug=True, host="0.0.0.0", port=8050)
