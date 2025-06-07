# sidebar.py
"""
Sidebar layout for navigation between dashboard tabs.
"""

import dash_bootstrap_components as dbc

sidebar = dbc.Nav(
    [
        dbc.NavLink("Overview", href="/", active="exact"),
        # Add more NavLinks for additional tabs here
    ],
    vertical=True,
    pills=True,
    className="bg-light sidebar",
)

__all__ = ["sidebar"]
