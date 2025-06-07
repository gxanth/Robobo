# register_callbacks.py
"""
Register all Dash callbacks for the dashboard.
"""


def register_callbacks(app):
    from .tabs.overview import callbacks as overview_callbacks

    overview_callbacks.register(app)
    # Register additional tab callbacks here
