"""
    These are built-in configurations for arena dimensions (in meters) to be used for simple tests.
    size (rows x columns) of SMALL arena grid is: 6x6
    size (rows x columns) of MEDIUM arena grid is: 10x20
    size (rows x columns) of LARGE arena grid is: 20x26
"""
DEMO_ARENA_CONFIG = {
    "SMALL": {"cell_size": 0.3, "height": 2.0, "width": 2.0},
    "MEDIUM": {"cell_size": 0.3, "height": 3.0, "width": 6.0},
    "LARGE": {"cell_size": 0.3, "height": 6.0, "width": 8.0}
}
