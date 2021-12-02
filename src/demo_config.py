"""
    These are built-in configurations for arena dimensions (in meters) to be used for simple tests.
    size (rows x columns) of SMALL arena grid is: 8x8
    size (rows x columns) of MEDIUM arena grid is: 10x20
    size (rows x columns) of LARGE arena grid is: 15x23
    All for cell size of 0.3m
"""
DEMO_ARENA_CONFIG = {
    "SMALL": {"cell_size": 0.3, "height": 2.5, "width": 2.5},
    "MEDIUM": {"cell_size": 0.3, "height": 3.0, "width": 6.0},
    "LARGE": {"cell_size": 0.3, "height": 4.5, "width": 7.0}
}
