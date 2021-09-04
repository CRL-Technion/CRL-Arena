# Overview

## Main files
- main_planner.py- Runs the visualization of the environment for creating the relevant files for the main planner (CBS).
- main_udp- This is a server that sends information from Motive to the client within a ROS node.
- main_combined- Runes the visualization and the server together (combination of main_planner.py and main_udp).
- robot_setup/multi_agent_start_script.py- ssh all the robots in the list of IP addresses.

-----

## General
### NatNet MotionClient
A Python NatNat client for motion tracking with [OptiTrack NatNetSDK](https://optitrack.com/software/natnet-sdk/) v3.1.0 (Motive v2.1).

Stream live motion capture (rigid bodies, markers, skeletons, etc) over a network.

- Python +2.7 compatible
- No 3rd party library dependencies

See [documentation](https://github.com/dudiw/natnet-motionclient/wiki) for usage 
and `sample.py` for a full example (Note: NatNet server / Motive is required for live data).
