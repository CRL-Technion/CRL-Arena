# Overview
This projcet includes the required code for preforming a robots experiment in the CRL lab.
It allows to read live data from the Motive system in the lab and  provides a pyGame based GUI that enables MAPF scenario's management.

## Documentation
For detailed documentation about the project and its multiple components, refer to [CRL Arena Documentation](https://github.com/CRL-Technion/CRL-Documents/blob/master/CRL/CRL%20Arena%20Documentation.pdf).

In order to run a simple demo, refer to our [Step-by-Step guide](https://github.com/CRL-Technion/CRL-Documents/blob/master/CRL/CRL_Arena_Step_by_Step_Guide.pdf).

Both are located under the CRL Documents repository, so ask for the relevant access permissions, if required.

# Main project files
- main.py - Initiates the program and runs the main loop which reads the data from Motive and updates the view on the screen.
  - This is a server that sends information from Motive to the client within a ROS node.

- Listener.py - An implementation of a Python NatNat client for motion tracking with [OptiTrack NatNetSDK](https://optitrack.com/software/natnet-sdk/) v3.1.0 (Motive v2.1). Streams live motion capture (rigid bodies, markers, skeletons, etc) over a network.
See [documentation](https://github.com/dudiw/natnet-motionclient/wiki) for usage.

- Grid.py - Allows to update the MAPF scenario map according to the live data and draw it to screen.
 
- udp_server.py - An implementation of a UDP server which allows to croadcast the live objects' data (to be used from different workstation for guiding the robots).


# Run
To run a basic scenario with random goal locations execute the main function from the root directory (and use the GUI to generate goals and run the planner):

`python main.py -c <cell_size> -he <env_height> -w <env_width> -m <map_filename.map> -s <scene_filename.scen>`

- The program can run a default scenario without any specified arguments.
- It is also possible to choose 1 of 3 built-in environment configurations using the `--config` flag.
- An explaination about the different program arguments can be found in the documents specified under Documentation section.
- **Note:** NatNet server / Motive is required for live data. I order to run a **Mocup environemnt**, comment-out the following line in **main.py**: `listener = mockup.simple_listener_mock`



# General
- Python +2.7 compatible
- No 3rd party library dependencies


