# RoboBuggy2
A complete re-write of the old RoboBuggy. This code was run for RD23, RD24 and RD25, on both NAND and Short Circuit.


## Table of Contents
 - Installation and Initial Setup
 - Launching Code
 - Infrastructure Documentation
 - Code Structure and Documentation


---
## Installation and Initial Setup
### Necessary + Recommended Software
- Docker
- Foxglove
- VSCode (recommended)
- Git (recommended)


### Docker
- Installation instructions here: https://docs.docker.com/get-docker/

### Foxglove
- Installation instructions here: https://foxglove.dev/

### VSCode
- https://code.visualstudio.com/download

### Git
- https://git-scm.com/downloads

### Install Softwares: WSL, Ubuntu (Windows only)
- Go to Microsoft Store to install "Ubuntu 20.04.6 LTS".

### Set up repo in WSL
- To set up ssh key, follow this link: [Connecting to GitHub with SSH](https://docs.github.com/en/authentication/connecting-to-github-with-ssh).
- Note: Ensure that the SSH keys are generated while in the WSL terminal
- In the website above, see these two pages: [Generating a new SSH key and adding it to the ssh-agent](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) and ["Adding a new SSH key to your GitHub account"](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).


### Apple Silicon Mac Only:
- In Docker Desktop App: go to settings -> general and turn on "Use Rosetta for x86/amd64 emulation on Apple Silicon"

### Set up repository
- To set up ssh key, follow this link: [Connecting to GitHub with SSH](https://docs.github.com/en/authentication/connecting-to-github-with-ssh).
- Note: Ensure that the SSH keys are generated while in the terminal
- In the website above, see these two pages: [Generating a new SSH key and adding it to the ssh-agent](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) and ["Adding a new SSH key to your GitHub account"](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).


### Clone the Repository
This is so you can edit our codebase locally, and sync your changes with the rest of the team through Git.
- In your terminal type: `$ git clone https://github.com/CMU-Robotics-Club/RoboBuggy2.git`.
- The clone link above is the URL or can be found above: code -> local -> Clone HTTPS.


### Foxglove Visualization
- Foxglove is used to visualize both the simulator and the actual buggy's movements.
- First, you need to import the layout definition into Foxglove. On the top bar, click Layout, then "Import from file".
- ![image](https://github.com/CMU-Robotics-Club/RoboBuggy2/assets/116482510/2aa04083-46b3-42a5-bcc1-99cf7ccdb3d2)
- Go to RoboBuggy2 and choose the file [telematics layout](telematics_layout.json)
- To visualize the simulator, launch the simulator and then launch Foxglove and select "Open Connection" on startup.
- Use this address `ws://localhost:8765` for Foxglove Websocket
- Open Foxglove, choose the third option "start link".
- ![image](https://github.com/CMU-Robotics-Club/RoboBuggy2/assets/116482510/66965d34-502b-4130-976e-1419c0ac5f69)



### X11 Setup (recommended)
- Install the appropriate X11 server on your computer for your respective operating systems (Xming for Windows, XQuartz for Mac, etc.).
- Mac: In XQuartz settings, ensure that the "Allow connections from network clients" under "Security" is checked.
- Windows: Make sure that you're using WSL 2 Ubuntu and NOT command prompt.
- While in a bash shell with the X11 server running, run `xhost +local:docker`.
- Boot up the docker container using the "Alternate Shortcut" above.
- Run `xeyes` while INSIDE the Docker container to test X11 forwarding. If this works, we're good.


## Launching Code
### Open Docker
- Use `cd` to change the working directory to be `RoboBuggy2`
- Then do `./setup_dev.sh` in the main directory (RoboBuggy2) to launch the docker container. Utilize the `--no-gpu`, `--force-gpu`, and `--run-testing` flags as necessary.
- Then you can go in the docker container using the `docker exec -it robobuggy2-main-1 bash`.
- When you are done, type Ctrl+C and use `$exit` to exit.

### ROS
- Navigate to `/rb_ws`. This is the catkin workspace where we will be doing all our ROS stuff.
- (This should only need to be run the first time you set up the repository) - to build the ROS workspace and source it, run:
        catkin_make
        source /rb_ws/devel/setup.bash  # sets variables so that our package is visible to ROS commands
- To learn ROS on your own, follow the guide on https://wiki.ros.org/ROS/Tutorials.

### 2D Simulation
- Boot up the docker container
- Run `roslaunch buggy sim_2d_single.launch` to simulate 1 buggy
- See `rb_ws/src/buggy/launch/sim_2d_single.launch` to view all available launch options
- Run `roslaunch buggy sim_2d_2buggies.launch` to simulate 2 buggies

<img width="612" alt="Screenshot 2023-11-13 at 3 18 30 PM" src="https://github.com/CMU-Robotics-Club/RoboBuggy2/assets/45720415/b204aa05-8792-414e-a868-6fbc0d11ab9d">

- See `rb_ws/src/buggy/launch/sim_2d_2buggies.launch` to view all available launch options
    - The buggy starting positions can be changed using the `sc_start_pos` and `nand_start_pos` arguments (can pass as a key to a dictionary of preset start positions in engine.py, a single float for starting distance along planned trajectory, or 3 comma-separated floats (utm east, utm north, and heading))
- To prevent topic name collision, a topic named `t` associated with buggy named `x` have format `x/t`. The names are `SC` and `Nand` in the 2 buggy simulator. In the one buggy simulator, the name can be defined as a launch arg.
- See [**Foxglove Visualization**](#foxglove-visualization) for visualizing the simulation. Beware that since topic names are user-defined, you will need to adjust the topic names in each panel.

### Connecting to and Launching the RoboBuggies
When launching Short Circuit:
- Connect to the Wi-Fi named ShortCircuit.
-	In the command line window:
SSH to the computer on ShortCircuit and go to folder
`$ ssh nuc@192.168.1.217`
Then `$ cd RoboBuggy2`
-	Setup the docker
`$ ./setup_prod.sh` (Utilize the `--no-gpu`, `--force-gpu`, and `--run-testing` flags as necessary.)
-	Go to docker container
`$ docker_exec`
-	Open foxglove and do local connection to “ws://192.168.1.217/8765”
-	Roslauch in docker container by `$ roslaunch buggy sc-main.launch`
(wait until no longer prints “waiting for covariance to be better”)

When launching NAND:
- Ask software lead (WIP)

When shutting down the buggy:
-	Stop roslauch
`$ ^C  (Ctrl+C)`
-	Leave the docker container
`$ exit`
-	Shutdown the ShortCircuit computer
`$ sudo shutdown now`

## Documentation
### Infrastructure Documentation
Ask Software Lead (WIP)

### Simulator notes
- Longitude + Latitude for Foxglove visualization on map: `/state/pose_navsat` (sensor_msgs/NavSatFix)
- UTM coordinates (assume we're in Zone 17N): `/sim_2d/utm` (geometry_msgs/Pose - position.x = Easting meters , position.y = Northing meters, position.z = heading in degrees from East axis + is CCW)
- INS Simulation: `/nav/odom` (nsg_msgs/Odometry) (**Noise** is implemented to vary ~1cm)
Commands:
- Steering angle: `/buggy/steering` in degrees (example_interfaces/Float64)
- Velocity: `/buggy/velocity` in m/s (example_interfaces/Float64)

### Auton Logic
Ask someone with experience (WIP)
