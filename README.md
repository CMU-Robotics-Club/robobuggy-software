# robobuggy-software
The software stack of the Robobuggy team at CMU. This code was run for Raceday '25, on both Short Circuit and NAND.

We push buggy and code, but not buggy code. 


## Table of Contents
 - Installation and Initial Setup
 - Launching Code

---
## Installation and Initial Setup
### Necessary + Recommended Software
- Docker
- Git
- Foxglove
- VSCode (recommended)

### Docker
- Installation instructions here: https://docs.docker.com/get-docker/

### Git
- https://git-scm.com/downloads

### Foxglove
- Installation instructions here: https://foxglove.dev/

### VSCode
- https://code.visualstudio.com/download

### Install Softwares: WSL, Ubuntu (Windows only)
- Go to Microsoft Store to install "Ubuntu 22.04 LTS".

### Apple Silicon Mac Only:
- In Docker Desktop App: go to settings -> general and turn on "Use Rosetta for x86/amd64 emulation on Apple Silicon"


### Clone the Repository
This is so you can edit our codebase locally, and sync your changes with the rest of the team through Git.
- In your terminal type: `$ git clone https://github.com/CMU-Robotics-Club/robobuggy-software.git`.
- The clone link above is the URL or can be found above: code -> local -> Clone HTTPS.


### Foxglove Visualization
- Foxglove is used to visualize both the simulator and the actual buggy's movements.
- First, you need to import the layout definition into Foxglove. On the top bar, click Layout, then "Import from file".
- ![image](https://github.com/CMU-Robotics-Club/RoboBuggy2/assets/116482510/2aa04083-46b3-42a5-bcc1-99cf7ccdb3d2)
- Go to repository and choose the file [telematics layout](telematics_layout.json)
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
- Use `$ cd` to change the working directory to be `robobuggy-software`
- (If you are on Windows, you need to run the rest of this guide within a linux shell. Type `$ bash` in the terminal to open the Ubuntu shell.)
- Then do `./setup_dev.sh` in the `robobuggy-software` directory to launch the docker container.
- Then you can execute and enter the docker container using `$ docker exec -it robobuggy-software-main-1 bash`.
- When you are done, click Ctrl+C and use Ctrl+D or `$ exit` to exit.

### ROS
- Make sure you are in the `rb_ws` directory. This is the workspace where we will be doing all our ROS stuff.
- To learn ROS on your own, follow the guide on https://wiki.ros.org/ROS/Tutorials.

### 2D Simulation
- Boot up the docker container (instructions above)
- Run `ros2 launch buggy sim_2d_single.xml` to simulate 1 buggy
- Run `ros2 launch buggy sim_2d_double.xml` to simulate 2 buggies

<img width="612" alt="Screenshot 2023-11-13 at 3 18 30 PM" src="https://github.com/CMU-Robotics-Club/RoboBuggy2/assets/45720415/b204aa05-8792-414e-a868-6fbc0d11ab9d">

<!-- - See `rb_ws/src/buggy/launch/sim_2d_2buggies.launch` to view all available launch options
    - The buggy starting positions can be changed using the `sc_start_pos` and `nand_start_pos` arguments (can pass as a key to a dictionary of preset start positions in engine.py, a single float for starting distance along planned trajectory, or 3 comma-separated floats (utm east, utm north, and heading))
- To prevent topic name collision, a topic named `t` associated with buggy named `x` have format `x/t`. The names are `SC` and `Nand` in the 2 buggy simulator. In the one buggy simulator, the name can be defined as a launch arg. -->
- See [**Foxglove Visualization**](#foxglove-visualization) for visualizing the simulation.
  <!-- Beware that since topic names are user-defined, you will need to adjust the topic names in each panel. -->

<br>

### Connecting to and Launching the RoboBuggies

#### To launching **Short Circuit**:
1. Connect to the Wi-Fi named **ShortCircuit**.  
2. In the command line window:
   - SSH to the computer on ShortCircuit:  
     ```bash
     ssh nuc@192.168.1.217
     ```
   - Attach to the tmux session (this will split your window into 4 panes):  
     ```bash
     tmux a -t buggy
     ```
     - **Top left pane**: system node  
     - **Top middle pane**: main (auton) node  
     - **Top right pane**: rosbag node (prefilled with a `startbag` command)  
     - **Bottom pane**: your working terminal  

     Refer to [this guide on tmux](https://hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/) for usage tips.
   - To restart the buggy stack, if needed:  
     ```bash
     sudo systemctl restart buggy
     ```
3. Open **Foxglove** and connect locally to:
   ```ws://192.168.1.217/8765```
<br>

#### To launch **NAND**:
1. Connect to the Wi-Fi named **NAND**.  
2. In the command line window:
- SSH to the computer on NAND:  
  ```bash
  ssh nuc@192.168.10.191
  ```
3. _Rest of the Terminal and Foxglove steps are the same as Short Circuit._
<br>

#### To shut down the buggy:
1. Stop the buggy stack
   ```bash
   sudo systemctl stop buggy
   ```
2.	Shutdown the ShortCircuit/NAND computer
   ```bash
   sudo shutdown now
   ```
