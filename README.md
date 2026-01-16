# Installation
## AutoDRIVE Simulator
### MacOS

The macOS installation guide is a bit lengthy, so please refer to [the macOS installation guide](macos.md).

## Windows

First, download the [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/releases/download/Simulator-0.3.0/AutoDRIVE_Simulator_Windows.zip).

Then, double-click on the executable file. Should be open :D

## Linux

First, download [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/releases/download/Simulator-0.3.0/AutoDRIVE_Simulator_Linux.zip).

Then, double-click on the executable file. Should be open :D

## Connecting to the simulator

### Installing uv

First, install uv, the Python package manager used by the UCI team.

#### Macos and Linux

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Then close and reopen your terminal.

#### Windows

```bash
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

Then close and reopen your terminal.

### Installing git

#### Macos

Install command line developer tools

```
xcode-select --install
```

#### Windows

Install [git for windows](https://git-scm.com/downloads/win). The easiest way to do this is with winget, so in a terminal:

```bash
winget install --id Git.Git -e --source winget
```

### Running example_f1tenth.py

UV will automatically download the correct Python version and the required packages in a virtual environment managed by UV. Therefore, please refrain from using pip or venv on top of UV. Now you are ready to run example_f1tenth.py!

```bash
git clone https://github.com/uci-f1tenth/UCI_AutoDRIVE_labs
cd UCI_AutoDRIVE_labs
uv run src/example/example_f1tenth.py
```

### Connecting to AutoDRIVE

While `example_f1tenth.py` is running, open the AutoDRIVE application. Then click on "Manual" to switch it to Autonomous mode, and then click on "Disconnected" to connect.

![AutoDRIVE buttons](images/AutoDRIVE_buttons.png "AutoDRIVE buttons")

Your car should now be driving forward automatically!

## ROS Setup

```bash
source autodrive_f1tenth/install/setup.bash && source /opt/ros/kilted/setup.bash
cd autodrive_f1tenth && colcon build && cd ..
ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py
```

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=autodrive_online_async.yaml
```