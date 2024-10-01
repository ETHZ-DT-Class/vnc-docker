# Troubleshooting guide

REQUIRED READING: [docs/vnc-docker_README.md](./vnc-docker_README.md)

## Issues

### - Failed to set up dynamic port forwarding connection over SSH to the VS Code Server.
When connecting to the Duckiebot using VS Code SSH:
```
Failed to set up dynamic port forwarding connection over SSH to the VS Code Server. (Show log)
Surce: Remote - SSH
```
**Solution**: you probably need to check that the laptop is connected to the same network of the Duckiebot, or that the Duckiebot's name is correct.

---
---

### - ssh: Could not resolve hostname duckiebolt2.local: Name or service not known

When connecting to the Duckiebot using native SSH:
```
ssh: Could not resolve hostname duckiebolt2.local: Name or service not known
```
**Solution**: you probably need to check that the laptop is connected to the same network of the Duckiebot, or that the Duckiebot's name is correct.

---
---

### - permission denied / insufficient permissions / file is unwritable
Likely when trying to modify from outside the docker container a file/folder that was created inside the container.

**Solution**> you need to change the ownership of the file/folder. Run `make chown` to change the ownership of all the files and folders inside this `vnc-docker` repository.

---
---

### - This application failed to start because no Qt platform plugin could be initialized
When running RQT or RViz or similar:

```
Authorization required, but no authorization protocol specified
qt.qpa.xcb: could not connect to display :0 
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found.
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.

Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, xcb.
```
**Solution**: you need to run `xhost +local:` before running the container (maybe run `export DISPLAY=:0` too).

---
---

### - pyglet.canvas.xlib.NoSuchDisplayException: Cannot connect to "None"
When running the simulator:

```
Authorization required, but no authorization protocol specified
Traceback (most recent call last):
  File "/code/catkin_ws/src/simulator/run.py", line 6, in <module>
    from src.SimulatorRosBridge import SimulatorRosBridge
  File "/code/catkin_ws/src/simulator/src/SimulatorRosBridge.py", line 24, in <module>
    import pyglet.window
  File "/usr/local/lib/python3.8/dist-packages/pyglet/window/__init__.py", line 1899, in <module>
    gl._create_shadow_window()
  File "/usr/local/lib/python3.8/dist-packages/pyglet/gl/__init__.py", line 206, in _create_shadow_window
    _shadow_window = Window(width=1, height=1, visible=False)
  File "/usr/local/lib/python3.8/dist-packages/pyglet/window/xlib/__init__.py", line 173, in __init__
    super(XlibWindow, self).__init__(*args, **kwargs)
  File "/usr/local/lib/python3.8/dist-packages/pyglet/window/__init__.py", line 585, in __init__
    display = pyglet.canvas.get_display()
  File "/usr/local/lib/python3.8/dist-packages/pyglet/canvas/__init__.py", line 94, in get_display
    return Display()
  File "/usr/local/lib/python3.8/dist-packages/pyglet/canvas/xlib.py", line 123, in __init__
    raise NoSuchDisplayException('Cannot connect to "%s"' % name)
pyglet.canvas.xlib.NoSuchDisplayException: Cannot connect to "None"
```
**Solution**: you need to run `xhost +local:` before running the container (maybe run `export DISPLAY=:0` too).

---
---

### - RLException: [`LAUNCH_NAME`.launch] is neither a launch file in package [`PACKAGE_NAME`] nor is [`PACKAGE_NAME`] a launch file name
When launchiing a launch file (or running the exercise ROS node using `make ex` or `roslaunch exerciseX exerciseX.launch`):

```
RLException: [`LAUNCH_NAME`.launch] is neither a launch file in package [`PACKAGE_NAME`] nor is [`PACKAGE_NAME`] a launch file name
```

**Solution**: Build the ROS package (`catkin build PACKAGE_NAME` at whatever subfolder level inside */code/catkin_ws/*, or `catkin build --this` if you are already inside *PACKAGE_NAME*), and finally source the workspace (`source /code/catkin_ws/devel/setup.bash`).

---
---

### - fatal: unable to access https<span>://</span>github.com/XXX/YYY/: Could not resolve host: github.com
When running `make build` to build the docker image:

```
fatal: unable to access 'https://github.com/XXX/YYY/': Could not resolve host: github.com
```

**Solution**: retry, internet connection problem.

---
---

### - fatal: detected dubious ownership in repository at '/code/catkin_ws/src/user_code/exerciseX'
Sometimes when using git (unfortunately given docker permissions):

```
fatal: detected dubious ownership in repository at '/code/catkin_ws/src/user_code/exerciseX' To add an exception for this directory, call:

git config --global --add safe.directory /code/catkin_ws/src/user_code/exerciseX
```

**Solution**: just run the suggested command.

---
---

### - Black screen on VNC client or browser
The desktop environment likely went in idle mode for inactivity. Theoretically this should not happen.

**Solution**: stop and restart the container: run `make stop` and then `make run`. 

---
---


### docker: error response from daemon: could not select device driver "" with capabilities: \[\[gpu\]\]
When running the simulation inside a container which was started with the flag `--gpus all`:

```
docker: error response from daemon: could not select device driver "" with capabilities: [[gpu]]
```
**Solution**: Contact the TAs. As last resort, set to `false` the Makefile parameter [GPU_DOCKER](../Makefile#L18).

---
---

### simulation is not running on GPU

When the simulation starts, some logging information will be displayed in the terminal, including if the simulation is running on the GPU.
If the simulation is not running on the GPU even though the container is running with the flag `--gpus all`:

**Solution**: Contact the TAs.