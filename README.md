# pmr_tp_final

## Running the simulation

This starts CrazySim, Crazyswarm2, and one controller node per configured
Crazyflie.

The easiest way to run the stack is with the helper script:

```bash
./pmr all
```

This starts the `crazysim` and `package-humble` containers, launches the
CrazySim SITL/Gazebo simulator and Crazyswarm2 in the background, sends a
takeoff command, waits briefly, then starts `pmr_tp_final` controllers in the
foreground. Press `Ctrl-C` to stop the foreground controllers and clean up the
Compose stack. If the controllers exit on their own, the helper also cleans up
the stack.

Useful helper commands:

```bash
./pmr up                 # start containers
./pmr rebuild            # rebuild images and recreate containers
./pmr sim-start          # start CrazySim SITL/Gazebo
./pmr swarm-start cflib  # start Crazyswarm2
./pmr takeoff            # command all Crazyflies to take off
./pmr controllers        # start controller.launch.py
./pmr shell sim          # shell into the CrazySim container
./pmr shell pkg          # shell into the controller container
./pmr down               # stop the stack
```

You can pass ROS launch arguments to the controller launch file, for example:

```bash
./pmr controllers total_robots:=4 robot_prefix:=cf_
```

By default, the launch file starts 4 controller nodes for `cf_0` through `cf_3`.

### Manual commands

If you prefer to run the steps manually, start the CrazySim container:

```bash
docker compose --profile full-sim up -d crazysim
```

Inside the `crazysim` container, start the simulated firmware:

```bash
docker exec -it crazysim bash
cd /CrazySim/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_text.sh -f agents.txt
```

In another terminal, start Crazyswarm2:

```bash
docker exec -it crazysim bash
ros2 launch crazyflie launch.py backend:=cflib gui:=False
```

Then start the controller container and launch one controller per robot:

```bash
docker compose --profile full-sim up -d package-humble
docker exec -it pmr_tp_final bash
ros2 launch pmr_tp_final controller.launch.py
```
