# pmr_tp_final

## Running the simulation

This starts CrazySim, Crazyswarm2, and one controller node per configured
Crazyflie.

### 1. Start the CrazySim container

```bash
docker compose up -d crazysim
```

Inside the `crazysim` container, start the simulated firmware:

```bash
docker exec -it crazysim bash
cd crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_text.sh -f agents.txt
```

In another terminal, start Crazyswarm2:

```bash
docker exec -it crazysim bash
ros2 launch crazyflie launch.py backend:=cflib gui:=False
```

### 2. Start the controller container

```bash
docker compose up -d package-humble
```

Inside the `pmr_tp_final` container, launch one controller per robot:

```bash
docker exec -it pmr_tp_final bash
ros2 launch pmr_tp_final controller.launch.py
```

By default, the launch file starts 4 controller nodes for `cf_0` through `cf_3`.
