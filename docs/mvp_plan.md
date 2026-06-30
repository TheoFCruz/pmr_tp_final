# MVP Plan: VO-CBF Crazyflie Controller

Goal: implement a first working version of the algorithm from **Multi-Agent Obstacle Avoidance using Velocity Obstacles and Control Barrier Functions** using Crazyswarm2 and CrazySim.

The MVP should prioritize a complete closed-loop simulation before implementing every term from the paper.

## Target MVP

Four Crazyflies run in CrazySim. Each drone has its own pseudo-decentralized controller node. Each controller:

1. Reads its own state and neighbor states from Crazyswarm2 logging.
2. Computes a reference acceleration toward a goal.
3. Solves a local safety-CBF QP for planar acceleration `[ax, ay]`.
4. Sends the acceleration through `cmd_full_state` using `AccelerationCommander`.

Success condition: drones move toward their goals while avoiding inter-agent collisions in simulation.

---

## Milestone 1: State and Command Interface

Status: partially implemented.

### Existing pieces

- `config/crazyflies.yaml` logs:
  - `stateEstimate.x`
  - `stateEstimate.y`
  - `stateEstimate.z`
  - `stateEstimate.vx`
  - `stateEstimate.vy`
  - `stateEstimate.vz`
- `controller.py` contains one controller node per drone.
- `commander.py` converts acceleration to `cmd_full_state` using one-step integration.

### Verify

After launching CrazySim/Crazyswarm2, check:

```bash
ros2 topic list
ros2 topic echo /cf_0/state
ros2 topic echo /cf_0/cmd_full_state
```

Expected `/cf_0/state` value order:

```python
x, y, z, vx, vy, vz = msg.values
```

---

## Milestone 2: Adapt Optimizer to Acceleration Control

The current optimizer structure should be adapted from velocity commands to acceleration commands.

### Current style

```text
decision variables: vx, vy, w
```

### MVP target

```text
decision variables: ax, ay
```

The initial QP should solve:

```text
minimize ||a - a_ref||²
subject to safety CBF constraints
           acceleration limits
```

where:

```text
a = [ax, ay]
```

### Recommended implementation

Create or adapt an optimizer interface for 2D acceleration:

```python
initialize_model(max_acceleration)
reset()
set_objective(a_ref)
add_linear_constraint(a_row, b)
solve() -> status, ax, ay
```

---

## Milestone 3: Implement Safety-CBF Constraint

Start with the safety-critical CBF from the paper before adding VO guidance.

For agents `i` and `j`, use planar states:

```text
p_i = [x_i, y_i]
v_i = [vx_i, vy_i]
p_j = [x_j, y_j]
v_j = [vx_j, vy_j]
```

Relative terms:

```text
p_ij = p_j - p_i
v_ij = v_j - v_i
d_ij = ||p_ij|| - r_i - r_j
p_hat = p_ij / ||p_ij||
nu_ij = min(0, v_ijᵀ p_hat)
```

Safety barrier:

```text
h_c,ij = d_ij - delta - nu_ij² / (2 a_max)
```

CBF condition:

```text
h_dot_c,ij + alpha_c h_c,ij >= 0
```

The implementation should output linear constraints:

```python
A_row @ a_i >= b
```

for each neighbor.

### Practical simplification for MVP

Assume neighbors keep constant velocity and only ego acceleration is controlled.

---

## Milestone 4: Reference Acceleration Toward Goals

Use a simple PD controller as the nominal acceleration:

```python
a_ref = kp * (p_goal - p_i) - kd * v_i
```

Then clip to the maximum acceleration before giving it to the QP.

Initial scenario:

- `cf_0`, `cf_1`, `cf_2`, `cf_3` start near square corners.
- Each drone's goal is the opposite corner.
- Altitude is fixed.

---

## Milestone 5: Closed-Loop Safety-CBF Simulation

Connect the full loop:

```text
/cf_i/state
    -> CbfAgentController
    -> compute a_ref
    -> compute safety constraints
    -> solve acceleration QP
    -> AccelerationCommander
    -> /cf_i/cmd_full_state
```

Run one controller instance per drone:

```text
robot_id=0, robot_prefix=cf_, total_robots=4
robot_id=1, robot_prefix=cf_, total_robots=4
robot_id=2, robot_prefix=cf_, total_robots=4
robot_id=3, robot_prefix=cf_, total_robots=4
```

MVP success criteria:

- all drones receive valid state logs;
- all controllers publish `cmd_full_state`;
- drones move toward goals;
- no inter-agent collision in the test scenario.

---

## Milestone 6: Add VO Guidance From the Paper

After the safety-CBF-only MVP works, add the relaxed VO-CBF part.

Paper objective:

```text
minimize ku ||u_i - u_ref,i||² + kvo Σ wij λ_ij²
```

with relaxed VO constraint:

```text
h_dot_vo,ij + alpha_vo h_vo,ij >= λ_ij
```

This requires:

- computing `h_vo` for each neighbor;
- computing approximate time-to-collision `T_col`;
- setting weights `w_ij = 1 / T_col`;
- adding slack variables, likely one per neighbor;
- penalizing slack in the QP objective.

---

## Suggested Implementation Order

1. Adapt optimizer to solve for `[ax, ay]`.
2. Implement safety-CBF constraint calculation.
3. Add goal/reference acceleration generation.
4. Wire QP into `CbfAgentController._compute_acceleration()`.
5. Launch one controller per drone in simulation.
6. Tune `kp`, `kd`, `a_max`, `delta`, and `alpha_c`.
7. Add VO guidance/slack objective after the safety-only loop works.

---

## Notes

- The paper's formal guarantees assume double-integrator dynamics. Crazyflies have inner-loop tracking dynamics, so use conservative safety margins.
- Keep altitude fixed for the first MVP and solve only in the horizontal plane.
- Use measured state at each control step; avoid integrating desired pose open-loop over many steps.
- Start with simulation only before moving to real hardware.
