# Konjugate Robotics Toolbox

A robotics extension for [Konjugate](https://github.com/zenineasa/Konjugate), the open-source, graph-native simulation engine. A robot's mechanism runs in [MuJoCo](https://mujoco.org), embedded in a Konjugate model, so the same model can hold the motors that drive it, the battery that feeds them, the heat in their windings and anything else Konjugate models, and be driven, forked, fitted and optimised like any other.

## Status

Early development. Where it is going and why: [docs/direction.md](docs/direction.md). What exists today:

- **Mechanisms in MuJoCo, inside Konjugate.** A Python computational-node provider (`packages/engine/providers/mechanism.py`) runs a MuJoCo model as one node of a Konjugate model, a co-simulation step per engine step. Each joint is driven by a torque, by the model's own servo to a setpoint, or left free; torques and setpoints come from shared parameters (live sliders) or from other nodes, such as a motor's. A body's mass and a joint's damping, friction and armature follow shared parameters, so a slider, a schedule, a fork or a fit reaches the physics. Runs match MuJoCo run on its own.
- **A component library plugin**, `konjugate.robotics.engine`: a **joint motor** (a DC motor and gearbox: current, torque at the joint, winding temperature) and a **joint drive** bundle wiring it to a battery and its joint's speed. A motor faster than the step runs in substeps of its own.
- **The Robotics Toolbox** add-on, `konjugate.robotics.toolbox`: a window that lists robots from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), shows a robot's licence before anything is downloaded, keeps the download on this computer, reads its joints and servos, and builds a mechanism from them into the canvas. The project carries the robot with its licence and where it came from. No robot is shipped with the toolbox.
- **Will these servos do it?** For the SO-ARM100, the window builds its pick-and-place move as its servos' setpoint schedules, holds its servos to their datasheet (current for torque, and the rated torque at stall in place of the model's own limit), adds a battery and each servo's winding, and puts a payload in its gripper as a slider: which servo reaches its limit, how hot it gets and how much charge the move takes, empty or loaded. A Compare step then forks the run from a chosen time, with a heavier payload or one servo held to a lower torque, and shows the baseline and the fork side by side, servo by servo, before opening the fork in the canvas.
- **Seeing it move.** A built robot reports each link's place in the world and carries each link's shape, so [Pose Visualizer](https://github.com/zenineasa/Konjugate-PoseVisualizer) shows it moving at its true size, once it is given a naming convention for poses by quaternion (x, y, z, qw, qx, qy, qz) under *Naming conventions*.

What it does not do yet: links as nodes of their own to wire to (until Konjugate has group providers); servo currents for the battery and heat; every action with a key and an undo, as the other toolboxes have. See [docs/direction.md](docs/direction.md), "Where milestone A stands".

## Running a robot

Konjugate runs the mechanism provider with the Python it is given for providers, which needs MuJoCo:

```
pip install mujoco
```

Then choose that Python under **Provider Toolchains** (⚒ in the titlebar). The window checks that Python for MuJoCo when it opens and before each comparison, and says how to install it into that very Python when it is missing (with a Konjugate that has the `pythonCheck` launcher feature; an older one only says MuJoCo is needed). Open the Robotics Toolbox from the toolstrip, choose a robot, read its licence, download it, set each joint's mode and build it into the canvas. Its servos' setpoints are live sliders there.

## Building from source

This repository builds against a sibling checkout of Konjugate, cloned side by side (`KONJUGATE_DIR` overrides that), whose engine is built:

```
npm run setup:python     # .venv with MuJoCo, for the tests (needs uv)
npm run build            # out/konjugate.robotics.engine-<version>.kjp and out/konjugate.robotics.toolbox-<version>.kja
npm run install:dev      # builds and installs both into your local Konjugate
npm run fetch:model trs_so_arm100 -- --commit 62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35   # a robot, for the tests
npm test                 # unit tests
npm run test:engine      # mechanisms through Konjugate's engine, against MuJoCo alone
npm run test:interaction # the window in the real app, offline
```

The tests use the robots fetched to `out/models/` (the SO-ARM100, UR5e, Panda and Go2 at the commit above); a test whose robot is not there is skipped and says so.

## Licence

[AGPL-3.0](LICENSE). The robots the toolbox fetches carry their own licences, shown before download and kept with every project built from them. MuJoCo, which runs the robots, is the user's own install (Apache-2.0), not shipped with the toolbox.
