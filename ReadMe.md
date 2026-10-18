# Konjugate Robotics Toolbox

Draw a robot, and get a [Konjugate](https://github.com/zenineasa/Konjugate) model of it. A simple CAD-like window: place solids, click a point on one solid and a point on another to join them, say how they join and what drives each joint, and press Build. In the canvas, **each solid is a node and each joint is an edge between two solids**, so the robot can be wired to motors, batteries, controllers and anything else Konjugate models, then run, driven with sliders, forked, fitted and optimised. The robot moves by its joints' own equations (Featherstone's articulated-body method, split along the joints), which Konjugate's engine runs like any other: nothing outside the graph solves it, and every equation can be read on its edge.

## Status

Early development; where it is going and why: [docs/direction.md](docs/direction.md). What works today:

- **The Robotics window.** Boxes, cylinders, spheres and capsules, sized, placed and turned by typing or by handles in the 3D view, each of a material (its mass from its density) or a mass given. The Join tool: click a point on the first solid (or the floor, for the Ground) and a point on the second; the second turns and moves so the two points meet, faces together (Shift-click leaves it where it is). Points snap to face centres and edge middles. Hinge, slider, ball and fixed joints, with limits, damping, friction and armature, and a drive: free, a torque, or a position servo with its setpoint, stiffness and most torque. Checks say what stops a build and what to know, each pointing at its part. Undo and redo for everything; every action by button, by gesture and by key, on Windows, Linux and macOS. Three examples to start from: a pendulum, a two-link arm on a waist, a cart carrying two pendulums. The drawing is kept with the project.
- **Build into the canvas.** The Ground, a node per solid (its joint's speed, position, acceleration and torque; its pose as `x`, `y`, `z`, `roll`, `pitch`, `yaw` and a rotation matrix; its energy; its shape at true size) and an edge per joint carrying the joint's equations, with its settings as parameters (the drive's torque or setpoint a live slider). Each solid's mass is a shared parameter. Pose Visualizer shows the robot moving.
- **Checked** against an independent Newton-Euler solution of the same drawing, agreeing to within 1e-9, and against MuJoCo run on its own where it is installed.
- **Not yet:** ball joints, closed loops (a four-bar linkage) and solids joined to nothing can be drawn but not built, and Build says so; contact; motors and batteries wired from the window; imported parts (STL, STEP, MJCF, URDF); keyframed moves. See the milestones in [docs/direction.md](docs/direction.md).

## Running a robot

Build, then run the model in the canvas. Nothing else is needed: no Python, no simulator. The run's step is set in the window (a millisecond by default); stiff servos on light solids need a shorter one.

## Building from source

This repository builds against a sibling checkout of Konjugate (`KONJUGATE_DIR` overrides that), with its engine built for the engine tests:

```
npm run build            # out/konjugate.robotics.toolbox-<version>.kja
npm run install:dev      # builds and installs it into your local Konjugate
npm test                 # unit tests
npm run test:window      # the window in a plain browser, as a Windows or Linux user and as a Mac user
npm run test:engine      # drawn robots through Konjugate's engine, against a Newton-Euler reference
npm run setup:python     # optional: .venv with MuJoCo (needs uv), so test:engine also compares with MuJoCo alone
```

## Licence

[AGPL-3.0](LICENSE). MuJoCo (Apache-2.0) is used only by the tests, from the developer's own install, and is not shipped. three.js (MIT) is vendored in `packages/toolbox/vendor/` with its licence.
