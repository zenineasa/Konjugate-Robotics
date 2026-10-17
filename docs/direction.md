<!-- Copyright © 2026 Zenin Easa Panthakkalakath -->

# Direction

Where the robotics toolbox is going and why. This is a first draft written before any code: the decisions below are working answers, each with its reasoning, and they change if building it teaches us otherwise. Open questions are listed at the end.

## The idea

A CAD-like window for Konjugate in which people build a robot from solids, or bring one in, and join the solids at explicitly chosen frames, getting a Konjugate model from the first click. Each mechanism becomes a subsystem in the Konjugate graph whose ports are its joints and named frames, so the robot is not a drawing exported to a simulator: it can be wired to motors, gearboxes, batteries, controllers and everything else Konjugate already models, then driven, forked, fitted and optimised.

The first question it answers is the one everyone building a robot has: **will these motors do it?** For a given move and payload, does each joint stay within its motor's torque and speed, how hot do the windings get, and how long does the battery last?

## What makes it different

1. **One graph from mechanism to digital twin.** CAD tools have geometry and parameters but no dynamics. Robot simulators have dynamics but no parameters and no link to the system around the robot. Here the mechanism, its electrical drive, its heat, its control and the business system it serves (a fleet of warehouse robots inside a logistics network, say) are one model.
2. **The whole robot sized at once.** Vendor sizing tools mostly look at one axis at a time. Here the joints are coupled, and each motor is checked against its torque-speed curve, its winding temperature (from the thermal templates core already has) and the shared battery over a whole duty cycle.
3. **Design in either direction.** Start from the motion (place joints, axes and ranges, simulate a stick figure, then grow solids around it), start from the parts (model solids, or import a robot or its parts, then pick joint frames on them), or mix the two. Both edit the same model. Later, a third direction: start from a goal (reach this point, lift this mass this fast) and let Konjugate propose the joints and sizes.
4. **Parametric parts with physics attached.** Link lengths, sections, motor and gearbox choices and controller gains are Konjugate parameters. Changing one changes the geometry, the mass properties and the dynamics together, and catalog parts bring their torque curves and limits with them.
5. **Drive it, fork it, fit it, optimise it.** Everything Konjugate already does applies to a robot: live sliders as a jog panel, schedules as trajectories, forks for what-if failures, Tuning to fit a real robot's logs, and an optimiser over the design and its controller together.
6. **Open and cross-platform,** for novices, regular users and power users alike, on Windows, Linux and macOS.

## The rule

As in logistics: everything that can be made from public robot descriptions and public data goes in before we approach anyone. Many real robots are already described in MJCF or URDF (MuJoCo Menagerie, the `robot_descriptions` collection, the SO-100 and Koch arms of LeRobot), and LeRobot publishes recorded joint data from real arms. A user should see a robot they know moving, wired to real motors, within the first minute.

The toolbox ships no robot and no dataset of anyone else's. Each is fetched when a user asks for it and kept on that user's device (see [Robots and data from public sources](#robots-and-data-from-public-sources)), so the toolbox redistributes nothing and its licence is unaffected by theirs. A project built from a robot carries it, with its licence, as a project built from a map carries its roads. Licences vary: Menagerie's differ by model (some meshes come from manufacturers) and each LeRobot dataset carries its own; in October 2026 every Menagerie model is under Apache-2.0, BSD or MIT, the SO-ARM100 under Apache-2.0, the UR5e under BSD-3-Clause, and LeRobot's SO-100 datasets under Apache-2.0.

## What Konjugate already has

Found while reading the core repository (October 2026):

- An Electron app with a three.js canvas, a C++ engine (Eigen, NLopt, METIS) and a WebAssembly web edition.
- STEP and STL import through `occt-import-js` (read and tessellate only, not a modelling kernel) and a shape library with robotics parts: articulated link, servo, parallel gripper, drive wheel, propeller.
- Component library templates for motors, batteries and thermal masses.
- Robotics examples: two-link arm, articulated arm, four-bar linkage, single joint, position-controlled joint, battery-powered joint, wheeled drivetrain. They say plainly what they cannot do: the arm's joints are uncoupled, and the four-bar holds its rods with stiff springs (2 to 3 percent length error, a 0.002 s step) because there is no constraint solver.
- The engine integrates with snapshot-coupled explicit Euler. Providers (C++ and Python) are scalar-in and scalar-out; computational-node providers have several outputs, their own checkpoint and restore, and run in process for C++, but are inline only, not packaged as plugins.
- Launcher add-ons: a window with an importer that builds a model (`runImport`), puts it in the canvas (`openInCanvas`, replacing the model), and keeps its own state with the project (`keepSession`, `restoreSession`). The logistics and fintech toolboxes work this way.
- Live parameters (sliders, programmatic control), shared parameters linked across many relationships, stored schedules (linear or hold), scenario forks with step, ramp, pulse and path interventions, and parameter fitting from CSV through NLopt.
- The Pose Visualizer add-on (its own repository, MPL-2.0), a result visualizer that plays a result's bodies in three.js, in step with the timeline and live while a run goes. A node is a body when its states match a naming convention (by default Konjugate's own `x`, `y`, `z`, `roll`, `pitch`, `yaw`, as in the Free body template), with as few as one degree of freedom; users add conventions or map one node by hand. It draws each node's shape or imported mesh (the canvas's mesh document: flat positions, normals and indices) with its frame axes, labels lines between bodies by the relationship's name rather than claiming a physical connection, and shares the editor's view cube. Two limits matter here: every shape is scaled to the same on-screen size and centred on its own middle, so a robot's links would not look like the robot; and orientation is three Euler angles only, which lock at a pitch of 90 degrees and wrap at 180.

## What a user does

1. **Start.** From a known robot (MJCF or URDF, from a file or fetched from Menagerie on request), from an example, from a blank skeleton, or from parts (STEP, STL or the catalog).
2. **Solids.** Primitives (box, cylinder, sphere, extrusion, revolve) and booleans, parametric robot templates (link between two joints, bracket, motor mount, servo horn, wheel, gripper finger, gear) and imported parts. Each solid carries its material, mass and inertia, computed from the geometry or entered.
3. **Joints, at frames chosen explicitly.** Fixed, revolute, prismatic, spherical and later planar and helical. A joint is a point and an axis on a frame of each solid, with limits on range, speed and effort. Frames snap to features (a face centre, a hole axis, an edge midpoint) but belong to the solid, so a joint survives when the geometry around it changes.
4. **Drive it.** Motors and gearboxes from the component library (or the catalog) wired to each actuated joint's port, with supplies and controllers in the canvas. Joint setpoints, gripper state and payload are live parameters: sliders while it runs, and keyframed poses saved as schedules for a pick-and-place cycle.
5. **Size it.** Each joint's operating points over its motor's torque-speed envelope, winding temperatures, the battery's state of charge over a shift, and which joint gives out first. Forks answer the follow-ups: a heavier payload at 5 s, a smaller gearbox, a worn joint, a stuck gripper.
6. **Simulate and compare.** Coupled multibody dynamics with gravity, closed loops and contact, seen moving in 3D. Two designs side by side.
7. **Fit it to a real robot.** Import joint logs as CSV (a user's own or a public LeRobot dataset) and fit friction, inertia or motor constants with Tuning.
8. **Optimise the design.** Choose parameters and bounds (link lengths, gear ratios, controller gains), an objective (cycle time, energy, mass, peak torque) and constraints (torque within the motor's limit, temperature below the winding's rating, the target reached), and compare the candidates as branches, or as a Pareto front when there are two objectives. Discrete choices (which motor from the catalog) run as one optimisation per choice.
9. **Hand it to a larger model.** A larger model (a warehouse in the logistics toolbox) would be too slow with full dynamics for every robot, so it gets a reduced robot: a table per task, measured by running the detailed robot across a grid of payloads (and distances, for a mobile robot), giving cycle time, energy and peak current at each point. A robot template in the larger model interpolates the table; its parameters (payload, task) stay live, so forks still work. Each table records the design and run it came from, and the reduced robot says it is one. A fitted dynamic model comes only if a table proves too coarse.
10. **Export.** MJCF and URDF for ROS, Gazebo, MuJoCo and Isaac users; STEP of the modelled parts.

## How it is built

The same shape as the logistics toolbox:

- `packages/engine`: a plugin with the component library (joint and frame nodes, servo, gearbox, sensors, robot-specific motor variants) and the mechanism provider.
- `packages/toolbox`: a launcher window with the 3D design view.

### The window owns the design, the model is built from it

The design (solids, frames, joints, parameters, keyframes) lives in the window's session, kept with the project. Each change rebuilds the model through the importer and puts it in the canvas, as Build does in logistics.

The window is CAD-like, but built for robots: it does not try to match FreeCAD, Onshape or Fusion feature for feature, and the model around the mechanism is what sets it apart. Geometry uses OpenCascade compiled to WebAssembly (opencascade.js) for B-rep modelling and STEP, Manifold where fast mesh booleans are enough, and three.js for display, reusing what Pose Visualizer and the canvas already do. OpenCascade is large (tens of megabytes), so it loads when the user first models or edits a solid, not when the window opens; importing a robot and driving it need only the meshes.

### A mechanism is one subsystem with ports

Each mechanism appears in the canvas as one subsystem. What people wire to are its joints (torque or setpoint in; angle, speed and servo current out) and its named frames (end effector, payload, sensor mount: pose and force). The solids sit inside it, so their poses and contact forces are result signals like any other.

Core has no declared ports today: a subsystem's ports are derived from the relationships that cross its boundary, and subsystems are flattened before the engine runs. So each joint and each named frame is a real node inside the subsystem, holding its states, with a stable key from the importer (`joint:<name>`, `frame:<name>`). The user's edges end on those nodes. A rebuild keeps every node whose key is still produced, with its id, so those edges survive; a renamed or deleted joint loses them, and the window warns before it builds. This depends on the owner keys of Part 1 of the core proposal. The subsystem is also the natural boundary for the group provider core needs to add (see [Needs from core](#needs-from-core)).

### Seeing it move: Pose Visualizer, not a second player

Playing a result in 3D and scrubbing through it is Pose Visualizer's job, so the toolbox does not build its own player. Each solid node carries Konjugate's pose states, so any robotics result plays in Pose Visualizer with nothing set up, even when the project is opened by someone without the toolbox. The window keeps to design (and to showing the jog panel's live pose, if the launcher can read live results). This needs from Pose Visualizer (see [Needs from Pose Visualizer](#needs-from-pose-visualizer)):

- **True scale.** A node whose mesh is in metres, marked so, is drawn at its real size about its own origin (the joint frame, not the mesh's middle). Today's normalised size stays the default for every other model.
- **Quaternions.** MuJoCo works in quaternions; Euler angles lock and wrap, which breaks playback between samples and Tuning across a wrap. Solid nodes carry `qw`, `qx`, `qy`, `qz` as well as `roll`, `pitch`, `yaw` (unwrapped, for reading). Pose Visualizer never guesses which to use: by default it turns them by the three angles, as Konjugate's own convention does, so a robot plays with nothing set up; a user who wants the quaternion ticks a convention that says so, under state names of their choosing.
- **Named frames** drawn as small axes on their solid, and contact forces as arrows, both optional.

The window reuses Pose Visualizer's view cube and camera (`viewCamera.mjs`) and its pose model (`poseModel.mjs`), so the two views move the same way. Shared files stay MPL-2.0 inside the AGPL toolbox, as MPL allows; changes to them go to Pose Visualizer first.

### The dynamics run in MuJoCo, inside one provider

A robot cannot be one ordinary edge per joint: the joints are coupled, closed loops need constraints, and explicit Euler on stiff joints is unstable. So the toolbox compiles a mechanism into a single group provider that embeds MuJoCo (DeepMind's C physics engine, Apache-2.0) and exchanges values with the rest of the graph once per engine step, much as an FMU does in co-simulation. MuJoCo is a library here, as Eigen and NLopt are to the core engine: users never see it as a separate program, and its own viewer is not used.

Each engine step, the provider:

1. reads its inputs from the graph: for each joint, a torque (torque mode) or a setpoint (servo mode, below), and external forces on named frames;
2. reads its parameters (payload mass, friction, joint damping, link offsets, servo gains and current limits) and writes those that changed into MuJoCo's model, then calls `mj_setConst` when a mass, inertia or body position changed, so derived quantities (subtree masses, reference inertias) do not go stale. Live sliders, schedules, fork interventions, Tuning and the optimiser all reach the physics this way;
3. steps MuJoCo with its own substeps (around 1 ms), holding torque inputs over the engine step and running each servo's inner loop at every substep;
4. writes back joint angles and speeds (which feed motor back-EMF and sensors), each servo's current, body poses (for the 3D view), contact forces and sensor readings as Konjugate states;
5. checkpoints and restores MuJoCo's full state, servo integrators included, so forks and restarts work as for any other node.

### Time steps and the two joint modes

The engine's explicit Euler needs a step well below the fastest time constant it integrates. A real small motor's electrical time constant L/R is around 1 ms (the core templates default to 0.5 H and 2 ohm, 250 ms, which hides this), and a real servo loop runs at around 1 kHz. A stiff PD controller held for 10 ms behaves differently from one running at 1 kHz, or goes unstable. So nothing fast is left to the engine step:

- **Servo mode** (the default for hobby and smart servos, such as the SO-100's Feetech STS3215, which close their own position loop). The servo's inner position loop runs inside the provider at MuJoCo's substep: built as the model's own position actuator on the joint (Menagerie's SO-ARM100 has one per joint, a gain of 50 N·m/rad limited to 3.5 N·m), which MuJoCo runs at every one of its steps, so its gain and torque limit are the model's for now; making them parameters, and a speed and current limit, come later. Konjugate sends setpoints at the engine step, as the real servo's bus does, and reads back the current it draws, which feeds the battery and the thermal model. The motor is still modelled once: its loop and mechanics in the provider, its losses and heat in Konjugate.
- **Torque mode** (for a motor driven by a controller the user builds in the canvas). A joint motor (`packages/engine/components/jointMotor.json`, wired by the joint drive bundle) computes the torque from its current, and the mechanism reads it. The motor keeps its inductance and runs in substeps of its own, a tenth of its electrical time constant each (24 of a 1 ms step for a winding of 0.42 ms), while the mechanism and the rest of the model keep the engine step: core runs each node at its own rate. Tried and measured (`tests/engine/arm.test.mjs`): with one substep, a winding of 1 ms on a 1 ms step overstates the peak current by a third, and one of 0.42 ms diverges. A quasi-static current (i = (V - k_e w) / R) would need a source term reading the joint's speed on another node, which core does not allow, and drops the inductance; it stays an option if substeps prove too slow. A controller in the canvas runs at the engine step, so the window suggests that step from the controller's gains.

The engine step is a choice the window makes from these (around 5 to 10 ms in servo mode), and the run report states it.

Why MuJoCo rather than Pinocchio: MuJoCo brings contact (grippers, wheels, feet), closed loops through equality constraints (the four-bar comes with the first milestone, not a later one) and MJCF natively, and much of the robotics community already uses it. Pinocchio's advantage, analytic derivatives, goes unused while design optimisation runs a whole simulation per evaluation with finite-difference gradients. Pinocchio can come later as a second provider if gradient-based optimisation through the dynamics is ever needed.

### Robots and data from public sources

Fetched on request, one model or dataset at a time, and kept on the user's device, as logistics fetches and keeps map extracts:

- **Licence before download.** The import lists each model with its licence and source, and shows them before fetching; one that is not freely usable (none in Menagerie today) is marked as such.
- **Pinned.** A download is of one commit of Menagerie (or one revision of a dataset), recorded, so a project runs the same after an upstream edit.
- **Kept on the device, carried by the project, as logistics does with maps.** The download goes to Konjugate's store of downloaded files (core's `storedFiles`, with where each came from), fetched once, used by every project built from it, and working offline after. The importer reads it from there (core's `binaryInputs`), as logistics' reads a map extract, and the project keeps what a run needs: the model's MJCF and the meshes, included files and textures it reads (1.5 MB in a saved project for the SO-ARM100), with its licence's text and its provenance (source, repository, folder, commit) beside them. So a project runs wherever it is opened, from the command line too, and forks and resumes with nothing to fetch; sharing it shares the robot with its notice, which every Menagerie licence allows. The readme, the changelog and Menagerie's pictures stay out.
- **Our own material is our responsibility.** An example, showcase project, thumbnail or video built from someone's robot is distributed by us: it fetches its robot when opened, or uses a model we are content to ship with its notices.
- **Names, not logos.** A model is named as its maker names it, to identify it; no logos or others' images, and previews are drawn by the toolbox.
- **Hosts.** `raw.githubusercontent.com` and `api.github.com` for Menagerie, `huggingface.co` for datasets, listed in the add-on's network hosts.

Nothing new from core: logistics already keeps map files this way. The importer runs in JavaScript, so it reads a model's joints and servos without MuJoCo's Python (the describer in `packages/engine/tools` is for scripts): from the MJCF itself, or MuJoCo's WebAssembly build, still to choose.

### Rules

- **Solids speak Konjugate's pose vocabulary.** Positions `x`, `y`, `z` in metres with z up, as the Free body template and Pose Visualizer expect, plus a quaternion; MuJoCo's own frames are converted at the provider's edge, never in the canvas.
- **Joints attach to frames, not to faces.** A frame belongs to its solid and snaps to features; a face may vanish when a part is edited, a frame does not.
- **Every dimension says which side drives it.** Driven by the joint (moving the joint stretches the link) or by the part (editing the part moves the joint). A lock per relationship, so the two never fight.
- **The provider reads parameters, never baked numbers.** Mass, inertia, link lengths and joint frames reach the provider as parameters, read every step, usually shared ones. Otherwise a live payload slider, a fitted inertia or an optimiser would change nothing. The window computes them from the geometry and writes them as parameters.
- **Template parts write their mass properties as equations of their own parameters** (a beam link: m = rho L w h, I = m L^2 / 12), so the optimiser can change a design without going back to the CAD kernel. The window regenerates the shape afterwards. Imported parts keep fixed mass properties.
- **Geometry changed during a run updates the dynamics at once and the shape at the next build;** the window shows when the two differ. Structural changes (adding a joint, a new mesh) always wait for the next build, as MuJoCo recompiles its model.
- **The motor's mechanical side lives in MuJoCo, its electrical side in Konjugate.** Rotor and reflected gearbox inertia go into MuJoCo's `armature` and damping, so the stiff loop between a light joint and its motor's back-EMF does not cross the engine step. MuJoCo's own actuators are plain torque inputs (servo mode applies its torque through the same input from inside the provider), so the motor is never modelled twice.
- **Contact is soft and says so.** MuJoCo's contact is tunable and needs small substeps (around 1 to 2 ms); the UI states this where contact matters.
- **Imported parts are editable, not parametric.** Push and pull faces, resize holes, stretch between two planes. We say so in the UI.
- The conventions of the other toolboxes: every action has a button or menu item, a gesture and a key, works on the selection and can be undone, on every platform; every input says where it came from; limits and bugs are stated plainly.

## Milestones

Each one ends in something that can be shown.

- **A. Mechanism provider on MuJoCo, fed by import.** MJCF and URDF import into a mechanism subsystem with joint and frame nodes, in servo mode and in torque mode with the existing motor templates. Compared against an analytic pendulum, against the two-link arm, and against MuJoCo running the same model on its own; the four-bar linkage again, now exact. Shown on an SO-100 or UR5 from Menagerie, playing at true scale in Pose Visualizer. Starts as an inline computational-node provider while core builds group providers.
- **B. Will these motors do it?** Motor, gearbox, winding temperature and battery per joint; a jog panel of live setpoints; keyframes saved as schedules; the torque-speed envelope view; a pick-and-place example with forks for a heavier payload and a weaker motor.
- **C. The CAD-like window.** Primitives and booleans, STEP and STL import, parametric robot templates, frames, the joint picker, joint-first skeleton design, Build into the canvas, the robot moving in 3D.
- **D. Fit to a real robot.** Tuning against a public LeRobot dataset of an SO-100 arm, then against a user's own logs.
- **E. Design optimisation.** Objectives and constraints over link lengths, gear ratios and controller gains; motor choice from a catalog of motors, gearboxes and bearings carrying their physics, in an outer loop; candidates as branches; a Pareto view.
- **F. Contact.** A gripper picking a box; a wheeled robot on the floor.
- **G. The joined demo.** A fleet of warehouse robots (autonomous mobile robots, built from the wheeled drivetrain) inside the logistics toolbox's warehouse model: battery, charging schedule, fleet size, cycle time and cost per pick in one model, with each robot reduced from its detailed model to a table (see What a user does, step 9).
- **H. Digital twin.** Live inputs from a joystick or a ROS topic.
- **I. Exchange.** MJCF and URDF export; STEP export of modelled parts.
- **Later.** Pinocchio as a second provider.

## Moving robotics out of core

Once the mechanism provider replaces them, six of the seven robotics examples move here, with the springs-versus-exact four-bar as a before and after. The position-controlled joint stays in core as a teaching example, since the web edition cannot install add-ons and welcome content or videos may point at it; before moving the others, check what links to them. The robotics shapes (articulated link, servo, parallel gripper, drive wheel, propeller) move with them. The generic motor, battery and thermal templates stay in core, as other toolboxes and examples use them. Pose Visualizer stays its own add-on, as it serves models not built in the toolbox, and becomes the toolbox's player (see [Seeing it move](#seeing-it-move-pose-visualizer-not-a-second-player)).

## Needs from core

Written up as a core proposal: [Toolbox mechanisms, model merging and design optimisation](https://github.com/zenineasa/Konjugate/blob/main/docs/proposals/toolboxMechanismsAndDesignOptimisation.md). In short:

1. A toolbox rebuild that updates only what it made, leaving what the user added by hand (a controller, a battery) in place. With a mechanism as one subsystem, this is mostly replacing the subsystem's inside while keeping, by their stable keys, the joint and frame nodes the user's edges end on. Later, reading the canvas model and hearing its changes, for editing from the canvas back to the window.
2. A provider that owns a group of nodes and edges (a mechanism subsystem) and is packaged in a plugin, with MuJoCo linked in.
3. Optimisation towards a design objective with constraints, not only fitting to data.

Each toolbox release needs a Konjugate release with what it requires, so core goes first.

### Where milestone A stands

Until group providers exist, milestone A runs on what core has (October 2026): a Python computational-node provider (`packages/engine/providers/mechanism.py`, MuJoCo through its Python package) on one mechanism node, with each joint's position and speed as its algebraic (`setsValue`) outputs. Each call is a co-simulation step, as an imported FMU's is: called at t with step h, it advances MuJoCo over [t, t + h] on torques read at t and reports the joints at t + h. A joint's torque comes from a state of the mechanism's own, set from a shared parameter, or from another node's state, such as a motor's: core now lets a node provider read other nodes' states through the step's synchronization snapshot, as edges do (added for this, with the validator, the partitioned backend's data exchange and code export following it).

Joint motors drive it: a two-link arm with two motors on one battery runs as MuJoCo alone runs it with the same motor equations integrated beside it, angles, currents, winding temperatures and charge alike, including a small motor in substeps. When MuJoCo finds the mechanism unstable it resets itself to its initial pose and carries on; the provider stops the run instead, saying why.

Shared parameters reach MuJoCo too (core lets a node provider read them, added for this): a body's mass (its inertia scaled with it) and a joint's damping, friction loss and armature, applied before a step whenever they change, so a live slider, a stored schedule, a fork's intervention and fitting reach the physics. The canvas, which used to drop a node's provider when it saved or ran a model, now keeps it and lists its parameters with the shared ones.

A real robot runs too: Menagerie's SO-ARM100, fetched at a pinned commit with its licence (Apache-2.0) and where it came from, into a folder on this computer standing in for Konjugate's store (`npm run fetch:model trs_so_arm100`), carried in the project with them (the provider loads the model from those files), and described by MuJoCo itself (`packages/engine/tools/describeMechanism.py`: its joints, ranges and servos, defaults and includes resolved). Driven by its own servos to setpoints from shared parameters, it runs as MuJoCo alone runs the same model file, positions and servo torques alike; each servo meets its 3.5 N·m limit at the start of a move, and the shoulder holds the arm 0.017 rad short of its setpoint at 0.85 N·m, as a gain of 50 says. A joint left free has its servo switched off. The window will read the model from Konjugate's store through its importer, and needs a way to describe a model without running Python of its own.

Still missing, and what a group provider would add: a node provider writes only its own node's states (so joints and solids are not nodes of their own, and Pose Visualizer has no solids to show), and is inline only, not packaged in a plugin. A run needs the Python with MuJoCo as the providers' interpreter, and core's Python SDK path. `npm run test:engine` runs the provider through Konjugate's engine and checks it against MuJoCo alone, the exact pendulum period, both ways of applying a torque, resuming from a checkpoint, the motor-driven arm, a damping and a scheduled payload mass reaching the physics, and the SO-ARM100 on its servos (fetched when not already on this computer, and skipped, saying so, when it cannot be).

## Needs from Pose Visualizer

Small and useful to any model with real geometry, not only robots: true scale for meshes marked as in metres, drawn about their own origin; conventions and mappings that turn bodies by a quaternion; optional frame axes and contact force arrows. Milestone A needs the first two. Quaternions are built (October 2026, not yet released): each convention, and each node's own mapping, chooses roll, pitch and yaw or a quaternion, with state names the user picks. Pose Visualizer releases before the toolbox release that relies on them.

## Out of scope

Topology optimisation and finite element analysis; drawings and full mechanical drafting; soft bodies and fluids around the robot; motion planning beyond keyframes and schedules (planners can come through providers later).

## Open questions

- **Licence:** AGPLv3 only (as logistics) or AGPLv3 with a commercial licence (as fintech)? MuJoCo (Apache-2.0) and OpenCascade (LGPL-2.1 with an exception) are compatible either way, as is Pinocchio (BSD-2) if it comes later.
- **First user:** students and hobbyists (the SO-100 and LeRobot community: public robots and data, quick to reach), robotics start-ups sizing actuators, or the warehouse customers of the logistics toolbox (mobile robot fleets)? The working answer is hobbyists and start-ups for milestones A to E, as they need no sales cycle, and warehouse fleets for G. It decides the order after B.
- **Web edition:** add-ons do not run there today. WebAssembly builds of MuJoCo exist, so the provider itself would not stand in the way; does the robotics toolbox need it, for students in particular?
- **Core proposal:** it still lists Pinocchio or MuJoCo and a node per body; it should be updated to match this document (MuJoCo, mechanism subsystems with joint and frame nodes under stable keys, servo mode).
