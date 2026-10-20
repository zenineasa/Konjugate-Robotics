<!-- Copyright © 2026 Zenin Easa Panthakkalakath -->

# Direction

Where the robotics toolbox is going and why. The decisions below are working answers, each with its reasoning; they change if building it teaches us otherwise. Open questions are listed at the end.

This is the second version of this document (October 2026). The first put importing known robots and sizing their servos first and the drawing window third; what was built from it worked, but it was not the toolbox we set out to make. This version puts the drawing window first, and keeps what was built as a later way in (see [What is kept from the first version](#what-is-kept-from-the-first-version)).

## The idea

**Draw a robot, and get a Konjugate model of it.** A simple CAD-like window in which people place solids, choose explicitly where two solids join and how, and press Build. The drawing becomes part of the Konjugate graph: **each solid is a node and each joint is an edge between two solids.** It is not a drawing exported to a simulator: in the canvas the robot can be wired to motors, gearboxes, batteries, controllers and everything else Konjugate models, then driven, forked, fitted and optimised like any other model.

Simple means: a few solid shapes, sizes typed or dragged, joints made by clicking a point on one solid and a point on another. Not sketches, constraints, fillets or drawings; the window does not try to match FreeCAD, Onshape or Fusion, and the model around the mechanism is what sets it apart.

## What makes it different

1. **The drawing is the model.** Solids and joints in the window are nodes and edges in the graph, under the same names, so a user who draws a two-link arm sees a two-link arm in the canvas, not a black box.
2. **One graph from mechanism to digital twin.** CAD tools have geometry but no dynamics; robot simulators have dynamics but no link to the system around the robot. Here the mechanism, its electrical drive, its heat, its control and the business system it serves (a fleet of warehouse robots inside a logistics network, say) are one model.
3. **Design in either direction.** Start from the parts (draw solids, then join them) or from the motion (place joints, axes and ranges, simulate a stick figure, then grow solids around it), or mix the two. Later, from a goal: reach this point, lift this mass this fast.
4. **Parametric parts with physics attached.** Sizes, masses, joint limits and drive settings are Konjugate parameters: changing one changes the geometry, the mass properties and the dynamics together.
5. **Drive it, fork it, fit it, optimise it,** with what Konjugate already does: live sliders as a jog panel, schedules as trajectories, forks for what-if failures, Tuning to fit a real robot's logs, and an optimiser over the design.
6. **Will these motors do it?** For a given move and payload, does each joint stay within its motor's torque and speed, how hot do the windings get, and how long does the battery last, with all joints coupled.
7. **Open and cross-platform,** for novices, regular users and power users alike, on Windows, Linux and macOS.

## What a user does

1. **Open the Robotics window** from the toolstrip. It shows a 3D view with a floor grid, the Ground (a fixed solid every robot can stand on) and a short hint of what to do first.
2. **Add solids.** Box, cylinder, sphere or capsule, from a toolbar button, a menu item or a key. Each has a name, a size (typed, or dragged by handles in the view), a place and a turn (typed, or by the move and turn handles), a colour and either a density (its mass worked out from its shape) or a mass. Later: extruded profiles, parts imported from STEP or STL, and parametric robot parts (link between two joints, bracket, motor mount, wheel, gripper finger).
3. **Choose where they join.** With the Joint tool, click a point on the surface of one solid, then a point on another. Each click makes a **frame** on its solid: a point with an axis, the axis along the surface's normal by default, snapped to the face's centre or an edge's middle when near one. A frame belongs to its solid and moves with it; its place and axis can be typed. The second click moves the second solid so the two frames meet, unless the user holds a key to keep it where it is.
4. **Say how they join.** The joint's kind: **hinge** (turns about the frame's axis), **slider** (slides along it), **ball** (turns about its point every way) or **fixed**. Its limits, damping and friction. Which solid carries which is shown by an arrow, from the parent (nearer the Ground) to the child.
5. **Say how each joint is driven.** **Free** (nothing drives it), **torque** (a torque the user sets, a slider in the canvas), **servo** (a position servo holding a setpoint, a slider in the canvas, with its gain and torque limit) or **motor** (a DC motor and gearbox on a battery, from the component library, in the canvas).
6. **Build.** The window checks the drawing (every solid joined to the Ground or explicitly free, no two joints on one child, no joint without two frames) and says plainly what is wrong and where. Then it builds the model into the canvas, as the logistics toolbox does, and keeps the drawing with the project.
7. **Run and watch it.** In the canvas, run it, move the sliders while it runs, and see it move in Pose Visualizer. Change the drawing, build again.

Every action has a button or menu item, a gesture in the view and a key, works on the selection and can be undone, on Windows, Linux and macOS alike.

Later steps, in later milestones: keyframed poses saved as schedules; the sizing view (each joint's operating points over its motor's envelope, winding temperatures, the battery's charge); forks for a heavier payload or a weaker motor; Tuning against a real robot's logs; optimising sizes and gear ratios; handing a reduced robot to a larger model; MJCF and URDF export.

## How the drawing becomes Konjugate

Build turns the drawing into these, side by side in the canvas, laid out from the Ground outwards (a subsystem would hide them behind a box the newcomer must open; that can come with merging, below):

| In the window | In the graph | What it holds |
|---|---|---|
| the Ground | a **node** | its place, which nothing changes, and its acceleration: upward at g, which is how gravity enters |
| a solid | a **node**, named as the solid | for a solid carried by a hinge or slider, that joint's speed and position (`jointSpeed`, then `jointAngle` or `jointPosition`, the only states it integrates), its acceleration and the drive's torque; its place in the world (`x`, `y`, `z` in metres, z up; `roll`, `pitch`, `yaw`, so Pose Visualizer shows it with nothing set up; and its rotation matrix `r11` to `r33`); its spatial velocity and acceleration; its articulated inertia and bias force; its kinetic and potential energy; its shape, drawn at true size |
| a joint | an **edge** from the solid nearer the Ground to the other, named as the joint | the joint's equations (below), and its parameters: damping and friction (live sliders), armature, lower and upper limit, and its drive: a torque (a live slider), or a servo's setpoint (a live slider), stiffness and torque limit (a live slider) |
| a solid's mass | a **shared parameter** `<solid> mass` | linked from the edge that carries the solid |
| a joint in motor mode | a **joint motor** node and a **battery** node from the component library, wired to the joint's edge | current, torque at the joint, winding temperature; the battery's charge (milestone B) |

So in the canvas a two-link arm reads as `Ground → Shoulder → Upper arm → Elbow → Forearm`, and a user adds a controller by wiring it to the `Forearm` node's `jointAngle` and to the `Elbow` edge's setpoint.

Each build replaces the model in the canvas, as Build in logistics does. Until core can merge a rebuild with what the user wired by hand (Part 1 of the core proposal), hand-made work around the robot is lost on the next build; the window should warn before that happens (still to do).

### The dynamics are the joints' own equations

Konjugate's main idea holds without exception: a node changes another only through an edge between them, and nothing outside the graph solves the model. So the robot moves by equations on its joint edges alone, with no solver node and no simulator behind it.

A robot's joints are coupled: how fast the elbow turns depends on the whole arm. One equation per joint, each reading only its own two solids, cannot capture that on its own. Featherstone's articulated-body method can, because it splits along the joints:

1. **Outward**, each joint sets its child's pose and velocity from its parent's and its own angle and speed.
2. **Inward**, each joint passes its child's articulated inertia and bias force to its parent (what the child and everything it carries resists, seen through the joint), each parent summing what its children pass and adding its own.
3. **Outward again**, each joint works out its own acceleration from its parent's acceleration and the articulated quantities, and sets its child's.

Every quantity is a spatial vector or matrix in the world frame, about the world's origin, so a joint needs no transforms between its two solids. Each joint edge carries these as one relationship with several equations, each setting a state on one of its two solids (core's algebraic relationships, below). The engine evaluates them in dependency order at every instant, so the three sweeps settle within one step, and each solid's node then integrates its joint's speed and position, the speed first and the position from the new speed (semi-implicit Euler), which keeps a swinging arm's energy bounded where explicit Euler would let it grow.

The equations are written out as MathJSON with LaTeX beside it, which Konjugate's canvas reads back as the same expressions, so a user can open a joint edge and read or edit any of its equations there, as they can every other relationship with several equations (a test checks every equation of every example). They are checked against a second, independent solution of the same drawing (`tests/engine/newtonEuler.mjs`: the recursive Newton-Euler method for the mass matrix and bias forces, then a linear solve, stepped the same way): pendulum, two-link arm, a cart carrying two pendulums on a slider, an arm with a gripper, two links on ball joints, a body joined to nothing falling with an arm turning on it, a four-bar linkage and an arm pinned at both ends, friction, armature, every shape and joints read either way round agree to within 1e-9. The reference holds a ball's and a free joint's turn as the built model does, as a quaternion stepped one component at a time, and closes a loop with the same rate and compliance, by its own Jacobian and mass matrix. MuJoCo run on its own gives a third opinion where a Python with MuJoCo is at hand; it treats damping implicitly and its servos as position actuators, so it agrees closely rather than to rounding, and the bound is stated in the test.

The same sweeps take a joint of any number of axes: a hinge or a slider one, a ball three (its angular velocity in its parent's frame, its turn a quaternion integrated from it, each component after the one before), and a free joint six, for **a solid joined to nothing**, which hangs from the Ground by a free joint edge of its own (its spatial velocity, centre and rotation integrated on its node). A joint of several axes solves with its D by the L D Lᵀ factors, written out as definitions on its edge.

**A closed loop** (a four-bar linkage) is a tree plus the joint that closes it. That joint is an edge between its two solids, holding what its kind holds with forces λ found in the graph each instant, as everything else is. It holds rows of motion along a frame fixed on its first solid (t₁, t₂ and its axis n): a ball its point (three rows), a hinge its point and its axis (five), a slider its axis's line and its turn (five), a fixed joint everything (six). Each row's rate is a component of G Δ, Δ the second solid's spatial velocity less the first's: the second's velocity at the first's point, relative to the first's, or their relative angular velocity. A robot may close any number of loops, R rows in all:

4. **Inward**, each joint on the way from either end of a loop back to the Ground passes its parent how its child's bias force moves with every λ (a 6 by R matrix, P), starting from each closing edge's own share at its ends (±Gᵀ).
5. **Outward**, each of those joints works out how its child's acceleration moves with every λ (another 6 by R, A).
6. Each closing edge reads its ends' accelerations and A, and hands the solid it holds its rows of K = G (A₂ - A₁) (how its rows' accelerations move with every λ) and of the acceleration wanted, -Ġ Δ - 2 ω c - ω² e - G (â₂ - â₁): c its rows' rates, e their gaps and twists, ω the closure's rate (a quarter of the time step's, critically damped: Baumgarte's terms). These pass in, solid to solid, to the Ground, where they gather; the edge from the Ground to each solid hanging from it solves (K + εI) λ = wanted for every loop at once (ε a small compliance, so a row the loop's other joints already hold, out of a planar linkage's plane, asks for no force), and the forces pass out again, solid to solid, to each closing edge.
7. **Inward and outward once more**, the change the forces make to the bias forces and then to every joint's acceleration below the loops, added to what the first three sweeps gave.

The gap opens a little while a linkage moves fast (within a twentieth of a millimetre in the four-bar example) and closes to nothing as it slows. **Contact** is not modelled yet: it needs forces between solids that are not joined, which are edges too, and come later.

### Time steps and how a joint is driven

The engine's step is the robot's step: a millisecond by default, which suits most drawings. Stiff servos on light solids need a shorter one; the run report states the step. The equations are cheap (a few hundred operations per joint), so a millisecond step runs much faster than real time for small robots.

- **Servo mode** (the default for hobby and smart servos, such as the SO-100's Feetech STS3215, which close their own position loop): a torque of stiffness times (setpoint minus position), limited to the servo's most torque, every step. Speed and current limits come later.
- **Torque mode** (for a motor driven by a controller the user builds in the canvas): a constant torque, a live slider. Wiring a joint motor's torque into a joint edge comes with milestone B.
- A joint's **limits** are a stiff spring and damper beyond them, scaled by the joint's effective inertia so they hold without a smaller step. **Friction** is smoothed over a hundredth of a radian per second (a tanh), so a joint at rest creeps very slowly rather than sticking.

### Seeing it move: Pose Visualizer, not a second player

Playing a result in 3D and scrubbing through it is Pose Visualizer's job, so the toolbox does not build its own player. Each solid node carries Konjugate's pose states, so any robotics result plays in Pose Visualizer with nothing set up, even when the project is opened by someone without the toolbox. This needs from Pose Visualizer (see [Needs from Pose Visualizer](#needs-from-pose-visualizer)):

- **True scale.** A node whose mesh is in metres, marked so, is drawn at its real size about its own origin.
- **Angles that do not lock.** Solid nodes carry their rotation matrix as well as `roll`, `pitch`, `yaw`; Pose Visualizer turns them by the three angles by default, as Konjugate's own convention does.
- **Named frames** drawn as small axes on their solid, optional.

### The window

`packages/toolbox`: a launcher window, as logistics and fintech have, built on three.js. The drawing (solids, frames, joints, drive settings) lives in the window's session, kept with the project, and has a small, versioned JSON form of its own (`lib/drawing.mjs`), so it can be tested without a browser and read by scripts. Turning a drawing into the graph (`lib/modelBuilder.mjs`, with `lib/expressions.mjs` for the equations) is a pure function of the drawing, tested on its own; so is turning it into MJCF (`lib/mjcfWriter.mjs`), used today only to check the build against MuJoCo and later for export.

Imported and extruded parts come later, with OpenCascade compiled to WebAssembly loaded only when first needed.

### Rules

- **Never outside the graph.** A node changes another only through an edge between them; no node solves others, and nothing runs the robot but the engine.
- **Solids speak Konjugate's pose vocabulary.** Positions in metres with z up.
- **Joints attach to frames, not to faces.** A frame belongs to its solid; a face may vanish when a part is edited, a frame does not.
- **Equations read parameters, never baked numbers, for what can change during a run** (mass, damping, friction, armature, limits, drive settings), so sliders, schedules, forks, Tuning and the optimiser reach the motion. What cannot (shapes, which solid carries which, a joint's kind, point and axis) waits for the next build, and the window says which is which.
- The conventions of the other toolboxes: every input says where it came from (typed, worked out from a density, a datasheet); limits and bugs are stated plainly.

## Needs from core

General additions, not robotics-specific, described in the core proposal [Toolbox mechanisms, model merging and design optimisation](https://github.com/zenineasa/Konjugate/blob/main/docs/proposals/toolboxMechanismsAndDesignOptimisation.md) as Part 2:

1. **Algebraic relationships** (built, October 2026, not yet released). A relationship may carry several equations (`equationModel.outputs`), each setting a state on one of its two endpoints. At each instant the engine evaluates every such equation across the model in dependency order, summing equations that set the same state, and refuses a model whose equations depend on each other in a loop. A launcher requires it as `algebraicRelationships`.
2. **Semi-implicit integration** (built, October 2026, not yet released). A node may integrate its states in the order listed, each from the ones updated before it (`numerics.integration: "semiImplicit"`).
3. **Arcsin, Arccos, Arctan, Arctan2 and Tanh** in equations (built, October 2026, not yet released).
4. **A resumed run steps exactly as the run it came from** (fixed, October 2026): the last step of a run resumed from a checkpoint is no longer a few ulps short.

Later, from the same proposal: merging a rebuild with hand-made work (Part 1), and design objectives with constraints (Part 3). Each toolbox release needs a Konjugate release with what it requires, so core goes first.

## Where milestone A stands

Built (October 2026): the core additions above, with engine and JavaScript tests; the window as described under What a user does, steps 1 to 7, except motor mode; the model builder with the articulated-body equations on the joint edges; seven examples (a pendulum, a two-link arm, a cart carrying two pendulums, an arm with a two-finger gripper, a four-bar linkage, a pendulum on two ball joints, a crank and slider); unit tests; engine tests against the Newton-Euler reference (and MuJoCo where it is installed); a window test that drives the page in a browser and has Konjugate's engine validate the model it builds.

Still to do in milestone A: a warning before a build replaces hand-made work; selecting several solids to move together; dragging a joint's point in the view.

Built in milestone A2 (October 2026): ball joints, with a torque or a servo to a turn and a most turn as their limit; solids joined to nothing; and loops closed by any kind of joint, any number to a robot, as above, each checked against the Newton-Euler reference (two loops in one robot, a weld, a slider on its rail, a hinge holding its axis), and against MuJoCo alone where it is installed; a four-bar linkage, a pendulum on two ball joints and a crank and slider among the examples. A joint's damping is taken at the step's end speed (added, times the step, to its effective inertia, as MuJoCo does), so heavy damping on a light link stays stable.

## Milestones

Each one ends in something that can be shown.

- **A. Draw it, build it, run it.** The window with boxes, cylinders, spheres and capsules; frames picked on surfaces; hinge, slider and fixed joints; free, torque and servo drives; undo, keys and menus. Build makes solid nodes and joint edges carrying the articulated-body equations. Checked against an independent Newton-Euler solution and against MuJoCo running the same MJCF on its own. Shown on a two-link arm and a cart carrying two pendulums.
- **A2. Loops, balls and floating solids.** Closed loops (a four-bar linkage) through constraint forces on the closing joint's edge; ball joints; a solid joined to nothing.
- **B. Drive it.** Motor mode (joint motor and battery from the component library); a jog panel of live setpoints; keyframed poses saved as schedules; a pick-and-place example.
- **C. Will these motors do it?** The sizing view and forks for a heavier payload or a weaker motor, as built in the first version, on drawn robots.
- **D. Start from a known robot.** MJCF and URDF import, and Menagerie fetched on request, into the drawing as solids (with their meshes) and joints, as built in the first version; then STL and STEP parts.
- **E. Parametric parts.** Extruded profiles, robot part templates, joint-first skeleton design, sizes as shared parameters.
- **F. Fit to a real robot.** Tuning against a public LeRobot dataset of an SO-100 arm, then a user's own logs.
- **G. Design optimisation.** Objectives and constraints over sizes, gear ratios and gains; motor choice from a catalog in an outer loop; candidates as branches.
- **H. Contact.** A gripper picking a box; a wheeled robot on the floor: contact forces as edges between the solids that touch.
- **I. The joined demo.** A fleet of warehouse robots inside the logistics toolbox's warehouse model, each reduced from its detailed model to a table of cycle time and energy per task.
- **J. Exchange.** MJCF and URDF export; STEP export of modelled parts.

## What is kept from the first version

The first version ran the robot in MuJoCo inside one computational node that set the states of every solid. That solved the robot outside the graph, against Konjugate's main idea, so it is gone, as are the core additions it needed (a node setting other nodes' states, and a relationship solved by a node), withdrawn from core before any release. Its code is in the repository's history. Kept from it:

- **The joint motor, servo winding and supply components**, in the history too, to come back with milestone B wired to joint edges.
- **Pose Visualizer additions:** quaternions, several bodies on one node, true-size parts.
- **Menagerie import with licences and provenance, and the SO-ARM100 servo sizing and fork comparison,** set aside for milestones C and D, where they come back on top of the drawing (up to commit `32ca8e2`).

### Robots and data from public sources

Fetched on request, one model or dataset at a time, and kept on the user's device, as logistics fetches and keeps map extracts:

- **Licence before download.** The import lists each model with its licence and source, and shows them before fetching; one that is not freely usable (none in Menagerie today) is marked as such.
- **Pinned.** A download is of one commit of Menagerie (or one revision of a dataset), recorded, so a project runs the same after an upstream edit.
- **Kept on the device, carried by the project, as logistics does with maps.** The download goes to Konjugate's store of downloaded files (core's `storedFiles`, with where each came from), fetched once, used by every project built from it, and working offline after. The importer reads it from there (core's `binaryInputs`), as logistics' reads a map extract, and the project keeps what a run needs: the model's MJCF and the meshes, included files and textures it reads (1.5 MB in a saved project for the SO-ARM100), with its licence's text and its provenance (source, repository, folder, commit) beside them. So a project runs wherever it is opened, from the command line too, and forks and resumes with nothing to fetch; sharing it shares the robot with its notice, which every Menagerie licence allows. The readme, the changelog and Menagerie's pictures stay out.
- **Our own material is our responsibility.** An example, showcase project, thumbnail or video built from someone's robot is distributed by us: it fetches its robot when opened, or uses a model we are content to ship with its notices.
- **Names, not logos.** A model is named as its maker names it, to identify it; no logos or others' images, and previews are drawn by the toolbox.
- **Hosts.** `raw.githubusercontent.com` and `api.github.com` for Menagerie, `huggingface.co` for datasets, listed in the add-on's network hosts.

Nothing new from core: logistics already keeps map files this way. The importer runs in JavaScript and reads a model's joints and servos from the MJCF itself.

## Moving robotics out of core

Once the toolbox replaces them, six of the seven robotics examples move here, with the springs-versus-exact four-bar as a before and after once loops are built. The position-controlled joint stays in core as a teaching example, since the web edition cannot install add-ons and welcome content or videos may point at it; before moving the others, check what links to them. The robotics shapes (articulated link, servo, parallel gripper, drive wheel, propeller) move with them. The generic motor, battery and thermal templates stay in core, as other toolboxes and examples use them. Pose Visualizer stays its own add-on, as it serves models not built in the toolbox, and becomes the toolbox's player (see [Seeing it move](#seeing-it-move-pose-visualizer-not-a-second-player)).

## Needs from Pose Visualizer

Built (October 2026, not yet released): quaternions (each convention, and each node's own mapping, chooses roll, pitch and yaw or a quaternion, with state names the user picks); several bodies on one node, by prefix; and a node's appearance parts drawn at true size about their bodies' origins. Still to come: frame axes for named frames and contact force arrows. Pose Visualizer, and Konjugate with appearance parts, release before the toolbox release that relies on them.

## Out of scope

Topology optimisation and finite element analysis; sketches, constraints, fillets, drawings and full mechanical drafting; soft bodies and fluids around the robot; motion planning beyond keyframes and schedules (planners can come through providers later).

## Open questions

- **Speed for large robots.** Each joint's equations are a few hundred operations, evaluated by the engine's interpreter. A humanoid with thirty joints at a millisecond step should still run faster than real time, but this is not yet measured.
- **Stiffness.** Semi-implicit Euler is stable for a swinging arm but still needs a step well below a stiff servo's time constant. Damping is now taken implicitly, inside the joint's equations; the servo's spring could be taken the same way if users hit it.
- **First user:** students and hobbyists, robotics start-ups sizing actuators, or the warehouse customers of the logistics toolbox? The working answer is hobbyists and students first: the drawing window serves them best, and they need no sales cycle.
- **Web edition:** add-ons do not run there today; nothing the toolbox builds needs more than the engine.
