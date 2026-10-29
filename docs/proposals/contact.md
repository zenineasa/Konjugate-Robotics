# Contact: a design

Milestone H in [direction.md](../direction.md): a gripper picking up a part, a wheeled robot on the floor, a walking robot on its feet. This is how contact comes into the toolbox without breaking Konjugate's main idea, and how it is checked. The first slice (the floor, and primitive pairs) is built (October 2026), as decided at the end; direction.md says how it was checked.

## The idea, kept

A node changes another only through an edge between them, and nothing outside the graph solves the robot. Contact is the hardest case for that: which solids touch changes from step to step, but the graph is fixed when it is built. So:

- **Every pair of solids that may touch is an edge, decided at Build.** A contact edge joins the two solids (or a solid and the Ground, for the floor). It is there whether or not they touch.
- **Each contact edge holds a fixed number of contact points,** as many as its two shapes can have at once (one for two spheres, four for a box on the floor). Each point is worked out by the edge's own equations at every instant: how far apart the shapes are there, along which normal, at which point. A point whose shapes are apart is a row with no room (its force held to nothing), as a joint's limit not reached is today.
- **Contact forces are rows found at the Ground with every other row,** the joints' limits and dry frictions, the couplings, the tendons and the loops, by the solve already there (projected Gauss-Seidel from the step before's forces, then polished). A contact edge hands its rows of K and its wanted accelerations to the solid holding it, exactly as a closing joint does today, and its forces come back the same way.

So contact is the loop machinery with bounds: a closing joint holds its rows always, a contact point only while it touches and only by pushing.

## What MuJoCo does, to match

Probed on MuJoCo 3.16 (a sphere, a box, a capsule and a cylinder resting on a plane, their contacts and constraint rows printed):

- **Shapes and pairs.** Each body's collision geoms (primitives, or a mesh's convex hull); every pair that may touch, less a body and its parent, filtered by `contype` and `conaffinity`, plus explicit `<pair>` and `<exclude>`. The floor is a plane geom of the world.
- **Points.** One collision function per pair of shape kinds, each giving up to a fixed number of contacts: a sphere on a plane one, a capsule lying on it two (its ends), a box four (its corners below the plane), a cylinder standing on it three (points of its rim). Each contact has a distance (negative when they overlap), a point (midway through the overlap), and a frame: its normal and two tangents.
- **Rows.** With the default pyramidal friction cone and condim 3, each contact is four rows, the normal plus and minus the friction coefficient times each tangent, each pushing only. Friction, condim, solref and solimp come from the two geoms (friction and condim by the larger, solref and solimp by `solmix`).
- **Softness.** As for limits: R = (1 - imp) w / imp and aref = -B v - K imp r, r the distance less the margin. w comes from the two bodies' `body_invweight0` and the friction (a 1 kg sphere on the floor with friction 1: w = 4, MuJoCo's diagApprox, so R = 0.324 at imp 0.925, 0.25 mm in). The exact rule for w with other frictions and two moving bodies is to be pinned down by probing, as the equality's was.

With these, a robot that touches things should still agree with MuJoCo to rounding, as limits, frictions, couplings and tendons do now. Where it cannot (contacts that come and go between steps differently), the difference is a finding to state.

## In the drawing

- **Each solid's colliders:** shapes in its own frame. A drawn solid's own shape by default; an imported body's collision geoms (its visible meshes stay its look). A solid may have none.
- **Each solid's contact settings:** friction (sliding, and later torsional and rolling), and how softly it is held (solref and solimp, MuJoCo's defaults otherwise).
- **The floor:** the Ground's plane, on or off.
- **Which pairs touch:** see the questions below.

## In the build

- A **contact edge** per pair: from the first solid (or the Ground) to the second. It reads both solids' poses, velocities and how their accelerations move with every row's force (the loops' sensitivities), and for each of its points:
  - the collision function as equations: distance, point, normal and tangents, each a `Which` over the cases (a box's corners, a capsule's ends);
  - the four pyramid rows' J as spatial vectors on the two solids (G, as a closing joint's), their rows of K from the two solids' sensitivities, and their wanted accelerations, handed to the solid holding the edge;
  - each row's compliance and bounds: 0 and 10³⁰ while the point touches, 0 and 0 while it does not;
  - the forces found, back from the holder, on each solid (as a closing joint's: on the second sweep, or through the joints' sensitivities with damping rows).
- **States worth plotting** on the holding solid: each point's distance and normal force, so a user can see in the canvas when and how hard two parts touch.
- **The part a gripper picks up** is a solid joined to nothing, which already falls by a free joint. Fingers and part are then in two trees, and their contact rows couple them at the Ground as a loop between two trees does today.

## Checks

- Against MuJoCo running the same model to rounding: a ball dropped onto the floor; a box sliding to rest with friction; a capsule rolling; a cylinder standing and tipping; a two-finger gripper lifting a box off the floor; each also written back as MJCF.
- Against the Newton-Euler reference, extended with its own collision functions, written independently in plain numbers, and the same row solve.
- Menagerie's arms with contact on, once their collision geoms come in.

## Cost

Each contact point is four rows. Every solid keeps 12 states per row (its sensitivities), and the Ground gathers K's lower triangle, R (R + 1) / 2 sums. A gripper and a part on the floor (two fingers against a box, the box on the floor: about 10 points, 40 rows) would roughly double or triple a run's time against the same robot without contact; a robot with many possible contacts grows as the square of their rows. Built so far against that: the Ground's sweeps and exact solve work only on rows with room and inside their bounds, each other row behind a choice the engine does not take (a gripper and a box, 24 rows: about 0.9 ms a step here). Later: gathering K per tree, so rows that cannot touch a tree are not carried through it.

## Slices

1. **The floor:** every primitive (sphere, capsule, box, cylinder) on the Ground's plane, with friction. A ball, a box sliding, a robot's foot on the floor.
2. **Primitive against primitive:** sphere and capsule against each other and against a box (box against box later: MuJoCo's needs up to eight points and a clipping step). A gripper with capsule or sphere fingertips picking up a box, and the pick-and-place example without its weld.
3. **Box against box and convex meshes:** MuJoCo's own box-box function, then meshes by their convex hulls (MuJoCo's iterative search, as a fixed number of iterations in equations, or a nearest-vertex approximation stated as such).
4. **Imported contact:** an MJCF's collision geoms, contype and conaffinity, pairs and exclusions, as solids' colliders and the drawing's pairs.

## Decided

1. **Which pairs touch:** both: every pair as MuJoCo's rule has it, and pairs the user names or excludes.
2. **Friction cone:** MuJoCo's default pyramidal cone.
3. **First slice:** the floor and primitive pairs together (slices 1 and 2), box against box and cylinders against the rest left for slice 3.
