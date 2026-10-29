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

## The solve in the joints' coordinates

Built October 2026, from the arm with a gripper with contact on (the floor against its six links: 24 points, about 110 rows, 6 joint coordinates), which built 75,000 equations of 3.7 million terms, 108 MB, and took the engine six minutes to check. Gathering K (R by R) and solving with it grows as R² and R³; but rows act on the robot only through its n joint coordinates, so K = J M⁻¹ Jᵀ has rank n at most, whatever R is. Finding the same forces from J (R by n) and the joints' mass matrix M (n by n), as MuJoCo does, grows as R n² instead. Nothing else changes: the rows, their references, compliances and bounds, the warm start, the sweeps and polishes, and the forces handed back are as now, so the runs agree with MuJoCo and the reference as before, to rounding. The loops without other rows (no limit, friction, coupling, tendon or contact anywhere) keep their own solve as now.

### What passes inward, edge by edge

In place of each solid's sensitivities (12 per row) and K's lower triangle (R (R + 1) / 2), with everything a spatial vector in the world frame about its origin, as now:

- **Each row's push, summed over a subtree.** A loop's or a contact's row pushes its second solid by Gᵀ and its first by -Gᵀ per unit of its force. Its relationship hands each end its share; each joint adds its child's and passes the sum to its parent: 6 numbers per row acting in the subtree.
- **Each row's J.** A joint's relationship works out each row's J on each of its axes: its axis Sᵀ times the row's push summed over its child's subtree; or, for a row of its own (a limit, a dry friction, its damping) or one a coupling or a tendon gives it, the row's own J as now. J's entries are passed inward to the Ground: per solid, the rows acting in its subtree times the axes in it.
- **The mass matrix, as composite inertias.** Each joint passes its child's subtree's inertia (its own plus its children's, 21 numbers) to its parent; each axis's column of force, F = Ic S, passes inward to every joint carrying it, which works out M's entry Sᵀ F with its own axes; the entries are passed to the Ground. A joint's armature joins its diagonal, as MuJoCo's qM has it. Entries between two trees (a robot and a part joined to nothing) are nothing, and drop out of every sum as the equations are written.
- **As now:** each row's wanted acceleration aref - J q̈₀, its compliance and bounds.

### At the Ground

On the solid hanging from the Ground that solves, with M = L Lᵀ (its Cholesky factors) and each row's ĵ = L⁻¹ Jᵀ (n numbers), so that K's entry between two rows is ĵ₁ · ĵ₂:

1. **Loops, taken out exactly.** Their rows are held with a compliance of 10⁻⁹. A Householder reflection of their ĵ (as many as the loops' rows) splits the joints' coordinates into what the loops hold and what they leave free; the other rows' ĵ are rewritten in it, the part the loops hold scaled by ε (T Tᵀ + ε)⁻¹ (T the loops' ĵ in the reflection's coordinates), so K less what the loops take is again ĵ'₁ · ĵ'₂, to rounding (checked against the row-space elimination on random rows, to 10⁻¹⁶ relative; scaling the loops' rows by 1 / ε instead loses up to half the digits). The loops' forces come from their few rows at the end, as now.
2. **Projected Gauss-Seidel, five sweeps, from the step before's forces.** Each row's residual is its wanted acceleration less ĵ' · u less its compliance times its force, u the sum of ĵ' times every row's force (n numbers), updated as each row's force changes: R n per sweep, not R².
3. **Polishing, three tries as now.** Each row inside its bounds or held at one, as now; the rows inside solved exactly by Woodbury's identity: H = I + Σ ĵ' ĵ'ᵀ / c over the rows inside (c each one's compliance; n by n), its Cholesky factors, then each row's force from H's solution: R n² + n³, not R³. The fits are checked as now.
4. **Damping rows** (with a tendon's damping): found after the others from them, as now, by L D Lᵀ over the damping rows alone, their K and their K with the others from ĵ.
5. **The joints' accelerations, with damping rows:** q̈ = q̈₀ + L⁻ᵀ Σ ĵ times each row's force, handed out to each joint (n numbers), in place of each joint's acceleration moved by every row's force. With a second sweep, the forces are handed out as now.

### What it costs

As built, the arm with a gripper with contact on: 18,800 equations of 227,000 terms (from 75,000 of 3.7 million), 8,900 states (from 55,000) and 7.5 MB (from 108); built in 0.4 s (from 11), checked by the engine in 3.3 s (from six minutes), and run for 2 s in about 7.5 s. The gripper lifting a box: 0.6 ms a step (from 0.9) and half a second to start (from 1.1). Menagerie's Panda as imported: 1.6 MB (from 4.5), run in half the time. Every run agrees with MuJoCo and the reference as it did, to the digits the tests print; the Panda's run with the solve before, to 10⁻¹⁶. Twice the possible contacts make a model about twice as large, not eight times. Later, as before: each tree solved on its own.

Checked: a unit test works out the forces the equations give for random rows of every kind (loops, contacts with and without room, dry frictions, a limit, a coupling, damping rows; more loop rows than coordinates; many more rows than coordinates) and holds them to what a solve over the rows has: each loop row's equation, each searched row inside its bounds with nothing left over or at a bound pushing against it, each damping row's equation, and the joints' accelerations M⁻¹ Jᵀ f. An engine test pins an arm at both ends with a dry friction on its pin, so the pivot's five rows hold more than its one coordinate can move.

## Slices

1. **The floor:** every primitive (sphere, capsule, box, cylinder) on the Ground's plane, with friction. A ball, a box sliding, a robot's foot on the floor.
2. **Primitive against primitive:** sphere and capsule against each other and against a box (box against box later: MuJoCo's needs up to eight points and a clipping step). A gripper with capsule or sphere fingertips picking up a box, and the pick-and-place example without its weld.
3. **Box against box and convex meshes:** MuJoCo's own box-box function, then meshes by their convex hulls (MuJoCo's iterative search, as a fixed number of iterations in equations, or a nearest-vertex approximation stated as such).
4. **Imported contact:** an MJCF's collision geoms, contype and conaffinity, pairs and exclusions, as solids' colliders and the drawing's pairs.

## Decided

1. **Which pairs touch:** both: every pair as MuJoCo's rule has it, and pairs the user names or excludes.
2. **Friction cone:** MuJoCo's default pyramidal cone.
3. **First slice:** the floor and primitive pairs together (slices 1 and 2), box against box and cylinders against the rest left for slice 3.
