# Working in this repository

The Robotics toolbox for Konjugate: a window add-on where people draw a robot from solids and joints and build it into the canvas (`packages/toolbox`). Each solid becomes a node and each joint an edge carrying the joint's equations; nothing outside the graph solves the robot. Never break that: a node changes another only through an edge between them.

Read first: [docs/direction.md](docs/direction.md), where the toolbox is going and why, and [docs/interaction.md](docs/interaction.md), how the window behaves for newcomers, regular users and power users.

Code map:

- `packages/toolbox/lib/drawing.mjs`: the drawing (solids, frames, joints) and every operation on it, each returning a new drawing; its structure from the Ground out (parents, loops, floating solids) and its checks.
- `packages/toolbox/lib/modelBuilder.mjs`: a drawing as a Konjugate document (a node per solid, an edge per joint carrying the articulated-body equations as algebraic relationships), and what it cannot build yet (`simulationLimits`). `lib/expressions.mjs`: the MathJSON and LaTeX those equations are written in. `lib/mjcfWriter.mjs`: a drawing as MJCF, for the comparison with MuJoCo and later export. `lib/shapes.mjs`: the four solid shapes. `lib/math.mjs`: vectors and quaternions. `lib/examples.mjs`: drawings to start from. All pure, no dependencies, tested without a browser.
- `packages/toolbox/toolbox.mjs`: the window; `lib/view.mjs`: its 3D view (three.js, vendored in `vendor/`); `lib/history.mjs`: undo and redo; `lib/platform.mjs`: keys per platform.
- `packages/toolbox/importers/drawing.mjs`: the importer (open a drawing file; build the model from the window's drawing).
- `tests/engine/newtonEuler.mjs`: an independent solution of a drawing's motion (recursive Newton-Euler), the reference the built model is checked against.

Commands: `npm test` (unit), `npm run test:window` (the window in a plain browser, Playwright from the Konjugate checkout; with an engine build it also validates the model it builds), `npm run test:engine` (drawn robots through Konjugate's engine against the Newton-Euler reference, and against MuJoCo alone when `npm run setup:python` has been run; needs the engine build). Konjugate's checkout is expected at `../konjugate` (or `KONJUGATE_DIR`).

Conventions:

- The owner commits; propose a one-sentence commit message describing the change in behaviour.
- A release needs a Konjugate release with every feature in `packages/toolbox/addon.json`'s `requires` (today `projectSession`, `keepSession` and `algebraicRelationships`: relationships with several equations setting their endpoints' states at each instant). When the toolbox needs something new from Konjugate, release Konjugate first.
- Every action has a button or field, a gesture in the view and a key, works on the selection and can be undone, on Windows, Linux and macOS alike; keep docs/interaction.md's table current.
- A mechanism is checked against the Newton-Euler reference to rounding, and against MuJoCo run on its own; a comparison that only matches after a shift or a tolerance is a finding to state, not to hide.
- No em dashes and no Oxford commas in docs, UI text or comments.
- Every input is labelled with where it came from; state limits and bugs plainly.
- Every fix comes with a test that would have caught it. Keep `ReadMe.md` and `docs/direction.md` current.
