# Working in this repository

The Robotics toolbox for Konjugate: a component library and the mechanism provider (`packages/engine`) and a window add-on (`packages/toolbox`).

Read first: [docs/direction.md](docs/direction.md), where the toolbox is going and why, and "Where milestone A stands" for what is built.

Code map:

- `packages/engine/providers/mechanism.py`: the mechanism provider, MuJoCo as a computational node (co-simulation steps; joints in torque, servo or free mode; parameters into MuJoCo's model). The model builder writes its configuration line.
- `packages/engine/components/`: the joint motor and the joint drive bundle. `packages/engine/tools/describeMechanism.py`: MuJoCo's own reading of a model, for scripts and tests.
- `packages/toolbox/lib/`: `mechanismModel.mjs` (a mechanism node from a model and its joints), `documentBuilder.mjs` and `driveModel.mjs` (placing templates, motors and drives), `mjcf.mjs` (reading a model's joints and servos in JavaScript), `menagerie.mjs` and `catalog.mjs` (fetching robots, the list offered, stored names).
- `packages/toolbox/importers/robot.mjs`: the importer (describe, build); `toolbox.mjs`: the window.

Commands: `npm test` (unit), `npm run test:engine` (through Konjugate's engine, needs its build and `npm run setup:python`), `npm run test:interaction` (the real app, offline), `npm run fetch:model <folder>` (a Menagerie robot into `out/models/`). Konjugate's checkout is expected at `../konjugate` (or `KONJUGATE_DIR`).

Conventions:

- The owner commits; propose a one-sentence commit message describing the change in behaviour.
- A release needs a Konjugate release with every feature in `packages/toolbox/addon.json`'s `requires` (today `nodeProviderInputs`, for a node provider reading other nodes' states and shared parameters, which the canvas keeps): when the toolbox needs something new from Konjugate, release Konjugate first.
- No robot or dataset of anyone else's is shipped: fetched on request, licence shown first, kept on the user's computer, carried by projects built from it with its licence and provenance.
- A mechanism is checked against MuJoCo run on its own; a comparison that only matches after a shift or a tolerance is a finding to state, not to hide.
- No em dashes and no Oxford commas in docs, UI text or comments.
- Every input is labelled with where it came from; state limits and bugs plainly.
- Every fix comes with a test that would have caught it. Keep `ReadMe.md` and `docs/direction.md` current.
