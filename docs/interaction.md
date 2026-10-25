<!-- Copyright © 2026 Zenin Easa Panthakkalakath -->

# Interaction

How the Robotics window should feel to three kinds of user, and what that means for every action in it. Keep this current when an action is added or changed. The rules are the Logistics toolbox's.

## Three users

- **A newcomer** reads the screen. Every action has a labelled button, a field or a list; nothing depends on knowing a gesture or a key. The hint at the foot of the view says what to do next, the Checks list says what stops a build and where, and every change can be undone.
- **A regular user** works in the view: click to select, drag the handles to move or turn, click two points to join.
- **A power user** keeps their hands on the keyboard: a key for every frequent action, several solids selected at a time, undo and redo for everything.

## Rules

1. **Every action three ways**: a button or field, a gesture in the view, a key. A key is never the only way.
2. **Shortcuts are taught where they are used**: in tooltips, in the hint and all of them under `?`.
3. **Act on the selection**: delete and duplicate act on everything selected, as one step.
4. **Undo everything**, and say what was undone. Typing in one field in quick succession is one step.
5. **Refuse with a reason, where the user is looking**: a message over the view, and the Checks list.
6. **Escape always backs out** one level: the first point of a joint, then the tool, then the selection.
7. **No surprises on keys**: keys act only when the user is not typing in a field.
8. **Every platform as its own users expect**: `lib/platform.mjs` decides which keys mean what, as in Logistics.
9. **Tests cover each way**: `tests/interaction/robotWindow.mjs` does the workflow in the real Konjugate app, opening the window from the toolstrip, by buttons, clicks in the view and keys, as a Windows or Linux user and as a Mac user.

## Actions

The command key is Ctrl on Windows and Linux and ⌘ on a Mac.

| Action | Newcomer | Regular | Power |
|---|---|---|---|
| Add a solid | Box, Cylinder, Sphere, Capsule | | `B`, `C`, `S`, `P` |
| Start from an example | Examples… | | |
| Import a robot from a MuJoCo model (Menagerie's, its licence first, or its files from this computer) | Import robot… | | `I` |
| Select | Its row under Parts; a check about it | Click it in the view | Shift-click or command-click to add |
| Rename, resize, change its shape, material, mass, colour | Its card | | |
| Move or turn a solid (and the solids it carries) | Its card: position and turn; Move, Turn | Drag the handles | `G`, `R` |
| Join two solids | Join (with the kind beside it), then click a point on each | Click a point on each | `J` |
| Join without moving | | Shift-click the second point | |
| Change a joint: kind, point, axis, limits, damping, drive (free, torque, servo or motor) | Its card | Click its mark to select it | |
| Set the battery the motors draw from (its voltages, capacity, internal resistance and most current), or a supply by name | The robot's card (nothing selected) | | |
| Set a joint's motor by name (a Feetech STS3215, say), and its protection (most voltage, cut-off current, duty and time) | Its card: Motor, Protection | | |
| Make every servo on a hinge a motor by name, on a supply, to size them | The robot's card: Servos to motors, Make them motors | | |
| Pose the robot (jog) | The sliders under Pose | | |
| Back to the robot as drawn | As drawn | | `Escape` |
| Record a pose as a keyframe | Add keyframe | | `K` |
| Show a keyframe | Show, beside it | Click its row | `[` and `]`: the one before and after |
| Retime, replace or remove a keyframe | Its time; Replace; ✕ | | |
| Make the pose the joints' setpoints | Set as setpoints | | |
| Flip a joint's axis, swap its solids | Its card | | |
| Duplicate | Duplicate | | command-`D` |
| Delete | Delete | | Delete or Backspace (⌫ on a Mac) |
| Undo, redo | Undo, Redo | | command-`Z`; Ctrl+`Y` or Ctrl+Shift+`Z` (⇧⌘Z on a Mac) |
| Fit the robot in the view, look from a side | Fit; Look from… | Drag to turn, right-drag to move, scroll to zoom | `F` |
| Snap moves to 5 mm and turns to 15° | Snap | | |
| Back out | Select | | `Escape` |
| Build into the canvas | Build into canvas | | command-`B` |
| Will these motors do it? (open and close the sizing panel; opened, it runs the robot as drawn if it has not run) | Size; ✕ | | `M`; `Escape` closes it when nothing is selected |
| Run the robot as drawn, or a what-if from a time | Run as drawn; What if, its fields, Run what-if | | |
| Open the runs in the canvas | Show in canvas | | |
| Keys and gestures | ? | | `?` |
