import sys
root = sys.argv[1]

def edit(path, pairs):
    p = f'{root}/{path}'
    s = open(p).read()
    for old, new in pairs:
        assert s.count(old) == 1, (path, old[:80])
        s = s.replace(old, new)
    open(p, 'w').write(s)

edit('docs/interaction.md', [
    ("| Change a joint: kind, point, axis, limits, damping, drive | Its card | Click its mark to select it | |",
     "| Change a joint: kind, point, axis, limits, damping, drive (free, torque, servo or motor) | Its card | Click its mark to select it | |\n| Set the battery the motors draw from | The robot's card (nothing selected) | | |\n| Pose the robot (jog) | The sliders under Pose | | |\n| Back to the robot as drawn | As drawn | | `Escape` |\n| Record a pose as a keyframe | Add keyframe | | `K` |\n| Show a keyframe | Show, beside it | Click its row | `[` and `]`: the one before and after |\n| Retime, replace or remove a keyframe | Its time; Replace; ✕ | | |\n| Make the pose the joints' setpoints | Set as setpoints | | |"),
])

edit('ReadMe.md', [
    ("Seven examples to start from: a pendulum, a two-link arm on a waist, a cart carrying two pendulums, an arm with a two-finger gripper, a four-bar linkage, a pendulum on two ball joints, and a crank driving a block along a rail.",
     "A joint can also be driven by a DC motor and gearbox on the robot's battery, its controller following a setpoint. A jog panel poses the robot with a slider per driven joint, to set setpoints or record keyframes: poses in time that the joints' setpoints follow. Eight examples to start from: a pendulum, a two-link arm on a waist, a cart carrying two pendulums, an arm with a two-finger gripper, the same arm on motors keyframed through a pick-and-place cycle, a four-bar linkage, a pendulum on two ball joints, and a crank driving a block along a rail."),
    ("Pose Visualizer shows the robot moving.",
     "A motor is a node of its own (its supply voltage, duty cycle, current, torque and winding temperature) with relationships to the battery (a node with its voltage and charge) and to its joint's solid; the keyframes are each driven joint's setpoint, a shared parameter with the keyframes as its schedule. Pose Visualizer shows the robot moving."),
    ("- **Not yet:** contact; motors and batteries wired from the window; imported parts (STL, STEP, MJCF, URDF); keyframed moves.",
     "- **Not yet:** contact (so a gripper cannot yet pick a part up: the pick-and-place example carries its part fixed to the hand); a motor's current limit and winding inductance; imported parts (STL, STEP, MJCF, URDF)."),
])

edit('docs/direction.md', [
    ("| a joint in motor mode | a **joint motor** node and a **battery** node from the component library, wired to the joint's edge | current, torque at the joint, winding temperature; the battery's charge (milestone B) |",
     "| a joint in motor mode | a **motor** node (`<joint> motor`) and the robot's one **battery** node, with relationships from the battery (supply), from the joint's solid (controller, windings) and to the battery (drain) | the motor's supply voltage, duty cycle, current, torque at the joint and winding temperature; the battery's voltage and charge; the motor's resistance, torque constant and gear ratio, and the battery's voltages and capacity, as shared parameters |\n| a keyframed joint | a **shared parameter** `<joint> setpoint` with the keyframes as its schedule, linked from the servo's or the motor controller's setpoint | its setpoint over time, in straight lines from one keyframe to the next, still a live slider |"),
    ("- **Torque mode** (for a motor driven by a controller the user builds in the canvas): a constant torque, a live slider. Wiring a joint motor's torque into a joint edge comes with milestone B.",
     "- **Torque mode** (for a motor driven by a controller the user builds in the canvas): a constant torque, a live slider.\n- **Motor mode** (a DC motor and gearbox on the robot's battery): a controller sets the duty cycle, the proportional gain times the setpoint left to reach less the derivative gain times the joint's speed, held to ±1; the current is the duty cycle times the battery's voltage less the back-EMF (torque constant × gear ratio × joint speed), over the winding's resistance; the torque at the joint is the gear ratio times the torque constant times the current, and the rotor's inertia times the ratio squared joins the joint's armature. The winding heats with its resistance times the current squared and cools to ambient through a thermal resistance; the battery's charge falls by the current the drive draws (duty × current) over its capacity, and its voltage falls in a straight line from full to empty with its charge. The winding's inductance is left out (its time constant is far below a step), and the current is not limited. A user's own controller can set the duty instead, with the gains at nothing.\n- **Keyframes**: poses in time, recorded from the jog panel, each driven joint's setpoint at that time. Build makes each keyframed joint's setpoint a shared parameter scheduled by them (Konjugate's own schedules, so a user can read and edit them in the parameters table), in straight lines between keyframes."),
    ("- **B. Drive it.** Motor mode (joint motor and battery from the component library); a jog panel of live setpoints; keyframed poses saved as schedules; a pick-and-place example.",
     "- **B. Drive it.** Motor mode (a motor node per joint and the robot's battery); a jog panel that poses the robot for setpoints and keyframes; keyframed poses saved as schedules; a pick-and-place example. Built (October 2026): each checked against the Newton-Euler reference, extended with the motors' electrics, the battery and the schedules, to rounding."),
    ("Later steps, in later milestones: keyframed poses saved as schedules; the sizing view",
     "Later steps, in later milestones: the sizing view"),
])

edit('AGENTS.md', [
    ("`lib/examples.mjs`: drawings to start from.",
     "`lib/pose.mjs`: the drawing posed by joint values, for the jog panel. `lib/examples.mjs`: drawings to start from."),
])
print('docs edited')
