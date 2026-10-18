/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Moves for a robot's servos, as keyframes: at each time, where each joint is to be. A move becomes a stored schedule
// on each servo's setpoint (lib/mechanismModel.mjs), linear between keyframes, so it plays in every run, a fork's
// included, and a slider set during a run takes over from it.

// The SO-ARM100 picking something up in front of it and putting it down to its left, from folded and back: the poses
// chosen by where MuJoCo puts the gripper (folded about 0.14 m up by the base, down at 0.1 m in front and to the side,
// lifted at 0.2 m), its jaw opened to pick and to let go.
const folded = { Rotation: 0, Pitch: -3, Elbow: 3, Wrist_Pitch: 0.5, Wrist_Roll: 0, Jaw: 0 };
const lifted = { Rotation: 0, Pitch: -1.6, Elbow: 1.6, Wrist_Pitch: 0.6, Wrist_Roll: 0 };
const down = { Rotation: 0, Pitch: -1, Elbow: 1.4, Wrist_Pitch: 0.8, Wrist_Roll: 0 };
const side = (pose) => ({ ...pose, Rotation: 1.2 });

export const moves = Object.freeze({
    trs_so_arm100: Object.freeze({
        pickAndPlace: Object.freeze({
            name: 'Pick and place',
            what: 'From folded, reach down in front, grip, lift, turn to the left, put it down, let go and fold again, in 8 seconds.',
            keyframes: [
                [0, folded],
                [1, { ...lifted, Jaw: 1 }],
                [2, { ...down, Jaw: 1 }],
                [2.5, { ...down, Jaw: 0 }],
                [3.5, { ...lifted, Jaw: 0 }],
                [4.5, { ...side(lifted), Jaw: 0 }],
                [5.5, { ...side(down), Jaw: 0 }],
                [6, { ...side(down), Jaw: 1 }],
                [7, { ...side(lifted), Jaw: 1 }],
                [8, folded]
            ]
        })
    })
});

export const moveFor = (folder, id) => moves[folder]?.[id] ?? null;

// A move's schedule for one joint ({ interpolation, samples }), and where it starts.
export function jointSchedule(move, joint) {
    const samples = move.keyframes.filter(([, pose]) => joint in pose).map(([time, pose]) => [time, pose[joint]]);
    return samples.length ? { schedule: { interpolation: 'linear', samples }, start: samples[0][1] } : null;
}
