/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The robots the window offers from MuJoCo Menagerie: arms on a fixed base, which the drawing holds today (a robot that
// walks or rolls needs contact, which comes later), each named as its maker names it. Nothing here is shipped with
// the toolbox: a robot is fetched when the user asks, its licence shown first (lib/menagerie.mjs).

export const robots = Object.freeze([
    { folder: 'trs_so_arm100', name: 'SO-ARM100', maker: 'The Robot Studio', what: 'A desktop arm of five joints and a gripper, on smart servos.' },
    { folder: 'universal_robots_ur5e', name: 'UR5e', maker: 'Universal Robots', what: 'An industrial arm of six joints.' },
    { folder: 'franka_emika_panda', name: 'Panda', maker: 'Franka Emika', what: 'A research arm of seven joints and a two-finger gripper.' }
]);

export const robotByFolder = (folder) => robots.find((robot) => robot.folder === folder) ?? null;

// A stored file's name for a path in a model's folder: the store keeps names to letters, digits and a few marks, so
// "/" is written as "^" (and the path is kept in the file's details too).
export const storedName = (path) => path.split('/').join('^');
