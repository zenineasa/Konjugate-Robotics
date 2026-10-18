/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The window's reading of a MuJoCo model (lib/mjcf.mjs) against MuJoCo's own (describeMechanism.py, which compiles
// the model): joints, actuators and bodies alike, on small models written to exercise the rules (degrees, default
// classes, childclass, actuator shortcuts, includes) and on the Menagerie robots fetched to this computer (the
// SO-ARM100, UR5e, Panda and Go2; npm run fetch:model <folder>). A robot not fetched is skipped, saying so.

import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { mkdtemp, readdir, readFile, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { describeModel, parseXml } from '../../packages/toolbox/lib/mjcf.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';

const python = process.env.ROBOTICS_PYTHON ?? join(roboticsRoot, '.venv', process.platform === 'win32' ? 'Scripts/python.exe' : 'bin/python');
const describer = join(roboticsRoot, 'packages', 'engine', 'tools', 'describeMechanism.py');
const commit = '62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35';

function mujocoDescription(path) {
    const result = spawnSync(python, ['-I', describer, path], { encoding: 'utf8' });
    assert.equal(result.status, 0, result.stderr);
    return JSON.parse(result.stdout);
}

const near = (actual, expected, label) => {
    if (expected === null || actual === null) return assert.equal(actual, expected, label);
    assert.equal(actual.length, expected.length, label);
    actual.forEach((value, index) => assert.ok(Math.abs(value - expected[index]) < 1e-6, `${label}: ${actual} against ${expected}`));
};

function sameAsMuJoCo(ours, theirs) {
    assert.deepEqual(ours.bodies, theirs.bodies, 'bodies');
    assert.deepEqual(ours.joints.map(({ name, kind }) => [name, kind]), theirs.joints.map(({ name, kind }) => [name, kind]), 'joints');
    ours.joints.forEach((joint, index) => {
        near(joint.range, theirs.joints[index].range, `${joint.name} range`);
        // A free joint's starting position is its body's place, which is not a joint setting.
        if (joint.kind !== 'free') near([joint.initialPosition], [theirs.joints[index].initialPosition], `${joint.name} starting position`);
    });
    assert.deepEqual(ours.actuators.map(({ name, joint, kind }) => [name, joint, kind]), theirs.actuators.map(({ name, joint, kind }) => [name, joint, kind]), 'actuators');
    // Each body's place at the reference pose; a quaternion and its negative are the same orientation.
    ours.frames.forEach((frame, index) => {
        near(frame.position, theirs.frames[index].position, `${ours.bodies[index]} position`);
        const sign = Math.sign(frame.quaternion.reduce((sum, value, at) => sum + value * theirs.frames[index].quaternion[at], 0)) || 1;
        near(frame.quaternion.map((value) => value * sign), theirs.frames[index].quaternion, `${ours.bodies[index]} orientation`);
    });
    ours.actuators.forEach((actuator, index) => {
        near([actuator.gain], [theirs.actuators[index].gain], `${actuator.name} gain`);
        near(actuator.forceRange, theirs.actuators[index].forceRange, `${actuator.name} force range`);
    });
}

async function describeBoth(files, main) {
    const directory = await mkdtemp(join(tmpdir(), 'robotics-mjcf-'));
    for (const [path, text] of Object.entries(files)) await writeFile(join(directory, path), text);
    return { ours: describeModel(files, main), theirs: mujocoDescription(join(directory, main)) };
}

test('degrees by default, default classes, childclass, ref and the actuator shortcuts read as MuJoCo reads them', async () => {
    const robot = `<mujoco>
  <default>
    <joint damping="0.1"/>
    <default class="arm">
      <joint axis="0 1 0" range="-90 90"/>
      <position kp="20" forcerange="-2 2"/>
      <default class="wrist"><joint range="-45 45" ref="10"/></default>
    </default>
  </default>
  <worldbody>
    <body name="base" childclass="arm">
      <joint name="shoulder"/>
      <geom size="0.05" mass="1"/>
      <body name="forearm" pos="0.3 0 0">
        <joint name="wrist" class="wrist"/>
        <joint name="slider" type="slide" range="0 0.2" axis="1 0 0"/>
        <geom size="0.04" mass="0.5"/>
      </body>
    </body>
    <body name="loose" pos="1 0 0" euler="10 20 30"><freejoint name="float"/><geom size="0.1" mass="1"/>
      <body name="tilted" pos="0 0.2 0" axisangle="1 1 0 45"><geom size="0.02" mass="0.1"/>
        <body name="aimed" pos="0.1 0 0" xyaxes="0 1 0 -1 0 0"><geom size="0.02" mass="0.1"/>
          <body name="pointed" zaxis="1 1 1"><geom size="0.02" mass="0.1"/></body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="shoulderServo" joint="shoulder" class="arm"/>
    <motor name="wristMotor" joint="wrist" gear="1"/>
    <velocity name="sliderSpeed" joint="slider" kv="3"/>
    <general name="slidePosition" joint="slider" gainprm="40" biasprm="0 -40 -2" biastype="affine"/>
  </actuator>
</mujoco>`;
    const scene = '<mujoco><compiler angle="degree"/><include file="robot.xml"/><worldbody><geom type="plane" size="1 1 0.1"/></worldbody></mujoco>';
    const { ours, theirs } = await describeBoth({ 'scene.xml': scene, 'robot.xml': robot }, 'scene.xml');
    sameAsMuJoCo(ours, theirs);
    assert.deepEqual(ours.actuators.map(({ kind }) => kind), ['position', 'motor', 'other', 'position']);
    assert.ok(Math.abs(ours.joints[0].range[1] - Math.PI / 2) < 1e-12, 'A range in degrees reads in radians.');
});

test('a model with markup it cannot follow, or a class it does not define, is refused with the reason', () => {
    assert.throws(() => parseXml('<mujoco><worldbody></mujoco>'), /closes <mujoco> where <worldbody> is open/);
    assert.throws(() => describeModel({ 'a.xml': '<mujoco><worldbody><body><joint name="j" class="missing"/></body></worldbody></mujoco>' }, 'a.xml'), /class missing/);
    assert.throws(() => describeModel({ 'a.xml': '<mujoco><include file="b.xml"/></mujoco>' }, 'a.xml'), /no file b\.xml/);
});

// The fetched folder's files, by path, as the importer is given them.
async function folderFiles(directory) {
    const files = {};
    for (const entry of await readdir(directory, { recursive: true, withFileTypes: true })) {
        if (!entry.isFile()) continue;
        const path = join(entry.parentPath, entry.name);
        files[path.slice(directory.length + 1).split('\\').join('/')] = new Uint8Array(await readFile(path));
    }
    return files;
}

for (const folder of ['trs_so_arm100', 'universal_robots_ur5e', 'franka_emika_panda', 'unitree_go2']) {
    const directory = join(roboticsRoot, 'out', 'models', 'menagerie', commit, folder);
    const fetched = existsSync(join(directory, 'scene.xml'));
    test(`Menagerie's ${folder} reads as MuJoCo reads it`, { skip: !fetched && `not fetched (npm run fetch:model ${folder} -- --commit ${commit})` }, async () => {
        sameAsMuJoCo(describeModel(await folderFiles(directory), 'scene.xml'), mujocoDescription(join(directory, 'scene.xml')));
    });
}
