/* Copyright © 2026 Zenin Easa Panthakkalakath */

// How a robot looks, as the importer gives it to Pose Visualizer (lib/mjcf.mjs bodyShapes: each body's visible geoms
// as one mesh in its own frame), against how MuJoCo places the same geoms (describeMechanism.py): each body's mesh,
// put where the body is at the reference pose, fills the same box in the world as MuJoCo's visible geoms of that body.
// On a small model of every primitive shape and placement, and on the Menagerie robots fetched to this computer.

import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { mkdtemp, readdir, readFile, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { compose } from '../../packages/toolbox/lib/frames.mjs';
import { readObj, readStl } from '../../packages/toolbox/lib/meshes.mjs';
import { bodyShapes, describeModel } from '../../packages/toolbox/lib/mjcf.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';

const python = process.env.ROBOTICS_PYTHON ?? join(roboticsRoot, '.venv', process.platform === 'win32' ? 'Scripts/python.exe' : 'bin/python');
const describer = join(roboticsRoot, 'packages', 'engine', 'tools', 'describeMechanism.py');
const commit = '62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35';

function mujocoDescription(path) {
    const result = spawnSync(python, ['-I', describer, path], { encoding: 'utf8', maxBuffer: 64 * 1024 * 1024 });
    assert.equal(result.status, 0, result.stderr);
    return JSON.parse(result.stdout);
}

// Each body's shape, placed where the body is at the reference pose, as a box in the world.
function worldBounds(files, main) {
    const { bodies, frames } = describeModel(files, main);
    const bounds = {};
    for (const [name, { mesh }] of bodyShapes(files, main)) {
        const frame = frames[bodies.indexOf(name)];
        const low = [Infinity, Infinity, Infinity];
        const high = [-Infinity, -Infinity, -Infinity];
        for (let at = 0; at < mesh.position.length; at += 3) {
            const { position } = compose(frame, { position: mesh.position.slice(at, at + 3), quaternion: [1, 0, 0, 0] });
            position.forEach((value, axis) => { low[axis] = Math.min(low[axis], value); high[axis] = Math.max(high[axis], value); });
        }
        bounds[name] = [low, high];
    }
    return bounds;
}

// The same bodies, filling the same boxes: to rounding for meshes (`tolerance`), and to the facets of a tessellation for
// primitive shapes (`primitiveTolerance`, a share of the box's size).
function sameBounds(ours, theirs, { tolerance = 2e-4, primitiveTolerance = 0, primitiveBodies = new Set() } = {}) {
    assert.deepEqual(Object.keys(ours).sort(), Object.keys(theirs).sort(), 'bodies with something to show');
    for (const [name, [low, high]] of Object.entries(theirs)) {
        const extent = Math.max(...high.map((value, axis) => value - low[axis]));
        const allowed = tolerance + (primitiveBodies.has(name) ? primitiveTolerance * extent : 0);
        [low, high].forEach((corner, side) => corner.forEach((value, axis) => {
            assert.ok(Math.abs(ours[name][side][axis] - value) <= allowed, `${name}: ${side ? 'highest' : 'lowest'} ${'xyz'[axis]} ${ours[name][side][axis]} against MuJoCo's ${value}`);
        }));
    }
}

async function onDisk(files) {
    const directory = await mkdtemp(join(tmpdir(), 'robotics-shapes-'));
    for (const [path, content] of Object.entries(files)) await writeFile(join(directory, path), content);
    return directory;
}

test('STL, text or binary, and OBJ files read as triangles', () => {
    const text = new TextEncoder().encode('solid one\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid one\n');
    assert.deepEqual(readStl(text), [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]]);
    const binary = new Uint8Array(84 + 50);
    const view = new DataView(binary.buffer);
    view.setUint32(80, 1, true);
    [0, 0, 0, 2, 0, 0, 0, 2, 0].forEach((value, index) => view.setFloat32(84 + 12 + index * 4, value, true));
    assert.deepEqual(readStl(binary), [[[0, 0, 0], [2, 0, 0], [0, 2, 0]]]);
    const obj = new TextEncoder().encode('v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1 4//1\n');
    assert.deepEqual(readObj(obj), [[[0, 0, 0], [1, 0, 0], [1, 1, 0]], [[0, 0, 0], [1, 1, 0], [0, 1, 0]]]);
});

test('every primitive shape and placement, and a scaled, moved mesh, fill the boxes MuJoCo puts them in', async () => {
    // A tetrahedron, as binary STL (MuJoCo takes no text STL, and a mesh needs a volume for its inertia).
    const corners = [[0, 0, 0], [0.1, 0, 0], [0, 0.2, 0], [0, 0, 0.05]];
    const faces = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
    const triangle = new Uint8Array(84 + faces.length * 50);
    const view = new DataView(triangle.buffer);
    view.setUint32(80, faces.length, true);
    faces.forEach((face, at) => face.forEach((corner, index) => corners[corner].forEach((value, axis) => view.setFloat32(84 + at * 50 + 12 + index * 12 + axis * 4, value, true))));
    const model = `<mujoco>
  <compiler angle="radian" meshdir="parts"/>
  <asset><mesh name="wedge" file="wedge.stl" scale="2 1 1" refpos="0.01 0 0"/><material name="red" rgba="1 0 0 1"/></asset>
  <default><default class="visual"><geom group="2" contype="0" conaffinity="0"/></default></default>
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <body name="base" pos="0 0 0.5" childclass="visual">
      <geom type="box" size="0.1 0.05 0.02" pos="0 0 0.1" euler="0 0 0.7" material="red"/>
      <geom type="mesh" mesh="wedge" pos="0.2 0 0" quat="0.9 0.1 0.2 0.3"/>
      <geom type="sphere" size="0.03" pos="0 0.2 0" group="3"/>
      <body name="arm" pos="0.3 0 0" axisangle="0 1 0 0.4">
        <joint type="hinge"/>
        <geom type="capsule" fromto="0 0 0 0.2 0.1 0" size="0.02"/>
        <geom type="cylinder" size="0.03 0.05" pos="0 0 0.1"/>
        <geom type="ellipsoid" size="0.02 0.03 0.04" pos="0.1 0 0" rgba="0 1 0 0"/>
      </body>
      <body name="ball" pos="0 0.3 0"><joint type="ball"/><geom type="sphere" size="0.04" rgba="0 0 1 1"/></body>
    </body>
  </worldbody>
</mujoco>`;
    const files = { 'scene.xml': model, 'parts/wedge.stl': triangle };
    const directory = await onDisk({ 'scene.xml': model });
    await import('node:fs/promises').then(({ mkdir }) => mkdir(join(directory, 'parts')));
    await writeFile(join(directory, 'parts', 'wedge.stl'), triangle);
    const shapes = bodyShapes(files, 'scene.xml');
    assert.equal(shapes.get('base').color, '#ff0000', 'A body takes the colour of its largest visible geom, here a material\'s.');
    sameBounds(worldBounds(files, 'scene.xml'), mujocoDescription(join(directory, 'scene.xml')).visualBounds,
        { primitiveTolerance: 0.03, primitiveBodies: new Set(['base', 'arm', 'ball']) });
});

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
    test(`Menagerie's ${folder} looks where MuJoCo puts it`, { skip: !fetched && `not fetched (npm run fetch:model ${folder} -- --commit ${commit})` }, async () => {
        const files = await folderFiles(directory);
        const description = mujocoDescription(join(directory, 'scene.xml'));
        sameBounds(worldBounds(files, 'scene.xml'), description.visualBounds, { primitiveTolerance: 0.03, primitiveBodies: new Set(description.bodies) });
    });
}
