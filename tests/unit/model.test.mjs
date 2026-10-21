/* Copyright © 2026 Zenin Easa Panthakkalakath */

import assert from 'node:assert/strict';
import test from 'node:test';
import { addFrameAt, addJoint, checkDrawing, structure, updateJoint } from '../../packages/toolbox/lib/drawing.mjs';
import { exampleDrawing, examples } from '../../packages/toolbox/lib/examples.mjs';
import { writeMjcf } from '../../packages/toolbox/lib/mjcfWriter.mjs';
import { add, evaluate, multiply } from '../../packages/toolbox/lib/expressions.mjs';
import { buildModel, camelKey, simulationLimits } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { shapeMesh, snapPoints, surfaceNormal, volume } from '../../packages/toolbox/lib/shapes.mjs';
import importDrawing from '../../packages/toolbox/importers/drawing.mjs';

test('every example is a drawing with nothing wrong in it', () => {
    for (const { id } of examples) assert.deepEqual(checkDrawing(exampleDrawing(id)).errors, [], id);
});

test('the MJCF nests bodies as the drawing joins them, a fixed joint welds, a servo is a position actuator', () => {
    const mjcf = writeMjcf(exampleDrawing('twoLinkArm'));
    assert.equal((mjcf.match(/<body /g) ?? []).length, 4);
    assert.equal((mjcf.match(/<joint /g) ?? []).length, 3, 'the fixed mount is no joint element');
    assert.equal((mjcf.match(/<position /g) ?? []).length, 3);
    assert.match(mjcf, /limited="true" range="-1\.57079632679 1\.57079632679"/);
    assert.match(mjcf, /contype="0" conaffinity="0"/);
    // Each body inside its parent's: the forearm's is the deepest, closed first.
    assert.match(mjcf, /<body name="s1"[\s\S]*<body name="s5"[\s\S]*<body name="s9"[\s\S]*<body name="s13"[\s\S]*<\/body>\s*<\/body>\s*<\/body>\s*<\/body>/);
});

test('the MJCF has a slider as a slide joint, and a solid joined to nothing floats on a free joint', () => {
    assert.match(writeMjcf(exampleDrawing('cartPendulums')), /<joint name="j\d+" type="slide"/);
    const pendulum = exampleDrawing('pendulum');
    const floating = { ...pendulum, joints: [], frames: [] };
    assert.equal((writeMjcf(floating).match(/<freejoint\/>/g) ?? []).length, 2);
    // Its angles are radians, as the drawing's are: MuJoCo would read a range in degrees otherwise.
    assert.match(writeMjcf(pendulum), /<compiler angle="radian"\/>/);
    // A ball's limit is its most turn, MuJoCo's range from 0.
    const limitedBall = updateJoint(pendulum, pendulum.joints.find(({ name }) => name === 'Pivot').id, { kind: 'ball' });
    const ballJoint = limitedBall.joints.find(({ name }) => name === 'Pivot');
    assert.match(writeMjcf(updateJoint(limitedBall, ballJoint.id, { limits: { lower: 0, upper: 0.3 } })), /type="ball" pos="[^"]*" limited="true" range="0 0.3"/);
});

test('the model: the Ground and a node per solid, a relationship per joint between the two solids it joins', () => {
    const drawing = exampleDrawing('twoLinkArm');
    const { document, data } = buildModel(drawing);
    assert.deepEqual(document.nodes.map(({ name }) => name), ['Ground', 'Base', 'Turret', 'Upper arm', 'Forearm']);
    assert.deepEqual(document.edges.map(({ name }) => name), ['Mount', 'Waist', 'Shoulder', 'Elbow']);
    const [ground, base, turret, upper] = document.nodes;
    assert.deepEqual(document.edges.map(({ source, target }) => [source.nodeId, target.nodeId]),
        [[ground.id, base.id], [base.id, turret.id], [turret.id, upper.id], [upper.id, document.nodes[4].id]]);
    // No node is computed outside the graph: no provider, no implementation, only relationships' equations.
    assert.ok(document.nodes.every((node) => !node.implementation));
    assert.ok(document.edges.every((edge) => !edge.implementation && edge.equationModel.outputs.length));
    assert.deepEqual(turret.states.slice(0, 4).map(({ symbol }) => symbol), ['jointSpeed', 'jointAngle', 'jointAcceleration', 'jointTorque']);
    assert.deepEqual(turret.numerics, { substepsPerGlobalStep: 1, integration: 'semiImplicit' });
    assert.deepEqual(turret.sourceTerms.map(({ state }) => state), ['jointSpeed', 'jointAngle'], 'the speed first, then the angle from it');
    assert.ok(!base.states.some(({ symbol }) => symbol === 'jointAngle'), 'a solid carried by a fixed joint has no joint states');
    assert.equal(turret.appearance.parts[0].prefix, '');
    // Each relationship writes only onto the two solids it joins: the child, and the parent (its share of the
    // articulated inertia and bias force), never the Ground.
    const owner = new Map(document.nodes.flatMap((node) => node.states.map((state) => [state.id, node.id])));
    for (const edge of document.edges) {
        for (const output of edge.equationModel.outputs) {
            const expected = output.role === 'target' ? edge.target.nodeId : edge.source.nodeId;
            assert.equal(owner.get(output.stateId), expected, `${edge.name}: ${output.role}`);
            assert.equal(output.setsValue, true);
            assert.notEqual(owner.get(output.stateId), ground.id);
        }
        for (const binding of edge.equationModel.bindings.filter(({ kind }) => kind === 'state')) {
            assert.ok([edge.source.nodeId, edge.target.nodeId].includes(owner.get(binding.stateId)), `${edge.name} reads ${binding.symbol} from a solid it does not join.`);
        }
    }
    // The shoulder's setpoint is a live parameter of its edge, read by its equations.
    const shoulder = document.edges[2];
    const setpoint = shoulder.parameters.find(({ symbol }) => symbol === 'setpoint');
    assert.equal(setpoint.mode, 'live');
    assert.deepEqual(setpoint.control, { minimum: -1.570796326795, maximum: 1.570796326795, step: 0.007853981634 });
    assert.ok(shoulder.equationModel.bindings.some(({ parameterId }) => parameterId === setpoint.id));
    // A solid's mass is one shared parameter, linked from the edge that carries it.
    assert.equal(document.sharedParameters.filter(({ unit }) => unit === 'kg').length, 4, 'a mass per solid');
    const mass = shoulder.parameters.find(({ symbol }) => symbol === 'mass');
    assert.equal(document.sharedParameters.find(({ id }) => id === mass.sharedParameterId).name, 'Upper arm mass');
    assert.equal(data.solids, 4);
    assert.equal(data.joints, 4);
    assert.equal(data.driven, 3);
    assert.equal(data.nodes, 5);
    assert.equal(data.edges, 4);
    assert.deepEqual(document.runConfigurations[0].globalTimeStep, 0.001);
});

test('a drawn equation reads as LaTeX and evaluates as the engine would', () => {
    const { document } = buildModel(exampleDrawing('pendulum'));
    const pivot = document.edges.find(({ name }) => name === 'Pivot');
    const acceleration = pivot.equationModel.outputs.find(({ stateId }) => document.nodes[2].states.find(({ id }) => id === stateId)?.symbol === 'jointAcceleration');
    assert.equal(acceleration.mathJson, 'qdd');
    const qdd = pivot.equationModel.definitions.find(({ symbol }) => symbol === 'qdd');
    assert.match(qdd.latex, /\\frac\{/);
    // Every definition and output is built from operations the engine knows.
    const known = new Set(['Add', 'Multiply', 'Negate', 'Divide', 'Sin', 'Cos', 'Tanh', 'Arcsin', 'Arctan2', 'Min', 'Max', 'Which', 'Greater', 'Less']);
    const operators = (expression, found = new Set()) => {
        if (Array.isArray(expression)) { found.add(expression[0]); expression.slice(1).forEach((item) => operators(item, found)); }
        return found;
    };
    for (const edge of document.edges) {
        for (const { mathJson } of [...edge.equationModel.definitions, ...edge.equationModel.outputs]) {
            for (const operator of operators(mathJson)) assert.ok(known.has(operator), `${edge.name} uses ${operator}.`);
        }
    }
});

test('add and multiply take lists of terms, but keep an expression whole', () => {
    // The bug this guards against: an expression passed as a term was flattened into its operator and arguments.
    assert.deepEqual(add('x', ['Multiply', 'y', 2]), ['Add', 'x', ['Multiply', 'y', 2]]);
    assert.deepEqual(add([['Multiply', 'y', 2], 'x'], 1), ['Add', ['Multiply', 'y', 2], 'x', 1]);
    assert.deepEqual(multiply('m', ['Add', 'a', 'b']), ['Multiply', 'm', ['Add', 'a', 'b']]);
    assert.equal(evaluate(add('x', multiply('y', 2), 3), { x: 1, y: 2 }), 8);
    assert.deepEqual(multiply(0, 'x'), 0);
    assert.deepEqual(add(1, 2), 3);
});

test('ball joints, solids joined to nothing and loops closed by any joint, several to a robot, all build', () => {
    const pendulum = exampleDrawing('pendulum');
    const arm = pendulum.solids.find(({ name }) => name === 'Arm').id;
    assert.deepEqual(simulationLimits(pendulum), []);
    // A ball: three speeds and a turn on the arm's node, integrated speeds first.
    const ball = updateJoint(pendulum, pendulum.joints.find(({ name }) => name === 'Pivot').id, { kind: 'ball' });
    const ballModel = buildModel(ball).document;
    const armNode = ballModel.nodes.find(({ name }) => name === 'Arm');
    assert.deepEqual(armNode.states.slice(0, 7).map(({ symbol }) => symbol), ['jointSpeed1', 'jointSpeed2', 'jointSpeed3', 'jointTurnW', 'jointTurnX', 'jointTurnY', 'jointTurnZ']);
    assert.deepEqual(armNode.sourceTerms.map(({ state }) => state), ['jointSpeed1', 'jointSpeed2', 'jointSpeed3', 'jointTurnW', 'jointTurnX', 'jointTurnY', 'jointTurnZ']);
    // Joined to nothing: a free joint from the Ground, a relationship like any other.
    const floating = { ...pendulum, joints: [], frames: [] };
    const floatingModel = buildModel(floating).document;
    assert.deepEqual(floatingModel.edges.map(({ name }) => name), ['Post floating', 'Arm floating']);
    assert.deepEqual(floatingModel.nodes.find(({ name }) => name === 'Post').sourceTerms.map(({ state }) => state),
        ['w1', 'w2', 'w3', 'vc1', 'vc2', 'vc3', 'x', 'y', 'z', 'turnW', 'turnX', 'turnY', 'turnZ']);
    // Its far end pinned to the Ground as well: a loop, closed by a relationship of its own.
    let frame = addFrameAt(pendulum, 'ground', [0.2, 0.035, 0.18], [0, 1, 0]);
    const groundFrame = frame.id;
    frame = addFrameAt(frame.drawing, arm, [0.2, 0.035, 0.18], [0, -1, 0]);
    const looped = addJoint(frame.drawing, groundFrame, frame.id, { kind: 'hinge', mate: false, name: 'Pin' }).drawing;
    assert.deepEqual(simulationLimits(looped), []);
    const loopModel = buildModel(looped).document;
    assert.deepEqual(loopModel.edges.map(({ name }) => name).sort(), ['Mount', 'Pin', 'Pivot']);
    // Which of the loop's joints closes it is the one the tree from the Ground reaches last; its force, moment, gap and
    // twist are states of the solid it holds.
    const closing = structure(looped).loops[0];
    const holderStates = loopModel.nodes.find(({ name }) => name === 'Arm').states.map(({ symbol }) => symbol);
    const key = camelKey(closing.name);
    for (const suffix of ['Force1', 'Force3', 'Moment1', 'Gap', 'Twist']) assert.ok(holderStates.includes(`${key}${suffix}`), `${key}${suffix}`);
    // Every kind of joint closes a loop, holding as many rows as it holds still: a ball 3, a hinge 5, a slider 5, fixed 6.
    for (const [kind, rows] of [['ball', 3], ['hinge', 5], ['slider', 5], ['fixed', 6]]) {
        const document = buildModel(updateJoint(looped, closing.id, { kind })).document;
        assert.equal(document.nodes.find(({ name }) => name === 'Ground').states.filter(({ symbol }) => symbol.startsWith('kb')).length, rows, kind);
    }
    // A second loop: the four-bar's crank pinned to its rocker as well.
    const fourBar = exampleDrawing('fourBar');
    const [crank, rocker] = ['Crank', 'Rocker'].map((name) => fourBar.solids.find((solid) => solid.name === name).id);
    frame = addFrameAt(fourBar, crank, [0.03, 0, 0.25], [0, 1, 0]);
    const onCrank = frame.id;
    frame = addFrameAt(frame.drawing, rocker, [0.03, 0, 0.25], [0, -1, 0]);
    const twice = addJoint(frame.drawing, onCrank, frame.id, { kind: 'hinge', mate: false, name: 'Second pin' }).drawing;
    assert.deepEqual(simulationLimits(twice), []);
    assert.equal(structure(twice).loops.length, 2);
    assert.equal(buildModel(twice).document.nodes.find(({ name }) => name === 'Ground').states.filter(({ symbol }) => symbol.startsWith('kb')).length, 10);
});

test('a drawing with an error is not built, and says why', () => {
    const drawing = { ...exampleDrawing('pendulum'), solids: [], frames: [], joints: [] };
    assert.throws(() => buildModel(drawing), /Add a solid to start/);
});

test('the importer builds from the window\'s drawing, and opens a drawing from a file', async () => {
    const built = await importDrawing({ options: { step: 'build', drawing: exampleDrawing('cartPendulums') } });
    assert.equal(built.ok, true);
    assert.match(built.report.summary, /Cart and pendulums: 4 solids and 4 joints/);
    assert.equal(built.document.runConfigurations[0].globalTimeStep, 0.001);
    const floating = await importDrawing({ options: { step: 'build', drawing: { ...exampleDrawing('pendulum'), joints: [], frames: [] } } });
    assert.equal(floating.ok, true);
    assert.match(floating.report.warnings.join(' '), /Post is not joined to the Ground, so it falls freely/);
    const pendulum = exampleDrawing('pendulum');
    const fixedPivot = updateJoint(pendulum, pendulum.joints.find(({ name }) => name === 'Pivot').id, { kind: 'fixed' });
    const refused = await importDrawing({ options: { step: 'build', drawing: { ...fixedPivot, solids: [], frames: [], joints: [] } } });
    assert.match(refused.report.errors[0], /Add a solid to start/);
    const opened = await importDrawing({ files: [{ role: 'drawing', name: 'arm.json', text: JSON.stringify(exampleDrawing('pendulum')) }], options: { step: 'open' } });
    assert.equal(opened.data.drawing.name, 'Pendulum');
    const unreadable = await importDrawing({ files: [{ role: 'drawing', name: 'x.json', text: '{}' }], options: { step: 'open' } });
    assert.match(unreadable.report.errors[0], /x\.json is not a drawing this window can open/);
});

test('shapes: volumes, outward normals, snap points on the surface, and closed meshes', () => {
    assert.ok(Math.abs(volume('capsule', { radius: 1, length: 2 }) - (2 * Math.PI + (4 / 3) * Math.PI)) < 1e-12);
    assert.deepEqual(surfaceNormal('box', { x: 0.1, y: 0.2, z: 0.3 }, [0.049, 0.01, 0.01]), [1, 0, 0]);
    assert.deepEqual(surfaceNormal('cylinder', { radius: 0.1, length: 0.4 }, [0, 0.01, 0.2]), [0, 0, 1]);
    assert.equal(snapPoints('box', { x: 1, y: 1, z: 1 }).length, 30);
    for (const shape of ['box', 'cylinder', 'sphere', 'capsule']) {
        const mesh = shapeMesh(shape, { x: 0.1, y: 0.1, z: 0.1, radius: 0.05, length: 0.1 });
        assert.equal(mesh.position.length, mesh.normal.length);
        assert.ok(mesh.index.every((index) => index < mesh.position.length / 3), shape);
    }
    assert.equal(camelKey('upper arm'), 'upperArm');
    assert.equal(camelKey('2nd link', new Set(), 'solid'), 'solid2ndLink');
});

test('structure is the same however often it is asked', () => {
    const drawing = exampleDrawing('cartPendulums');
    assert.deepEqual([...structure(drawing).parentJoint.keys()], [...structure(drawing).parentJoint.keys()]);
});
