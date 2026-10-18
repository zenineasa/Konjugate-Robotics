/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A scenario's two branches, the baseline and the fork, put side by side servo by servo: how hard each servo worked
// (its peak torque, and about how long it spent at its limit), how hot its winding got, and how much charge the battery
// gave. From the series a scenario run answers with (`branches[].series[node][state] = [[time, value], ...]`, thinned
// by the host to a few hundred points, so time at a limit is counted from those and is about right, not exact).
// No dependencies.

// A servo is at its limit when its torque is within this share of it.
const atLimitShare = 0.995;

function seriesOf(branch, node, symbol) {
    return branch.series?.[node]?.[symbol] ?? [];
}

// The time each point stands for: half the gap to the point before it and half to the one after.
function weights(points) {
    return points.map((point, index) => {
        const before = index > 0 ? point[0] - points[index - 1][0] : 0;
        const after = index + 1 < points.length ? points[index + 1][0] - point[0] : 0;
        return (before + after) / 2;
    });
}

// `servos` are [{ name (the joint), key (its states' prefix) }]; `limit(branchId, joint, time)` the servo's torque
// limit there, in N·m; `mechanism` the mechanism node's name. Returns { servos: [{ name, baseline, scenario }], battery }.
export function compareServos(answer, { servos, limit, mechanism }) {
    const branches = Object.fromEntries(answer.branches.map((branch) => [branch.id, branch]));
    const side = (branchId, servo) => {
        const branch = branches[branchId];
        const torques = seriesOf(branch, mechanism, `${servo.key}ServoTorque`);
        const temperatures = seriesOf(branch, `${servo.name} winding`, 'windingTemperature');
        const spans = weights(torques);
        return {
            peakTorque: torques.reduce((largest, [, value]) => Math.max(largest, Math.abs(value ?? 0)), 0),
            secondsAtLimit: torques.reduce((sum, [time, value], index) => (Math.abs(value ?? 0) >= atLimitShare * limit(branchId, servo.name, time) ? sum + spans[index] : sum), 0),
            peakWinding: temperatures.length ? Math.max(...temperatures.map(([, value]) => value ?? -Infinity)) - 273.15 : null
        };
    };
    const used = (branchId) => {
        const charge = seriesOf(branches[branchId], 'Battery', 'stateOfCharge');
        return charge.length ? charge[0][1] - charge.at(-1)[1] : null;
    };
    return {
        servos: servos.map((servo) => ({ name: servo.name, baseline: side('baseline', servo), scenario: side('scenario', servo) })),
        battery: { baseline: used('baseline'), scenario: used('scenario') }
    };
}

// What a fork changes, as the toolbox's scenarios take it: `built` is what the importer's build step answers with
// (mechanism, servos, stallTorque, payload, runTime). For heavierPayload, `mass` is the payload from the fork on, in kg;
// for weakerServo, `joint`'s servo is held to `torqueLimit` N·m from the fork on. Returns the run's options for
// runScenario (supplied paths are counted from the fork), the torque limit each servo had (for compareServos) and a
// sentence saying what changed.
export function forkChange(built, { scenarioId, forkAt, mass, joint, torqueLimit }) {
    const at = Math.min(Math.max(Number(forkAt) || 0, 0), built.runTime);
    const held = (value) => [[0, value], [Math.max(built.runTime - at, 0.01), value]];
    const options = (parameter, entity, value) => ({ forkAt: at, runTime: built.runTime, supplied: { byParameter: { [parameter]: { entities: [entity], samples: { [entity]: held(value) } } } }, signals: comparisonSignals(built.servos) });
    if (scenarioId === 'heavierPayload') {
        if (!built.payload) throw new Error('This robot was built without a payload to change.');
        const payload = Math.max(Number(mass) || 0, 0);
        return {
            run: options('payloadMass', built.payload.body, built.payload.own + payload),
            limit: () => built.stallTorque,
            change: `${payload.toFixed(2)} kg in the gripper from ${at} s, against ${built.payload.mass.toFixed(2)} kg as built`
        };
    }
    if (scenarioId === 'weakerServo') {
        if (!built.servos.some(({ name }) => name === joint)) throw new Error(`${joint} has no servo to weaken.`);
        const value = Math.max(Number(torqueLimit) || 0.01, 0.01);
        return {
            run: options('servoTorqueLimit', joint, value),
            limit: (branch, name, time) => (branch === 'scenario' && name === joint && time >= at ? value : built.stallTorque),
            change: `${joint}'s servo held to ${value} N·m from ${at} s, against ${built.stallTorque.toFixed(2)} N·m as built`
        };
    }
    throw new Error(`There is no scenario ${scenarioId}.`);
}

// The states a scenario run is asked to answer with, for compareServos.
export function comparisonSignals(servos) {
    return [...servos.map((servo) => `${servo.key}ServoTorque`), 'windingTemperature', 'stateOfCharge'];
}

// A sentence saying what changed most: the servo whose time at its limit grew most, or whose winding warmed most.
export function comparisonHeadline(comparison) {
    const worse = comparison.servos
        .map((servo) => ({ name: servo.name, extra: servo.scenario.secondsAtLimit - servo.baseline.secondsAtLimit, warmer: (servo.scenario.peakWinding ?? 0) - (servo.baseline.peakWinding ?? 0) }))
        .sort((a, b) => b.extra - a.extra || b.warmer - a.warmer);
    const first = worse[0];
    if (!first || (first.extra < 0.01 && first.warmer < 0.1)) return 'The servos meet it as they met the baseline: none spends longer at its limit or gets noticeably hotter.';
    const atLimit = worse.filter((servo) => servo.extra >= 0.01).map((servo) => servo.name);
    return atLimit.length
        ? `${atLimit.join(', ')} ${atLimit.length === 1 ? 'spends' : 'spend'} longer at ${atLimit.length === 1 ? 'its' : 'their'} limit; ${first.name} by about ${first.extra.toFixed(2)} s.`
        : `No servo spends longer at its limit; ${first.name}'s winding gets ${first.warmer.toFixed(1)} K hotter.`;
}
