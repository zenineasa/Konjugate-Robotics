/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Will these motors do it? The sizing view's sums, from a run Konjugate made of the built robot: the series its
// launcher host answers with (`branch.series[node][symbol] = [[time, value], ...]`, thinned by the host to a few
// hundred points, so a share of time is counted from those and is about right, not exact), and the drawing.
//
// A motor in motor mode, at its joint: its current is (duty × supply voltage - kt N ω) / R and its torque N kt times
// that (lib/modelBuilder.mjs), the duty held to ±1. So at a speed ω and a supply voltage V the torque it can give lies
// between N kt (-V - kt N ω) / R and N kt (V - kt N ω) / R: a band of slope -(kt N)² / R, as wide as twice its stall
// torque N kt V / R, and an operating point on its edge is a motor at full duty, giving all it can. The band narrows
// as the battery drains. Pure, no dependencies.

import { batteryOf, defaultMotor, structure } from './drawing.mjs';

// The states a sizing run asks the host for: each motor's, its joint's speed, the battery's.
export const sizingSignals = Object.freeze(['duty', 'current', 'torque', 'windingTemperature', 'supplyVoltage', 'jointSpeed', 'voltage', 'stateOfCharge']);

// At or beyond this share of full duty, a motor counts as giving all it can.
const fullDuty = 0.999;
// A winding this close to its limit (kelvin) is called close.
const closeToLimit = 15;
// A motor at full duty for more than this share of the run struggles to keep up.
const struggling = 0.2;

// The robot's motors: each motor-mode joint, its node's name in the model, the solid it turns (whose node holds the
// joint's speed) and its motor with every field filled in.
export function motorsOf(drawing) {
    const { parentJoint } = structure(drawing);
    const carried = new Map([...parentJoint].map(([solidId, { joint }]) => [joint.id, solidId]));
    return drawing.joints.filter((joint) => joint.drive?.mode === 'motor' && carried.has(joint.id)).map((joint) => ({
        joint: joint.id, name: joint.name, node: `${joint.name} motor`, linear: joint.kind === 'slider',
        solid: drawing.solids.find(({ id }) => id === carried.get(joint.id)).name,
        motor: { ...defaultMotor(joint.kind), ...joint.drive.motor }
    }));
}

// At its joint, on a supply of `voltage`: the torque it gives held still at full duty (stall), the speed it reaches
// with no load (no-load), and the most and least torque it can give at a speed.
export function envelope(motor, voltage) {
    const { resistance: R, torqueConstant: kt, gearRatio: N } = motor;
    const constant = kt * N;
    return {
        stallTorque: (constant * voltage) / R,
        noLoadSpeed: voltage / constant,
        most: (speed) => (constant * (voltage - constant * speed)) / R,
        least: (speed) => (constant * (-voltage - constant * speed)) / R
    };
}

const seriesOf = (branch, node, symbol) => branch?.series?.[node]?.[symbol] ?? [];

// The time each point stands for: half the gap to the point before it and half to the one after.
function spans(points) {
    return points.map((point, index) => {
        const before = index > 0 ? point[0] - points[index - 1][0] : 0;
        const after = index + 1 < points.length ? points[index + 1][0] - point[0] : 0;
        return (before + after) / 2;
    });
}

// One run, motor by motor: its operating points (speed and torque at the joint), how hard it worked and how hot it
// got, with a verdict. Then the battery: the charge the run used and its lowest voltage (`battery`), and the battery
// itself (`supply`). `drawing` is the robot the run was of: for a what-if, the robot with its change, so the band is
// the changed motor's or battery's.
export function summarize(branch, drawing) {
    const battery = batteryOf(drawing);
    const motors = motorsOf(drawing).map((entry) => {
        const torques = seriesOf(branch, entry.node, 'torque');
        const speeds = seriesOf(branch, entry.solid, 'jointSpeed');
        const duties = seriesOf(branch, entry.node, 'duty');
        const temperatures = seriesOf(branch, entry.node, 'windingTemperature');
        const currents = seriesOf(branch, entry.node, 'current');
        const weights = spans(duties);
        const total = weights.reduce((sum, weight) => sum + weight, 0);
        const atFullDuty = total ? duties.reduce((sum, [, duty], index) => (Math.abs(duty ?? 0) >= fullDuty ? sum + weights[index] : sum), 0) / total : 0;
        const peak = (points) => points.reduce((largest, [, value]) => Math.max(largest, Math.abs(value ?? 0)), 0);
        const hottest = temperatures.length ? Math.max(...temperatures.map(([, value]) => value ?? -Infinity)) : entry.motor.ambientTemperature;
        const full = envelope(entry.motor, battery.fullVoltage);
        const empty = envelope(entry.motor, battery.emptyVoltage);
        const summary = {
            ...entry,
            points: torques.map(([time, torque], index) => [speeds[index]?.[1] ?? 0, torque ?? 0, time]),
            peakTorque: peak(torques), peakSpeed: peak(speeds), peakCurrent: peak(currents), atFullDuty, hottest,
            stallTorque: full.stallTorque, noLoadSpeed: full.noLoadSpeed, emptyStallTorque: empty.stallTorque
        };
        return { ...summary, verdict: verdictOf(summary) };
    });
    const charge = seriesOf(branch, 'Battery', 'stateOfCharge');
    const voltages = seriesOf(branch, 'Battery', 'voltage');
    return {
        motors,
        // The battery the run had, full and empty, for the bands.
        supply: battery,
        battery: {
            chargeUsed: charge.length ? charge[0][1] - charge.at(-1)[1] : null,
            lowestVoltage: voltages.length ? Math.min(...voltages.map(([, value]) => value)) : null
        }
    };
}

// What a motor's run says about it: `level` is 'ok', 'close' (warm, or at full duty now and then) or 'over' (its
// winding past its limit, or at full duty for much of the run), with the reasons in words.
export function verdictOf({ hottest, motor, atFullDuty }) {
    const reasons = [];
    let level = 'ok';
    const raise = (to) => { if (to === 'over' || level === 'ok') level = to; };
    const celsius = (kelvin) => `${(kelvin - 273.15).toFixed(0)} °C`;
    if (hottest > motor.windingLimit) { raise('over'); reasons.push(`its winding reaches ${celsius(hottest)}, past its ${celsius(motor.windingLimit)}`); }
    else if (hottest > motor.windingLimit - closeToLimit) { raise('close'); reasons.push(`its winding reaches ${celsius(hottest)}, close to its ${celsius(motor.windingLimit)}`); }
    if (atFullDuty > struggling) { raise('over'); reasons.push(`it is at full duty ${(100 * atFullDuty).toFixed(0)}% of the run, giving all it can`); }
    else if (atFullDuty > 0) { raise('close'); reasons.push(`it is at full duty ${(100 * atFullDuty).toFixed(0)}% of the run`); }
    return { level, reasons };
}

// The what-ifs, as the toolbox's scenarios run them (packages/toolbox/addon.json). Each changes one thing from a time in
// the run (`forkAt`, seconds) to its end (`runTime`):
// - heavierSolid: `solid` (a name) weighs `mass` kilograms;
// - weakerMotor: `motor` (a joint's name) gives `share` (0 to 1) of its torque per ampere;
// - tiredBattery: the battery gives `drop` volts less at every charge.
// Returns the options for runScenario, a sentence saying what changed and the drawing with the change made (for the
// band of the changed motor or battery). Throws a message a user can act on.
export function whatIf(drawing, { scenarioId, forkAt, runTime, solid, mass, motor, share, drop }) {
    if (!(runTime > 0)) throw new Error('Set how long to run first.');
    if (!(forkAt >= 0 && forkAt < runTime)) throw new Error(`The change must start at or after 0 s and before the run ends at ${runTime} s.`);
    const held = (value) => [[0, value], [runTime - forkAt, value]];
    const options = (byParameter) => ({ forkAt, runTime, supplied: { byParameter }, signals: [...sizingSignals] });
    const from = forkAt > 0 ? ` from ${forkAt} s` : '';
    const changed = structuredClone(drawing);
    if (scenarioId === 'heavierSolid') {
        const target = changed.solids.find(({ name }) => name === solid);
        if (!target) throw new Error('Choose the solid that gets heavier.');
        if (!(mass > 0)) throw new Error('Its mass must be more than zero.');
        target.mass = { by: 'mass', value: mass };
        return { options: options({ solidMass: { entities: [solid], samples: { [solid]: held(mass) } } }), changed, change: `${solid} weighs ${formatMass(mass)}${from}` };
    }
    if (scenarioId === 'weakerMotor') {
        const joint = changed.joints.find(({ name }) => name === motor);
        if (!joint || joint.drive?.mode !== 'motor') throw new Error('Choose the motor that gets weaker.');
        if (!(share > 0 && share <= 2)) throw new Error('Its share of the torque must be more than 0% and at most 200%.');
        const full = { ...defaultMotor(joint.kind), ...joint.drive.motor };
        const torqueConstant = full.torqueConstant * share;
        joint.drive.motor = { ...full, torqueConstant };
        return { options: options({ torqueConstant: { entities: [motor], samples: { [motor]: held(torqueConstant) } } }), changed, change: `${motor}'s motor gives ${(100 * share).toFixed(0)}% of its torque per ampere${from}` };
    }
    if (scenarioId === 'tiredBattery') {
        const battery = batteryOf(drawing);
        if (!(drop > 0 && drop < battery.emptyVoltage)) throw new Error(`The drop must be more than 0 V and less than the battery's empty voltage, ${battery.emptyVoltage} V.`);
        changed.battery = { ...battery, fullVoltage: battery.fullVoltage - drop, emptyVoltage: battery.emptyVoltage - drop };
        return {
            options: options({
                batteryFullVoltage: { entities: ['Battery'], samples: { Battery: held(changed.battery.fullVoltage) } },
                batteryEmptyVoltage: { entities: ['Battery'], samples: { Battery: held(changed.battery.emptyVoltage) } }
            }),
            changed, change: `the battery gives ${drop} V less${from}`
        };
    }
    throw new Error(`There is no what-if ${scenarioId}.`);
}

const formatMass = (kilograms) => (kilograms < 1 ? `${(1000 * kilograms).toFixed(0)} g` : `${kilograms.toFixed(2)} kg`);
