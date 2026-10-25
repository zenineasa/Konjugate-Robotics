/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Motors and supplies a joint's motor and the robot's battery can be set from, by name, as their makers give them.
//
// A motor is given by its datasheet: at its rated voltage V, its stall torque Ts and stall current Is, its no-load
// speed ω0 and its rated torque Tr (the most it gives for long), all at its output shaft, with its gear ratio N. The
// motor mode's model (lib/modelBuilder.mjs) has a winding resistance R, a torque constant kt, a gear efficiency η and a
// thermal resistance; from the datasheet:
// - held still at full duty the current is V / R, so R = V / Is;
// - free at full duty the back-EMF meets the supply, kt N ω0 = V, so kt = V / (N ω0);
// - held still the torque is η kt N Is = Ts, so η = Ts / (kt N Is): the share of the motor's torque the gears pass on;
// - giving its rated torque it draws Tr / (η kt N) and heats by R times that squared; taking that as what it can
//   give for long, its winding settles at its hottest from the default ambient, so the thermal resistance is the
//   hottest less 20 °C over that heat.
// So the model gives the datasheet's stall torque, stall current and no-load speed exactly, at any supply (a 7.4 V
// servo on 5 V gives 5 / 7.4 of each). Its heat capacity and rotor inertia are not on datasheets; each entry says
// where its own come from. A smart servo's protection comes with it as its datasheet gives it (`protection`: the most
// voltage it takes, and the current and share of its full output past which, for so many seconds on end, it turns
// itself off; lib/drawing.mjs), its output taken as the motor's duty. Pure, no dependencies.

import { batteryOf, defaultMotor, drivenJoints } from './drawing.mjs';

const kilogramCentimetre = 0.0980665; // N·m
const perSixtyDegrees = (seconds) => Math.PI / 3 / seconds; // rad/s, from a servo's "s/60°"
const ambient = defaultMotor('hinge').ambientTemperature;

export const motorCatalog = Object.freeze([
    {
        id: 'sts3215-c001', name: 'Feetech STS3215 (7.4 V, 1:345)', maker: 'Feetech', kind: 'hinge',
        what: 'The SO-ARM100\'s and SO-101\'s smart servo (ST-3215-C001), run on 5 V in their kits.',
        voltage: 7.4, stallTorque: 19.5 * kilogramCentimetre, stallCurrent: 2.5, noLoadSpeed: perSixtyDegrees(0.192),
        ratedTorque: 5 * kilogramCentimetre, gearRatio: 345,
        // Its rotor's inertia, so that at the joint it is MuJoCo Menagerie's SO-ARM100 armature, 0.1 kg·m².
        rotorInertia: 0.1 / 345 ** 2,
        // An estimate, for a motor of its size (about 15 g of copper and iron).
        thermalCapacitance: 7,
        // The servo turns its torque off past 70 °C.
        windingLimit: 273.15 + 70,
        protection: { mostVoltage: 7.4, cutoffCurrent: 2, cutoffDuty: 0.8, cutoffTime: 2 },
        source: 'Feetech STS3215 datasheet: 19.5 kg·cm and 2.5 A held still, 0.192 s/60° free, 5 kg·cm rated, at 7.4 V; 4 to 7.4 V; off past 2 A, or past 80% of its output, for 2 s'
    },
    {
        id: 'sts3215-c018', name: 'Feetech STS3215 (12 V, 1:345)', maker: 'Feetech', kind: 'hinge',
        what: 'The 12 V smart servo (ST-3215-C018) of the SO-101 Pro\'s follower arm, run on 12 V.',
        voltage: 12, stallTorque: 30 * kilogramCentimetre, stallCurrent: 2.7, noLoadSpeed: perSixtyDegrees(0.222),
        ratedTorque: 10 * kilogramCentimetre, gearRatio: 345,
        rotorInertia: 0.1 / 345 ** 2,
        thermalCapacitance: 7,
        windingLimit: 273.15 + 70,
        // Its specification gives its voltage range, and says it has overcurrent and overload protection without
        // saying where; none is assumed.
        protection: { mostVoltage: 14 },
        source: 'Feetech ST-3215-C018 specification: 30 kg·cm and 2.7 A held still, 0.222 s/60° free, 10 kg·cm rated, at 12 V; 4 to 14 V'
    }
]);

export const motorById = (id) => motorCatalog.find((entry) => entry.id === id) ?? null;

// A catalog motor as a joint's motor fields.
export function motorFromDatasheet(entry) {
    const { voltage: V, stallTorque, stallCurrent, noLoadSpeed, ratedTorque, gearRatio: N } = entry;
    const resistance = V / stallCurrent;
    const torqueConstant = V / (N * noLoadSpeed);
    const gearEfficiency = stallTorque / (torqueConstant * N * stallCurrent);
    if (!(gearEfficiency > 0 && gearEfficiency <= 1)) throw new Error(`${entry.name}'s datasheet gives a gear efficiency of ${gearEfficiency}, not between 0 and 1.`);
    const ratedCurrent = ratedTorque / (gearEfficiency * torqueConstant * N);
    return {
        ...defaultMotor(entry.kind),
        resistance, torqueConstant, gearRatio: N, gearEfficiency, rotorInertia: entry.rotorInertia,
        thermalResistance: (entry.windingLimit - ambient) / (resistance * ratedCurrent ** 2),
        thermalCapacitance: entry.thermalCapacitance, windingLimit: entry.windingLimit,
        ...(entry.protection ?? {})
    };
}

// The catalog motor a joint's motor is, or null when any of its fields differ (to a part in a million).
export function catalogMotorOf(motor) {
    const fields = Object.keys(defaultMotor('hinge'));
    const full = { ...defaultMotor('hinge'), ...motor };
    return motorCatalog.find((entry) => {
        const listed = motorFromDatasheet(entry);
        return fields.every((field) => Math.abs(full[field] - listed[field]) <= 1e-6 * Math.max(Math.abs(listed[field]), 1e-12));
    }) ?? null;
}

// Supplies the robot's battery can be set to. A supply on the mains does not run down: its full and empty voltages are
// the same, and its capacity is far more than any run takes; it holds its voltage up to its most current, and past it
// gives no more, its voltage falling. The LiPos' internal resistance is left at nothing (it varies with the cells and
// their age; type your own).
const mains = 1000 * 3600; // A·s
// The SO-ARM100 and SO-101 kits run their 7.4 V servos on a 5 V supply, and the 12 V ones on 12 V: The Robot Studio
// asks for a 12 V supply of at least 5 A for those. The 5 V one is taken at 4 A, as such supplies are commonly sold;
// the kits' makers do not say. Check the one you have.
export const supplyCatalog = Object.freeze([
    { id: 'lipo3s', name: '3-cell LiPo, 2.2 Ah', fullVoltage: 12.6, emptyVoltage: 9.9, capacity: 7920, internalResistance: 0, currentLimit: 0 },
    { id: 'lipo2s', name: '2-cell LiPo, 2.2 Ah', fullVoltage: 8.4, emptyVoltage: 6.6, capacity: 7920, internalResistance: 0, currentLimit: 0 },
    { id: 'mains5', name: '5 V, 4 A mains supply', fullVoltage: 5, emptyVoltage: 5, capacity: mains, internalResistance: 0, currentLimit: 4 },
    { id: 'mains12', name: '12 V, 5 A mains supply', fullVoltage: 12, emptyVoltage: 12, capacity: mains, internalResistance: 0, currentLimit: 5 }
]);
const supplyFields = ['fullVoltage', 'emptyVoltage', 'capacity', 'internalResistance', 'currentLimit'];
export const batteryOfSupply = (entry) => Object.fromEntries(supplyFields.map((field) => [field, entry[field]]));

export const supplyOf = (battery) => supplyCatalog.find((entry) => supplyFields.every((field) => Math.abs(entry[field] - (battery[field] ?? 0)) < 1e-9)) ?? null;

// A servo drive as a motor drive with `motor` on a supply of `voltage`, holding the joint as the servo held it: near
// its setpoint, with the duty below full, the motor gives η kt N (duty V - kt N ω) / R, so a proportional gain of the
// servo's stiffness over the motor's stall torque η kt N V / R keeps its stiffness, and a derivative gain of what is
// left of the servo's damping, after the motor's own back-EMF (η (kt N)² / R), over the stall torque keeps its damping.
// The setpoint is kept. Unlike the servo, its torque is held by the motor's band rather than a most torque.
export function motorDriveForServo(drive, motor, voltage) {
    const constant = motor.torqueConstant * motor.gearRatio;
    const stallTorque = (motor.gearEfficiency * constant * voltage) / motor.resistance;
    const backEmfDamping = (motor.gearEfficiency * constant * constant) / motor.resistance;
    return {
        drive: {
            ...drive, mode: 'motor',
            setpoint: drive.setpoint ?? 0,
            proportional: (drive.gain ?? 0) / stallTorque,
            derivative: Math.max(0, (drive.damping ?? 0) - backEmfDamping) / stallTorque,
            motor: { ...motor }
        },
        stallTorque
    };
}

// Every servo on a hinge in the robot's tree as catalog motor `id`, on the battery's full voltage, the battery first
// set to catalog supply `supply` when one is given. Returns the drawing and what was done, joint by joint: each turned
// servo with the most torque it had and the stall torque its motor gives; the servos left as they were (a slider's, as
// a rotary motor does not drive one).
export function servosToMotors(drawing, id, supply = null) {
    const entry = motorById(id);
    if (!entry) throw new Error(`There is no motor ${id} in the catalog.`);
    const next = structuredClone(drawing);
    if (supply) {
        const chosen = supplyCatalog.find((candidate) => candidate.id === supply);
        if (!chosen) throw new Error(`There is no supply ${supply} in the catalog.`);
        next.battery = batteryOfSupply(chosen);
    }
    const motor = motorFromDatasheet(entry);
    const voltage = batteryOf(next).fullVoltage;
    const turned = [];
    const left = [];
    for (const { id: jointId } of drivenJoints(drawing)) {
        const joint = next.joints.find((candidate) => candidate.id === jointId);
        if (joint.drive?.mode !== 'servo') continue;
        if (joint.kind !== entry.kind) { left.push(joint.name); continue; }
        const { drive, stallTorque } = motorDriveForServo(joint.drive, motor, voltage);
        turned.push({ name: joint.name, torqueLimit: joint.drive.torqueLimit, stallTorque });
        joint.drive = drive;
    }
    return { drawing: next, turned, left, motor: entry, voltage };
}
