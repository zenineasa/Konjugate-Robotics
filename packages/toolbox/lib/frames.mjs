/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Frames as MuJoCo's models write them: a position and a unit quaternion [w, x, y, z], composed parent to child.
// An element's orientation in MJCF is one of quat, axisangle, euler, xyaxes or zaxis (MuJoCo refuses more than one);
// `orientationOf` reads whichever it has, in the compiler's angle unit and Euler sequence. No dependencies.

export const identity = Object.freeze({ position: [0, 0, 0], quaternion: [1, 0, 0, 0] });

export function multiply([aw, ax, ay, az], [bw, bx, by, bz]) {
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw
    ];
}

export function normalise(quaternion) {
    const length = Math.hypot(...quaternion);
    if (!(length > 0)) throw new Error('A quaternion of zero length has no orientation.');
    return quaternion.map((value) => value / length);
}

// A vector turned by a unit quaternion.
export function rotate(quaternion, [x, y, z]) {
    const [, rx, ry, rz] = multiply(multiply(quaternion, [0, x, y, z]), [quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]]);
    return [rx, ry, rz];
}

// The child's frame in the parent's parent: the parent's frame followed by the child's, relative to it.
export function compose(parent, child) {
    const turned = rotate(parent.quaternion, child.position);
    return {
        position: parent.position.map((value, index) => value + turned[index]),
        quaternion: normalise(multiply(parent.quaternion, child.quaternion))
    };
}

export function fromAxisAngle([x, y, z], angle) {
    const length = Math.hypot(x, y, z);
    if (!(length > 0)) throw new Error('An axis of zero length has no direction.');
    const half = angle / 2;
    const scale = Math.sin(half) / length;
    return [Math.cos(half), x * scale, y * scale, z * scale];
}

// From a rotation matrix's rows (row-major, 3 by 3).
export function fromMatrix([m00, m01, m02, m10, m11, m12, m20, m21, m22]) {
    const trace = m00 + m11 + m22;
    if (trace > 0) {
        const s = Math.sqrt(trace + 1) * 2;
        return normalise([0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]);
    }
    if (m00 > m11 && m00 > m22) {
        const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
        return normalise([(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]);
    }
    if (m11 > m22) {
        const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
        return normalise([(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]);
    }
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    return normalise([(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]);
}

const cross = ([ax, ay, az], [bx, by, bz]) => [ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx];
const unit = (vector) => {
    const length = Math.hypot(...vector);
    if (!(length > 0)) throw new Error('A direction of zero length.');
    return vector.map((value) => value / length);
};
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

const numbers = (text) => String(text).trim().split(/\s+/).filter(Boolean).map(Number);

// An element's orientation from its attributes (resolved already, defaults applied), as MuJoCo reads it. `angle(value)`
// converts the compiler's unit to radians; `sequence` is the compiler's eulerseq.
export function orientationOf(attributes, { angle = (value) => value, sequence = 'xyz' } = {}) {
    const given = ['quat', 'axisangle', 'euler', 'xyaxes', 'zaxis'].filter((name) => attributes[name] !== undefined);
    if (given.length > 1) throw new Error(`An element is given its orientation as ${given.join(' and ')}; MuJoCo takes one.`);
    if (attributes.quat !== undefined) return normalise(numbers(attributes.quat));
    if (attributes.axisangle !== undefined) {
        const [x, y, z, value] = numbers(attributes.axisangle);
        return fromAxisAngle([x, y, z], angle(value));
    }
    if (attributes.euler !== undefined) {
        const values = numbers(attributes.euler).map(angle);
        if (!/^[xyzXYZ]{3}$/.test(sequence)) throw new Error(`The Euler sequence ${sequence} is not three axes.`);
        let quaternion = [1, 0, 0, 0];
        [...sequence].forEach((letter, index) => {
            const axis = { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] }[letter.toLowerCase()];
            const turn = fromAxisAngle(axis, values[index]);
            // Lower case: about the axis as it has turned so far (intrinsic); upper case: about the fixed axis.
            quaternion = letter === letter.toLowerCase() ? multiply(quaternion, turn) : multiply(turn, quaternion);
        });
        return normalise(quaternion);
    }
    if (attributes.xyaxes !== undefined) {
        const values = numbers(attributes.xyaxes);
        const x = unit(values.slice(0, 3));
        const yRaw = values.slice(3, 6);
        const y = unit(yRaw.map((value, index) => value - dot(yRaw, x) * x[index]));
        const z = cross(x, y);
        return fromMatrix([x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]]);
    }
    if (attributes.zaxis !== undefined) {
        // The shortest turn taking z to the given direction.
        const z = unit(numbers(attributes.zaxis));
        const axis = cross([0, 0, 1], z);
        const sine = Math.hypot(...axis);
        if (sine < 1e-12) return z[2] > 0 ? [1, 0, 0, 0] : [0, 1, 0, 0];
        return fromAxisAngle(axis, Math.atan2(sine, z[2]));
    }
    return [1, 0, 0, 0];
}

export function frameOf(attributes, options) {
    return { position: attributes.pos === undefined ? [0, 0, 0] : numbers(attributes.pos), quaternion: orientationOf(attributes, options) };
}
