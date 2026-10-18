/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Smart servos by their datasheets, for a robot's servo joints: how much current one draws for the torque it gives, and
// how much torque it can give at all. Each figure says where it is from; a figure worked out from others says how.
//
// A model in MuJoCo Menagerie may give its servos a different torque limit from the servo's own (the SO-ARM100's model
// allows 3.5 N·m; its servos are rated at 1.91 N·m at stall): building with a servo here holds each to its datasheet.

const kilogramCentimetre = 9.80665 / 100;

export const servos = Object.freeze({
    sts3215: Object.freeze({
        name: 'Feetech STS3215 (7.4 V, 19.5 kg·cm)',
        // Vendor listings give these at 6 V: Core Electronics and RobotShop, October 2026.
        sources: [
            'https://core-electronics.com.au/feetech-sts3215-smart-servo.html',
            'https://www.robotshop.com/products/magnetic-encoding-servo-sts3215-74v-19kgcm'
        ],
        voltage: 7.4,
        stallTorque: 19.5 * kilogramCentimetre,
        stallCurrent: 2,
        idleCurrent: 0.15,
        // Worked out: the torque at stall over the current beyond idle it draws for it.
        get torqueConstant() { return this.stallTorque / (this.stallCurrent - this.idleCurrent); },
        // Worked out: the listed 6 V over the 2 A at stall, when the motor turns no more and its winding takes it all.
        windingResistance: 6 / 2
    })
});

export const servoById = (id) => servos[id] ?? null;
