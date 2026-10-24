/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The drawing importer, in the two steps the window asks for:
// - open: reads a drawing from a JSON file the user chose (role `drawing`: one written by a script, say, or kept from
//   another project), checks it and returns it for the window to show;
// - build: builds the Konjugate model from the drawing the window sends (options.drawing; lib/modelBuilder.mjs).

import { readDrawing } from '../lib/drawing.mjs';
import { buildModel } from '../lib/modelBuilder.mjs';

const failure = (message) => ({ ok: false, report: { errors: [message], warnings: [] } });

export default async function importDrawing({ files = [], options = {} }) {
    if (options.step === 'open') {
        const file = files.find((candidate) => candidate.role === 'drawing');
        if (!file) return failure('Choose a drawing file first.');
        try {
            const drawing = readDrawing(JSON.parse(file.text));
            return { ok: true, data: { step: 'open', drawing, name: file.name }, report: { errors: [], warnings: [] } };
        } catch (error) {
            return failure(`${file.name} is not a drawing this window can open: ${error.message}`);
        }
    }
    if (options.step !== 'build') return failure(`The drawing importer has no step "${options.step}".`);
    let drawing;
    try {
        drawing = readDrawing(options.drawing);
    } catch (error) {
        return failure(error.message);
    }
    try {
        const built = buildModel(drawing, { timeStep: options.timeStep ?? 0.001, runTime: options.runTime ?? 10 });
        const { document, data, report } = built;
        return {
            ok: true, document, parameterIndex: built.parameterIndex, data,
            report: {
                errors: [], warnings: report.warnings, notes: report.notes,
                summary: `${drawing.name || 'The robot'}: ${data.solids} solids and ${data.joints} joints`
            }
        };
    } catch (error) {
        return failure(error.message);
    }
}
