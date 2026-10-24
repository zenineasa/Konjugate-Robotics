/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The drawing importer, in the steps the window asks for:
// - open: reads a drawing from a JSON file the user chose (role `drawing`: one written by a script, say, or kept from
//   another project), checks it and returns it for the window to show;
// - model: reads a MuJoCo model (role `model`: its MJCF and the meshes it reads, kept on this computer, downloaded from
//   MuJoCo Menagerie or brought in from disk) into a drawing, with a report of what came in and what was left out
//   (lib/mjcf.mjs, lib/mjcfImport.mjs). It derives the solids' looks (role `looks`), and a Menagerie robot's licence
//   (role `notice`), which the project keeps in place of the model's files: a project built from it shows the robot
//   and carries its notice wherever it is opened, with nothing to fetch;
// - looks: the looks kept with the project, for the window to show the imported solids again;
// - build: builds the Konjugate model from the drawing the window sends (options.drawing; lib/modelBuilder.mjs), the
//   imported solids looking as their looks have them.

import { readDrawing } from '../lib/drawing.mjs';
import { bodyLook, readModel } from '../lib/mjcf.mjs';
import { importModel } from '../lib/mjcfImport.mjs';
import { buildModel } from '../lib/modelBuilder.mjs';

const failure = (message) => ({ ok: false, report: { errors: [message], warnings: [] } });

// The looks kept with the project ({ format, looks: { solidId: { mesh, color } } }), or none.
function keptLooks(files) {
    const file = files.find((candidate) => candidate.role === 'looks');
    if (!file) return {};
    const parsed = JSON.parse(file.text);
    if (parsed?.format !== 'konjugate.robotics.looks') throw new Error(`${file.name} is not the looks of an imported robot.`);
    return parsed.looks ?? {};
}

// The model's files by their path in its folder: a stored file's path is the window's (options.paths, by name) or its
// name, with "^" for "/" as a Menagerie download is named. A path the model asks for that is not there is found by its
// file name, so files brought in from disk without their folders still read.
function modelFiles(files, options) {
    const byPath = {};
    for (const file of files.filter((candidate) => candidate.role === 'model' && candidate.data)) {
        byPath[options.paths?.[file.name] ?? file.name.split('^').join('/')] = file.data;
    }
    const byName = new Map();
    for (const path of Object.keys(byPath)) {
        const name = path.split('/').pop();
        byName.set(name, byName.has(name) ? null : path);
    }
    return new Proxy(byPath, {
        get(target, path) {
            if (typeof path !== 'string' || path in target) return target[path];
            const found = byName.get(path.split('/').pop());
            return found ? target[found] : undefined;
        }
    });
}

// The model's main file: the one named, or its scene, or the one MJCF no other includes.
function mainFile(files, named) {
    const paths = Object.keys(files);
    if (named) return named;
    const xml = paths.filter((path) => path.toLowerCase().endsWith('.xml'));
    if (xml.includes('scene.xml')) return 'scene.xml';
    const text = (path) => (typeof files[path] === 'string' ? files[path] : new TextDecoder().decode(files[path]));
    const included = new Set(xml.flatMap((path) => [...text(path).matchAll(/<include\s+file\s*=\s*["']([^"']+)["']/g)].map(([, file]) => file.split('/').pop())));
    const tops = xml.filter((path) => !included.has(path.split('/').pop()) && /<mujoco[\s>]/.test(text(path)));
    if (tops.length !== 1) throw new Error(tops.length ? `Say which of ${tops.join(', ')} is the model.` : 'Choose the model\'s MJCF (.xml) file among the files.');
    return tops[0];
}

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
    if (options.step === 'model') {
        const model = modelFiles(files, options);
        if (!Object.keys(model).length) return failure('Choose a robot\'s model first: download one from Menagerie, or bring its files in from this computer.');
        try {
            const main = mainFile(model, options.main);
            const read = readModel(model, main);
            const { drawing, looks, report } = importModel(read, read.bodies.map((body) => bodyLook(read, body)), { name: options.name ?? read.name ?? 'Robot' });
            if (options.source) drawing.source = options.source;
            const notice = model.LICENSE ?? null;
            const derived = [{ role: 'looks', name: 'looks.json', text: JSON.stringify({ format: 'konjugate.robotics.looks', version: 1, looks }), source: main }];
            if (notice) derived.push({ role: 'notice', name: 'LICENSE', text: typeof notice === 'string' ? notice : new TextDecoder().decode(notice), source: 'LICENSE' });
            return {
                ok: true, derived,
                data: { step: 'model', drawing, looks, report, main },
                report: { errors: [], warnings: report.leftOut, summary: `${drawing.name}: ${report.solids} solids and ${report.joints} joints, ${report.drives} driven` }
            };
        } catch (error) {
            return failure(`The model could not be read: ${error.message}`);
        }
    }
    if (options.step === 'looks') {
        try {
            return { ok: true, data: { step: 'looks', looks: keptLooks(files) }, report: { errors: [], warnings: [] } };
        } catch (error) {
            return failure(error.message);
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
        const built = buildModel(drawing, { timeStep: options.timeStep ?? 0.001, runTime: options.runTime ?? 10, looks: keptLooks(files) });
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
