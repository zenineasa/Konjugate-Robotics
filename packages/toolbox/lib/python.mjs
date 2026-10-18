/* Copyright © 2026 Zenin Easa Panthakkalakath */

// What the window says about the Python that runs the mechanism provider, from the host's check of it
// (konjugateLauncher.checkPython, Konjugate's pythonCheck feature): whether it imports MuJoCo, and if not, what to do.
// No dependencies.

export const pythonModules = ['mujoco'];

// `answer` is checkPython's answer, or null when this Konjugate cannot check. Returns { ready, kind, text }:
// ready is true or false when known, null when not checked.
export function pythonAdvice(answer) {
    if (!answer) {
        return { ready: null, kind: '', text: 'Running it, or comparing, needs Python with MuJoCo (pip install mujoco) as the Python for providers, under Provider toolchains (⚒ in the titlebar). This version of Konjugate cannot check it from here.' };
    }
    const mujoco = answer.modules?.mujoco;
    if (answer.found && mujoco?.found) {
        return { ready: true, kind: 'ok', text: `Python ${answer.version} has MuJoCo${mujoco.version ? ` ${mujoco.version}` : ''}: runs and comparisons will work (${answer.executable ?? answer.interpreter}).` };
    }
    if (!answer.found) {
        return { ready: false, kind: 'warning', text: `${answer.message} Install Python 3 with MuJoCo (pip install mujoco), or choose a Python under Provider toolchains (⚒ in the titlebar), then check again. Building works without it; running does not.` };
    }
    const why = mujoco?.error && !/^ModuleNotFoundError/.test(mujoco.error) ? ` It is there but does not load: ${mujoco.error}.` : '';
    return {
        ready: false, kind: 'warning',
        text: `Python ${answer.version} at ${answer.executable ?? answer.interpreter} does not have MuJoCo.${why} Install it with "${answer.executable ?? answer.interpreter}" -m pip install mujoco, or choose a Python that has it under Provider toolchains (⚒ in the titlebar), then check again. Building works without it; running does not.`
    };
}
