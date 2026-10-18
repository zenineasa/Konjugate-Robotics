/* Copyright © 2026 Zenin Easa Panthakkalakath */

// What the window says about the providers' Python (lib/python.mjs), from the host's check of it.

import assert from 'node:assert/strict';
import test from 'node:test';
import { pythonAdvice, pythonModules } from '../../packages/toolbox/lib/python.mjs';

const checked = (mujoco, extra = {}) => ({ found: true, interpreter: 'python3', executable: '/venv/bin/python', version: '3.12.4', modules: { mujoco }, message: 'm', ...extra });

test('the check asks for MuJoCo, and a Python that has it is ready', () => {
    assert.deepEqual(pythonModules, ['mujoco']);
    const advice = pythonAdvice(checked({ found: true, version: '3.16.0' }));
    assert.equal(advice.ready, true);
    assert.equal(advice.kind, 'ok');
    assert.match(advice.text, /^Python 3\.12\.4 has MuJoCo 3\.16\.0: runs and comparisons will work \(\/venv\/bin\/python\)\.$/);
});

test('a Python without MuJoCo is told how to install it into that very Python, and one where it does not load says why', () => {
    const missing = pythonAdvice(checked({ found: false, error: "ModuleNotFoundError: No module named 'mujoco'" }));
    assert.equal(missing.ready, false);
    assert.match(missing.text, /does not have MuJoCo\. Install it with "\/venv\/bin\/python" -m pip install mujoco, or choose a Python .*Provider toolchains/);
    const broken = pythonAdvice(checked({ found: false, error: 'ImportError: dlopen failed' }));
    assert.match(broken.text, /It is there but does not load: ImportError: dlopen failed\./);
});

test('no Python at all is said plainly, and a Konjugate that cannot check says that instead of guessing', () => {
    const none = pythonAdvice({ found: false, interpreter: null, modules: { mujoco: { found: false, error: null } }, message: 'Python could not be run (the Python found on the PATH): spawn python3 ENOENT' });
    assert.equal(none.ready, false);
    assert.match(none.text, /^Python could not be run .*ENOENT Install Python 3 with MuJoCo/);
    const unknown = pythonAdvice(null);
    assert.equal(unknown.ready, null);
    assert.match(unknown.text, /cannot check it from here/);
});
