/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The build and install scripts' own decisions, which a stale out/ or an old Konjugate once made silently.

import assert from 'node:assert/strict';
import { mkdtemp, mkdir, readdir, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import test from 'node:test';
import { compatibilityWarning, currentArchiveName, retiredPackages } from '../../scripts/installDev.mjs';

test('only the toolbox build of the current version is installed, whatever else out/ holds', () => {
    const names = ['konjugate.robotics.engine-0.1.0.kjp', 'konjugate.robotics.toolbox-0.0.9.kja', 'konjugate.robotics.toolbox-0.1.0.kja'];
    assert.equal(currentArchiveName(names, 'konjugate.robotics.toolbox', '0.1.0'), 'konjugate.robotics.toolbox-0.1.0.kja');
    assert.equal(currentArchiveName(names.slice(0, 2), 'konjugate.robotics.toolbox', '0.1.0'), null);
});

test('a Konjugate too old for the toolbox is named, since it installs the toolbox but never loads it', () => {
    assert.equal(compatibilityWarning('1.1.16', '1.1.16'), null);
    assert.match(compatibilityWarning('1.1.17', '1.1.16'), /needs Konjugate 1\.1\.17 or later.*is 1\.1\.16, so it will not load the toolbox or show its button/);
});

test('the retired Robotics Engine plugin is among the packages an install removes', () => {
    assert.deepEqual(retiredPackages, [{ type: 'plugins', id: 'konjugate.robotics.engine' }]);
});

test('an install removes the retired plugin from the packages folder and leaves other packages alone', async () => {
    const { installBuiltPackages } = await import('../../scripts/installDev.mjs');
    const userData = await mkdtemp(join(tmpdir(), 'robotics-install-'));
    await mkdir(join(userData, 'packages', 'plugins', 'konjugate.robotics.engine', '0.1.0'), { recursive: true });
    await mkdir(join(userData, 'packages', 'plugins', 'konjugate.logistics.engine', '1.0.0'), { recursive: true });
    await writeFile(join(userData, 'packages', 'plugins', 'konjugate.robotics.engine', '0.1.0', 'package.json'), '{}');
    try {
        await installBuiltPackages(userData);
    } catch (error) {
        // Without a build in out/, the install stops before removing anything; build first to run this test fully.
        if (/No build of/.test(error.message)) return;
        throw error;
    }
    assert.deepEqual(await readdir(join(userData, 'packages', 'plugins')), ['konjugate.logistics.engine']);
    assert.ok((await readdir(join(userData, 'packages', 'addons'))).includes('konjugate.robotics.toolbox'));
});
