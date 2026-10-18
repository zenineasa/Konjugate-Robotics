/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Builds the Robotics Toolbox add-on into out/ (konjugate.robotics.toolbox-<version>.kja): the window and its importer.
// What it builds runs on Konjugate's engine alone, so there is no plugin or provider to ship with it.

import { mkdir, readdir, readFile, writeFile } from 'node:fs/promises';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
import { konjugateModule, roboticsRoot } from './konjugatePaths.mjs';

const { createPackageArchive } = await import(pathToFileURL(konjugateModule('src/packageArchive.mjs')));

// package.json is the single source of the version and of the lowest Konjugate these packages run on.
const { version, minimumKonjugateVersion } = JSON.parse(await readFile(join(roboticsRoot, 'package.json'), 'utf8'));
if (!minimumKonjugateVersion) throw new Error('package.json must declare minimumKonjugateVersion, the lowest Konjugate these packages run on.');
const outputDirectory = join(roboticsRoot, 'out');
await mkdir(outputDirectory, { recursive: true });

const toolboxDirectory = join(roboticsRoot, 'packages', 'toolbox');
const toolboxManifest = { ...JSON.parse(await readFile(join(toolboxDirectory, 'addon.json'), 'utf8')), version };
const toolboxFiles = {};
for (const entry of await readdir(toolboxDirectory, { recursive: true, withFileTypes: true })) {
    if (!entry.isFile()) continue;
    const relative = join(entry.parentPath, entry.name).slice(toolboxDirectory.length + 1).split('\\').join('/');
    if (relative !== 'addon.json' && !relative.endsWith('.DS_Store')) toolboxFiles[relative] = await readFile(join(toolboxDirectory, relative));
}
const toolboxTarget = join(outputDirectory, `${toolboxManifest.addonId}-${version}.kja`);
await writeFile(toolboxTarget, createPackageArchive({
    packageManifest: {
        format: 'konjugate-package', formatVersion: 1, packageType: 'addon',
        packageId: toolboxManifest.addonId, name: toolboxManifest.name, version, minimumKonjugateVersion, contents: { manifest: 'addon.json' }
    },
    contributionManifest: toolboxManifest,
    files: toolboxFiles
}));
console.log(`Built ${toolboxTarget}`);
