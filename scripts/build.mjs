/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Builds the two packages into out/: the Robotics Engine plugin (konjugate.robotics.engine-<version>.kjp, the component
// templates) and the Robotics Toolbox add-on (konjugate.robotics.toolbox-<version>.kja, the window and its importer).
// The add-on carries the mechanism provider's source as engine/mechanismProvider.json, since an importer reads only
// JSON files of its own package. No robot is in either: a robot is fetched when a user asks for one.

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

// ---- the Robotics Engine plugin ---------------------------------------------------------------------------------
const engineDirectory = join(roboticsRoot, 'packages', 'engine');
const pluginManifest = { ...JSON.parse(await readFile(join(engineDirectory, 'plugin.json'), 'utf8')), version };
const pluginFiles = {};
for (const contribution of pluginManifest.contributes) pluginFiles[contribution.entry] = await readFile(join(engineDirectory, contribution.entry));
const pluginTarget = join(outputDirectory, `${pluginManifest.pluginId}-${version}.kjp`);
await writeFile(pluginTarget, createPackageArchive({
    packageManifest: {
        format: 'konjugate-package', formatVersion: 1, packageType: 'plugin',
        packageId: pluginManifest.pluginId, name: pluginManifest.name, version, minimumKonjugateVersion, contents: { manifest: 'plugin.json' }
    },
    contributionManifest: pluginManifest,
    files: pluginFiles
}));
console.log(`Built ${pluginTarget}`);

// ---- the Robotics Toolbox add-on ----------------------------------------------------------------------------------
const toolboxDirectory = join(roboticsRoot, 'packages', 'toolbox');
const toolboxManifest = { ...JSON.parse(await readFile(join(toolboxDirectory, 'addon.json'), 'utf8')), version };
const toolboxFiles = {};
for (const entry of await readdir(toolboxDirectory, { recursive: true, withFileTypes: true })) {
    if (!entry.isFile()) continue;
    const relative = join(entry.parentPath, entry.name).slice(toolboxDirectory.length + 1).split('\\').join('/');
    if (relative !== 'addon.json' && !relative.endsWith('.DS_Store')) toolboxFiles[relative] = await readFile(join(toolboxDirectory, relative));
}
toolboxFiles['engine/mechanismProvider.json'] = Buffer.from(JSON.stringify({
    source: await readFile(join(engineDirectory, 'providers', 'mechanism.py'), 'utf8')
}));
for (const contribution of pluginManifest.contributes) {
    toolboxFiles[`engine/components/${contribution.componentId}.json`] = await readFile(join(engineDirectory, contribution.entry));
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
