/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Checks the built toolbox in out/ against Konjugate core's registry/ and fails (non-zero exit) unless it comes back
// "verified": a check that the release's signing step produced something the registry agrees with, not a gate in
// Konjugate itself (see docs/registry.md: nothing refuses to install an unsigned or unverified package).

import { readFile, readdir } from 'node:fs/promises';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
import { currentArchiveName } from './installDev.mjs';
import { konjugateModule, roboticsRoot } from './konjugatePaths.mjs';

const { loadNamespaceRegistry, verifyPackageArchive } = await import(pathToFileURL(konjugateModule('src/packageArchive.mjs')));
const namespaces = await loadNamespaceRegistry(konjugateModule('registry'));

const outputDirectory = join(roboticsRoot, 'out');
const { version } = JSON.parse(await readFile(join(roboticsRoot, 'package.json'), 'utf8'));
const { addonId } = JSON.parse(await readFile(join(roboticsRoot, 'packages', 'toolbox', 'addon.json'), 'utf8'));
const name = currentArchiveName(await readdir(outputDirectory).catch(() => []), addonId, version);
if (!name) throw new Error(`No build of ${addonId} ${version} in ${outputDirectory}; run the build (and sign) first.`);
const result = verifyPackageArchive(await readFile(join(outputDirectory, name)), { namespaces });
console.log(`${name}: ${result.status}${result.reason ? ` (${result.reason})` : ''}`);
if (result.status !== 'verified') throw new Error('The package did not verify. See docs/registry.md if this is unexpected.');
