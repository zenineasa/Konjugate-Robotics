/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Signs the built toolbox in out/ under the konjugate.robotics namespace prefix (reserved in Konjugate core's
// registry/, see that repository's docs/registry.md). The private key never lives in this repository; it is read
// from an environment variable, so it can come from a CI secret (see .github/workflows/release.yml) or a local,
// gitignored file (.keys/) for a manual release build.
//
// Usage: ROBOTICS_SIGNING_PRIVATE_KEY="$(cat .keys/robotics.pem)" node scripts/signPackage.mjs

import { readFile, readdir, writeFile } from 'node:fs/promises';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
import { currentArchiveName } from './installDev.mjs';
import { konjugateModule, roboticsRoot } from './konjugatePaths.mjs';

const { signPackageArchive } = await import(pathToFileURL(konjugateModule('src/packageArchive.mjs')));

const privateKey = process.env.ROBOTICS_SIGNING_PRIVATE_KEY;
if (!privateKey) throw new Error('ROBOTICS_SIGNING_PRIVATE_KEY is not set, so there is nothing to sign with. See docs/registry.md in the Konjugate core repository for how a key is made and registered.');

// The build of the current version only: out/ may keep earlier ones.
const outputDirectory = join(roboticsRoot, 'out');
const { version } = JSON.parse(await readFile(join(roboticsRoot, 'package.json'), 'utf8'));
const { addonId } = JSON.parse(await readFile(join(roboticsRoot, 'packages', 'toolbox', 'addon.json'), 'utf8'));
const name = currentArchiveName(await readdir(outputDirectory).catch(() => []), addonId, version);
if (!name) throw new Error(`No build of ${addonId} ${version} in ${outputDirectory}; run npm run build first.`);
const path = join(outputDirectory, name);
await writeFile(path, signPackageArchive(await readFile(path), { privateKey, prefix: 'konjugate.robotics' }));
console.log(`Signed ${name}`);
