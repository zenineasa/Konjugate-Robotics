/* Copyright © 2026 Zenin Easa Panthakkalakath */

import { readFile, readdir, rm } from 'node:fs/promises';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
import { defaultUserData, konjugateModule, roboticsRoot } from './konjugatePaths.mjs';

const { installPackageArchive, packageCompatibility } = await import(pathToFileURL(konjugateModule('src/packageArchive.mjs')));

// Packages this repository once built and no longer does: the Robotics Engine plugin carried the MuJoCo mechanism
// provider and its components, retired when the robot came to move by its joints' own equations. Left installed, it
// would still be listed and loaded beside the toolbox.
export const retiredPackages = Object.freeze([{ type: 'plugins', id: 'konjugate.robotics.engine' }]);

// The build of the current version, as scripts/build.mjs names it, not every archive out/ holds: out/ keeps earlier
// builds, and archives of packages this repository no longer builds.
export function currentArchiveName(names, addonId, version) {
    const name = `${addonId}-${version}.kja`;
    return names.includes(name) ? name : null;
}

// Why the sibling Konjugate checkout will not load this build, or null when it will: a package asking for a newer
// Konjugate is installed but never loaded, so its window has no button.
export function compatibilityWarning(minimumKonjugateVersion, konjugateVersion) {
    const verdict = packageCompatibility({ minimumKonjugateVersion }, konjugateVersion);
    return verdict.compatible ? null
        : `This toolbox needs Konjugate ${verdict.required} or later, and the Konjugate at ${konjugateModule('')} is ${verdict.running}, so it will not load the toolbox or show its button. Run a Konjugate of ${verdict.required} or later.`;
}

// Installs the built toolbox into <userData>/packages with the same code path the app uses, in place of any other
// installed version (Konjugate loads one version of an add-on, and with two installed it may load the older), and
// removes the packages this repository no longer builds.
// Usage: node scripts/installDev.mjs [userDataDirectory]
export async function installBuiltPackages(userData = defaultUserData()) {
    const outputDirectory = join(roboticsRoot, 'out');
    const { version, minimumKonjugateVersion } = JSON.parse(await readFile(join(roboticsRoot, 'package.json'), 'utf8'));
    const { addonId } = JSON.parse(await readFile(join(roboticsRoot, 'packages', 'toolbox', 'addon.json'), 'utf8'));
    const name = currentArchiveName(await readdir(outputDirectory).catch(() => []), addonId, version);
    if (!name) throw new Error(`No build of ${addonId} ${version} in ${outputDirectory}; run npm run build first.`);
    for (const { type, id } of retiredPackages) {
        const directory = join(userData, 'packages', type, id);
        const versions = await readdir(directory).catch(() => []);
        if (!versions.length) continue;
        await rm(directory, { recursive: true, force: true });
        console.log(`Removed ${id} (${versions.join(', ')}), which this repository no longer builds, from ${join(userData, 'packages')}`);
    }
    const installed = await installPackageArchive(await readFile(join(outputDirectory, name)), {
        extension: '.kja', directory: join(userData, 'packages'), overwrite: true, replaceOtherVersions: true
    });
    console.log(`Installed ${name} into ${join(userData, 'packages')}`);
    const konjugateVersion = JSON.parse(await readFile(konjugateModule('package.json'), 'utf8')).version;
    const warning = compatibilityWarning(minimumKonjugateVersion, konjugateVersion);
    if (warning) console.warn(`Warning: ${warning}`);
    return installed;
}

if (import.meta.url === pathToFileURL(process.argv[1]).href) await installBuiltPackages(process.argv[2]);
