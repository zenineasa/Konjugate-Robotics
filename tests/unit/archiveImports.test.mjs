/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Building a .kja proves the files exist; it doesn't prove they resolve at runtime -- a vendored
// ES module can import a sibling file that never got vendored, and nothing about a successful build
// or a correct file listing would catch that (exactly what happened to Konjugate-PoseVisualizer:
// three.module.js's own `import ... from './three.core.js'` silently broke once three.core.js wasn't
// vendored alongside it, and the add-on rendered nothing with no visible error). This toolbox vendors
// three.js and its controls the same way, in packages/toolbox/vendor/. This statically checks every relative
// import in a built .js/.mjs file, and every relative <script src>/<link href> in a built .html
// file, resolves to another file actually present in the same archive -- no browser needed.

import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readdirSync, readFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { join } from 'node:path';
import test from 'node:test';
import { konjugateDir, roboticsRoot as repoRoot } from '../../scripts/konjugatePaths.mjs';

// This repo declares no npm dependencies of its own -- fflate only exists in Konjugate core's
// node_modules (see scripts/konjugatePaths.mjs's own reasoning for the same thing). createRequire
// against a file inside core resolves it from there, the same way the build scripts borrow core's
// own source modules, just for an npm package instead.
const { strFromU8, unzipSync } = createRequire(join(konjugateDir, 'package.json'))('fflate');

function resolveRelative(fromPath, relative) {
    const parts = fromPath.split('/').slice(0, -1).concat(relative.split('/'));
    const resolved = [];
    for (const part of parts) {
        if (part === '.' || part === '') continue;
        if (part === '..') resolved.pop(); else resolved.push(part);
    }
    return resolved.join('/');
}

test('every relative import/script/stylesheet reference in the built package resolves to a file inside it', () => {
    execFileSync('node', ['scripts/build.mjs'], { cwd: repoRoot, stdio: 'inherit' });
    const outputDirectory = join(repoRoot, 'out');
    const { version } = JSON.parse(readFileSync(join(repoRoot, 'package.json'), 'utf8'));
    const { addonId } = JSON.parse(readFileSync(join(repoRoot, 'packages', 'toolbox', 'addon.json'), 'utf8'));
    // The build just made, not an earlier one out/ may keep.
    const archiveName = readdirSync(outputDirectory).find((name) => name === `${addonId}-${version}.kja`);
    assert.ok(archiveName, 'build did not produce a .kja/.kjp in out/');
    const entries = unzipSync(readFileSync(join(outputDirectory, archiveName)));
    const paths = new Set(Object.keys(entries));

    for (const [path, bytes] of Object.entries(entries)) {
        const text = strFromU8(bytes);
        if (path.endsWith('.mjs') || path.endsWith('.js')) {
            for (const match of text.matchAll(/\bfrom\s+['"](\.[^'"]+)['"]/g)) {
                const resolved = resolveRelative(path, match[1]);
                assert.ok(paths.has(resolved), `${path} imports "${match[1]}" -> resolved to "${resolved}", which is not in the built archive`);
            }
        }
        if (path.endsWith('.html')) {
            for (const match of text.matchAll(/<(?:script[^>]+src|link[^>]+href)=["'](?!https?:)([^"']+)["']/g)) {
                const resolved = resolveRelative(path, match[1]);
                assert.ok(paths.has(resolved), `${path} references "${match[1]}" -> resolved to "${resolved}", which is not in the built archive`);
            }
        }
    }
});
