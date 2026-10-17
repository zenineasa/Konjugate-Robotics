/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Fetches one MuJoCo Menagerie model to this computer, as the toolbox does when a user asks for one, into
// out/models/menagerie/<commit>/<folder>/ with a provenance.json beside its files (where it came from, the commit and
// its licence). Standing in for Konjugate's store of downloaded files in scripts and tests until the window exists.
//
//   node scripts/fetchMenagerieModel.mjs <folder> [--commit <sha>]
//
// Prints the model's directory. A model already fetched at that commit is not fetched again.

import { existsSync } from 'node:fs';
import { mkdir, readFile, writeFile } from 'node:fs/promises';
import { dirname, join } from 'node:path';
import { fetchModel, latestCommit } from '../packages/toolbox/lib/menagerie.mjs';
import { roboticsRoot } from './konjugatePaths.mjs';

const fetchOk = async (url) => {
    const response = await fetch(url, { headers: { 'User-Agent': 'konjugate-robotics' } });
    if (!response.ok) throw new Error(`${url}: ${response.status} ${response.statusText}`);
    return response;
};
const fetchJson = async (url) => (await fetchOk(url)).json();
const fetchBytes = async (url) => new Uint8Array(await (await fetchOk(url)).arrayBuffer());

// The model's directory, fetching it first if it is not already on this computer.
export async function menagerieModel(folder, { commit } = {}) {
    const pinned = commit ?? await latestCommit(fetchJson);
    const directory = join(roboticsRoot, 'out', 'models', 'menagerie', pinned, folder);
    const provenancePath = join(directory, 'provenance.json');
    if (existsSync(provenancePath)) return { directory, provenance: JSON.parse(await readFile(provenancePath, 'utf8')) };
    const write = async (path, bytes) => {
        const target = join(directory, path);
        await mkdir(dirname(target), { recursive: true });
        await writeFile(target, bytes);
    };
    const provenance = await fetchModel({ folder, commit: pinned, fetchJson, fetchBytes, write });
    await writeFile(provenancePath, `${JSON.stringify(provenance, null, 2)}\n`);
    return { directory, provenance };
}

if (process.argv[1] && import.meta.url.endsWith(process.argv[1].split('/').pop())) {
    const [folder, ...rest] = process.argv.slice(2);
    if (!folder) {
        console.error('Usage: node scripts/fetchMenagerieModel.mjs <folder> [--commit <sha>]');
        process.exit(64);
    }
    const commitIndex = rest.indexOf('--commit');
    const { directory, provenance } = await menagerieModel(folder, { commit: commitIndex >= 0 ? rest[commitIndex + 1] : undefined });
    console.log(`${directory}\n${provenance.source} ${provenance.folder} at ${provenance.commit.slice(0, 7)}, licence ${provenance.licence.name ?? 'not recognised (see LICENSE)'}`);
}
