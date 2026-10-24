/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Robots from MuJoCo Menagerie (github.com/google-deepmind/mujoco_menagerie), fetched when a user asks for one and kept
// on their device, never shipped with the toolbox (docs/direction.md, "Robots and data from public sources").
//
// A fetch is of one model's folder at one commit, recorded with the model's licence and where it came from, so a
// project that uses it runs the same after an upstream edit and says whose it is. Menagerie's own pictures of its
// robots (images at the top of a model's folder) are left out: the toolbox draws its own. Images further in are
// textures the model uses, and stay.
//
// No dependencies and no I/O of its own: `fetchJson(url)` and `fetchBytes(url)` come from the caller (the window's
// network bridge, or Node's fetch in scripts and tests), as does `write(path, bytes)`.

export const repository = { owner: 'google-deepmind', name: 'mujoco_menagerie', url: 'https://github.com/google-deepmind/mujoco_menagerie' };
export const hosts = ['api.github.com', 'raw.githubusercontent.com'];

const pictures = /^[^/]+\.(png|jpe?g|gif|webp|svg)$/i;

// The commit `branch` points at now: a fetch pins to it.
export async function latestCommit(fetchJson, branch = 'main') {
    const commit = await fetchJson(`https://api.github.com/repos/${repository.owner}/${repository.name}/commits/${branch}`);
    if (!/^[0-9a-f]{40}$/.test(commit?.sha ?? '')) throw new Error('Menagerie did not say which commit its main branch is at.');
    return commit.sha;
}

// The files of one model's folder at `commit` (paths relative to the folder), its pictures left out. Listed folder by
// folder through GitHub's contents API: the whole repository's file list is over the megabyte a window may fetch.
export async function modelFiles(fetchJson, commit, folder) {
    if (!/^[A-Za-z0-9_.-]+$/.test(folder)) throw new Error(`"${folder}" is not a Menagerie model folder name.`);
    const files = [];
    const list = async (path) => {
        const entries = await fetchJson(`https://api.github.com/repos/${repository.owner}/${repository.name}/contents/${path.split('/').map(encodeURIComponent).join('/')}?ref=${commit}`);
        if (!Array.isArray(entries)) throw new Error(`Menagerie has no model folder "${folder}" at commit ${commit}.`);
        for (const entry of entries) {
            if (entry.type === 'dir') await list(entry.path);
            else if (entry.type === 'file') files.push({ path: entry.path.slice(folder.length + 1), size: entry.size, url: rawUrl(commit, entry.path) });
        }
    };
    await list(folder);
    return files.filter(({ path }) => !pictures.test(path));
}

export function rawUrl(commit, path) {
    return `https://raw.githubusercontent.com/${repository.owner}/${repository.name}/${commit}/${path.split('/').map(encodeURIComponent).join('/')}`;
}

// The licence a LICENSE file's text is, by its opening words; null when it is none of these, so it is shown as found.
export function licenceName(text) {
    const start = String(text).slice(0, 2000);
    if (/Apache License[\s\S]*Version 2\.0/i.test(start)) return 'Apache-2.0';
    if (/BSD 3-Clause Clear/i.test(start) || /The Clear BSD License/i.test(start)) return 'BSD-3-Clause-Clear';
    if (/Redistribution and use in source and binary forms/i.test(start)) {
        return /Neither the name/i.test(start) ? 'BSD-3-Clause' : 'BSD-2-Clause';
    }
    if (/Permission is hereby granted, free of charge/i.test(start)) return 'MIT';
    return null;
}

// Fetches one model's folder at `commit` (the latest when not given), writing each file with `write(path, bytes)`;
// returns its provenance: where it came from, at which commit, under what licence, and which files.
export async function fetchModel({ folder, commit, fetchJson, fetchBytes, write, now = () => new Date() }) {
    const pinned = commit ?? await latestCommit(fetchJson);
    const files = await modelFiles(fetchJson, pinned, folder);
    let licenceText = null;
    for (const file of files) {
        const bytes = await fetchBytes(file.url);
        if (file.size !== undefined && bytes.byteLength !== file.size) throw new Error(`${folder}/${file.path} came back as ${bytes.byteLength} bytes, not ${file.size}.`);
        await write(file.path, bytes);
        if (file.path === 'LICENSE') licenceText = new TextDecoder().decode(bytes);
    }
    return {
        source: 'MuJoCo Menagerie', repository: repository.url, folder, commit: pinned, fetchedAt: now().toISOString(),
        licence: { name: licenceText === null ? null : licenceName(licenceText), file: licenceText === null ? null : 'LICENSE' },
        files: files.map(({ path, size }) => ({ path, size }))
    };
}
