/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Fetching a robot from MuJoCo Menagerie (lib/menagerie.mjs), against a GitHub of its own: the commit pinned, the
// folder listed directory by directory, its pictures left out but textures kept, each file checked for its size, and
// the licence named from its text. And the names the store keeps a robot's files by (lib/catalog.mjs).

import assert from 'node:assert/strict';
import test from 'node:test';
import { pathOfStoredName, robots, storedName } from '../../packages/toolbox/lib/catalog.mjs';
import { fetchModel, licenceName, rawUrl } from '../../packages/toolbox/lib/menagerie.mjs';

const commit = 'a'.repeat(40);
const files = {
    'arm/LICENSE': 'Apache License\n                           Version 2.0, January 2004',
    'arm/scene.xml': '<mujoco/>',
    'arm/arm.png': 'a picture of the arm',
    'arm/assets/link.stl': 'solid link',
    'arm/assets/skin.png': 'a texture'
};

function github({ corrupt = null } = {}) {
    const requests = [];
    const listing = (path) => {
        const prefix = `${path}/`;
        const children = new Map();
        for (const [file, text] of Object.entries(files)) {
            if (!file.startsWith(prefix)) continue;
            const [name, ...rest] = file.slice(prefix.length).split('/');
            children.set(name, rest.length ? { name, path: `${path}/${name}`, type: 'dir', size: 0 } : { name, path: file, type: 'file', size: text.length });
        }
        return children.size ? [...children.values()] : { message: 'Not Found' };
    };
    return {
        requests,
        fetchJson: async (url) => {
            requests.push(url);
            if (url.endsWith('/commits/main')) return { sha: commit };
            const match = url.match(/\/contents\/(.+)\?ref=([0-9a-f]+)$/);
            assert.ok(match && match[2] === commit, `An unexpected address: ${url}`);
            return listing(decodeURIComponent(match[1]));
        },
        fetchBytes: async (url) => {
            requests.push(url);
            const path = decodeURIComponent(url.split(`/${commit}/`)[1]);
            const text = path === corrupt ? `${files[path]} and more` : files[path];
            return new TextEncoder().encode(text);
        }
    };
}

test('a robot is fetched at a pinned commit, its pictures left out and its textures kept, with its licence named', async () => {
    const server = github();
    const written = {};
    const provenance = await fetchModel({ folder: 'arm', ...server, write: (path, bytes) => { written[path] = new TextDecoder().decode(bytes); }, now: () => new Date('2026-10-17T12:00:00Z') });
    assert.deepEqual(Object.keys(written).sort(), ['LICENSE', 'assets/link.stl', 'assets/skin.png', 'scene.xml']);
    assert.equal(provenance.commit, commit);
    assert.deepEqual(provenance.licence, { name: 'Apache-2.0', file: 'LICENSE' });
    assert.equal(provenance.fetchedAt, '2026-10-17T12:00:00.000Z');
    assert.ok(!server.requests.some((url) => url.endsWith('arm.png')), 'The picture is never fetched.');
    assert.ok(server.requests.every((url) => url.includes(commit) || url.endsWith('/commits/main')), 'Everything is fetched at the pinned commit.');
});

test('a file that comes back the wrong size, or a folder Menagerie does not have, stops the fetch', async () => {
    await assert.rejects(fetchModel({ folder: 'arm', commit, ...github({ corrupt: 'arm/scene.xml' }), write: () => {} }), /scene\.xml came back as/);
    await assert.rejects(fetchModel({ folder: 'legs', commit, ...github(), write: () => {} }), /no model folder "legs"/);
    await assert.rejects(fetchModel({ folder: '../etc', commit, ...github(), write: () => {} }), /not a Menagerie model folder name/);
});

test('licences are named from their text, or not named when unknown', () => {
    assert.equal(licenceName('Apache License\nVersion 2.0, January 2004'), 'Apache-2.0');
    assert.equal(licenceName('Copyright 2020\nRedistribution and use in source and binary forms, with or without modification...\nNeither the name of the copyright holder'), 'BSD-3-Clause');
    assert.equal(licenceName('Redistribution and use in source and binary forms, with or without modification...'), 'BSD-2-Clause');
    assert.equal(licenceName('The Clear BSD License\nRedistribution and use...'), 'BSD-3-Clause-Clear');
    assert.equal(licenceName('Permission is hereby granted, free of charge, to any person'), 'MIT');
    assert.equal(licenceName('All rights reserved.'), null);
});

test('a file\'s path survives the name the store keeps it by, and every listed robot has a main file', () => {
    assert.equal(storedName('assets/meshes/link.stl'), 'assets^meshes^link.stl');
    assert.equal(pathOfStoredName(storedName('assets/meshes/link.stl')), 'assets/meshes/link.stl');
    assert.equal(rawUrl(commit, 'arm/a b.stl'), `https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/${commit}/arm/a%20b.stl`);
    assert.ok(robots.every((robot) => /^[a-z0-9_]+$/.test(robot.folder) && robot.main && robot.name && robot.maker));
});
