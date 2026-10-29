/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Where two shapes touch (lib/collision.mjs), against MuJoCo 3.16's own contacts for the same shapes where MuJoCo put
// them (tests/fixtures/contact/mujocoContacts.json: random poses, and poses square to each other, for each pair the
// toolbox collides); and the same functions written as equations (lib/modelBuilder.mjs's expressionAlgebra) giving
// the same numbers.

import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { join } from 'node:path';
import test from 'node:test';
import { collide, contactSlots, numbers, pairKey } from '../../packages/toolbox/lib/collision.mjs';
import * as E from '../../packages/toolbox/lib/expressions.mjs';
import { expressionAlgebra } from '../../packages/toolbox/lib/modelBuilder.mjs';
import { roboticsRoot } from '../../scripts/konjugatePaths.mjs';

const { cases } = JSON.parse(await readFile(join(roboticsRoot, 'tests', 'fixtures', 'contact', 'mujocoContacts.json'), 'utf8'));
const differ = (a, b) => Math.max(...a.map((value, index) => Math.abs(value - b[index])));

test('each pair\'s contacts are MuJoCo\'s: as many, each at its distance, point and frame', () => {
    const seen = new Map();
    for (const { first, second, contacts } of cases) {
        const key = pairKey(first.kind, second.kind);
        const found = collide(numbers, first, second, 0).filter(({ active }) => active);
        assert.equal(found.length, contacts.length, `${key}: ${found.length} contacts, MuJoCo ${contacts.length}`);
        for (const contact of contacts) {
            const match = found.reduce((best, slot) => (differ(slot.pos, contact.pos) < differ(best.pos, contact.pos) ? slot : best));
            const error = Math.max(differ(match.pos, contact.pos), Math.abs(match.dist - contact.dist), differ(match.frame, contact.frame));
            assert.ok(error < 1e-12, `${key}: differs from MuJoCo by ${error}`);
        }
        seen.set(key, (seen.get(key) ?? 0) + contacts.length);
    }
    for (const key of Object.keys(contactSlots)) assert.ok(seen.get(key) > 0, `${key} should touch in some case`);
});

test('written as equations, each pair\'s functions give the numbers they give', () => {
    for (const { first, second } of cases.filter((_, index) => index % 3 === 0)) {
        const definitions = new Map();
        const algebra = expressionAlgebra((symbol, expression) => {
            if (typeof expression === 'number') return expression;
            assert.ok(!definitions.has(symbol), `${symbol} is defined twice`);
            definitions.set(symbol, expression);
            return symbol;
        });
        // Each shape's pose given as symbols, as the model's states are.
        const values = {};
        const symbolic = (shape, tag) => ({
            kind: shape.kind, size: shape.size,
            pos: shape.pos.map((value, i) => { values[`${tag}p${i}`] = value; return `${tag}p${i}`; }),
            mat: shape.mat.map((value, i) => { values[`${tag}m${i}`] = value; return `${tag}m${i}`; })
        });
        const slots = collide(algebra, symbolic(first, 'a'), symbolic(second, 'b'), 0);
        const resolved = new Map();
        const evaluate = (expression) => {
            for (const symbol of E.symbolsOf(expression)) {
                if (symbol in values || !definitions.has(symbol)) continue;
                if (!resolved.has(symbol)) resolved.set(symbol, evaluate(definitions.get(symbol)));
                values[symbol] = resolved.get(symbol);
            }
            return E.evaluate(expression, values);
        };
        const expected = collide(numbers, first, second, 0);
        slots.forEach((slot, k) => {
            assert.equal(evaluate(slot.active) > 0.5, expected[k].active, `${first.kind} and ${second.kind}, slot ${k + 1}: whether it touches`);
            if (!expected[k].active) return;
            const error = Math.max(Math.abs(evaluate(slot.dist) - expected[k].dist), differ(slot.pos.map(evaluate), expected[k].pos), differ(slot.frame.map(evaluate), expected[k].frame));
            assert.ok(error < 1e-12, `${first.kind} and ${second.kind}, slot ${k + 1}: differs by ${error}`);
        });
    }
});
