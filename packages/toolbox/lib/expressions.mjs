/* Copyright © 2026 Zenin Easa Panthakkalakath */

// A small builder for the equations Build writes into a robot's relationships, as MathJSON (what Konjugate's engine
// runs) with LaTeX beside it (what the canvas shows). Sums and products drop zeros and ones and fold numbers, so a
// joint along a plain axis gets plain equations. No dependencies.
//
// An expression is a number, a symbol (a string) or [operator, ...arguments].

const isNumber = (value) => typeof value === 'number';
// An expression is [operator, ...]; any other array passed to add or multiply is a list of terms.
const isExpression = (value) => Array.isArray(value) && typeof value[0] === 'string' && /^[A-Z]/.test(value[0]);
const terms = (values) => values.flatMap((value) => (Array.isArray(value) && !isExpression(value) ? terms(value) : [value]));

export const round = (value) => (Math.abs(value) < 1e-15 ? 0 : Number(value.toPrecision(15)));

export function add(...items) {
    const flat = [];
    let constant = 0;
    for (const term of terms(items)) {
        if (isNumber(term)) constant += term;
        else if (Array.isArray(term) && term[0] === 'Add') flat.push(...term.slice(1));
        else flat.push(term);
    }
    if (constant !== 0) flat.push(round(constant));
    if (!flat.length) return 0;
    return flat.length === 1 ? flat[0] : ['Add', ...flat];
}

export function multiply(...factors) {
    const flat = [];
    let constant = 1;
    for (const factor of terms(factors)) {
        if (isNumber(factor)) constant *= factor;
        else if (Array.isArray(factor) && factor[0] === 'Multiply') flat.push(...factor.slice(1));
        else flat.push(factor);
    }
    if (constant === 0) return 0;
    if (!flat.length) return round(constant);
    if (constant === -1) return negate(flat.length === 1 ? flat[0] : ['Multiply', ...flat]);
    if (constant !== 1) flat.unshift(round(constant));
    return flat.length === 1 ? flat[0] : ['Multiply', ...flat];
}

export function negate(value) {
    if (isNumber(value)) return round(-value);
    if (Array.isArray(value) && value[0] === 'Negate') return value[1];
    return ['Negate', value];
}

export const subtract = (left, right) => add(left, negate(right));

export function divide(numerator, denominator) {
    if (isNumber(numerator) && numerator === 0) return 0;
    if (isNumber(denominator) && denominator === 1) return numerator;
    if (isNumber(numerator) && isNumber(denominator)) return round(numerator / denominator);
    return ['Divide', numerator, denominator];
}

const unary = (name, fold) => (value) => (isNumber(value) ? round(fold(value)) : [name, value]);
export const sin = unary('Sin', Math.sin);
export const cos = unary('Cos', Math.cos);
export const tanh = unary('Tanh', Math.tanh);
export const asin = unary('Arcsin', Math.asin);
export const atan2 = (y, x) => (isNumber(y) && isNumber(x) ? round(Math.atan2(y, x)) : ['Arctan2', y, x]);
export const min = (a, b) => ['Min', a, b];
export const max = (a, b) => ['Max', a, b];
// A piecewise value: [[condition, value], ...], then the value otherwise.
export const cases = (pairs, otherwise) => ['Which', ...pairs.flat(), 'True', otherwise];
export const greater = (a, b) => ['Greater', a, b];
export const less = (a, b) => ['Less', a, b];

// Sum of products of two equally long lists.
export const dot = (left, right) => add(left.map((value, index) => multiply(value, right[index])));
// A 3 by 3 matrix (rows) times a vector.
export const matrixVector = (matrix, vector) => matrix.map((row) => dot(row, vector));
// Two 3 by 3 matrices (rows) multiplied.
export const matrixMatrix = (left, right) => left.map((row) => [0, 1, 2].map((column) => dot(row, right.map((line) => line[column]))));
export const cross = (a, b) => [
    subtract(multiply(a[1], b[2]), multiply(a[2], b[1])),
    subtract(multiply(a[2], b[0]), multiply(a[0], b[2])),
    subtract(multiply(a[0], b[1]), multiply(a[1], b[0]))
];

// ---- LaTeX, for reading in the canvas -------------------------------------------------------------------------------

const precedence = { Add: 1, Negate: 2, Multiply: 3, Divide: 3 };

function symbolLatex(name) {
    // "ia14" reads as ia_{14}, "parentR12" as parentR_{12}.
    const match = /^([A-Za-z]+?)(\d+)$/.exec(name);
    const base = match ? match[1] : name;
    const text = base.length > 1 ? `\\mathrm{${base}}` : base;
    return match ? `${text}_{${match[2]}}` : text;
}

export function toLatex(expression, parent = 0) {
    if (isNumber(expression)) {
        const text = String(round(expression));
        return expression < 0 && parent > 1 ? `(${text})` : text;
    }
    if (typeof expression === 'string') return expression === 'True' ? '\\text{otherwise}' : symbolLatex(expression);
    const [operator, ...args] = expression;
    const wrap = (text, own) => (own < parent ? `\\left(${text}\\right)` : text);
    switch (operator) {
    case 'Add': return wrap(args.map((arg, index) => {
        const text = toLatex(arg, 1);
        return index && !text.startsWith('-') ? `+${text}` : text;
    }).join(''), 1);
    case 'Multiply': return wrap(args.map((arg) => toLatex(arg, 3)).join('\\,'), 3);
    case 'Negate': return wrap(`-${toLatex(args[0], 3)}`, 2);
    case 'Divide': return `\\frac{${toLatex(args[0])}}{${toLatex(args[1])}}`;
    case 'Sin': case 'Cos': case 'Tanh': return `\\${operator.toLowerCase()}\\left(${toLatex(args[0])}\\right)`;
    case 'Arcsin': return `\\arcsin\\left(${toLatex(args[0])}\\right)`;
    case 'Arctan2': return `\\operatorname{atan2}\\left(${toLatex(args[0])},${toLatex(args[1])}\\right)`;
    case 'Min': case 'Max': return `\\${operator.toLowerCase()}\\left(${toLatex(args[0])},${toLatex(args[1])}\\right)`;
    case 'Greater': return `${toLatex(args[0])}>${toLatex(args[1])}`;
    case 'Less': return `${toLatex(args[0])}<${toLatex(args[1])}`;
    case 'Which': {
        const rows = [];
        for (let index = 0; index < args.length; index += 2) rows.push(`${toLatex(args[index + 1])} & ${toLatex(args[index])}`);
        return `\\begin{cases}${rows.join('\\\\')}\\end{cases}`;
    }
    default: return `\\operatorname{${operator}}\\left(${args.map((arg) => toLatex(arg)).join(',')}\\right)`;
    }
}

// The symbols an expression reads.
export function symbolsOf(expression, found = new Set()) {
    if (typeof expression === 'string') { if (expression !== 'True') found.add(expression); }
    else if (Array.isArray(expression)) expression.slice(1).forEach((arg) => symbolsOf(arg, found));
    return found;
}

// Evaluates an expression with `values` (a Map or object of symbol values): for tests, and to check the model in
// JavaScript before the engine runs it.
export function evaluate(expression, values) {
    const get = (name) => (values instanceof Map ? values.get(name) : values[name]);
    if (isNumber(expression)) return expression;
    if (typeof expression === 'string') {
        const value = get(expression);
        if (value === undefined) throw new Error(`No value for ${expression}.`);
        return value;
    }
    const [operator, ...args] = expression;
    if (operator === 'Which') {
        for (let index = 0; index < args.length; index += 2) {
            if (args[index] === 'True' || evaluate(args[index], values)) return evaluate(args[index + 1], values);
        }
        throw new Error('No branch matched.');
    }
    const value = args.map((arg) => evaluate(arg, values));
    switch (operator) {
    case 'Add': return value.reduce((sum, item) => sum + item, 0);
    case 'Multiply': return value.reduce((product, item) => product * item, 1);
    case 'Negate': return -value[0];
    case 'Divide': return value[0] / value[1];
    case 'Sin': return Math.sin(value[0]);
    case 'Cos': return Math.cos(value[0]);
    case 'Tanh': return Math.tanh(value[0]);
    case 'Arcsin': return Math.asin(value[0]);
    case 'Arctan2': return Math.atan2(value[0], value[1]);
    case 'Min': return Math.min(value[0], value[1]);
    case 'Max': return Math.max(value[0], value[1]);
    case 'Greater': return value[0] > value[1] ? 1 : 0;
    case 'Less': return value[0] < value[1] ? 1 : 0;
    default: throw new Error(`Unknown operator ${operator}.`);
    }
}
