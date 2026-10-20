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
export const sqrt = unary('Sqrt', Math.sqrt);
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

// ---- LaTeX, for reading and editing in the canvas -------------------------------------------------------------------
// Written as Konjugate's canvas writes MathJSON (its equationModel.mjs, mathJsonToLatex), so the canvas reads it back as
// the same expression and a user can edit any equation Build wrote: a symbol of one letter as itself, a longer one
// upright, a number in plain digits or as a power of ten, products with \cdot, the two-argument arctangent as
// \arctan(y, x).

const latexFunctions = {
    Sin: '\\sin', Cos: '\\cos', Tanh: '\\tanh', Arcsin: '\\arcsin', Min: '\\min', Max: '\\max', Arctan2: '\\arctan'
};
const latexComparisons = { Less: '<', Greater: '>' };

function numberLatex(value) {
    const text = String(value);
    if (!/e/i.test(text)) return text;
    const [mantissa, exponent] = value.toExponential().split('e');
    return `${mantissa}\\times10^{${Number(exponent)}}`;
}

const parenthesize = (text) => `\\left(${text}\\right)`;
const isNegative = (expression) => (isNumber(expression) && expression < 0) || (Array.isArray(expression) && expression[0] === 'Negate');

export function toLatex(expression) {
    if (isNumber(expression)) return numberLatex(expression);
    if (typeof expression === 'string') {
        if (expression === 'True') return '\\text{otherwise}';
        return expression.length === 1 ? expression : `\\mathrm{${expression}}`;
    }
    const [operator, ...operands] = expression;
    const factor = (operand) => {
        const text = toLatex(operand);
        return isNegative(operand) || (Array.isArray(operand) && operand[0] === 'Add') ? parenthesize(text) : text;
    };
    if (latexFunctions[operator]) return `${latexFunctions[operator]}${parenthesize(operands.map(toLatex).join(','))}`;
    if (latexComparisons[operator]) return operands.map(toLatex).join(latexComparisons[operator]);
    switch (operator) {
    case 'Add': return operands.map((operand, index) => {
        if (index === 0) return toLatex(operand);
        if (isNumber(operand) && operand < 0) return numberLatex(operand);
        if (Array.isArray(operand) && operand[0] === 'Negate') return `-${factor(operand[1])}`;
        return `+${toLatex(operand)}`;
    }).join('');
    case 'Multiply': return operands.map(factor).join('\\cdot ');
    case 'Negate': return `-${factor(operands[0])}`;
    case 'Divide': return `\\frac{${toLatex(operands[0])}}{${toLatex(operands[1])}}`;
    case 'Sqrt': return `\\sqrt{${toLatex(operands[0])}}`;
    case 'Which': {
        const rows = [];
        for (let index = 0; index < operands.length; index += 2) rows.push(`${toLatex(operands[index + 1])}&${toLatex(operands[index])}`);
        return `\\begin{cases}${rows.join('\\\\')}\\end{cases}`;
    }
    default: throw new Error(`The operation ${operator} cannot be written as LaTeX.`);
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
    case 'Sqrt': return Math.sqrt(value[0]);
    case 'Arctan2': return Math.atan2(value[0], value[1]);
    case 'Min': return Math.min(value[0], value[1]);
    case 'Max': return Math.max(value[0], value[1]);
    case 'Greater': return value[0] > value[1] ? 1 : 0;
    case 'Less': return value[0] < value[1] ? 1 : 0;
    default: throw new Error(`Unknown operator ${operator}.`);
    }
}
