/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Places this toolbox's component templates (packages/engine/components, and core's own where wanted) into a project
// document, the way the app places them: a node template becomes a node with its source terms, a bundle becomes its
// edges between the given nodes. Ids follow whatever the document already holds.
//
// Shared parameters are resolved by key within a placement scope (a Map, one per placed thing, such as one joint's
// motor and its drive): the first template in the scope to declare a key creates the shared parameter, later ones
// reuse it, so a motor and its drive have one winding resistance between them.
//
// A bundle endpoint may name which of its node's states plays a port: { node, ports: { speed: stateId } } binds that
// state as `sourceSpeed` (or `targetSpeed`) in the edge's equation, and only that one of the node's states, for a node
// such as a mechanism whose joints each have their own. Otherwise every state of the node is bound, as the app does.
//
// It has no dependencies of its own: Konjugate's equation helpers (validateEquationLatex, timeBinding) come from the
// host or, in scripts and tests, from the Konjugate checkout.

const upperFirst = (text) => text[0].toUpperCase() + text.slice(1);

// The time binding the node editor adds to every equation, `t` (or `t2`, ... if `t` is taken), for a host whose
// helpers do not include it (an importer is given only the equation validator and binding reconciler).
function timeBinding(usedSymbols = new Set(), preferred = 't') {
    let symbol = preferred;
    for (let suffix = 2; usedSymbols.has(symbol); suffix += 1) symbol = `${preferred}${suffix}`;
    return { kind: 'time', symbol, label: `${symbol} (time)` };
}

// The largest id anywhere in a document: every persistent entity's `id` shares one id space.
function largestId(value) {
    if (Array.isArray(value)) return value.reduce((largest, item) => Math.max(largest, largestId(item)), 0);
    if (!value || typeof value !== 'object') return 0;
    return Object.entries(value).reduce((largest, [key, item]) => Math.max(largest, key === 'id' && Number.isSafeInteger(item) ? item : largestId(item)), 0);
}

export class DocumentBuilder {
    // `templates` is a Map of component templates by id; `helpers` are Konjugate's equation helpers.
    constructor(document, templates, helpers) {
        if (!helpers?.validateEquationLatex) throw new Error('DocumentBuilder needs Konjugate\'s equation helpers.');
        this.document = document;
        document.nodes ??= [];
        document.edges ??= [];
        document.sharedParameters ??= [];
        this.templates = templates;
        this.helpers = helpers;
        this.nextId = largestId(document) + 1;
    }

    id() {
        return this.nextId++;
    }

    template(id, kind) {
        const template = this.templates.get(id);
        if (!template || template.kind !== kind) throw new Error(`No ${kind} template "${id}".`);
        return template;
    }

    // Creates the shared parameters `declared` names that `scope` does not yet hold, with a symbol unique in the
    // document; `values` overrides a declared value by key.
    resolveShared(declared = [], values = {}, scope = new Map()) {
        for (const parameter of declared) {
            if (scope.has(parameter.key)) continue;
            let symbol = parameter.symbol;
            for (let suffix = 2; this.document.sharedParameters.some((shared) => shared.symbol === symbol); suffix += 1) symbol = `${parameter.symbol}${suffix}`;
            const mode = parameter.mode ?? 'constant';
            const created = {
                id: this.id(), name: parameter.name, symbol, value: values[parameter.key] ?? parameter.value, unit: parameter.unit ?? '', mode,
                ...(mode === 'live' && parameter.control ? { control: { ...parameter.control } } : {})
            };
            this.document.sharedParameters.push(created);
            scope.set(parameter.key, created);
        }
        return scope;
    }

    parameter(parameter, scope) {
        const shared = parameter.shared === undefined ? null : scope.get(parameter.shared);
        if (parameter.shared !== undefined && !shared) throw new Error(`No shared parameter "${parameter.shared}" in scope.`);
        const source = shared ?? parameter;
        return {
            id: this.id(), name: parameter.name, symbol: parameter.symbol, value: Number(source.value) || 0, unit: source.unit ?? '',
            mode: source.mode ?? 'constant', ...(source.control ? { control: { ...source.control } } : {}), ...(shared ? { sharedParameterId: shared.id } : {})
        };
    }

    // `substeps` is the node's substeps per global step, for a node faster than the rest of the model.
    placeNode(templateId, { name, position = [0, 0, 0], initialValues = {}, values = {}, scope = new Map(), substeps = 1 } = {}) {
        const template = this.template(templateId, 'node');
        this.resolveShared(template.sharedParameters, values, scope);
        const node = {
            id: this.id(), name: name ?? template.name, type: template.name, position, sourceTerms: [], numerics: { substepsPerGlobalStep: substeps },
            states: template.states.map((state) => ({
                id: this.id(), name: state.label, symbol: state.symbol, initialValue: initialValues[state.symbol] ?? state.initialValue, unit: state.unit ?? ''
            })),
            appearance: { type: 'primitive', shape: template.shape ?? 'box', color: template.color ?? '#34727a' }
        };
        for (const term of template.sourceTerms ?? []) {
            const parameters = (term.parameters ?? []).map((parameter) => this.parameter(parameter, scope));
            const bindings = [
                ...node.states.map((state) => ({ kind: 'state', nodeId: node.id, stateId: state.id, symbol: state.symbol })),
                ...parameters.map((parameter) => ({ kind: 'parameter', parameterId: parameter.id, symbol: parameter.symbol }))
            ];
            bindings.push((this.helpers.timeBinding ?? timeBinding)(new Set(bindings.map((binding) => binding.symbol))));
            const validation = this.helpers.validateEquationLatex(term.expression, bindings);
            if (!validation.valid) throw new Error(`${node.name} ${term.state}: ${validation.errors.join(' ')}`);
            const output = node.states.find((state) => state.symbol === term.state);
            node.sourceTerms.push({
                id: this.id(), state: term.state, expression: term.expression, parameters, ...(term.setsValue ? { setsValue: true } : {}),
                expressionModel: { latex: term.expression, bindings, output: { stateId: output.id }, mathJson: validation.mathJson }
            });
        }
        this.document.nodes.push(node);
        return node;
    }

    // `endpoints` maps each of the bundle's endpoint ids to a placed node, or to { node, ports: { port: stateId } }.
    applyBundle(templateId, endpoints, { values = {}, scope = new Map(), name } = {}) {
        const template = this.template(templateId, 'bundle');
        const resolved = Object.fromEntries(template.endpoints.map((endpoint) => {
            const given = endpoints[endpoint.id];
            if (!given) throw new Error(`${template.name}: no node for endpoint "${endpoint.id}".`);
            return [endpoint.id, given.node ? given : { node: given, ports: {} }];
        }));
        this.resolveShared(template.sharedParameters, values, scope);
        return template.edges.map((edge) => {
            const ends = { source: resolved[edge.from], target: resolved[edge.to] };
            // The state that plays a port: the one the endpoint names for it, or the node's state of that symbol.
            const stateFor = (role, port) => {
                const { node, ports } = ends[role];
                const stateId = ports[port];
                const state = node.states.find((candidate) => (stateId === undefined ? candidate.symbol === port : candidate.id === stateId));
                if (!state) throw new Error(`${template.name} / ${edge.name}: ${node.name} has no state for "${port}".`);
                return state;
            };
            const bindings = [];
            for (const role of ['source', 'target']) {
                const { node, ports } = ends[role];
                if (Object.keys(ports).length) {
                    for (const port of Object.keys(ports)) bindings.push({ kind: 'state', role, nodeId: node.id, stateId: stateFor(role, port).id, symbol: `${role}${upperFirst(port)}` });
                } else {
                    for (const state of node.states) bindings.push({ kind: 'state', role, nodeId: node.id, stateId: state.id, symbol: `${role}${upperFirst(state.symbol)}` });
                }
            }
            const parameters = (edge.parameters ?? []).map((parameter) => this.parameter(parameter, scope));
            bindings.push(...parameters.map((parameter) => ({ kind: 'parameter', parameterId: parameter.id, symbol: parameter.symbol })));
            bindings.push((this.helpers.timeBinding ?? timeBinding)(new Set(bindings.map((binding) => binding.symbol))));
            const validation = this.helpers.validateEquationLatex(edge.latex, bindings);
            if (!validation.valid) throw new Error(`${template.name} / ${edge.name}: ${validation.errors.join(' ')}`);
            const sourceState = stateFor('source', edge.ports.source);
            const targetState = stateFor('target', edge.ports.target);
            const output = edge.output.role === 'source' ? sourceState : targetState;
            const definition = {
                id: this.id(), name: `${name ? `${name} ` : ''}${edge.name}: ${ends.source.node.name} → ${ends.target.node.name}`,
                source: { nodeId: ends.source.node.id, stateId: sourceState.id }, target: { nodeId: ends.target.node.id, stateId: targetState.id },
                directionality: 'directed', equation: edge.latex,
                equationModel: { latex: edge.latex, output: { role: edge.output.role, stateId: output.id }, bindings, mathJson: validation.mathJson },
                parameters, appearance: { color: edge.color ?? template.color ?? '#9c83c4', offset: 0 }
            };
            this.document.edges.push(definition);
            return definition;
        });
    }
}
