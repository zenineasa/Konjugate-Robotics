/* Copyright © 2026 Zenin Easa Panthakkalakath */

// Undo and redo over whole drawings: each step keeps the drawing as it was and what it was ("Move Base"), so undo can
// say what it undid. Changes of one kind in quick succession (a size typed figure by figure, a solid nudged with the
// arrows) are one step: a change with the same `coalesce` key within `window` milliseconds of the last replaces it.

export class History {
    constructor(drawing, { limit = 200, window = 1200, now = () => Date.now() } = {}) {
        this.current = drawing;
        this.undoStack = [];
        this.redoStack = [];
        this.limit = limit;
        this.window = window;
        this.now = now;
        this.last = null;
    }

    // Makes `drawing` current, as the step `label`. Returns it.
    change(drawing, label, coalesce = null) {
        if (drawing === this.current) return drawing;
        const time = this.now();
        const joins = coalesce && this.last && this.last.coalesce === coalesce && time - this.last.time <= this.window && this.undoStack.length;
        if (!joins) {
            this.undoStack.push({ drawing: this.current, label });
            if (this.undoStack.length > this.limit) this.undoStack.shift();
        }
        this.current = drawing;
        this.redoStack = [];
        this.last = { coalesce, time };
        return drawing;
    }

    // Steps back; returns { drawing, label } of what was undone, or null.
    undo() {
        const step = this.undoStack.pop();
        if (!step) return null;
        this.redoStack.push({ drawing: this.current, label: step.label });
        this.current = step.drawing;
        this.last = null;
        return { drawing: this.current, label: step.label };
    }

    redo() {
        const step = this.redoStack.pop();
        if (!step) return null;
        this.undoStack.push({ drawing: this.current, label: step.label });
        this.current = step.drawing;
        this.last = null;
        return { drawing: this.current, label: step.label };
    }

    // Starts again from `drawing`, forgetting every step (a session restored, say).
    reset(drawing) {
        this.current = drawing;
        this.undoStack = [];
        this.redoStack = [];
        this.last = null;
    }

    get canUndo() { return this.undoStack.length > 0; }
    get canRedo() { return this.redoStack.length > 0; }
    get undoLabel() { return this.undoStack.at(-1)?.label ?? null; }
    get redoLabel() { return this.redoStack.at(-1)?.label ?? null; }
}
