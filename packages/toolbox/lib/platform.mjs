/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The keys of the platform the window runs on, so shortcuts behave and read as its users expect: ⌘ and ⌫ on a Mac,
// where Control-click opens a menu; Ctrl and Delete on Windows and Linux. As the Logistics toolbox's.

export function platformKeys(platform = globalThis.navigator?.platform ?? '') {
    const mac = /Mac|iPhone|iPad/.test(platform);
    return mac ? {
        mac, modifier: '⌘', undo: '⌘Z', redo: '⇧⌘Z', duplicate: '⌘D', build: '⌘B', delete: '⌫',
        deleteKeys: '⌫ (delete) or ⌦', addClick: 'Shift-click or ⌘-click'
    } : {
        mac, modifier: 'Ctrl', undo: 'Ctrl+Z', redo: 'Ctrl+Y or Ctrl+Shift+Z', duplicate: 'Ctrl+D', build: 'Ctrl+B', delete: 'Delete',
        deleteKeys: 'Delete or Backspace', addClick: 'Shift-click or Ctrl-click'
    };
}

// The platform's command modifier held: ⌘ on a Mac, Ctrl elsewhere.
export const commandHeld = (event, { mac }) => (mac ? event.metaKey : event.ctrlKey);

// A click that adds to the selection: Shift, or the command modifier.
export const addsToSelection = (event, keys) => event.shiftKey || commandHeld(event, keys);
