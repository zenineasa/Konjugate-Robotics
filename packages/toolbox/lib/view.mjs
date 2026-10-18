/* Copyright © 2026 Zenin Easa Panthakkalakath */

// The 3D view of a drawing: the floor, the solids, the joints, and what the pointer is over. It draws what it is given
// (setDrawing, setSelection) and says what the user points at and drags (the callbacks); it never changes the drawing
// itself. Space is z up, metres, as in the drawing.
//
// Pointer: left-drag on empty space turns the view, right-drag (or Shift-left-drag) moves it, the wheel zooms; a
// click (a press and release without moving) picks. With a solid selected and the Move or Turn tool, its handles move or
// turn it.

import * as THREE from '../vendor/three.module.js';
import { OrbitControls } from '../vendor/OrbitControls.js';
import { TransformControls } from '../vendor/TransformControls.js';
import { frameById, frameInWorld, groundId, solidById, structure } from './drawing.mjs';
import { halfExtents, nearestSnap, surfaceNormal } from './shapes.mjs';

const toThree = (q) => new THREE.Quaternion(q[1], q[2], q[3], q[0]);
const fromThree = (q) => [q.w, q.x, q.y, q.z];

function geometryOf(solid) {
    const { shape, size } = solid;
    let geometry;
    if (shape === 'box') geometry = new THREE.BoxGeometry(size.x, size.y, size.z);
    else if (shape === 'cylinder') geometry = new THREE.CylinderGeometry(size.radius, size.radius, size.length, 40).rotateX(Math.PI / 2);
    else if (shape === 'sphere') geometry = new THREE.SphereGeometry(size.radius, 40, 20);
    else geometry = new THREE.CapsuleGeometry(size.radius, size.length, 10, 32).rotateX(Math.PI / 2);
    return geometry;
}

export class DrawingView {
    constructor(container, callbacks = {}) {
        this.container = container;
        this.callbacks = callbacks;
        this.drawing = null;
        this.selection = [];
        this.meshes = new Map();
        this.carried = [];
        this.tool = 'select';

        this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
        this.renderer.setPixelRatio(Math.min(2, globalThis.devicePixelRatio ?? 1));
        this.renderer.setClearColor(0x1b232b);
        container.appendChild(this.renderer.domElement);
        this.renderer.domElement.className = 'viewCanvas';
        this.renderer.domElement.tabIndex = 0;
        this.renderer.domElement.setAttribute('aria-label', '3D view of the robot');

        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(40, 1, 0.001, 200);
        this.camera.up.set(0, 0, 1);
        this.camera.position.set(0.6, -0.9, 0.55);
        this.scene.add(new THREE.HemisphereLight(0xdfe8f0, 0x2a3138, 1.6));
        const sun = new THREE.DirectionalLight(0xffffff, 1.6);
        sun.position.set(1, -1.5, 2.5);
        this.scene.add(sun);

        // The floor: a grid of 5 cm squares, 2 m across, with a line for each axis.
        const grid = new THREE.GridHelper(2, 40, 0x56636f, 0x323c46);
        grid.rotateX(Math.PI / 2);
        this.scene.add(grid);
        this.scene.add(new THREE.AxesHelper(0.1));
        // Clicks on the floor are clicks on the Ground.
        this.floor = new THREE.Mesh(new THREE.PlaneGeometry(20, 20), new THREE.MeshBasicMaterial({ visible: false }));
        this.floor.name = 'floor';
        this.scene.add(this.floor);

        this.solidGroup = new THREE.Group();
        this.jointGroup = new THREE.Group();
        this.markerGroup = new THREE.Group();
        this.scene.add(this.solidGroup, this.jointGroup, this.markerGroup);

        this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
        this.orbit.target.set(0, 0, 0.08);
        this.orbit.enableDamping = false;
        this.orbit.screenSpacePanning = true;
        this.orbit.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN };
        this.orbit.keys = {};
        this.orbit.addEventListener('change', () => this.render());

        this.gizmo = new TransformControls(this.camera, this.renderer.domElement);
        this.gizmo.setSize(0.8);
        this.gizmoHelper = this.gizmo.getHelper();
        this.scene.add(this.gizmoHelper);
        this.gizmo.addEventListener('change', () => this.render());
        this.gizmo.addEventListener('dragging-changed', (event) => {
            this.orbit.enabled = !event.value;
            const object = this.gizmo.object;
            if (!object) return;
            if (event.value) {
                this.dragStart = { position: object.position.clone(), quaternion: object.quaternion.clone() };
                this.carriedStart = this.carried.map((id) => {
                    const mesh = this.meshes.get(id);
                    return { id, mesh, position: mesh.position.clone(), quaternion: mesh.quaternion.clone() };
                });
            } else if (this.dragStart) {
                const start = this.dragStart;
                this.dragStart = null;
                this.callbacks.onTransformEnd?.(object.userData.solidId, {
                    from: { position: start.position.toArray(), rotation: fromThree(start.quaternion) },
                    to: { position: object.position.toArray(), rotation: fromThree(object.quaternion) }
                });
            }
        });
        this.gizmo.addEventListener('objectChange', () => {
            const object = this.gizmo.object;
            if (!object || !this.dragStart) return;
            // The solids it carries follow, as one piece.
            const turn = object.quaternion.clone().multiply(this.dragStart.quaternion.clone().invert());
            for (const item of this.carriedStart ?? []) {
                const offset = item.position.clone().sub(this.dragStart.position).applyQuaternion(turn);
                item.mesh.position.copy(object.position).add(offset);
                item.mesh.quaternion.copy(turn).multiply(item.quaternion);
            }
            this.callbacks.onTransform?.(object.userData.solidId, { position: object.position.toArray(), rotation: fromThree(object.quaternion) });
        });

        this.raycaster = new THREE.Raycaster();
        this.pointer = { down: null };
        const canvas = this.renderer.domElement;
        canvas.addEventListener('pointerdown', (event) => {
            this.pointer.down = { x: event.clientX, y: event.clientY, button: event.button };
        });
        canvas.addEventListener('pointerup', (event) => {
            const down = this.pointer.down;
            this.pointer.down = null;
            if (!down || down.button !== 0 || this.gizmo.dragging) return;
            if (Math.hypot(event.clientX - down.x, event.clientY - down.y) > 4) return;
            this.callbacks.onClick?.(this.pickAt(event.clientX, event.clientY), event);
        });
        canvas.addEventListener('pointermove', (event) => {
            if (this.pointer.down) return;
            this.callbacks.onHover?.(this.pickAt(event.clientX, event.clientY), event);
        });
        canvas.addEventListener('pointerleave', () => this.callbacks.onHover?.(null));
        canvas.addEventListener('contextmenu', (event) => event.preventDefault());

        this.resizeObserver = new ResizeObserver(() => this.resize());
        this.resizeObserver.observe(container);
        this.resize();
    }

    resize() {
        const { clientWidth: width, clientHeight: height } = this.container;
        if (!width || !height) return;
        this.renderer.setSize(width, height, false);
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.render();
    }

    render() {
        if (this.renderPending) return;
        this.renderPending = true;
        requestAnimationFrame(() => {
            this.renderPending = false;
            this.renderer.render(this.scene, this.camera);
        });
    }

    // ---- what is drawn -------------------------------------------------------------------------------------------

    setDrawing(drawing) {
        this.drawing = drawing;
        const keep = new Set(drawing.solids.map(({ id }) => id));
        for (const [id, mesh] of this.meshes) {
            if (keep.has(id)) continue;
            if (this.gizmo.object === mesh) this.gizmo.detach();
            this.solidGroup.remove(mesh);
            mesh.geometry.dispose();
            this.meshes.delete(id);
        }
        for (const solid of drawing.solids) {
            let mesh = this.meshes.get(solid.id);
            const shapeKey = JSON.stringify([solid.shape, solid.size]);
            if (!mesh || mesh.userData.shapeKey !== shapeKey) {
                if (mesh) { this.solidGroup.remove(mesh); mesh.geometry.dispose(); }
                const geometry = geometryOf(solid);
                mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: solid.color, roughness: 0.65, metalness: 0.05 }));
                mesh.add(new THREE.LineSegments(new THREE.EdgesGeometry(geometry, 30), new THREE.LineBasicMaterial({ color: 0x0b0f13, transparent: true, opacity: 0.45 })));
                mesh.userData = { solidId: solid.id, shapeKey };
                this.solidGroup.add(mesh);
                this.meshes.set(solid.id, mesh);
                if (this.gizmo.object?.userData.solidId === solid.id) this.gizmo.attach(mesh);
            }
            mesh.material.color.set(solid.color);
            mesh.position.set(...solid.position);
            mesh.quaternion.copy(toThree(solid.rotation));
        }
        this.drawJoints();
        this.applySelection();
        this.render();
    }

    // A small mark at each joint: a ring about a hinge's axis, an arrow along a slider's, a ball, or a cube when fixed.
    drawJoints() {
        for (const child of [...this.jointGroup.children]) {
            this.jointGroup.remove(child);
            child.traverse((part) => { part.geometry?.dispose(); });
        }
        const drawing = this.drawing;
        const built = structure(drawing);
        const loops = new Set(built.loops.map(({ id }) => id));
        for (const joint of drawing.joints) {
            const parentFrame = frameById(drawing, joint.parent);
            const childFrame = frameById(drawing, joint.child);
            if (!parentFrame || !childFrame) continue;
            const anchor = frameInWorld(drawing, parentFrame);
            const size = this.markSize(parentFrame.solid, childFrame.solid);
            const selected = this.selection.includes(joint.id);
            const color = selected ? 0xffd166 : loops.has(joint.id) ? 0xb08cff : 0x4fd1c5;
            const material = new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.95 });
            const mark = new THREE.Group();
            mark.userData = { jointId: joint.id };
            mark.renderOrder = 10;
            const axis = new THREE.Vector3(...anchor.axis);
            const along = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), axis);
            if (joint.kind === 'hinge') {
                const ring = new THREE.Mesh(new THREE.TorusGeometry(size, size * 0.12, 8, 40), material);
                ring.quaternion.copy(along);
                mark.add(ring);
            } else if (joint.kind === 'slider') {
                const rod = new THREE.Mesh(new THREE.CylinderGeometry(size * 0.1, size * 0.1, size * 3, 12).rotateX(Math.PI / 2), material);
                rod.quaternion.copy(along);
                mark.add(rod);
            } else if (joint.kind === 'ball') {
                mark.add(new THREE.Mesh(new THREE.SphereGeometry(size * 0.6, 20, 12), material));
            } else {
                mark.add(new THREE.Mesh(new THREE.BoxGeometry(size, size, size), material));
            }
            if (joint.kind === 'hinge' || joint.kind === 'slider') {
                const arrow = new THREE.ArrowHelper(axis, new THREE.Vector3(), size * 2.4, color, size * 0.7, size * 0.4);
                arrow.traverse((part) => { if (part.material) { part.material.depthTest = false; part.renderOrder = 11; } });
                mark.add(arrow);
            }
            mark.position.set(...anchor.point);
            mark.traverse((part) => { part.renderOrder = 10; });
            this.jointGroup.add(mark);
        }
    }

    // A mark's size: a fifth of the smaller solid it joins, at least 5 mm.
    markSize(...solidIds) {
        const sizes = solidIds.filter((id) => id !== groundId).map((id) => solidById(this.drawing, id)).filter(Boolean)
            .map((solid) => Math.min(...halfExtents(solid.shape, solid.size)));
        return Math.max(0.005, Math.min(...sizes, 0.05) * 0.6);
    }

    setSelection(ids) {
        this.selection = [...ids];
        this.applySelection();
        this.drawJoints();
        this.render();
    }

    applySelection() {
        for (const [id, mesh] of this.meshes) {
            const selected = this.selection.includes(id);
            mesh.material.emissive.set(selected ? 0x4a3a10 : 0x000000);
        }
        this.updateGizmo();
    }

    // The tool decides the handles: Move and Turn put them on the one selected solid.
    setTool(tool, carried = []) {
        this.tool = tool;
        this.carried = carried;
        this.updateGizmo();
        this.render();
    }

    updateGizmo() {
        const solidIds = this.selection.filter((id) => this.meshes.has(id));
        const wants = (this.tool === 'move' || this.tool === 'turn') && solidIds.length === 1;
        if (!wants) {
            if (this.gizmo.object) this.gizmo.detach();
            return;
        }
        this.gizmo.setMode(this.tool === 'move' ? 'translate' : 'rotate');
        this.gizmo.setSpace(this.tool === 'move' ? 'world' : 'local');
        this.gizmo.translationSnap = this.snapping ? 0.005 : null;
        this.gizmo.rotationSnap = this.snapping ? THREE.MathUtils.degToRad(15) : null;
        const mesh = this.meshes.get(solidIds[0]);
        if (this.gizmo.object !== mesh) this.gizmo.attach(mesh);
    }

    setSnapping(on) {
        this.snapping = on;
        this.updateGizmo();
    }

    // ---- what the pointer is over ----------------------------------------------------------------------------------

    // The solid (or the Ground) under a point of the screen, the point on it and the surface's outward direction there,
    // both in the world, snapped to a face's centre or an edge's middle when near one: { solid, point, normal, snapped },
    // or null over nothing. A joint's mark under the point comes first: { joint }.
    pickAt(clientX, clientY) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        const ndc = new THREE.Vector2(((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
        this.raycaster.setFromCamera(ndc, this.camera);
        if (this.tool !== 'join') {
            const markHit = this.raycaster.intersectObjects(this.jointGroup.children, true)[0];
            if (markHit) {
                let object = markHit.object;
                while (object && !object.userData.jointId) object = object.parent;
                if (object) return { joint: object.userData.jointId };
            }
        }
        const hit = this.raycaster.intersectObjects([...this.meshes.values()], false)[0];
        if (hit) {
            const solid = this.drawing.solids.find(({ id }) => id === hit.object.userData.solidId);
            const inverse = hit.object.matrixWorld.clone().invert();
            const local = hit.point.clone().applyMatrix4(inverse).toArray();
            const half = halfExtents(solid.shape, solid.size);
            const snap = nearestSnap(solid.shape, solid.size, local, Math.min(...half) * 0.35);
            const point = snap ? snap.point : local;
            const normal = snap ? snap.normal : surfaceNormal(solid.shape, solid.size, local);
            return {
                solid: solid.id,
                point: new THREE.Vector3(...point).applyMatrix4(hit.object.matrixWorld).toArray(),
                normal: new THREE.Vector3(...normal).applyQuaternion(hit.object.quaternion).normalize().toArray(),
                snapped: Boolean(snap)
            };
        }
        const floorHit = this.raycaster.intersectObject(this.floor, false)[0];
        if (floorHit) {
            // The floor snaps to a 5 mm grid.
            const point = floorHit.point.toArray().map((value, axis) => (axis < 2 ? Math.round(value / 0.005) * 0.005 : 0));
            return { solid: groundId, point, normal: [0, 0, 1], snapped: true };
        }
        return null;
    }

    // A mark where a joint would go: a dot and an arrow along the surface's direction. `kind` is 'hover' or 'chosen'.
    showMarker(kind, pick) {
        const name = `marker-${kind}`;
        const old = this.markerGroup.getObjectByName(name);
        if (old) this.markerGroup.remove(old);
        if (pick?.point) {
            const size = pick.solid === groundId ? 0.012 : this.markSize(pick.solid);
            const color = kind === 'chosen' ? 0xffd166 : pick.snapped ? 0x7ee787 : 0xe6edf3;
            const group = new THREE.Group();
            group.name = name;
            const dot = new THREE.Mesh(new THREE.SphereGeometry(size * 0.35, 16, 10), new THREE.MeshBasicMaterial({ color, depthTest: false }));
            group.add(dot);
            const arrow = new THREE.ArrowHelper(new THREE.Vector3(...pick.normal), new THREE.Vector3(), size * 3, color, size * 0.9, size * 0.5);
            arrow.traverse((part) => { if (part.material) part.material.depthTest = false; });
            group.add(arrow);
            group.position.set(...pick.point);
            group.traverse((part) => { part.renderOrder = 12; });
            this.markerGroup.add(group);
        }
        this.render();
    }

    // ---- the camera ----------------------------------------------------------------------------------------------------

    // Frames every solid (or the floor's middle when there are none).
    fit() {
        const box = new THREE.Box3();
        for (const mesh of this.meshes.values()) box.expandByObject(mesh);
        if (box.isEmpty()) box.set(new THREE.Vector3(-0.15, -0.15, 0), new THREE.Vector3(0.15, 0.15, 0.15));
        const sphere = box.getBoundingSphere(new THREE.Sphere());
        const radius = Math.max(sphere.radius, 0.05);
        const direction = this.camera.position.clone().sub(this.orbit.target).normalize();
        const distance = radius / Math.sin(THREE.MathUtils.degToRad(this.camera.fov / 2)) * 1.15;
        this.orbit.target.copy(sphere.center);
        this.camera.position.copy(sphere.center).add(direction.multiplyScalar(distance));
        this.camera.near = Math.max(0.0005, distance / 500);
        this.camera.far = distance * 50;
        this.camera.updateProjectionMatrix();
        this.orbit.update();
        this.render();
    }

    // Looks from a side: 'front' (from -y), 'right' (from +x), 'top' (from +z) or 'iso'.
    look(side) {
        const directions = { front: [0, -1, 0.0001], right: [1, 0, 0.0001], top: [0.0001, -0.0001, 1], iso: [0.6, -0.9, 0.55] };
        const distance = this.camera.position.distanceTo(this.orbit.target);
        const direction = new THREE.Vector3(...(directions[side] ?? directions.iso)).normalize();
        this.camera.position.copy(this.orbit.target).add(direction.multiplyScalar(distance));
        this.orbit.update();
        this.render();
    }

    dispose() {
        this.resizeObserver.disconnect();
        this.gizmo.dispose();
        this.orbit.dispose();
        this.renderer.dispose();
    }
}
