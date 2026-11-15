import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {toCreasedNormals} from 'three/addons/utils/BufferGeometryUtils.js';

const BUILD = 'b1';
const el = document.getElementById('viewer'), info = document.getElementById('vinfo');
const Q = new URLSearchParams(location.search);
const scene = new THREE.Scene();
const cam = new THREE.PerspectiveCamera(30, el.clientWidth / el.clientHeight, 5, 6000);
const r = new THREE.WebGLRenderer({antialias: true, alpha: true, preserveDrawingBuffer: true});
r.setPixelRatio(Math.min(devicePixelRatio, 2)); r.setSize(el.clientWidth, el.clientHeight);
r.shadowMap.enabled = true; r.shadowMap.type = THREE.PCFSoftShadowMap;
r.toneMapping = THREE.ACESFilmicToneMapping; r.toneMappingExposure = 1.0;
el.appendChild(r.domElement);
const ctl = new OrbitControls(cam, r.domElement); ctl.enableDamping = true; ctl.minDistance = 150; ctl.maxDistance = 2000;
const TGT = new THREE.Vector3(0, 165, 0);
// robot frame: x forward, y left, z up  ->  three: (x, z, -y)
const VIEWS = {front: [900, 190, 0], '34': [640, 330, -560], side: [0, 190, -900], back: [-900, 190, 0]};
function setView(n) { const v = VIEWS[n] || VIEWS['34']; cam.position.set(...v); ctl.target.copy(TGT); ctl.update();
  document.querySelectorAll('[data-view]').forEach(b => b.classList.toggle('on', b.dataset.view === n)); }

scene.add(new THREE.HemisphereLight(0xfff8ee, 0x8a8577, 1.25));
const key = new THREE.DirectionalLight(0xffffff, 2.0); key.position.set(450, 800, -300); key.castShadow = true;
key.shadow.mapSize.set(2048, 2048); Object.assign(key.shadow.camera, {left: -300, right: 300, top: 300, bottom: -300, near: 100, far: 2000});
key.shadow.bias = -0.0005; key.shadow.normalBias = 1.2; scene.add(key);
const fill = new THREE.DirectionalLight(0xdfeaff, 0.8); fill.position.set(300, 200, 600); scene.add(fill);
const rim = new THREE.DirectionalLight(0xffe2c4, 0.9); rim.position.set(-600, 400, 200); scene.add(rim);
const floor = new THREE.Mesh(new THREE.CircleGeometry(900, 72), new THREE.ShadowMaterial({opacity: 0.18}));
floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
const ring = new THREE.Mesh(new THREE.RingGeometry(150, 152, 96), new THREE.MeshBasicMaterial({color: 0xcfc6b3}));
ring.rotation.x = -Math.PI / 2; ring.position.y = 0.3; scene.add(ring);
const root = new THREE.Group(); root.rotation.x = -Math.PI / 2; scene.add(root);

const COL = {shell: 0xece4d0, accent: 0x2c9a92, dark: 0x3b4048, servo: 0x1d1f23, compute: 0x3a7bd5, power: 0xe8892c, sensor: 0x30a46c, display: 0x111418};
const ELEC = {xiao: 'compute', camera: 'compute', adapter: 'compute', lcd: 'display', imu: 'sensor', touch: 'sensor', buzzer: 'sensor',
  cell_l: 'power', cell_r: 'power', bms: 'power', buck: 'power', switch: 'power', harness: 'power'};
const shellMats = [];
function mat(c, o = {}) { return new THREE.MeshStandardMaterial(Object.assign({color: c, roughness: 0.55, metalness: 0.02}, o)); }
const links = {}, labels = []; const JOINTS = {};
const SHORT = {cell_l: 'LiFePO4 cells', bms: 'BMS', adapter: 'Servo bus adapter', imu: 'IMU', switch: 'Switch + charge jack', xiao: 'XIAO ESP32-S3', camera: 'Camera eye', lcd: 'Face LCD', buck: '5 V regulator', buzzer: 'Buzzer', touch: 'Touch pad'};
let xray = false, showLabels = Q.get('labels') !== '0', play = null;

info.textContent = 'Loading the CAD model...';
Promise.all([
  fetch('assets/robot_info.json?v=' + BUILD).then(x => x.json()),
  new Promise((ok, bad) => new GLTFLoader().load('assets/bleep.gltf?v=' + BUILD, ok, undefined, bad)),
  fetch('assets/walk_replay.json?v=' + BUILD).then(x => x.ok ? x.json() : null).catch(() => null)
]).then(([ri, gltf, replay]) => {
  const g = gltf.scene; root.add(g);
  g.traverse(o => {
    // GLTFLoader strips ':' from node names, so 'link:base' arrives as 'linkbase'
    const nm = o.name.replace(/^(link|part|servo|elec)(?!:)/, '$1:');
    if (nm.startsWith('link:') && !o.isMesh) links[nm.slice(5)] = o;
    if (!o.isMesh) return;
    const n = nm.includes(':') ? nm : (o.parent.name || '').replace(/^(link|part|servo|elec)(?!:)/, '$1:');
    let m;
    const pn = n.replace(/^part:/, ''), pi = ri.parts[pn];
    if (n.startsWith('part:') && pi) {
      m = mat(COL[pi.color] || COL.shell, {roughness: pi.color === 'dark' ? 0.7 : 0.48});
      if (pi.color !== 'dark') shellMats.push(m);
    } else if (n.startsWith('servo:')) m = mat(COL.servo, {roughness: 0.4, metalness: 0.15});
    else if (n.startsWith('elec:')) { const k = n.slice(5); m = mat(COL[ELEC[k]] || COL.compute, {roughness: 0.45, emissive: k === 'lcd' ? 0x0b3a52 : 0x000000}); }
    else m = mat(COL.shell);
    o.geometry = toCreasedNormals(o.geometry.index ? o.geometry.toNonIndexed() : o.geometry, THREE.MathUtils.degToRad(32));
    o.material = m; o.castShadow = true; o.receiveShadow = true;
    if (n.startsWith('part:')) { const e = new THREE.LineSegments(new THREE.EdgesGeometry(o.geometry, 40), new THREE.LineBasicMaterial({color: 0x000000, transparent: true, opacity: 0.12})); o.add(e); }
  });
  for (const [n, jj] of Object.entries(ri.joints)) JOINTS[n] = jj.axis;
  for (const l of Object.values(links)) l.userData.q0 = l.quaternion.clone(), l.userData.p0 = l.position.clone();
  const layer = document.getElementById('labels');
  for (const L of ri.labels) {
    const node = links[L.node.slice(5)]; if (!node) continue;
    const a = new THREE.Object3D(); a.position.set(...L.local); node.add(a);
    const d = document.createElement('button'); d.className = 'lbl ' + L.group;
    const k = L.key.split(':')[1]; d.innerHTML = `<i></i><span>${SHORT[k] || L.title.split(':')[0]}</span>`;
    d.onclick = () => { info.innerHTML = `<b>${L.title}</b><br>${L.spec}<br><em>${L.kind === 'real' ? 'Real vendor CAD' : L.kind === 'modeled' ? 'Modelled from datasheet' : 'Size envelope (not detailed CAD)'}</em>`; };
    layer.appendChild(d); labels.push({a, d, inner: L.group === 'electronics' && !['lcd', 'camera'].includes(k)});
  }
  info.innerHTML = 'Drag to orbit, scroll to zoom. Click a label for its spec. X-ray shows the electronics inside.';
  if (replay) setupReplay(ri, replay); else document.getElementById('play').disabled = true;
  if (Q.get('pose') === 'home') setPose({l_hip_pitch: -0.42, l_knee: 0.84, l_ankle: -0.42, r_hip_pitch: -0.42, r_knee: 0.84, r_ankle: -0.42});
  applyXray(); document.body.classList.add('loaded');
}).catch(e => { info.textContent = 'Could not load the model: ' + e; });

function setPose(j, base) {
  const ax = new THREE.Vector3();
  for (const [n, v] of Object.entries(j)) {
    const l = links[n === 'neck_yaw' ? 'head' : n]; if (!l) continue;
    const a = JOINTS[n]; if (!a) continue; ax.set(...a);
    l.quaternion.copy(l.userData.q0).multiply(new THREE.Quaternion().setFromAxisAngle(ax, v));
  }
  const b = links.base;
  if (base) { b.position.set(base.p[0] * 1000, base.p[1] * 1000, base.p[2] * 1000); b.quaternion.set(base.q[1], base.q[2], base.q[3], base.q[0]); }
}
function setupReplay(ri, rp) {
  const btn = document.getElementById('play'), sel = document.getElementById('clip');
  for (const c of Object.keys(rp.clips)) { const o = document.createElement('option'); o.textContent = c; sel.appendChild(o); }
  btn.onclick = () => {
    if (play) { play = null; btn.textContent = '\u25B6 Play sim walk'; resetPose(); return; }
    play = {frames: rp.clips[sel.value], fps: rp.fps, t0: performance.now()}; btn.textContent = '\u25A0 Stop';
  };
  sel.onchange = () => { if (play) play = {frames: rp.clips[sel.value], fps: rp.fps, t0: performance.now()}; };
}
function resetPose() { for (const l of Object.values(links)) { l.quaternion.copy(l.userData.q0); l.position.copy(l.userData.p0); } }
function stepReplay() {
  if (!play) return;
  const f = play.frames; const i = Math.floor((performance.now() - play.t0) / 1000 * play.fps) % f.length;
  const fr = f[i], f0 = f[0];
  // follow the robot: keep the replay centred on the start point but show drift within the clip
  setPose(fr.j, {p: [fr.p[0] - f0.p[0], fr.p[1] - f0.p[1], fr.p[2]], q: fr.q});
}

document.querySelectorAll('[data-view]').forEach(b => b.onclick = () => setView(b.dataset.view));
function applyXray() { document.getElementById('xray').classList.toggle('on', xray);
  for (const m of shellMats) { m.transparent = xray; m.opacity = xray ? 0.18 : 1; m.depthWrite = !xray; m.needsUpdate = true; } }
document.getElementById('xray').onclick = () => { xray = !xray; applyXray(); };
const lb = document.getElementById('lbls'); lb.classList.toggle('on', showLabels);
lb.onclick = () => { showLabels = !showLabels; lb.classList.toggle('on', showLabels); };
if (Q.get('xray') === '1') xray = true;
setView(Q.get('view') || '34');

const v = new THREE.Vector3();
function placeLabels() {
  const w = el.clientWidth, h = el.clientHeight;
  for (const L of labels) {
    if (!showLabels || (L.inner && !xray)) { L.d.style.display = 'none'; continue; }
    L.a.getWorldPosition(v); v.project(cam);
    if (v.z > 1) { L.d.style.display = 'none'; continue; }
    L.d.style.display = ''; L.d.style.transform = `translate(${(v.x * .5 + .5) * w}px,${(-v.y * .5 + .5) * h}px)`;
  }
}
new ResizeObserver(() => { cam.aspect = el.clientWidth / el.clientHeight; cam.updateProjectionMatrix(); r.setSize(el.clientWidth, el.clientHeight); }).observe(el);
(function loop() { requestAnimationFrame(loop); stepReplay(); ctl.update(); placeLabels(); r.render(scene, cam); })();
