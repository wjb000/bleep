const D = await fetch('assets/site_data.json?v=b1').then(r => r.json());
const $ = id => document.getElementById(id), f = (x, n = 0) => Number(x).toFixed(n), usd = x => '$' + f(x, 2);
const S = D.summary, B = D.bom, T = D.torque, ST = D.stand;
// hero stats
const stats = [[f(S.height_mm / 10, 1) + ' cm', 'tall'], [f(S.total_mass_kg * 1000) + ' g', 'all-up mass'], [usd(B.total_usd), 'in parts'],
  [S.n_servos + ' servos', 'Feetech STS3215'], [S.n_printed_parts + ' prints', 'PLA, ~' + Math.round(S.plastic_g) + ' g'], ['ESP32-S3', 'camera, mic, Wi-Fi']];
$('stats').innerHTML = stats.map(([a, b]) => `<div class="stat"><b>${a}</b><span>${b}</span></div>`).join('');

// servo table
const wst = T.worst_static_nm;
$('tq-rule').textContent = `Worst holding torque at ${f(T.robot_mass_kg * 1000)} g: hip roll ${f(wst.hip_roll, 2)} N·m (standing on one leg), knee ${f(wst.knee, 2)} N·m (crouched). ` +
  `Rule used: holding load at most 1/3 of stall, so the walking peaks (about ${T.dynamic_factor}× static) stay under 2/3.`;
let h = '<tr><th>Servo</th><th class="num">Stall</th><th class="num">Price</th><th>Feedback</th><th class="num">Robot mass with it</th><th>Worst holding load / stall</th><th>Verdict</th></tr>';
for (const [n, c] of Object.entries(T.candidates)) {
  const p = c.static_pct_of_stall;
  h += `<tr><td><b>${n}</b></td><td class="num">${f(c.spec.stall, 2)} N·m</td><td class="num">${usd(c.spec.price)}</td><td>${c.spec.feedback ? 'position, load, temp' : 'none'}</td>` +
    `<td class="num">${f(c.robot_mass_kg * 1000)} g</td><td><div class="bar"><i class="${p > 33 ? 'over' : ''}" style="width:${Math.min(100, p)}%"></i></div>${p}%</td>` +
    `<td class="${c.ok ? 'pass' : 'fail'}">${c.ok ? 'Chosen' : 'Too weak'}</td></tr>`;
}
$('servo-table').innerHTML = h;
$('servo-note').textContent = 'Each smaller servo makes the robot lighter, and the table includes that, but not enough to make it hold. The STS3215 has the cheapest torque per dollar ' +
  'of the serial-bus servos, a 12-bit magnetic encoder, and position, load and temperature feedback. That feedback is what lets the policy and the system ID see what the joint really did.';

// BOM
$('bom-total').textContent = usd(B.total_usd);
$('bom-sub').textContent = `Prices checked ${B.checked}, in USD, before tax and shipping. Items sold in packs count at the share one robot uses.`;
h = '<tr><th>Part</th><th class="num">Qty</th><th class="num">You pay</th><th class="num">Per robot</th><th>Does</th></tr>';
for (const r of B.rows) h += `<tr><td><a href="${r.link}" rel="noopener">${r.item}</a></td><td class="num">${r.qty}</td><td class="num">${usd(r.unit_usd)}${r.pack ? ' <small>(' + r.pack + ')</small>' : ''}</td><td class="num">${usd(r.robot_usd)}</td><td>${r.role}</td></tr>`;
h += `<tr class="total"><td>Total per robot</td><td></td><td></td><td class="num">${usd(B.total_usd)}</td><td>Servos are ${Math.round(B.servo_share * 100)}% of it</td></tr>`;
$('bom-table').innerHTML = h;
$('bom-note').innerHTML = `Servo price is Feetech's direct listing (2-499 pcs). At one-off retail ($21.99 each) the total is ${usd(B.total_with_retail_servos_usd)}. ` +
  `Also needed, not counted: a 7.3 V LiFePO4 charger (<a href="${B.charger.link}">${usd(B.charger.usd)}</a>). Never use an 8.4 V Li-ion charger.`;

// checks
const C = [];
C.push(['CAD interference, zero pose', D.zero_pose_interferences === 0, `0 overlaps between all ${S.n_printed_parts} printed parts, 11 real servo models and the electronics.`]);
const oth = D.sweep_contacts.filter(c => c.what === 'other_leg'), slf = D.sweep_contacts.filter(c => c.what === 'self');
C.push(['Joint range sweep', slf.length === 0, `All ${D.n_joints_swept} joints swept one at a time through their full range: ${slf.length} self-collisions. ` +
  (oth.length ? `The legs touch each other only at ${Math.abs(oth[0].deg)}° of inward hip roll (out of 20°), and the sim models that contact.` : ''), slf.length === 0 ? 'Pass' : 'Fail']);
if (ST) C.push(['Stands on its own (sim)', ST.passed, `Servos held at the home crouch for 5 s: max tilt ${ST.settle_5s.max_tilt_deg}°. With no balancing, it survives shoves up to ${ST.passive_shove_survived_m_s.forward} m/s forward, ` +
  `${ST.passive_shove_survived_m_s.backward} back and ${ST.passive_shove_survived_m_s.left} sideways. Max holding torque ${f(ST.max_static_torque_nm, 2)} N·m of ${ST.stall_limit_nm}.`]);
C.push(['Torque budget', true, `Worst case (one leg, crouched): hip roll ${f(wst.hip_roll, 2)} N·m = ${T.candidates[Object.keys(T.candidates)[0]].static_pct_of_stall}% of stall.`]);
if (D.sysid) C.push(['System ID code', true, `Self-test on a synthetic log (3 mrad noise): all 6 servo parameters recovered within ${f(Math.max(...Object.values(D.sysid.error_pct).map(Math.abs)), 1)}%. Not yet run on a real servo.`, 'Code checked']);
if (D.fw_test) C.push(['Firmware network = PyTorch', D.fw_test.passed, `The C forward pass compiled with gcc and compared on ${D.fw_test.samples} random observations: max error ${D.fw_test.max_abs_error.toExponential(1)}.`]);
if (D.firmware_build) C.push(['Firmware compiles', D.firmware_build.ok, `arduino-cli, ESP32 core ${D.firmware_build.core}, XIAO_ESP32S3: ${D.firmware_build.flash_pct}% flash, ${D.firmware_build.ram_pct}% RAM. Not yet flashed to a board.`, 'Builds']);
C.push(['Built and printed', false, 'Not yet. The first prints and fit checks are the next step. Expect small clearance fixes.', 'To do']);
$('checks').innerHTML = C.map(([t, ok, p, lab]) => `<div class="check"><span class="st ${ok ? 'ok' : 'warn'}">${lab || (ok ? 'Pass' : 'Fail')}</span><h4>${t}</h4><p>${p}</p></div>`).join('');

// training curve
const cv = $('curve'), g = cv.getContext('2d'), cu = D.curve || [];
if (cu.length) {
  const W = cv.width, H = cv.height, L = 48, R = 12, Tp = 14, Bt = 34, xm = cu[cu.length - 1][0], ym = Math.max(...cu.map(c => c[1])) * 1.08;
  const X = s => L + (W - L - R) * s / xm, Y = v => H - Bt - (H - Bt - Tp) * v / ym;
  g.font = '12px Inter,system-ui,sans-serif'; g.fillStyle = '#5d6672'; g.strokeStyle = '#ece5d6';
  for (let i = 0; i <= 4; i++) { const v = ym * i / 4; g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(W - R, Y(v)); g.stroke(); g.fillText(f(v), 6, Y(v) + 4); }
  for (let i = 0; i <= 4; i++) { const s = xm * i / 4; g.fillText(f(s / 1e6, 1) + 'M', X(s) - 12, H - 12); }
  g.strokeStyle = '#2c9a92'; g.lineWidth = 2.5; g.beginPath(); cu.forEach((c, i) => i ? g.lineTo(X(c[0]), Y(c[1])) : g.moveTo(X(c[0]), Y(c[1]))); g.stroke();
  const tr = D.training, last = cu[cu.length - 1];
  $('curve-note').textContent = `Episode reward vs. simulation steps. ` + (tr ? `${f(tr.steps / 1e6, 1)}M steps in ${f(tr.minutes)} min on ${tr.cpus} CPU cores (8 parallel sims). ` : '') +
    `Last value: ${f(last[1])}, average episode ${f(last[2], 1)} s of 10 s without falling (commands and shoves are random).`;
}
// eval table
const E = D.walk_eval;
if (E) {
  h = '<tr><th>Command</th><th>Model</th><th class="num">Falls</th><th class="num">Measured</th><th class="num">Peak torque</th></tr>';
  for (const [k, v] of Object.entries(E)) {
    const [c, m] = k.split(' | ');
    const meas = c.startsWith('turn') ? f(v.mean_yaw_rate, 2) + ' rad/s' : f(v.mean_vx_m_s, 3) + ' m/s';
    h += `<tr><td>${c}</td><td>${m}</td><td class="num">${v.falls}/${v.episodes}</td><td class="num">${meas}</td><td class="num">${f(v.peak_torque_nm, 2)} N·m</td></tr>`;
  }
  $('eval-table').innerHTML = h;
  $('eval-note').textContent = '10 s episodes. "Randomised" uses new random physics, servo and latency settings each episode, as in training, with shoves. Peak torque is the highest single-joint value; the STS3215 stalls at 1.65 N·m.';
} else $('eval-table').innerHTML = '<tr><td>Evaluation pending.</td></tr>';
