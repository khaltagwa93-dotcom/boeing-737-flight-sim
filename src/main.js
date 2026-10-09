/**
 * Boeing 737-800 Realistic Passenger Flight Simulator
 * Professional 6DOF Flight Dynamics Model + Three.js PBR
 * Improvements: Fixed body axes, ground start, better stall/takeoff, polished HUD & model
 */

import * as THREE from 'three';

// ==================== AIRCRAFT CONFIG (Boeing 737-800 approx) ====================
const CFG = {
  mass: 70000,          // kg (operating empty + fuel)
  wingArea: 125,        // m²
  wingSpan: 35.8,       // m
  mac: 3.9,             // mean aerodynamic chord m
  // Aero (per radian where applicable)
  CL0: 0.22,
  CLa: 5.7,
  CLmax: 1.55,
  CD0: 0.022,
  K: 0.042,             // induced drag factor
  Cm0: -0.04,
  Cma: -1.5,
  Cl_beta: -0.12,
  Cn_beta: 0.08,
  Clp: -0.45,
  Cnr: -0.18,
  // Control power
  elevPower: 0.9,
  ailPower: 0.55,
  rudPower: 0.45,
  // Engines (2 × CFM56-7B)
  maxThrust: 2 * 121400, // N total
  // Inertia (approx kg·m²)
  Ix: 1.15e6,
  Iy: 3.8e6,
  Iz: 4.5e6,
};

// ==================== STATE ====================
const S = {
  pos: new THREE.Vector3(0, 3.2, 0),   // start on runway (gear height)
  vel: new THREE.Vector3(0, 0, 0),     // world m/s
  quat: new THREE.Quaternion(),       // orientation
  p: 0, q: 0, r: 0,                   // body rates rad/s (roll, pitch, yaw)
  elev: 0, ail: 0, rud: 0,            // controls -1..1
  throttle: 0,                        // 0..1
  enginesOn: false,
  // derived
  airspeedKt: 0,
  alpha: 0,
  beta: 0,
  altFt: 10,
  hdg: 0,
  pitchDeg: 0,
  bankDeg: 0,
  vsFpm: 0,
  stalled: false,
  onGround: true,
  cam: 'chase',                       // 'chase' | 'cockpit'
};

// ==================== INPUT ====================
const keys = Object.create(null);
window.addEventListener('keydown', e => {
  keys[e.code] = true;
  if (['Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.code)) e.preventDefault();
});
window.addEventListener('keyup', e => { keys[e.code] = false; });

function updateInput(dt) {
  // Elevator (nose up/down)
  if (keys.KeyW || keys.ArrowUp)   S.elev = Math.min(1,  S.elev + 2.2 * dt);
  else if (keys.KeyS || keys.ArrowDown) S.elev = Math.max(-1, S.elev - 2.2 * dt);
  else S.elev *= 0.88;

  // Aileron
  if (keys.KeyA || keys.ArrowLeft)  S.ail = Math.min(1,  S.ail + 2.5 * dt);
  else if (keys.KeyD || keys.ArrowRight) S.ail = Math.max(-1, S.ail - 2.5 * dt);
  else S.ail *= 0.85;

  // Rudder
  if (keys.KeyQ) S.rud = Math.min(1,  S.rud + 2.0 * dt);
  else if (keys.KeyE) S.rud = Math.max(-1, S.rud - 2.0 * dt);
  else S.rud *= 0.88;

  // Throttle
  if (keys.ShiftLeft || keys.ShiftRight) S.throttle = Math.min(1, S.throttle + 0.55 * dt);
  if (keys.ControlLeft || keys.ControlRight) S.throttle = Math.max(0, S.throttle - 0.55 * dt);

  // Engine start/stop
  if (keys.Space) {
    keys.Space = false;
    S.enginesOn = !S.enginesOn;
    if (!S.enginesOn) S.throttle = 0;
  }

  // Camera toggle
  if (keys.KeyC) {
    keys.KeyC = false;
    S.cam = S.cam === 'chase' ? 'cockpit' : 'chase';
  }

  // Reset
  if (keys.KeyR) {
    keys.KeyR = false;
    reset();
  }
}

function reset() {
  S.pos.set(0, 3.2, 0);
  S.vel.set(0, 0, 0);
  S.quat.identity();
  S.p = S.q = S.r = 0;
  S.elev = S.ail = S.rud = 0;
  S.throttle = 0;
  S.enginesOn = false;
  S.onGround = true;
  S.stalled = false;
}

// ==================== ISA DENSITY ====================
function density(altM) {
  const T = Math.max(216.65, 288.15 - 0.0065 * altM);
  return 1.225 * Math.pow(T / 288.15, 4.256);
}

// ==================== 6DOF FLIGHT DYNAMICS ====================
function stepPhysics(dt) {
  const altM = S.pos.y;
  const rho = density(altM);
  const V = S.vel.length();
  S.airspeedKt = V * 1.943844;

  // Body velocity (X=forward, Y=up, Z=right) – quaternion maps body→world
  const qInv = S.quat.clone().invert();
  const bodyV = S.vel.clone().applyQuaternion(qInv);
  const u = bodyV.x;
  const v = bodyV.y;
  const w = bodyV.z;

  // AoA & sideslip
  S.alpha = Math.atan2(-v, Math.max(0.1, u));
  S.beta  = Math.asin(THREE.MathUtils.clamp(w / Math.max(0.1, V), -1, 1));

  const qbar = 0.5 * rho * V * V;

  // Lift & Drag coefficients
  let CL = CFG.CL0 + CFG.CLa * S.alpha + CFG.elevPower * S.elev * 0.35;
  CL = THREE.MathUtils.clamp(CL, -0.9, CFG.CLmax);

  // Stall
  const stallAlpha = 0.26; // ~15°
  S.stalled = Math.abs(S.alpha) > stallAlpha || (V < 48 && altM > 15);
  if (S.stalled) CL *= 0.4;

  const CD = CFG.CD0 + CFG.K * CL * CL
           + 0.015 * Math.abs(S.elev)
           + 0.012 * Math.abs(S.ail)
           + 0.01  * Math.abs(S.rud);

  const L = qbar * CFG.wingArea * CL;
  const D = qbar * CFG.wingArea * CD;
  const Y = -qbar * CFG.wingArea * 1.1 * S.beta; // side force

  // Thrust along body X
  const thrust = S.enginesOn ? S.throttle * CFG.maxThrust * (rho / 1.225) : 0;

  // Body forces (X forward, Y up, Z right)
  const Fx = thrust - D * Math.cos(S.alpha) + L * Math.sin(S.alpha);
  const Fy = L * Math.cos(S.alpha) + D * Math.sin(S.alpha);
  const Fz = Y;

  // Transform body force → world
  const Fbody = new THREE.Vector3(Fx, Fy, Fz);
  const Fworld = Fbody.applyQuaternion(S.quat);
  Fworld.y -= 9.80665 * CFG.mass; // gravity

  // Moments (simplified stability derivatives)
  const Cm = CFG.Cm0 + CFG.Cma * S.alpha + CFG.elevPower * S.elev;
  const My = qbar * CFG.wingArea * CFG.mac * Cm;

  const Cl = -CFG.ailPower * S.ail
           + CFG.Cl_beta * S.beta
           + CFG.Clp * S.p * (CFG.wingSpan / (2 * Math.max(1, V)));
  const Mx = qbar * CFG.wingArea * CFG.wingSpan * Cl;

  const Cn = -CFG.rudPower * S.rud
           + CFG.Cn_beta * S.beta
           + CFG.Cnr * S.r * (CFG.wingSpan / (2 * Math.max(1, V)));
  const Mz = qbar * CFG.wingArea * CFG.wingSpan * Cn;

  // Angular acceleration
  S.p += (Mx / CFG.Ix) * dt;
  S.q += (My / CFG.Iy) * dt;
  S.r += (Mz / CFG.Iz) * dt;

  // Rate damping
  S.p *= 0.975;
  S.q *= 0.98;
  S.r *= 0.975;

  // Linear integration
  const acc = Fworld.divideScalar(CFG.mass);
  S.vel.addScaledVector(acc, dt);

  // Ground interaction (simple but stable)
  const gearH = 3.0;
  if (S.pos.y < gearH) {
    S.pos.y = gearH;
    if (S.vel.y < 0) S.vel.y *= -0.15; // soft bounce
    // Rolling resistance & wheel friction
    const spd = S.vel.length();
    if (spd < 35) {
      S.vel.x *= 0.992;
      S.vel.z *= 0.992;
      S.p *= 0.4;
      S.q *= 0.55;
      // keep wings level-ish on ground
      const e = new THREE.Euler().setFromQuaternion(S.quat, 'YXZ');
      e.z *= 0.85;
      e.x = THREE.MathUtils.clamp(e.x, -0.08, 0.12);
      S.quat.setFromEuler(e);
    }
    S.onGround = spd < 40;
  } else {
    S.onGround = false;
  }

  // Position
  S.pos.addScaledVector(S.vel, dt);

  // Orientation integration (body rates → quaternion)
  const halfDt = 0.5 * dt;
  const dq = new THREE.Quaternion(
    S.p * halfDt,
    S.q * halfDt,
    S.r * halfDt,
    0
  ).multiply(S.quat);
  S.quat.x += dq.x;
  S.quat.y += dq.y;
  S.quat.z += dq.z;
  S.quat.w += dq.w;
  S.quat.normalize();

  // Derived flight instruments
  const euler = new THREE.Euler().setFromQuaternion(S.quat, 'YXZ');
  S.pitchDeg = THREE.MathUtils.radToDeg(euler.x);
  S.bankDeg  = THREE.MathUtils.radToDeg(euler.z);
  S.hdg      = (THREE.MathUtils.radToDeg(euler.y) + 360) % 360;
  S.altFt    = S.pos.y * 3.28084;
  S.vsFpm    = S.vel.y * 196.85;
}

// ==================== SCENE ====================
let scene, camera, renderer, aircraft, clock;
let fanL, fanR;

function createAircraft() {
  const g = new THREE.Group();

  const white = new THREE.MeshStandardMaterial({ color: 0xf2f2f2, metalness: 0.55, roughness: 0.28 });
  const wingM = new THREE.MeshStandardMaterial({ color: 0xe6e6e6, metalness: 0.5, roughness: 0.35 });
  const dark  = new THREE.MeshStandardMaterial({ color: 0x222222, metalness: 0.8, roughness: 0.4 });
  const blue  = new THREE.MeshStandardMaterial({ color: 0x003087, metalness: 0.4, roughness: 0.45 });
  const glass = new THREE.MeshStandardMaterial({ color: 0x0a0a18, metalness: 0.95, roughness: 0.05 });

  // Fuselage (X-forward)
  const fus = new THREE.Mesh(new THREE.CylinderGeometry(1.85, 1.65, 30, 20), white);
  fus.rotation.z = Math.PI / 2;
  g.add(fus);

  // Nose cone
  const nose = new THREE.Mesh(new THREE.ConeGeometry(1.65, 5.5, 16), white);
  nose.rotation.z = -Math.PI / 2;
  nose.position.x = 17.75;
  g.add(nose);

  // Cockpit
  const cock = new THREE.Mesh(new THREE.BoxGeometry(3.2, 1.4, 2.4), glass);
  cock.position.set(13.2, 0.9, 0);
  g.add(cock);

  // Wings
  const wing = new THREE.Mesh(new THREE.BoxGeometry(4.2, 0.28, 19.5), wingM);
  wing.position.set(-0.5, -0.25, 0);
  g.add(wing);

  // Winglets
  const wlGeo = new THREE.BoxGeometry(1.8, 0.18, 2.4);
  const wlL = new THREE.Mesh(wlGeo, wingM);
  wlL.position.set(-0.5, 0.55, 10.2);
  wlL.rotation.x = 0.35;
  g.add(wlL);
  const wlR = wlL.clone();
  wlR.position.z = -10.2;
  wlR.rotation.x = -0.35;
  g.add(wlR);

  // Engines
  const engGeo = new THREE.CylinderGeometry(1.15, 1.05, 5.2, 14);
  const engL = new THREE.Mesh(engGeo, dark);
  engL.rotation.z = Math.PI / 2;
  engL.position.set(1.5, -1.9, 5.8);
  g.add(engL);
  const engR = engL.clone();
  engR.position.z = -5.8;
  g.add(engR);

  // Fan disks (will spin)
  const fanGeo = new THREE.CylinderGeometry(1.0, 1.0, 0.35, 18);
  fanL = new THREE.Mesh(fanGeo, new THREE.MeshStandardMaterial({ color: 0x555555, metalness: 0.95 }));
  fanL.rotation.z = Math.PI / 2;
  fanL.position.set(4.2, -1.9, 5.8);
  g.add(fanL);
  fanR = fanL.clone();
  fanR.position.z = -5.8;
  g.add(fanR);

  // Horizontal stabilizer
  const hstab = new THREE.Mesh(new THREE.BoxGeometry(2.8, 0.18, 9.5), wingM);
  hstab.position.set(-14.2, 0.6, 0);
  g.add(hstab);

  // Vertical stabilizer
  const vstab = new THREE.Mesh(new THREE.BoxGeometry(4.2, 5.2, 0.28), wingM);
  vstab.position.set(-14.8, 3.1, 0);
  g.add(vstab);

  // Blue livery stripes
  const stripe = new THREE.Mesh(new THREE.BoxGeometry(28, 0.18, 0.1), blue);
  stripe.position.set(0, 0.35, 1.85);
  g.add(stripe);
  const stripe2 = stripe.clone();
  stripe2.position.z = -1.85;
  g.add(stripe2);

  // Landing gear
  const gearGeo = new THREE.CylinderGeometry(0.18, 0.18, 2.4);
  const gearC = new THREE.Mesh(gearGeo, dark);
  gearC.position.set(9, -2.4, 0);
  g.add(gearC);
  const gearL = gearC.clone();
  gearL.position.set(-1.5, -2.7, 3.2);
  g.add(gearL);
  const gearR = gearC.clone();
  gearR.position.set(-1.5, -2.7, -3.2);
  g.add(gearR);

  // Wheel disks
  const wheelGeo = new THREE.CylinderGeometry(0.55, 0.55, 0.35, 12);
  const wheelM = new THREE.MeshStandardMaterial({ color: 0x111111 });
  [[9, -3.5, 0], [-1.5, -3.8, 3.2], [-1.5, -3.8, -3.2]].forEach(([x, y, z]) => {
    const w = new THREE.Mesh(wheelGeo, wheelM);
    w.rotation.z = Math.PI / 2;
    w.position.set(x, y, z);
    g.add(w);
  });

  g.scale.setScalar(1.15);
  return g;
}

function createWorld() {
  // Ground
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(12000, 12000),
    new THREE.MeshStandardMaterial({ color: 0x2a5a28, roughness: 0.92 })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  // Runway
  const rwy = new THREE.Mesh(
    new THREE.PlaneGeometry(55, 3200),
    new THREE.MeshStandardMaterial({ color: 0x2c2c2c, roughness: 0.8 })
  );
  rwy.rotation.x = -Math.PI / 2;
  rwy.position.y = 0.08;
  scene.add(rwy);

  // Centerline dashes
  for (let i = -30; i <= 30; i++) {
    const dash = new THREE.Mesh(
      new THREE.PlaneGeometry(1.2, 28),
      new THREE.MeshBasicMaterial({ color: 0xffffff })
    );
    dash.rotation.x = -Math.PI / 2;
    dash.position.set(0, 0.1, i * 50);
    scene.add(dash);
  }

  // Threshold bars
  for (let i = 0; i < 10; i++) {
    const bar = new THREE.Mesh(
      new THREE.PlaneGeometry(3.2, 35),
      new THREE.MeshBasicMaterial({ color: 0xffffff })
    );
    bar.rotation.x = -Math.PI / 2;
    bar.position.set(-22 + i * 4.8, 0.11, 1550);
    scene.add(bar);
  }

  // Taxiway
  const taxi = new THREE.Mesh(
    new THREE.PlaneGeometry(28, 500),
    new THREE.MeshStandardMaterial({ color: 0x333333, roughness: 0.82 })
  );
  taxi.rotation.x = -Math.PI / 2;
  taxi.position.set(90, 0.06, 1200);
  scene.add(taxi);

  // Terminal
  const term = new THREE.Mesh(
    new THREE.BoxGeometry(90, 28, 45),
    new THREE.MeshStandardMaterial({ color: 0x7a8a9a, metalness: 0.25, roughness: 0.55 })
  );
  term.position.set(170, 14, 1100);
  term.castShadow = true;
  scene.add(term);

  // Tower
  const tower = new THREE.Mesh(
    new THREE.BoxGeometry(14, 52, 14),
    new THREE.MeshStandardMaterial({ color: 0x4a4a4a })
  );
  tower.position.set(130, 26, 980);
  tower.castShadow = true;
  scene.add(tower);

  // Simple trees (instanced feel)
  const treeGeo = new THREE.ConeGeometry(4, 12, 6);
  const treeMat = new THREE.MeshStandardMaterial({ color: 0x1a4a1a });
  for (let i = 0; i < 40; i++) {
    const t = new THREE.Mesh(treeGeo, treeMat);
    const ang = Math.random() * Math.PI * 2;
    const dist = 200 + Math.random() * 600;
    t.position.set(Math.cos(ang) * dist, 6, Math.sin(ang) * dist + 400);
    scene.add(t);
  }
}

function init() {
  const container = document.getElementById('canvas-container');
  scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x87b8d8, 0.00022);

  camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, 0.8, 15000);
  camera.position.set(-45, 18, 0);

  renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setSize(innerWidth, innerHeight);
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  container.appendChild(renderer.domElement);

  // Lighting
  const sun = new THREE.DirectionalLight(0xfff4e0, 2.4);
  sun.position.set(400, 700, 250);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.near = 50;
  sun.shadow.camera.far = 2500;
  sun.shadow.camera.left = sun.shadow.camera.bottom = -500;
  sun.shadow.camera.right = sun.shadow.camera.top = 500;
  scene.add(sun);
  scene.add(new THREE.AmbientLight(0x6a8aaa, 0.4));
  scene.add(new THREE.HemisphereLight(0x87ceeb, 0x3a5f2a, 0.4));

  // Sky sphere
  const sky = new THREE.Mesh(
    new THREE.SphereGeometry(9000, 24, 16),
    new THREE.MeshBasicMaterial({ color: 0x87ceeb, side: THREE.BackSide })
  );
  scene.add(sky);

  createWorld();
  aircraft = createAircraft();
  aircraft.position.copy(S.pos);
  aircraft.castShadow = true;
  scene.add(aircraft);

  clock = new THREE.Clock();

  window.addEventListener('resize', () => {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  });

  document.getElementById('loading').style.display = 'none';
  animate();
}

function updateCamera() {
  if (S.cam === 'cockpit') {
    const offset = new THREE.Vector3(12.5, 1.4, 0).applyQuaternion(S.quat);
    camera.position.copy(S.pos).add(offset);
    const look = new THREE.Vector3(50, 0, 0).applyQuaternion(S.quat).add(S.pos);
    camera.lookAt(look);
  } else {
    const offset = new THREE.Vector3(-38, 14, 0).applyQuaternion(S.quat);
    camera.position.lerp(S.pos.clone().add(offset), 0.09);
    camera.lookAt(S.pos);
  }
}

function updateHUD() {
  document.getElementById('spd').textContent = Math.round(S.airspeedKt);
  document.getElementById('alt').textContent = Math.round(S.altFt);
  document.getElementById('hdg').textContent = Math.round(S.hdg).toString().padStart(3, '0');
  document.getElementById('vs').textContent = Math.round(S.vsFpm);
  document.getElementById('thr').textContent = Math.round(S.throttle * 100);
  document.getElementById('pitch').textContent = S.pitchDeg.toFixed(1);
  document.getElementById('bank').textContent = S.bankDeg.toFixed(1);

  const st = document.getElementById('status');
  if (!S.enginesOn) {
    st.textContent = 'المحركات متوقفة — اضغط SPACE للتشغيل  |  Shift لزيادة الدفع';
  } else if (S.onGround && S.airspeedKt < 20) {
    st.textContent = 'جاهز للإقلاع — زد الدفع (Shift) وارفع الأنف (W) عند ~145 kt';
  } else if (S.stalled) {
    st.textContent = '⚠ STALL — ادفع الأنف للأسفل (S) وزد السرعة';
  } else if (S.onGround) {
    st.textContent = `على المدرج — السرعة ${Math.round(S.airspeedKt)} kt`;
  } else {
    st.textContent = `طيران — ${Math.round(S.airspeedKt)} kt  |  ${Math.round(S.altFt)} ft`;
  }

  document.getElementById('stall-warning').style.display = S.stalled ? 'block' : 'none';
}

function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), 0.033);

  updateInput(dt);
  stepPhysics(dt);

  aircraft.position.copy(S.pos);
  aircraft.quaternion.copy(S.quat);

  // Spin fans
  if (S.enginesOn && S.throttle > 0.03) {
    const spin = S.throttle * 1.4;
    fanL.rotation.x += spin;
    fanR.rotation.x += spin;
  }

  updateCamera();
  updateHUD();
  renderer.render(scene, camera);
}

init();
