/* ═══════════════════════════════════════════════════════════════
   app.js  —  LionChief Web UI JavaScript
   Datix AI  |  July 2026
═══════════════════════════════════════════════════════════════ */

// ── SocketIO connection ───────────────────────────────────────────
const socket = io();

// ── Cached DOM references ─────────────────────────────────────────
const cam          = document.getElementById('camera');
const dotA         = document.getElementById('dot-a');
const dotB         = document.getElementById('dot-b');
const zoneBadge    = document.getElementById('zone-badge');
const gapVal       = document.getElementById('gap-val');
const modeVal      = document.getElementById('mode-val');
const confAFill    = document.getElementById('conf-a-fill');
const confBFill    = document.getElementById('conf-b-fill');
const spdANum      = document.getElementById('spd-a-num');
const spdBNum      = document.getElementById('spd-b-num');
const spdABar      = document.getElementById('spd-a-bar');
const spdBBar      = document.getElementById('spd-b-bar');
const confirmOv    = document.getElementById('confirm-overlay');
const pausedOv     = document.getElementById('paused-overlay');
const instrBox     = document.getElementById('instruction-box');
const btnMode      = document.getElementById('btn-mode');

// ══════════════════════════════════════════════════════════════════
//  SEND COMMAND
// ══════════════════════════════════════════════════════════════════
async function sendCmd(cmd, val = null) {
  try {
    await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd, val }),
    });
  } catch (e) {
    console.error('Command failed:', cmd, e);
  }
}

// ══════════════════════════════════════════════════════════════════
//  MOUSE — box drawing on camera image
// ══════════════════════════════════════════════════════════════════
let isDragging = false;

function camCoords(e) {
  const rect   = cam.getBoundingClientRect();
  const scaleX = 960 / cam.clientWidth;
  const scaleY = 540 / cam.clientHeight;
  return {
    x: Math.round((e.clientX - rect.left) * scaleX),
    y: Math.round((e.clientY - rect.top)  * scaleY),
  };
}

async function sendMouse(event, e) {
  const { x, y } = camCoords(e);
  try {
    await fetch('/api/mouse', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ event, x, y }),
    });
  } catch (_) {}
}

cam.addEventListener('mousedown', (e) => {
  e.preventDefault();
  isDragging = true;
  sendMouse('down', e);
});
cam.addEventListener('mousemove', (e) => {
  if (isDragging) sendMouse('move', e);
});
cam.addEventListener('mouseup', (e) => {
  if (isDragging) {
    isDragging = false;
    sendMouse('up', e);
  }
});
cam.addEventListener('mouseleave', (e) => {
  if (isDragging) {
    isDragging = false;
    sendMouse('up', e);
  }
});
// Prevent context menu on right-click over camera
cam.addEventListener('contextmenu', (e) => e.preventDefault());

// Touch support (tablet/phone)
cam.addEventListener('touchstart', (e) => {
  e.preventDefault();
  isDragging = true;
  sendMouse('down', e.touches[0]);
}, { passive: false });
cam.addEventListener('touchmove', (e) => {
  e.preventDefault();
  if (isDragging) sendMouse('move', e.touches[0]);
}, { passive: false });
cam.addEventListener('touchend', (e) => {
  e.preventDefault();
  if (isDragging) {
    isDragging = false;
    sendMouse('up', e.changedTouches[0]);
  }
}, { passive: false });

// ══════════════════════════════════════════════════════════════════
//  SOCKETIO — real-time status updates
// ══════════════════════════════════════════════════════════════════

socket.on('connect', () => {
  console.log('WebSocket connected');
});

socket.on('status', (data) => {
  updateStatus(data);
});

socket.on('confirm_needed', () => {
  confirmOv.classList.remove('hidden');
});

socket.on('auto_paused', () => {
  pausedOv.classList.remove('hidden');
});

socket.on('auto_resumed', () => {
  pausedOv.classList.add('hidden');
});

// ══════════════════════════════════════════════════════════════════
//  UPDATE UI
// ══════════════════════════════════════════════════════════════════

const ZONE_CLASSES = ['zone-DANGER','zone-WARNING','zone-CAUTION',
                      'zone-SAFE','zone-FAR','zone-ESCAPE','zone-unknown'];

function updateStatus(d) {
  // BLE dots
  dotA.className = 'dot ' + (d.ble_a ? 'connected' : 'connecting');
  dotB.className = 'dot ' + (d.ble_b ? 'connected' : 'connecting');

  // Zone badge
  zoneBadge.textContent = d.zone || 'UNKNOWN';
  ZONE_CLASSES.forEach(c => zoneBadge.classList.remove(c));
  zoneBadge.classList.add('zone-' + (d.zone || 'unknown'));

  // Gap
  gapVal.textContent = d.dist ? d.dist + 'px' : '---';

  // Mode
  if (d.waiting) {
    modeVal.textContent = 'SELECT MODE';
    modeVal.style.color = '#ffcc00';
  } else if (d.paused) {
    modeVal.textContent = 'AUTO-PAUSED';
    modeVal.style.color = '#ff4444';
  } else if (d.manual) {
    modeVal.textContent = 'MANUAL';
    modeVal.style.color = '#00ccff';
    btnMode.textContent = 'M  Switch to AUTO';
  } else {
    modeVal.textContent = 'AUTO';
    modeVal.style.color = '#22cc55';
    btnMode.textContent = 'M  Switch to MANUAL';
  }

  // Speeds
  spdANum.textContent = d.spd_a || 0;
  spdBNum.textContent = d.spd_b || 0;
  spdABar.style.width = ((d.spd_a || 0) / 7 * 100) + '%';
  spdBBar.style.width = ((d.spd_b || 0) / 7 * 100) + '%';

  // Confidence bars
  confAFill.style.width = (d.conf_a || 0) + '%';
  confBFill.style.width = (d.conf_b || 0) + '%';
  confAFill.style.background = confColor(d.conf_a);
  confBFill.style.background = confColor(d.conf_b);

  // Auto-pause overlay
  if (d.paused) {
    pausedOv.classList.remove('hidden');
  } else {
    pausedOv.classList.add('hidden');
  }

  // Confirm overlay
  if (d.waiting) {
    confirmOv.classList.remove('hidden');
  } else {
    confirmOv.classList.add('hidden');
  }

  // Speed bar color danger
  if (d.zone === 'DANGER') {
    spdBBar.style.background = 'linear-gradient(90deg, #660000, #ff2200)';
  } else {
    spdBBar.style.background = 'linear-gradient(90deg, #c04000, #ff8030)';
  }
}

function confColor(pct) {
  if (pct > 75) return '#22cc55';
  if (pct > 40) return '#ffaa00';
  return '#ff4422';
}

// ══════════════════════════════════════════════════════════════════
//  KEYBOARD SHORTCUTS (still work in browser)
// ══════════════════════════════════════════════════════════════════
document.addEventListener('keydown', (e) => {
  // Ignore if typing in an input
  if (e.target.tagName === 'INPUT') return;

  const key = e.key;
  if      (key === '[')       sendCmd('a_slower');
  else if (key === ']')       sendCmd('a_faster');
  else if (key === 'F1')      { e.preventDefault(); sendCmd('a_stop'); }
  else if (key === 'F2')      { e.preventDefault(); sendCmd('a_resume'); }
  else if (key === 's' || key === 'S') sendCmd('b_stop');
  else if (key === 'r' || key === 'R') sendCmd('b_resume');
  else if (key === 'm' || key === 'M') sendCmd('toggle_manual');
  else if (key === 'e' || key === 'E') sendCmd('stop_both');
  else if (key === 'a' || key === 'A') sendCmd('reselect_a');
  else if (key === 'b' || key === 'B') sendCmd('reselect_b');
  else if (key === 't' || key === 'T') sendCmd('redraw_table');
  else if (key === 'h' || key === 'H') sendCmd('horn');
  else if (key === 'l' || key === 'L') sendCmd('lights');
  else if (key === '+' || key === '=') sendCmd('b_faster');
  else if (key === '-' || key === '_') sendCmd('b_slower');
  else if (key === 'ArrowUp')          { e.preventDefault(); sendCmd('b_faster'); }
  else if (key === 'ArrowDown')        { e.preventDefault(); sendCmd('b_slower'); }
  else if ('1234567'.includes(key))    sendCmd('b_speed', parseInt(key));
  else if (key === 'y' || key === 'Y') sendCmd('start_auto');
  else if (key === 'n' || key === 'N') sendCmd('start_manual');
});

// ── Initial status fetch ──────────────────────────────────────────
fetch('/api/status')
  .then(r => r.json())
  .then(d => updateStatus(d))
  .catch(() => {});
