#pragma once
// Auto-generated from gimbal_dashboard.html — do not edit manually.
// Served by the ESP32 WebServer at GET /

static const char GIMBAL_PAGE[] PROGMEM = R"GIMBAL_HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>HAMR Gimbal Tuner</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;600&family=Barlow+Condensed:wght@300;500;700&display=swap');

  :root {
    --bg0: #080c10;
    --bg1: #0e1318;
    --bg2: #141b22;
    --bg3: #1c2530;
    --border: #1f2d3d;
    --border-hi: #2a3f55;
    --accent: #00d4ff;
    --accent2: #ff6b35;
    --accent3: #39ff8f;
    --muted: #3a5068;
    --text: #c8dae8;
    --text-dim: #5a7a90;
    --roll-color: #00d4ff;
    --roll-raw: rgba(0,212,255,0.35);
    --pitch-color: #ff6b35;
    --pitch-raw: rgba(255,107,53,0.35);
    --err-roll: #ff3d71;
    --err-pitch: #ffc107;
    --speed-cmd: rgba(180,180,255,0.7);
    --speed-mot: rgba(100,255,180,0.9);
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg0);
    color: var(--text);
    font-family: 'JetBrains Mono', monospace;
    font-size: 13px;
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* ── Header ───────────────────────────────────────────────── */
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 12px 24px;
    background: var(--bg1);
    border-bottom: 1px solid var(--border);
    position: sticky;
    top: 0;
    z-index: 100;
  }

  .logo {
    font-family: 'Barlow Condensed', sans-serif;
    font-weight: 700;
    font-size: 20px;
    letter-spacing: 3px;
    color: var(--accent);
    text-transform: uppercase;
  }
  .logo span { color: var(--text-dim); font-weight: 300; }

  .status-row {
    display: flex;
    align-items: center;
    gap: 20px;
  }

  .pill {
    display: flex;
    align-items: center;
    gap: 6px;
    padding: 4px 10px;
    border-radius: 20px;
    background: var(--bg2);
    border: 1px solid var(--border);
    font-size: 11px;
    color: var(--text-dim);
  }
  .pill .dot {
    width: 7px; height: 7px;
    border-radius: 50%;
    background: var(--muted);
    transition: background 0.4s;
  }
  .pill.live .dot { background: var(--accent3); box-shadow: 0 0 6px var(--accent3); }
  .pill.live { color: var(--text); border-color: var(--border-hi); }

  #hz-counter { font-weight: 600; color: var(--accent); font-size: 12px; }

  /* ── Main layout ──────────────────────────────────────────── */
  .layout {
    display: grid;
    grid-template-columns: 280px 1fr;
    gap: 0;
    height: calc(100vh - 49px);
  }

  /* ── Sidebar ─────────────────────────────────────────────── */
  .sidebar {
    background: var(--bg1);
    border-right: 1px solid var(--border);
    padding: 16px;
    overflow-y: auto;
    display: flex;
    flex-direction: column;
    gap: 20px;
  }

  .section-label {
    font-family: 'Barlow Condensed', sans-serif;
    font-size: 11px;
    font-weight: 500;
    letter-spacing: 3px;
    text-transform: uppercase;
    color: var(--text-dim);
    margin-bottom: 10px;
    padding-bottom: 6px;
    border-bottom: 1px solid var(--border);
  }

  /* ── Sliders ─────────────────────────────────────────────── */
  .slider-group { display: flex; flex-direction: column; gap: 14px; }

  .slider-row {
    display: flex;
    flex-direction: column;
    gap: 5px;
  }

  .slider-header {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
  }

  .slider-name {
    font-family: 'Barlow Condensed', sans-serif;
    font-size: 14px;
    font-weight: 500;
    letter-spacing: 1px;
    color: var(--text);
  }

  .slider-val {
    font-size: 12px;
    font-weight: 600;
    color: var(--accent);
    min-width: 40px;
    text-align: right;
    transition: color 0.2s;
  }

  input[type=range] {
    -webkit-appearance: none;
    width: 100%;
    height: 3px;
    background: var(--bg3);
    border-radius: 2px;
    outline: none;
    cursor: pointer;
    border: 1px solid var(--border);
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 14px; height: 14px;
    border-radius: 50%;
    background: var(--accent);
    box-shadow: 0 0 8px var(--accent);
    border: 2px solid var(--bg0);
    cursor: pointer;
    transition: transform 0.15s, box-shadow 0.15s;
  }
  input[type=range]::-webkit-slider-thumb:hover {
    transform: scale(1.3);
    box-shadow: 0 0 14px var(--accent);
  }
  input[type=range].roll { accent-color: var(--roll-color); }
  input[type=range].roll::-webkit-slider-thumb { background: var(--roll-color); box-shadow: 0 0 8px var(--roll-color); }
  input[type=range].pitch { accent-color: var(--pitch-color); }
  input[type=range].pitch::-webkit-slider-thumb { background: var(--pitch-color); box-shadow: 0 0 8px var(--pitch-color); }

  /* ── Send button ─────────────────────────────────────────── */
  .send-btn {
    width: 100%;
    padding: 10px;
    background: transparent;
    border: 1px solid var(--accent);
    color: var(--accent);
    font-family: 'Barlow Condensed', sans-serif;
    font-size: 14px;
    font-weight: 700;
    letter-spacing: 3px;
    text-transform: uppercase;
    cursor: pointer;
    border-radius: 3px;
    transition: background 0.2s, box-shadow 0.2s;
    position: relative;
    overflow: hidden;
  }
  .send-btn:hover {
    background: rgba(0,212,255,0.08);
    box-shadow: 0 0 20px rgba(0,212,255,0.2);
  }
  .send-btn:active { background: rgba(0,212,255,0.18); }
  .send-btn.sent { border-color: var(--accent3); color: var(--accent3); }

  /* ── Live readouts ───────────────────────────────────────── */
  .readout-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }

  .readout-card {
    background: var(--bg2);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 8px 10px;
  }
  .readout-card .r-label {
    font-size: 9px;
    letter-spacing: 2px;
    text-transform: uppercase;
    color: var(--text-dim);
    margin-bottom: 3px;
  }
  .readout-card .r-val {
    font-size: 18px;
    font-weight: 600;
    letter-spacing: -0.5px;
    transition: color 0.2s;
  }
  .readout-card .r-unit {
    font-size: 10px;
    color: var(--text-dim);
    margin-left: 2px;
  }

  .r-roll { color: var(--roll-color); }
  .r-pitch { color: var(--pitch-color); }
  .r-err { color: var(--err-roll); }
  .r-err-p { color: var(--err-pitch); }
  .r-dps { color: var(--accent3); }

  /* ── Charts area ─────────────────────────────────────────── */
  .charts {
    display: grid;
    grid-template-rows: repeat(3, 1fr);
    gap: 1px;
    background: var(--border);
    overflow: hidden;
  }

  .chart-row {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 1px;
    background: var(--border);
  }

  .chart-wrap {
    background: var(--bg0);
    padding: 10px 14px 8px;
    position: relative;
    display: flex;
    flex-direction: column;
  }

  .chart-title {
    font-family: 'Barlow Condensed', sans-serif;
    font-size: 11px;
    font-weight: 500;
    letter-spacing: 2px;
    text-transform: uppercase;
    color: var(--text-dim);
    margin-bottom: 6px;
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .chart-title .tag {
    padding: 1px 6px;
    border-radius: 2px;
    font-size: 9px;
    letter-spacing: 1px;
  }
  .tag-roll  { background: rgba(0,212,255,0.12); color: var(--roll-color); border: 1px solid rgba(0,212,255,0.2); }
  .tag-pitch { background: rgba(255,107,53,0.12); color: var(--pitch-color); border: 1px solid rgba(255,107,53,0.2); }

  .chart-wrap canvas { flex: 1; min-height: 0; }

  /* ── Axis toggle tabs ────────────────────────────────────── */
  .axis-tabs {
    display: flex;
    gap: 6px;
    margin-bottom: 4px;
  }
  .axis-tab {
    padding: 3px 10px;
    border-radius: 2px;
    font-size: 10px;
    letter-spacing: 1px;
    text-transform: uppercase;
    cursor: pointer;
    border: 1px solid var(--border);
    background: transparent;
    color: var(--text-dim);
    font-family: 'JetBrains Mono', monospace;
    transition: all 0.2s;
  }
  .axis-tab.active-roll  { border-color: var(--roll-color); color: var(--roll-color); background: rgba(0,212,255,0.08); }
  .axis-tab.active-pitch { border-color: var(--pitch-color); color: var(--pitch-color); background: rgba(255,107,53,0.08); }
</style>
</head>
<body>

<header>
  <div class="logo">HAMR <span>/</span> Gimbal Tuner</div>
  <div class="status-row">
    <div id="status-pill" class="pill">
      <div class="dot"></div>
      <span id="status-text">Connecting…</span>
    </div>
    <div style="color:var(--text-dim);font-size:11px"><span id="hz-counter">—</span> Hz</div>
    <div style="color:var(--text-dim);font-size:11px">samples: <span id="sample-count" style="color:var(--text)">0</span></div>
  </div>
</header>

<div class="layout">

  <!-- ── Sidebar ── -->
  <aside class="sidebar">

    <!-- Live readouts -->
    <div>
      <div class="section-label">Live Telemetry</div>
      <div class="readout-grid">
        <div class="readout-card">
          <div class="r-label">Roll angle</div>
          <div class="r-val r-roll" id="v-roll">—<span class="r-unit">°</span></div>
        </div>
        <div class="readout-card">
          <div class="r-label">Pitch angle</div>
          <div class="r-val r-pitch" id="v-pitch">—<span class="r-unit">°</span></div>
        </div>
        <div class="readout-card">
          <div class="r-label">Roll error</div>
          <div class="r-val r-err" id="v-rerr">—<span class="r-unit">°</span></div>
        </div>
        <div class="readout-card">
          <div class="r-label">Pitch error</div>
          <div class="r-val r-err-p" id="v-perr">—<span class="r-unit">°</span></div>
        </div>
        <div class="readout-card">
          <div class="r-label">Roll cmd</div>
          <div class="r-val r-dps" id="v-rcmd">—<span class="r-unit">dps</span></div>
        </div>
        <div class="readout-card">
          <div class="r-label">Pitch cmd</div>
          <div class="r-val r-dps" id="v-pcmd">—<span class="r-unit">dps</span></div>
        </div>
      </div>
    </div>

    <!-- Roll PID -->
    <div>
      <div class="section-label">Roll PID</div>
      <div class="slider-group">
        <div class="slider-row">
          <div class="slider-header">
            <span class="slider-name">Kp</span>
            <span class="slider-val" id="roll-kp-val">0.12</span>
          </div>
          <input type="range" id="roll-kp" class="roll" min="0" max="5" step="0.01" value="0.12"
                 oninput="updateVal('roll-kp-val', this.value)">
        </div>
        <div class="slider-row">
          <div class="slider-header">
            <span class="slider-name">Ki</span>
            <span class="slider-val" id="roll-ki-val">0.00</span>
          </div>
          <input type="range" id="roll-ki" class="roll" min="0" max="2" step="0.005" value="0.00"
                 oninput="updateVal('roll-ki-val', this.value)">
        </div>
        <div class="slider-row">
          <div class="slider-header">
            <span class="slider-name">Kd</span>
            <span class="slider-val" id="roll-kd-val">0.02</span>
          </div>
          <input type="range" id="roll-kd" class="roll" min="0" max="1" step="0.005" value="0.02"
                 oninput="updateVal('roll-kd-val', this.value)">
        </div>
      </div>
    </div>

    <!-- Pitch PID -->
    <div>
      <div class="section-label">Pitch PID</div>
      <div class="slider-group">
        <div class="slider-row">
          <div class="slider-header">
            <span class="slider-name">Kp</span>
            <span class="slider-val" id="pitch-kp-val">0.30</span>
          </div>
          <input type="range" id="pitch-kp" class="pitch" min="0" max="5" step="0.01" value="0.30"
                 oninput="updateVal('pitch-kp-val', this.value)">
        </div>
        <div class="slider-row">
          <div class="slider-header">
            <span class="slider-name">Ki</span>
            <span class="slider-val" id="pitch-ki-val">0.00</span>
          </div>
          <input type="range" id="pitch-ki" class="pitch" min="0" max="2" step="0.005" value="0.00"
                 oninput="updateVal('pitch-ki-val', this.value)">
        </div>
        <div class="slider-row">
          <div class="slider-header">
            <span class="slider-name">Kd</span>
            <span class="slider-val" id="pitch-kd-val">0.08</span>
          </div>
          <input type="range" id="pitch-kd" class="pitch" min="0" max="1" step="0.005" value="0.08"
                 oninput="updateVal('pitch-kd-val', this.value)">
        </div>
      </div>
    </div>

    <!-- Max DPS -->
    <div>
      <div class="section-label">Speed Limit</div>
      <div class="slider-group">
        <div class="slider-row">
          <div class="slider-header">
            <span class="slider-name">MAX_DPS</span>
            <span class="slider-val" id="max-dps-val">100</span>
          </div>
          <input type="range" id="max-dps" min="10" max="300" step="5" value="100"
                 oninput="updateVal('max-dps-val', this.value)"
                 style="accent-color:#39ff8f">
          <style>#max-dps::-webkit-slider-thumb{background:var(--accent3);box-shadow:0 0 8px var(--accent3);}</style>
        </div>
      </div>
    </div>

    <!-- Send -->
    <button class="send-btn" id="send-btn" onclick="sendParams()">⬆ Apply to Motor</button>
    <div id="send-status" style="text-align:center;font-size:10px;color:var(--text-dim);margin-top:-12px;min-height:14px;"></div>

  </aside>

  <!-- ── Charts ── -->
  <main class="charts">

    <!-- Row 0: Angle -->
    <div class="chart-row">
      <div class="chart-wrap">
        <div class="chart-title"><span class="tag tag-roll">ROLL</span> Angle (deg)</div>
        <canvas id="c-roll-angle"></canvas>
      </div>
      <div class="chart-wrap">
        <div class="chart-title"><span class="tag tag-pitch">PITCH</span> Angle (deg)</div>
        <canvas id="c-pitch-angle"></canvas>
      </div>
    </div>

    <!-- Row 1: Speed -->
    <div class="chart-row">
      <div class="chart-wrap">
        <div class="chart-title"><span class="tag tag-roll">ROLL</span> Speed (dps) — <span style="color:#b4b4ff;font-size:9px">dashed=cmd  solid=motor</span></div>
        <canvas id="c-roll-speed"></canvas>
      </div>
      <div class="chart-wrap">
        <div class="chart-title"><span class="tag tag-pitch">PITCH</span> Speed (dps)</div>
        <canvas id="c-pitch-speed"></canvas>
      </div>
    </div>

    <!-- Row 2: Error -->
    <div class="chart-row">
      <div class="chart-wrap">
        <div class="chart-title"><span class="tag tag-roll">ROLL</span> Error (deg)</div>
        <canvas id="c-roll-err"></canvas>
      </div>
      <div class="chart-wrap">
        <div class="chart-title"><span class="tag tag-pitch">PITCH</span> Error (deg)</div>
        <canvas id="c-pitch-err"></canvas>
      </div>
    </div>

  </main>

</div>

<script>
// ─── Config ──────────────────────────────────────────────────────────
const POLL_MS   = 50;    // fetch /data every 50 ms  (20 Hz display)
const WIN       = 150;   // samples shown in window

// ─── Ring buffer helper ───────────────────────────────────────────────
function makeRing(n) {
  const a = new Array(n).fill(0);
  let head = 0;
  return {
    push(v) { a[head] = v; head = (head + 1) % n; },
    snapshot() { return [...a.slice(head), ...a.slice(0, head)]; }
  };
}

const bufs = {
  roll_raw:        makeRing(WIN),
  roll_filt:       makeRing(WIN),
  roll_cmd_dps:    makeRing(WIN),
  roll_err:        makeRing(WIN),
  roll_mot_speed:  makeRing(WIN),
  pitch_raw:       makeRing(WIN),
  pitch_filt:      makeRing(WIN),
  pitch_cmd_dps:   makeRing(WIN),
  pitch_err:       makeRing(WIN),
  pitch_mot_speed: makeRing(WIN),
};

const labels = new Array(WIN).fill('');

// ─── Chart factory ────────────────────────────────────────────────────
const CHART_DEFAULTS = {
  responsive: true,
  maintainAspectRatio: false,
  animation: false,
  interaction: { mode: 'none' },
  plugins: { legend: { display: false }, tooltip: { enabled: false } },
  scales: {
    x: {
      display: false,
      type: 'category',
    },
    y: {
      grid: { color: 'rgba(255,255,255,0.04)', drawBorder: false },
      ticks: {
        color: '#3a5068',
        font: { family: 'JetBrains Mono', size: 10 },
        maxTicksLimit: 5,
      },
      border: { display: false },
    }
  },
  elements: { point: { radius: 0 }, line: { tension: 0.2 } }
};

function mkChart(id, datasets, yMin, yMax) {
  const ctx = document.getElementById(id).getContext('2d');
  return new Chart(ctx, {
    type: 'line',
    data: { labels, datasets },
    options: {
      ...CHART_DEFAULTS,
      scales: {
        ...CHART_DEFAULTS.scales,
        y: {
          ...CHART_DEFAULTS.scales.y,
          min: yMin, max: yMax,
        }
      }
    }
  });
}

// ─── Charts ───────────────────────────────────────────────────────────
const charts = {
  rollAngle: mkChart('c-roll-angle', [
    { label: 'raw',      data: bufs.roll_raw.snapshot(),  borderColor: 'rgba(0,212,255,0.35)', borderWidth: 1 },
    { label: 'filtered', data: bufs.roll_filt.snapshot(), borderColor: '#00d4ff',              borderWidth: 2 },
  ], -45, 45),

  pitchAngle: mkChart('c-pitch-angle', [
    { label: 'raw',      data: bufs.pitch_raw.snapshot(),  borderColor: 'rgba(255,107,53,0.35)', borderWidth: 1 },
    { label: 'filtered', data: bufs.pitch_filt.snapshot(), borderColor: '#ff6b35',               borderWidth: 2 },
  ], -45, 45),

  rollSpeed: mkChart('c-roll-speed', [
    { label: 'cmd',   data: bufs.roll_cmd_dps.snapshot(),   borderColor: 'rgba(180,180,255,0.8)', borderWidth: 1.5, borderDash: [4,3] },
    { label: 'motor', data: bufs.roll_mot_speed.snapshot(), borderColor: '#39ff8f',               borderWidth: 2 },
  ], -110, 110),

  pitchSpeed: mkChart('c-pitch-speed', [
    { label: 'cmd',   data: bufs.pitch_cmd_dps.snapshot(),   borderColor: 'rgba(180,180,255,0.8)', borderWidth: 1.5, borderDash: [4,3] },
    { label: 'motor', data: bufs.pitch_mot_speed.snapshot(), borderColor: '#4db6ac',               borderWidth: 2 },
  ], -110, 110),

  rollErr: mkChart('c-roll-err', [
    { label: 'error', data: bufs.roll_err.snapshot(), borderColor: '#ff3d71', borderWidth: 2,
      fill: true, backgroundColor: 'rgba(255,61,113,0.07)' },
  ], -25, 25),

  pitchErr: mkChart('c-pitch-err', [
    { label: 'error', data: bufs.pitch_err.snapshot(), borderColor: '#ffc107', borderWidth: 2,
      fill: true, backgroundColor: 'rgba(255,193,7,0.07)' },
  ], -25, 25),
};

// ─── Slider helpers ───────────────────────────────────────────────────
function updateVal(id, val) {
  document.getElementById(id).textContent = parseFloat(val).toFixed(
    id.includes('dps') ? 0 : 3
  );
}

// ─── Hz tracking ─────────────────────────────────────────────────────
let lastSampleCount = 0;
let sampleCount = 0;
setInterval(() => {
  const hz = sampleCount - lastSampleCount;
  document.getElementById('hz-counter').textContent = hz;
  lastSampleCount = sampleCount;
}, 1000);

// ─── Data polling ─────────────────────────────────────────────────────
let connected = false;

async function poll() {
  try {
    const r   = await fetch('/data', { signal: AbortSignal.timeout(300) });
    const d   = await r.json();

    bufs.roll_raw.push(d.roll_raw);
    bufs.roll_filt.push(d.roll_filt);
    bufs.roll_cmd_dps.push(d.roll_cmd_dps);
    bufs.roll_err.push(d.roll_err);
    bufs.roll_mot_speed.push(d.roll_mot_speed);
    bufs.pitch_raw.push(d.pitch_raw);
    bufs.pitch_filt.push(d.pitch_filt);
    bufs.pitch_cmd_dps.push(d.pitch_cmd_dps);
    bufs.pitch_err.push(d.pitch_err);
    bufs.pitch_mot_speed.push(d.pitch_mot_speed);

    sampleCount++;

    // Update readouts
    document.getElementById('v-roll').innerHTML  = d.roll_filt.toFixed(2)  + '<span class="r-unit">°</span>';
    document.getElementById('v-pitch').innerHTML = d.pitch_filt.toFixed(2) + '<span class="r-unit">°</span>';
    document.getElementById('v-rerr').innerHTML  = d.roll_err.toFixed(2)   + '<span class="r-unit">°</span>';
    document.getElementById('v-perr').innerHTML  = d.pitch_err.toFixed(2)  + '<span class="r-unit">°</span>';
    document.getElementById('v-rcmd').innerHTML  = d.roll_cmd_dps.toFixed(1)  + '<span class="r-unit">dps</span>';
    document.getElementById('v-pcmd').innerHTML  = d.pitch_cmd_dps.toFixed(1) + '<span class="r-unit">dps</span>';
    document.getElementById('sample-count').textContent = sampleCount;

    // Sync slider positions from ESP if params changed server-side
    if (d.params) {
      syncSliders(d.params);
    }

    if (!connected) {
      connected = true;
      document.getElementById('status-pill').classList.add('live');
      document.getElementById('status-text').textContent = 'Live';
    }

  } catch (e) {
    if (connected) {
      connected = false;
      document.getElementById('status-pill').classList.remove('live');
      document.getElementById('status-text').textContent = 'Disconnected';
    }
  }
}

// ─── Chart update loop ────────────────────────────────────────────────
function refreshCharts() {
  charts.rollAngle.data.datasets[0].data  = bufs.roll_raw.snapshot();
  charts.rollAngle.data.datasets[1].data  = bufs.roll_filt.snapshot();
  charts.pitchAngle.data.datasets[0].data = bufs.pitch_raw.snapshot();
  charts.pitchAngle.data.datasets[1].data = bufs.pitch_filt.snapshot();
  charts.rollSpeed.data.datasets[0].data  = bufs.roll_cmd_dps.snapshot();
  charts.rollSpeed.data.datasets[1].data  = bufs.roll_mot_speed.snapshot();
  charts.pitchSpeed.data.datasets[0].data = bufs.pitch_cmd_dps.snapshot();
  charts.pitchSpeed.data.datasets[1].data = bufs.pitch_mot_speed.snapshot();
  charts.rollErr.data.datasets[0].data    = bufs.roll_err.snapshot();
  charts.pitchErr.data.datasets[0].data   = bufs.pitch_err.snapshot();

  for (const c of Object.values(charts)) c.update('none');
  requestAnimationFrame(refreshCharts);
}

// ─── Send params to ESP ───────────────────────────────────────────────
async function sendParams() {
  const params = new URLSearchParams({
    roll_kp:  document.getElementById('roll-kp').value,
    roll_ki:  document.getElementById('roll-ki').value,
    roll_kd:  document.getElementById('roll-kd').value,
    pitch_kp: document.getElementById('pitch-kp').value,
    pitch_ki: document.getElementById('pitch-ki').value,
    pitch_kd: document.getElementById('pitch-kd').value,
    max_dps:  document.getElementById('max-dps').value,
  });

  const btn = document.getElementById('send-btn');
  const st  = document.getElementById('send-status');
  btn.disabled = true;

  try {
    const r = await fetch('/setParams?' + params.toString(), { method: 'GET' });
    if (r.ok) {
      btn.classList.add('sent');
      btn.textContent = '✓ Applied';
      st.textContent  = 'Parameters updated successfully';
      st.style.color  = 'var(--accent3)';
    } else {
      throw new Error('Bad response');
    }
  } catch {
    st.textContent = '✗ Failed to apply';
    st.style.color = 'var(--err-roll)';
  }

  setTimeout(() => {
    btn.disabled    = false;
    btn.classList.remove('sent');
    btn.textContent = '⬆ Apply to Motor';
    st.textContent  = '';
  }, 2000);
}

function syncSliders(p) {
  const set = (id, valId, v, decimals) => {
    document.getElementById(id).value = v;
    document.getElementById(valId).textContent = parseFloat(v).toFixed(decimals);
  };
  if (p.roll_kp  !== undefined) set('roll-kp',  'roll-kp-val',  p.roll_kp,  3);
  if (p.roll_ki  !== undefined) set('roll-ki',  'roll-ki-val',  p.roll_ki,  3);
  if (p.roll_kd  !== undefined) set('roll-kd',  'roll-kd-val',  p.roll_kd,  3);
  if (p.pitch_kp !== undefined) set('pitch-kp', 'pitch-kp-val', p.pitch_kp, 3);
  if (p.pitch_ki !== undefined) set('pitch-ki', 'pitch-ki-val', p.pitch_ki, 3);
  if (p.pitch_kd !== undefined) set('pitch-kd', 'pitch-kd-val', p.pitch_kd, 3);
  if (p.max_dps  !== undefined) set('max-dps',  'max-dps-val',  p.max_dps,  0);
}

// ─── Boot ─────────────────────────────────────────────────────────────
setInterval(poll, POLL_MS);
requestAnimationFrame(refreshCharts);
</script>
</body>
</html>

)GIMBAL_HTML";