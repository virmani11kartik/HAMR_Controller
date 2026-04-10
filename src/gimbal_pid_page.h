const char* gimbal_pid_page = R"=====(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Gimbal Roll Tuning</title>
<style>
  :root { --fg:#f4f7fb; --bg:#0c1220; --card:#162033; --muted:#a7b5d1; --accent:#3ecf8e; --accent2:#59a8ff; }
  * { box-sizing: border-box; font-family: Segoe UI, Arial, sans-serif; }
  body { margin: 0; background: radial-gradient(circle at top, #1d315a 0%, var(--bg) 60%); color: var(--fg); }
  .wrap { max-width: 760px; margin: 24px auto; padding: 0 16px; }
  .card { background: rgba(22,32,51,0.92); border-radius: 18px; padding: 22px; box-shadow: 0 18px 48px rgba(0,0,0,0.28); }
  h1 { margin: 0 0 8px; font-size: 1.55rem; }
  .small { color: var(--muted); margin-bottom: 20px; line-height: 1.45; }
  .row { display: grid; grid-template-columns: 72px 1fr 104px; gap: 12px; align-items: center; margin: 16px 0; }
  label { font-weight: 700; }
  input[type="range"] { width: 100%; accent-color: var(--accent); }
  input[type="number"] { width: 100%; padding: 10px; border-radius: 10px; border: 1px solid #37507f; background: #0f1727; color: var(--fg); }
  .btns { display: flex; gap: 10px; margin-top: 18px; flex-wrap: wrap; }
  button { padding: 10px 14px; border: 0; border-radius: 10px; background: var(--accent); color: #071019; font-weight: 700; cursor: pointer; }
  button.alt { background: var(--accent2); color: white; }
  button.ghost { background: #24344f; color: var(--fg); }
  .status { margin-top: 16px; color: var(--muted); white-space: pre-line; min-height: 72px; }
  .live { display: grid; grid-template-columns: repeat(4, minmax(0,1fr)); gap: 12px; margin-top: 18px; }
  .tile { background: #0f1727; border-radius: 12px; padding: 12px; }
  .tile b { display: block; font-size: 1.2rem; margin-top: 6px; }
  .chartWrap { margin-top: 18px; background: #0f1727; border-radius: 14px; padding: 12px; }
  canvas { width: 100%; height: 260px; display: block; background: #0a1020; border-radius: 10px; }
  .legend { display: flex; gap: 14px; flex-wrap: wrap; font-size: 0.92rem; color: var(--muted); margin-top: 10px; }
  .legend span::before { content: ""; display: inline-block; width: 12px; height: 3px; margin-right: 6px; vertical-align: middle; border-radius: 2px; }
  .l1::before { background: #ffd166; }
  .l2::before { background: #3ecf8e; }
  .l3::before { background: #59a8ff; }
  .l4::before { background: #ff6b6b; }
</style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>Roll Kp/Kd Tuning</h1>
      <div class="small">Connect to the ESP32 access point and open <code>192.168.50.1</code>. The outer loop now commands motor speed, not position, so you can tune the roll stabilizer against real speed response.</div>

      <div class="row">
        <label for="kp">Kp</label>
        <input id="kp_range" type="range" min="0" max="20" step="0.05">
        <input id="kp" type="number" min="0" max="20" step="0.05">
      </div>

      <div class="row">
        <label for="kd">Kd</label>
        <input id="kd_range" type="range" min="0" max="2" step="0.01">
        <input id="kd" type="number" min="0" max="2" step="0.01">
      </div>

      <div class="btns">
        <button id="applyBtn">Apply</button>
        <button id="refreshBtn" class="alt">Refresh</button>
        <button id="defaultsBtn" class="ghost">Reset Defaults</button>
      </div>

      <div class="live">
        <div class="tile">Roll<b id="rollLive">--</b></div>
        <div class="tile">Filtered<b id="rollFiltLive">--</b></div>
        <div class="tile">Cmd Speed<b id="cmdLive">--</b></div>
        <div class="tile">Motor Speed<b id="motorLive">--</b></div>
      </div>

      <div class="chartWrap">
        <canvas id="plot" width="720" height="260"></canvas>
        <div class="legend">
          <span class="l1">IMU Roll</span>
          <span class="l2">Filtered</span>
          <span class="l3">Cmd Speed</span>
          <span class="l4">Motor Speed</span>
        </div>
      </div>

      <div id="status" class="status">Loading gains...</div>
    </div>
  </div>

<script>
  const s = (id) => document.getElementById(id);
  const statusEl = s('status');
  const plot = s('plot');
  const ctx = plot.getContext('2d');
  const history = [];
  const MAX_POINTS = 140;
  const ANGLE_RANGE = 20;
  const SPEED_RANGE = 400;

  function setFields(kp, kd) {
    s('kp').value = Number(kp).toFixed(3);
    s('kd').value = Number(kd).toFixed(3);
    s('kp_range').value = kp;
    s('kd_range').value = kd;
  }

  function setLive(j) {
    s('rollLive').textContent = Number(j.roll ?? 0).toFixed(2);
    s('rollFiltLive').textContent = Number(j.roll_filtered ?? 0).toFixed(2);
    s('cmdLive').textContent = Number(j.roll_cmd ?? 0).toFixed(1);
    s('motorLive').textContent = j.motor_valid ? Number(j.motor_speed ?? 0).toFixed(1) : '--';
  }

  function pushSample(j) {
    history.push({
      roll: Number(j.roll ?? 0),
      filtered: Number(j.roll_filtered ?? 0),
      cmd: Number(j.roll_cmd ?? 0),
      motor: j.motor_valid ? Number(j.motor_speed ?? 0) : null
    });
    while (history.length > MAX_POINTS) history.shift();
  }

  function mapAngleY(v) {
    const clamped = Math.max(-ANGLE_RANGE, Math.min(ANGLE_RANGE, v));
    return ((ANGLE_RANGE - clamped) / (ANGLE_RANGE * 2)) * (plot.height * 0.48);
  }

  function mapSpeedY(v) {
    const clamped = Math.max(-SPEED_RANGE, Math.min(SPEED_RANGE, v));
    const top = plot.height * 0.54;
    const height = plot.height * 0.42;
    return top + ((SPEED_RANGE - clamped) / (SPEED_RANGE * 2)) * height;
  }

  function drawSeries(key, color, mapper) {
    if (!history.length) return;
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    let started = false;
    history.forEach((sample, i) => {
      const value = sample[key];
      if (value === null || Number.isNaN(value)) return;
      const x = (i / Math.max(1, MAX_POINTS - 1)) * plot.width;
      const y = mapper(value);
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    });
    ctx.stroke();
  }

  function drawPlot() {
    ctx.clearRect(0, 0, plot.width, plot.height);
    ctx.fillStyle = '#0a1020';
    ctx.fillRect(0, 0, plot.width, plot.height);

    ctx.strokeStyle = 'rgba(255,255,255,0.10)';
    ctx.lineWidth = 1;
    for (let deg = -20; deg <= 20; deg += 10) {
      const y = mapAngleY(deg);
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(plot.width, y);
      ctx.stroke();
    }

    for (let spd = -400; spd <= 400; spd += 200) {
      const y = mapSpeedY(spd);
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(plot.width, y);
      ctx.stroke();
    }

    ctx.fillStyle = 'rgba(255,255,255,0.7)';
    ctx.font = '12px Segoe UI';
    ctx.fillText('Angle (deg)', 8, 14);
    ctx.fillText('Speed (deg/s)', 8, plot.height * 0.54 - 6);

    drawSeries('roll', '#ffd166', mapAngleY);
    drawSeries('filtered', '#3ecf8e', mapAngleY);
    drawSeries('cmd', '#59a8ff', mapSpeedY);
    drawSeries('motor', '#ff6b6b', mapSpeedY);
  }

  function status(msg) { statusEl.textContent = msg; }

  async function refreshPID() {
    status('Reading current tuning...');
    try {
      const r = await fetch('/getPID', { cache: 'no-store' });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      const j = await r.json();
      setFields(j.kp ?? 0, j.kd ?? 0);
      setLive(j);
      pushSample(j);
      drawPlot();
      status(`Current roll gains:\nKp=${Number(j.kp).toFixed(3)}  Kd=${Number(j.kd).toFixed(3)}\nAngles are top plot band, speeds are bottom plot band.`);
    } catch (e) {
      status('Failed to read tuning: ' + e);
    }
  }

  async function applyPID(kp, kd) {
    const body = new URLSearchParams({ kp, kd });
    status('Updating gains...');
    try {
      const r = await fetch('/updatePID', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body
      });
      const j = await r.json();
      setFields(j.kp ?? kp, j.kd ?? kd);
      setLive(j);
      pushSample(j);
      drawPlot();
      status(`Applied:\nKp=${Number(j.kp).toFixed(3)}  Kd=${Number(j.kd).toFixed(3)}`);
    } catch (e) {
      status('Failed to update tuning: ' + e);
    }
  }

  function sendFromFields() {
    const kp = Number(s('kp').value);
    const kd = Number(s('kd').value);
    if ([kp, kd].some(v => Number.isNaN(v))) {
      status('Please enter numeric Kp and Kd values.');
      return;
    }
    applyPID(kp, kd);
  }

  function bindPair(rangeId, numberId) {
    const range = s(rangeId);
    const number = s(numberId);
    range.addEventListener('input', () => {
      number.value = Number(range.value).toFixed(3);
      sendFromFields();
    });
    number.addEventListener('change', () => {
      range.value = number.value;
      sendFromFields();
    });
  }

  s('applyBtn').addEventListener('click', sendFromFields);
  s('refreshBtn').addEventListener('click', refreshPID);
  s('defaultsBtn').addEventListener('click', () => {
    setFields(8.000, 0.350);
    sendFromFields();
  });

  bindPair('kp_range', 'kp');
  bindPair('kd_range', 'kd');

  refreshPID();
  setInterval(refreshPID, 1500);
</script>
</body>
</html>)=====";
