// Path recorder for the splat viewer.
// Activate by appending ?record=1 to the URL.
// UI lets you capture the current camera, play back the captured path, and export as JSON.

(function () {
  if (!new URLSearchParams(location.search).has("record")) return;

  const keyframes = [];
  let playing = false;
  let playStart = 0;
  let durationPerSegmentMs = 2000; // 2s between captured points

  // --- Math helpers ---------------------------------------------------------

  // Camera world position from view matrix (column-major 4x4): position = -R^T * t,
  // where R is the upper-left 3x3 and t is the last column's xyz.
  function cameraPosition(v) {
    const r00 = v[0], r10 = v[1], r20 = v[2];
    const r01 = v[4], r11 = v[5], r21 = v[6];
    const r02 = v[8], r12 = v[9], r22 = v[10];
    const tx = v[12], ty = v[13], tz = v[14];
    return [
      -(r00 * tx + r10 * ty + r20 * tz),
      -(r01 * tx + r11 * ty + r21 * tz),
      -(r02 * tx + r12 * ty + r22 * tz),
    ];
  }

  // Camera-to-world rotation 3x3 (row-major) from view matrix.
  // View has R(world->cam); cam->world is R^T.
  function cameraRotation(v) {
    return [
      v[0], v[1], v[2],
      v[4], v[5], v[6],
      v[8], v[9], v[10],
    ];
  }

  function rotMatrixToQuat(m) {
    // m is row-major 3x3 of cam->world.
    const m00 = m[0], m01 = m[1], m02 = m[2];
    const m10 = m[3], m11 = m[4], m12 = m[5];
    const m20 = m[6], m21 = m[7], m22 = m[8];
    const tr = m00 + m11 + m22;
    let qw, qx, qy, qz;
    if (tr > 0) {
      const s = Math.sqrt(tr + 1.0) * 2;
      qw = 0.25 * s;
      qx = (m21 - m12) / s;
      qy = (m02 - m20) / s;
      qz = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
      const s = Math.sqrt(1.0 + m00 - m11 - m22) * 2;
      qw = (m21 - m12) / s;
      qx = 0.25 * s;
      qy = (m01 + m10) / s;
      qz = (m02 + m20) / s;
    } else if (m11 > m22) {
      const s = Math.sqrt(1.0 + m11 - m00 - m22) * 2;
      qw = (m02 - m20) / s;
      qx = (m01 + m10) / s;
      qy = 0.25 * s;
      qz = (m12 + m21) / s;
    } else {
      const s = Math.sqrt(1.0 + m22 - m00 - m11) * 2;
      qw = (m10 - m01) / s;
      qx = (m02 + m20) / s;
      qy = (m12 + m21) / s;
      qz = 0.25 * s;
    }
    const n = Math.hypot(qx, qy, qz, qw);
    return [qx / n, qy / n, qz / n, qw / n];
  }

  function quatToRotMatrix(q) {
    const [x, y, z, w] = q;
    return [
      1 - 2 * (y * y + z * z),     2 * (x * y - z * w),         2 * (x * z + y * w),
      2 * (x * y + z * w),         1 - 2 * (x * x + z * z),     2 * (y * z - x * w),
      2 * (x * z - y * w),         2 * (y * z + x * w),         1 - 2 * (x * x + y * y),
    ];
  }

  function slerp(a, b, t) {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    let bb = b;
    if (dot < 0) { bb = [-b[0], -b[1], -b[2], -b[3]]; dot = -dot; }
    if (dot > 0.9995) {
      const r = [
        a[0] + t * (bb[0] - a[0]),
        a[1] + t * (bb[1] - a[1]),
        a[2] + t * (bb[2] - a[2]),
        a[3] + t * (bb[3] - a[3]),
      ];
      const n = Math.hypot(r[0], r[1], r[2], r[3]);
      return [r[0] / n, r[1] / n, r[2] / n, r[3] / n];
    }
    const theta0 = Math.acos(dot);
    const theta = theta0 * t;
    const s1 = Math.sin(theta0 - theta) / Math.sin(theta0);
    const s2 = Math.sin(theta) / Math.sin(theta0);
    return [
      s1 * a[0] + s2 * bb[0],
      s1 * a[1] + s2 * bb[1],
      s1 * a[2] + s2 * bb[2],
      s1 * a[3] + s2 * bb[3],
    ];
  }

  // Build a view matrix from a camera world position + cam->world rotation 3x3 (row-major).
  function makeViewMatrix(pos, camToWorld) {
    // World->cam = transpose of cam->world.
    const r00 = camToWorld[0], r01 = camToWorld[1], r02 = camToWorld[2];
    const r10 = camToWorld[3], r11 = camToWorld[4], r12 = camToWorld[5];
    const r20 = camToWorld[6], r21 = camToWorld[7], r22 = camToWorld[8];
    // World->cam columns are the rows of cam->world (the right/up/forward basis vectors of camera in world).
    // Translation = -worldToCam * pos.
    const t0 = -(r00 * pos[0] + r10 * pos[1] + r20 * pos[2]);
    const t1 = -(r01 * pos[0] + r11 * pos[1] + r21 * pos[2]);
    const t2 = -(r02 * pos[0] + r12 * pos[1] + r22 * pos[2]);
    return [
      r00, r01, r02, 0,
      r10, r11, r12, 0,
      r20, r21, r22, 0,
      t0,  t1,  t2,  1,
    ];
  }

  // Decompose all keyframes once when playback starts.
  let decomposed = [];
  function rebuildDecomposed() {
    decomposed = keyframes.map((v) => ({
      pos: cameraPosition(v),
      quat: rotMatrixToQuat(cameraRotation(v)),
    }));
  }

  function getPlaybackMatrix() {
    if (decomposed.length === 0) return null;
    if (decomposed.length === 1) return keyframes[0];

    const t = (performance.now() - playStart) / durationPerSegmentMs;
    const total = decomposed.length; // loop back to first
    const segIndex = Math.floor(t) % total;
    const segT = t - Math.floor(t);
    const a = decomposed[segIndex];
    const b = decomposed[(segIndex + 1) % total];

    const pos = [
      a.pos[0] + (b.pos[0] - a.pos[0]) * segT,
      a.pos[1] + (b.pos[1] - a.pos[1]) * segT,
      a.pos[2] + (b.pos[2] - a.pos[2]) * segT,
    ];
    const q = slerp(a.quat, b.quat, segT);
    return makeViewMatrix(pos, quatToRotMatrix(q));
  }

  // --- UI -------------------------------------------------------------------

  function makeUI() {
    const panel = document.createElement("div");
    panel.id = "path-recorder-panel";
    panel.innerHTML = `
      <style>
        #path-recorder-panel {
          position: fixed;
          top: 16px;
          right: 16px;
          z-index: 9999;
          background: rgba(10, 20, 36, 0.92);
          color: #fff;
          font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
          font-size: 13px;
          padding: 14px 16px;
          border-radius: 10px;
          border: 1px solid #ffffff22;
          box-shadow: 0 8px 24px #0008;
          min-width: 220px;
          backdrop-filter: blur(8px);
        }
        #path-recorder-panel h4 {
          margin: 0 0 10px;
          font-size: 12px;
          letter-spacing: 0.6px;
          text-transform: uppercase;
          color: #39f;
        }
        #path-recorder-panel button {
          display: block;
          width: 100%;
          margin: 4px 0;
          padding: 8px 10px;
          background: #0080ff22;
          color: #fff;
          border: 1px solid #0080ff55;
          border-radius: 6px;
          cursor: pointer;
          font-size: 13px;
          font-weight: 600;
          transition: background 0.15s;
        }
        #path-recorder-panel button:hover { background: #0080ff44; }
        #path-recorder-panel button.danger { border-color: #ff555588; background: #ff555522; }
        #path-recorder-panel button.danger:hover { background: #ff555544; }
        #path-recorder-panel .count {
          font-size: 12px;
          color: #ffffffaa;
          margin: 6px 0 10px;
        }
        #path-recorder-panel .row {
          display: flex;
          gap: 6px;
          align-items: center;
          font-size: 12px;
          color: #ffffffaa;
          margin: 8px 0 4px;
        }
        #path-recorder-panel input[type="number"] {
          width: 70px;
          padding: 4px 6px;
          background: #0006;
          color: #fff;
          border: 1px solid #ffffff22;
          border-radius: 4px;
          font-size: 12px;
        }
      </style>
      <h4>Path Recorder</h4>
      <div class="count" id="pr-count">0 keyframes</div>
      <button id="pr-capture">Capture (C)</button>
      <button id="pr-play">Play (P)</button>
      <button id="pr-export">Export JSON</button>
      <div class="row">
        <span>Seg. duration (ms):</span>
        <input type="number" id="pr-duration" value="2000" min="100" step="100" />
      </div>
      <button id="pr-reset" class="danger">Reset</button>
    `;
    document.body.appendChild(panel);

    const countEl = panel.querySelector("#pr-count");
    const playBtn = panel.querySelector("#pr-play");

    function updateCount() {
      countEl.textContent = `${keyframes.length} keyframe${keyframes.length === 1 ? "" : "s"}`;
    }

    panel.querySelector("#pr-capture").addEventListener("click", capture);
    panel.querySelector("#pr-reset").addEventListener("click", () => {
      keyframes.length = 0;
      playing = false;
      window.__pathRecorder.playing = false;
      playBtn.textContent = "Play (P)";
      updateCount();
    });
    panel.querySelector("#pr-export").addEventListener("click", exportJSON);
    panel.querySelector("#pr-duration").addEventListener("input", (e) => {
      const v = parseInt(e.target.value, 10);
      if (v >= 100) durationPerSegmentMs = v;
    });
    playBtn.addEventListener("click", togglePlay);

    function capture() {
      if (!window.__viewMatrix) {
        console.warn("[recorder] no viewMatrix available yet");
        return;
      }
      keyframes.push([...window.__viewMatrix]);
      updateCount();
      console.log("[recorder] captured keyframe", keyframes.length);
    }

    function togglePlay() {
      if (keyframes.length < 2) {
        console.warn("[recorder] need at least 2 keyframes to play");
        return;
      }
      playing = !playing;
      window.__pathRecorder.playing = playing;
      if (playing) {
        rebuildDecomposed();
        playStart = performance.now();
        playBtn.textContent = "Stop (P)";
      } else {
        playBtn.textContent = "Play (P)";
      }
    }

    function exportJSON() {
      const data = {
        scene: "Cargo_Container_cutted.splat",
        durationPerSegmentMs,
        keyframes,
      };
      const json = JSON.stringify(data, null, 2);
      console.log("[recorder] exported path:\n", json);
      const blob = new Blob([json], { type: "application/json" });
      const a = document.createElement("a");
      a.href = URL.createObjectURL(blob);
      a.download = "camera-path.json";
      document.body.appendChild(a);
      a.click();
      a.remove();
    }

    // Keyboard shortcuts
    window.addEventListener("keydown", (e) => {
      if (e.target.tagName === "INPUT") return;
      if (e.key === "c" || e.key === "C") { capture(); }
      else if (e.key === "p" || e.key === "P") { togglePlay(); }
    });
  }

  // --- Bootstrap ------------------------------------------------------------

  window.__pathRecorder = { playing: false, getPlaybackMatrix };

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", makeUI);
  } else {
    makeUI();
  }
})();
