# 🚀 Rocket Telemetry Desktop Console

A high-performance, real-time **Tkinter-based telemetry dashboard** for rocket systems.  
Designed for long-duration runs, high data rates, and post-flight analysis without the overhead of a web UI.
<img width="1919" height="1031" alt="image" src="https://github.com/user-attachments/assets/d1eb3d0d-2d3d-4b13-89fa-34ced32f0d81" />

---

## 🔥 Core Capabilities

### 📡 Live Telemetry
- Serial input (COM + baud configurable)
- Built-in simulator for testing
- Handles high-rate telemetry without UI slowdown

Real-time display:
- Altitude (5-minute sliding window)
- Speed (5-minute sliding window)
- Roll / Pitch / Yaw
- Rocket body-axis attitude view
- Artificial horizon
- GPS map with live tracking

---

### 🧠 Attitude Visualization

- Rocket-shaped vehicle representation
- Body-axis aligned:
  - **+X = Nose (points upward on screen)**
  - +Y / +Z labeled
- "Body/Nose view" — like looking out the rocket

---

### 🗺️ Map System

- Street + Satellite tiles
- Fully interactive:
  - Mouse wheel zoom
  - Click + drag pan
  - Recenter button
- Historical tracking:
  - Uses full telemetry buffer
  - Downsamples automatically for performance
- Graceful fallback if map tiles fail

---

### 🎮 Playback Mode

Replay recorded telemetry:

- Load CSV logs
- Replay entire flight through:
  - charts
  - map
  - attitude view

Controls:
- Pause / Resume
- Timeline scrubber (jump anywhere)
- Adjustable speed (0.5x, 1x, 2x)

---

### 💾 Data System

#### Automatic Logging
- Starts on connect (Serial or Simulator)
- Saves to:
```
telemetry_logs/
```

#### Save Buffer
- Manual export of ALL in-memory data
- Includes full history, not just chart window

---

### 🎯 Tare System

Button: **Tare IMU+Alt**

Zeroes:
- roll
- pitch
- yaw
- altitude

Used for:
- defining launch reference
- recalibration mid-run

---

## ⚡ Performance Design

This system is built to **run for hours without degrading**.

### Key optimizations:
- Charts use **5-minute sliding window**
- UI redraw throttling:
  - Charts: ~10–12 FPS
  - Attitude: ~30 FPS
  - Map: ~5–6 FPS
- Packet processing is NOT throttled
  - all data still logged + buffered
- Map rendering decoupled from telemetry rate
- Matplotlib redraw minimized

👉 Result: smooth UI even at high telemetry rates

---

## 📦 Installation

### 1. Python
Requires:
- Python 3.9+

---

### 2. Install dependencies

```bash
pip install -r requirements.txt
```

---

## ▶️ Running

```bash
python rocket_telemetry_tk_v8_5min_window.py
```

---

## 🧭 Usage Guide

### 1. Select Source
- Simulator → test UI
- Serial → live hardware
- Playback → replay logs

---

### 2. Serial Mode

Configure:
- COM port
- Baud rate (typically 115200)

Click **Connect**

#### Expected data format:
Each line must be JSON:

```json
{"ts":50.0,"roll":1.2,"pitch":10.5,"yaw":0.3,"alt_rel_m":5.2,"lat":28.33,"lon":-80.61,"N":0.1,"E":0.2}
```

Supported:
- `alt_m` or `alt_rel_m`
- `yaw` or `yawNav`
- speed auto-calculated from N/E if missing

---

### 3. Playback Mode

Steps:
1. Click **Open Log**
2. Select CSV
3. Click **Connect**

Controls:
- Pause / Resume
- Timeline slider
- Speed control

---

### 4. Map Controls

| Action | Control |
|------|--------|
| Zoom | Mouse wheel / buttons |
| Pan | Click + drag |
| Recenter | Button |
| Track | Full buffer |

---

### 5. Charts

- Displays last **5 minutes only**
- Prevents performance degradation
- Full data still available in:
  - CSV logs
  - Save Buffer
  - Playback

---

### 6. Save Buffer

Click:
```
Save Buffer
```

Exports entire in-memory dataset.

---

## 📁 CSV Format

```
ts
roll_raw, pitch_raw, yaw_raw, alt_m_raw
roll_tared, pitch_tared, yaw_tared, alt_m_tared
spd_mps
lat, lon
rssi_dbm
rate_hz
server_rx_ts
```

---

## ⚠️ Troubleshooting

### No data updating:
- Check baud rate
- Check JSON formatting
- Verify newline-delimited packets

### Map not loading:
- Internet issue OR tile server blocked
- Fallback grid will still render

### UI lag:
- Should not happen unless extreme data rates
- Charts are already throttled

---

## 🚀 Recommended Workflow

1. Run **Simulator**
2. Verify UI behavior
3. Switch to **Serial**
4. Log a full session
5. Analyze using **Playback**
6. Scrub + inspect flight behavior

---

## 🧠 Future Expansion Ideas

- Flight event detection (liftoff, apogee)
- Auto markers on charts/map
- Multi-vehicle tracking
- 3D trajectory visualization
- Network telemetry (UDP/WebSocket)
- GPU rendering

---

## 👌 Summary

This is a **flight-grade telemetry console**:

- Fast
- Stable over long runs
- Designed for analysis, not just display
- Handles real-world data rates

No browser. No lag. No nonsense.

