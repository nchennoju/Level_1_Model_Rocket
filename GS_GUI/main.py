import csv
import json
import math
import os
import queue
import random
import threading
import time
from dataclasses import dataclass
from io import BytesIO
from typing import Dict, List, Optional, Tuple
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from PIL import Image, ImageTk, ImageDraw
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    import requests
except Exception:
    requests = None

try:
    import serial
    from serial.tools import list_ports
except Exception:
    serial = None
    list_ports = None


APP_TITLE = 'Rocket Telemetry Console'
MAX_POINTS = 1000
CHART_WINDOW_SEC = 300.0  # live chart sliding window: last 5 minutes
DEFAULT_SIM_HZ = 20.0


THEMES = {
    'dark': {
        'bg': '#0b0f17',
        'fg': '#e7eef9',
        'muted': '#9bb0c9',
        'panel': '#121826',
        'tile': '#0f1421',
        'accent': '#4cc9f0',
        'green': '#30e88f',
        'red': '#ff5e57',
        'warn': '#ffd166',
        'horizon_bg': '#0c1220',
        'sky': '#2a6f97',
        'ground': '#aa7f39',
        'line': '#e7eef9',
        'grid': '#1e2a3c',
    },
    'light': {
        'bg': '#f6f8fc',
        'fg': '#0c1220',
        'muted': '#51627a',
        'panel': '#ffffff',
        'tile': '#f1f4f9',
        'accent': '#1e88e5',
        'green': '#30e88f',
        'red': '#ff5e57',
        'warn': '#d99a00',
        'horizon_bg': '#eef3fb',
        'sky': '#9bd0ff',
        'ground': '#e4c8a0',
        'line': '#223344',
        'grid': '#d4deeb',
    }
}


@dataclass
class TelemetryPacket:
    ts: float
    roll: float
    pitch: float
    yaw: float
    alt_m: float
    spd_mps: float
    lat: float
    lon: float
    rssi_dbm: float
    rate_hz: float = 0.0
    server_rx_ts: float = 0.0


class TelemetrySource:
    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError


class SimulatorSource(TelemetrySource):
    def __init__(self, callback, hz=DEFAULT_SIM_HZ):
        self.callback = callback
        self.hz = hz
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        t0 = time.time()
        lat0 = 28.485
        lon0 = -80.544
        alt0 = 10.0
        while not self._stop.is_set():
            t = time.time() - t0
            alt = max(0.0, alt0 + 5*t + 0.4*(t**2) - 0.025*(t**3))
            spd = max(0.0, 5 + 0.6*t - 0.015*(t**2))
            pkt = TelemetryPacket(
                ts=time.time(),
                roll=20 * math.sin(0.7 * t),
                pitch=10 * math.sin(0.4 * t + 1.0),
                yaw=(12 * t) % 360,
                alt_m=alt,
                spd_mps=spd,
                lat=lat0 + 0.0001 * math.sin(0.05 * t),
                lon=lon0 + 0.0001 * math.cos(0.05 * t),
                rssi_dbm=-55 - 10 * math.log10(1 + 0.2 * t) + random.uniform(-1.5, 1.5),
            )
            self.callback(pkt)
            time.sleep(max(0.001, 1.0 / self.hz))


class SerialSource(TelemetrySource):
    def __init__(self, callback, port: str, baudrate: int):
        self.callback = callback
        self.port = port
        self.baudrate = baudrate
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if serial is None:
            raise RuntimeError('pyserial is not installed. Install it with: pip install pyserial')
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        ser = None
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            buf = b''
            while not self._stop.is_set():
                chunk = ser.read(4096)
                if not chunk:
                    time.sleep(0.01)
                    continue
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        obj = json.loads(line.decode('utf-8'))

                        alt = float(obj.get('alt_m', obj.get('alt_rel_m', 0.0)))
                        yaw = float(obj.get('yaw', obj.get('yawNav', 0.0)))

                        if 'spd_mps' in obj:
                            spd = float(obj['spd_mps'])
                        else:
                            n = float(obj.get('N', 0.0))
                            e = float(obj.get('E', 0.0))
                            spd = math.sqrt(n * n + e * e)

                        pkt = TelemetryPacket(
                            ts=float(obj.get('ts', time.time())),
                            roll=float(obj.get('roll', 0.0)),
                            pitch=float(obj.get('pitch', 0.0)),
                            yaw=yaw,
                            alt_m=alt,
                            spd_mps=spd,
                            lat=float(obj.get('lat', 0.0)),
                            lon=float(obj.get('lon', 0.0)),
                            rssi_dbm=float(obj.get('rssi_dbm', -60.0)),
                        )
                        self.callback(pkt)
                    except Exception:
                        # Bad line? Yeet it and keep going.
                        continue
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass




class PlaybackSource(TelemetrySource):
    def __init__(self, callback, filepath: str, speed: float = 1.0):
        self.callback = callback
        self.filepath = filepath
        self.speed = max(0.01, float(speed))
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if not self.filepath:
            raise RuntimeError('Pick a playback CSV file first.')
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _row_float(self, row, *keys, default=0.0):
        for key in keys:
            if key in row and row[key] not in (None, ''):
                try:
                    return float(row[key])
                except Exception:
                    pass
        return float(default)

    def _run(self):
        with open(self.filepath, 'r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            prev_ts = None
            for row in reader:
                if self._stop.is_set():
                    break
                ts = self._row_float(row, 'ts', default=time.time())
                pkt = TelemetryPacket(
                    ts=ts,
                    roll=self._row_float(row, 'roll_raw', 'roll', default=0.0),
                    pitch=self._row_float(row, 'pitch_raw', 'pitch', default=0.0),
                    yaw=self._row_float(row, 'yaw_raw', 'yaw', default=0.0),
                    alt_m=self._row_float(row, 'alt_m_raw', 'alt_raw', 'alt_m', 'alt_rel_m', default=0.0),
                    spd_mps=self._row_float(row, 'spd_mps', default=0.0),
                    lat=self._row_float(row, 'lat', default=0.0),
                    lon=self._row_float(row, 'lon', default=0.0),
                    rssi_dbm=self._row_float(row, 'rssi_dbm', default=-60.0),
                )
                if prev_ts is not None:
                    dt = max(0.0, ts - prev_ts)
                    time.sleep(min(1.0, dt / self.speed))
                prev_ts = ts
                self.callback(pkt)

class TileMapCanvas(tk.Canvas):
    TILE_SIZE = 256

    def __init__(self, master, theme_getter, *args, **kwargs):
        super().__init__(master, *args, highlightthickness=0, **kwargs)
        self.theme_getter = theme_getter
        self.map_style = 'street'
        self.track: List[Tuple[float, float]] = []
        self.center: Optional[Tuple[float, float]] = None
        self.zoom = 14
        self.marker = None
        self.follow_latest = True
        self._drag_start = None
        self._drag_center = None
        self._photo = None
        self._last_fetch_key = None
        self._cache: Dict[Tuple[str, int, int, int], Image.Image] = {}
        self._fetch_lock = threading.Lock()
        self.bind('<Configure>', lambda e: self.redraw())
        self.bind('<MouseWheel>', self._on_mousewheel)      # Windows/macOS
        self.bind('<Button-4>', self._on_mousewheel)        # Linux scroll up
        self.bind('<Button-5>', self._on_mousewheel)        # Linux scroll down
        self.bind('<ButtonPress-1>', self._on_drag_start)
        self.bind('<B1-Motion>', self._on_drag_move)

    def set_style(self, style: str):
        self.map_style = style
        self.redraw(force=True)

    def set_track(self, points: List[Tuple[float, float]], follow_latest: bool = True):
        self.track = [(float(lat), float(lon)) for lat, lon in points if lat is not None and lon is not None]
        self.follow_latest = follow_latest
        if self.track and (follow_latest or self.center is None):
            self.center = self.track[-1]
        self.redraw(force=True)

    def add_point(self, lat: float, lon: float):
        self.track.append((lat, lon))
        if self.follow_latest or self.center is None:
            self.center = (lat, lon)
        self.redraw()

    def zoom_in(self):
        self.zoom = min(19, self.zoom + 1)
        self.redraw(force=True)

    def zoom_out(self):
        self.zoom = max(2, self.zoom - 1)
        self.redraw(force=True)

    def recenter_latest(self):
        if self.track:
            self.follow_latest = True
            self.center = self.track[-1]
            self.redraw(force=True)

    def _on_mousewheel(self, event):
        if getattr(event, 'num', None) == 4 or getattr(event, 'delta', 0) > 0:
            self.zoom_in()
        elif getattr(event, 'num', None) == 5 or getattr(event, 'delta', 0) < 0:
            self.zoom_out()

    def _on_drag_start(self, event):
        if self.center is None:
            return
        self.follow_latest = False
        self._drag_start = (event.x, event.y)
        self._drag_center = self.center

    def _on_drag_move(self, event):
        if self._drag_start is None or self._drag_center is None:
            return
        dx = event.x - self._drag_start[0]
        dy = event.y - self._drag_start[1]
        cx, cy = self._deg2num(self._drag_center[0], self._drag_center[1], self.zoom)
        new_x = cx - dx / self.TILE_SIZE
        new_y = cy - dy / self.TILE_SIZE
        self.center = self._num2deg(new_x, new_y, self.zoom)
        self.redraw(force=True)

    def redraw(self, force: bool = False):
        w = max(10, self.winfo_width())
        h = max(10, self.winfo_height())
        self.delete('all')
        theme = THEMES[self.theme_getter()]
        self.configure(bg=theme['horizon_bg'])
        if not self.center:
            self.create_text(w/2, h/2, text='Waiting for coordinates…', fill=theme['muted'], font=('Segoe UI', 14, 'bold'))
            return

        key = (self.map_style, round(self.center[0], 5), round(self.center[1], 5), self.zoom, w, h)
        if force or self._last_fetch_key != key:
            self._last_fetch_key = key
            if requests is not None:
                threading.Thread(target=self._fetch_and_draw, args=(key,), daemon=True).start()
            else:
                self._draw_fallback_grid(w, h)
        else:
            self._draw_from_last()

    def _draw_from_last(self):
        if self._photo is not None:
            self.create_image(0, 0, anchor='nw', image=self._photo)
            self._overlay_track()
        else:
            self._draw_fallback_grid(self.winfo_width(), self.winfo_height())

    def _fetch_and_draw(self, key):
        if self.center is None:
            return
        with self._fetch_lock:
            try:
                img = self._compose_tiles(*key[:-2], width=key[-2], height=key[-1])
                photo = ImageTk.PhotoImage(img)
                def apply():
                    self._photo = photo
                    self.delete('all')
                    self.create_image(0, 0, anchor='nw', image=self._photo)
                    self._overlay_track()
                self.after(0, apply)
            except Exception:
                self.after(0, lambda: self._draw_fallback_grid(self.winfo_width(), self.winfo_height()))

    def _draw_fallback_grid(self, w, h):
        theme = THEMES[self.theme_getter()]
        self.create_rectangle(0, 0, w, h, fill=theme['horizon_bg'], outline='')
        for x in range(0, w, 40):
            self.create_line(x, 0, x, h, fill=theme['grid'])
        for y in range(0, h, 40):
            self.create_line(0, y, w, y, fill=theme['grid'])
        self.create_text(10, 10, anchor='nw', text=f'{self.map_style.title()} map fallback', fill=theme['muted'], font=('Segoe UI', 10, 'bold'))
        self._overlay_track(normalized=True)

    def _overlay_track(self, normalized: bool = False):
        if not self.track:
            return
        theme = THEMES[self.theme_getter()]
        w = max(10, self.winfo_width())
        h = max(10, self.winfo_height())
        points = self.track
        if len(points) > 10000:
            step = max(1, len(points) // 10000)
            points = points[::step]
            if points[-1] != self.track[-1]:
                points.append(self.track[-1])
        if normalized or self.center is None:
            lats = [p[0] for p in points]
            lons = [p[1] for p in points]
            lat_min, lat_max = min(lats), max(lats)
            lon_min, lon_max = min(lons), max(lons)
            lat_span = max(1e-6, lat_max - lat_min)
            lon_span = max(1e-6, lon_max - lon_min)
            xy = [((lon - lon_min) / lon_span * (w - 40) + 20,
                   h - (((lat - lat_min) / lat_span * (h - 40)) + 20)) for lat, lon in points]
        else:
            xy = [self._latlon_to_canvas(lat, lon) for lat, lon in points]
        flat = [c for pt in xy for c in pt]
        if len(flat) >= 4:
            self.create_line(*flat, fill=theme['accent'], width=3, smooth=True)
        x, y = xy[-1]
        self.create_oval(x-6, y-6, x+6, y+6, fill=theme['red'], outline='white')

    def _compose_tiles(self, style: str, lat: float, lon: float, zoom: int, width: int, height: int) -> Image.Image:
        cx, cy = self._deg2num(lat, lon, zoom)
        tile_x = int(cx)
        tile_y = int(cy)
        frac_x = cx - tile_x
        frac_y = cy - tile_y
        cols = math.ceil(width / self.TILE_SIZE) + 2
        rows = math.ceil(height / self.TILE_SIZE) + 2
        canvas = Image.new('RGB', (cols * self.TILE_SIZE, rows * self.TILE_SIZE))
        start_x = tile_x - cols // 2
        start_y = tile_y - rows // 2
        for ix in range(cols):
            for iy in range(rows):
                tx = start_x + ix
                ty = start_y + iy
                tile = self._get_tile(style, zoom, tx, ty)
                canvas.paste(tile, (ix * self.TILE_SIZE, iy * self.TILE_SIZE))
        offset_x = int((cols // 2 + frac_x) * self.TILE_SIZE - width / 2)
        offset_y = int((rows // 2 + frac_y) * self.TILE_SIZE - height / 2)
        crop = canvas.crop((offset_x, offset_y, offset_x + width, offset_y + height))
        draw = ImageDraw.Draw(crop)
        draw.rectangle((8, 8, 175, 30), fill=(0, 0, 0, 110))
        draw.text((14, 12), f'{style.title()} map', fill='white')
        return crop

    def _get_tile(self, style: str, zoom: int, x: int, y: int) -> Image.Image:
        key = (style, zoom, x, y)
        if key in self._cache:
            return self._cache[key]
        max_tile = 2 ** zoom
        x = x % max_tile
        y = max(0, min(max_tile - 1, y))
        if style == 'satellite':
            url = f'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{zoom}/{y}/{x}'
        else:
            url = f'https://tile.openstreetmap.org/{zoom}/{x}/{y}.png'
        resp = requests.get(url, timeout=4, headers={'User-Agent': 'RocketTelemetryTk/1.0'})
        resp.raise_for_status()
        img = Image.open(BytesIO(resp.content)).convert('RGB')
        self._cache[key] = img
        if len(self._cache) > 200:
            self._cache.pop(next(iter(self._cache)))
        return img

    def _latlon_to_canvas(self, lat: float, lon: float) -> Tuple[float, float]:
        if self.center is None:
            return 0.0, 0.0
        w = max(10, self.winfo_width())
        h = max(10, self.winfo_height())
        center_x, center_y = self._deg2num(self.center[0], self.center[1], self.zoom)
        px, py = self._deg2num(lat, lon, self.zoom)
        dx = (px - center_x) * self.TILE_SIZE
        dy = (py - center_y) * self.TILE_SIZE
        return w / 2 + dx, h / 2 + dy

    @staticmethod
    def _deg2num(lat_deg: float, lon_deg: float, zoom: int) -> Tuple[float, float]:
        lat_rad = math.radians(max(-85.0511, min(85.0511, lat_deg)))
        n = 2.0 ** zoom
        xtile = (lon_deg + 180.0) / 360.0 * n
        ytile = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
        return xtile, ytile

    @staticmethod
    def _num2deg(xtile: float, ytile: float, zoom: int) -> Tuple[float, float]:
        n = 2.0 ** zoom
        lon_deg = xtile / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * ytile / n)))
        lat_deg = math.degrees(lat_rad)
        return lat_deg, lon_deg


class TelemetryApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry('1600x980')
        self.minsize(1280, 760)

        self.theme_name = 'dark'
        self.rx_queue: queue.Queue = queue.Queue(maxsize=5000)
        self.source: Optional[TelemetrySource] = None
        self.connected = False
        self.start_time = time.time()
        self.stream_t0 = None
        self.last_pkt_ts = None
        self.rate_ema = 0.0
        self.alt_x: List[float] = []
        self.alt_y: List[float] = []
        self.spd_x: List[float] = []
        self.spd_y: List[float] = []
        self.current = TelemetryPacket(time.time(), 0, 0, 0, 0, 0, 28.485, -80.544, -60)
        self.last_tile_style = 'street'
        self.tare_active = False
        self.tare_offsets = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'alt_m': 0.0}
        self.playback_file_var = tk.StringVar()
        self.playback_speed_var = tk.StringVar(value='1.0')
        self.csv_file = None
        self.csv_writer = None
        self.log_path = None

        self.buffer_rows: List[Dict[str, float]] = []
        self.playback_packets: List[TelemetryPacket] = []
        self.playback_index = 0
        self.playback_paused = False
        self.playback_after_id = None
        self.playback_active = False

        # UI performance throttles.
        # Telemetry/logging still processes every packet; only expensive drawings are rate-limited.
        self.last_readout_draw = 0.0
        self.last_chart_draw = 0.0
        self.last_attitude_draw = 0.0
        self.last_map_draw = 0.0
        self.readout_draw_period = 1.0 / 30.0   # labels/tiles
        self.chart_draw_period = 1.0 / 12.0     # matplotlib is expensive
        self.attitude_draw_period = 1.0 / 30.0  # canvases are cheaper
        self.map_draw_period = 1.0 / 6.0        # tile/map overlays are expensive

        self._build_style()
        self._build_ui()
        self._update_theme()
        self.after(100, self._poll_rx_queue)
        self.after(250, self._tick_clock)
        self.after(1500, self.refresh_ports)
        self.protocol('WM_DELETE_WINDOW', self.on_close)

    def _build_style(self):
        self.style = ttk.Style(self)
        self.style.theme_use('clam')

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        self.topbar = ttk.Frame(self, padding=(14, 10))
        self.topbar.grid(row=0, column=0, sticky='ew')
        self.topbar.columnconfigure(1, weight=1)

        self.brand = ttk.Label(self.topbar, text='🚀 Rocket Dashboard', font=('Segoe UI', 16, 'bold'))
        self.brand.grid(row=0, column=0, sticky='w')

        controls = ttk.Frame(self.topbar)
        controls.grid(row=0, column=2, sticky='e')

        ttk.Label(controls, text='Source').grid(row=0, column=0, padx=(0, 6))
        self.source_var = tk.StringVar(value='Simulator')
        self.source_combo = ttk.Combobox(controls, width=11, textvariable=self.source_var, state='readonly', values=['Simulator', 'Serial', 'Playback'])
        self.source_combo.grid(row=0, column=1, padx=(0, 8))
        self.source_combo.bind('<<ComboboxSelected>>', lambda e: self._toggle_source_fields())

        ttk.Label(controls, text='Port').grid(row=0, column=2, padx=(0, 6))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(controls, width=12, textvariable=self.port_var)
        self.port_combo.grid(row=0, column=3, padx=(0, 8))

        ttk.Label(controls, text='Baud').grid(row=0, column=4, padx=(0, 6))
        self.baud_var = tk.StringVar(value='115200')
        self.baud_entry = ttk.Entry(controls, width=9, textvariable=self.baud_var)
        self.baud_entry.grid(row=0, column=5, padx=(0, 8))

        self.refresh_btn = ttk.Button(controls, text='Refresh Ports', command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=6, padx=(0, 8))

        self.playback_btn = ttk.Button(controls, text='Open Log', command=self.select_playback_file)
        self.playback_btn.grid(row=0, column=7, padx=(0, 8))

        self.playback_file_label = ttk.Label(controls, text='No log loaded')
        self.playback_file_label.grid(row=0, column=8, padx=(0, 8))

        ttk.Label(controls, text='Replay x').grid(row=0, column=9, padx=(0, 6))
        self.playback_speed_entry = ttk.Entry(controls, width=6, textvariable=self.playback_speed_var)
        self.playback_speed_entry.grid(row=0, column=10, padx=(0, 8))

        ttk.Label(controls, text='Map').grid(row=0, column=11, padx=(0, 6))
        self.map_style_var = tk.StringVar(value='street')
        self.map_style_combo = ttk.Combobox(controls, width=10, textvariable=self.map_style_var, state='readonly', values=['street', 'satellite'])
        self.map_style_combo.grid(row=0, column=12, padx=(0, 8))
        self.map_style_combo.bind('<<ComboboxSelected>>', lambda e: self.map_canvas.set_style(self.map_style_var.get()))

        self.theme_btn = ttk.Button(controls, text='🌙', width=4, command=self.toggle_theme)
        self.theme_btn.grid(row=0, column=13, padx=(0, 10))

        self.conn_canvas = tk.Canvas(controls, width=14, height=14, highlightthickness=0, bd=0)
        self.conn_canvas.grid(row=0, column=14, padx=(0, 4))
        self.conn_dot = self.conn_canvas.create_oval(2, 2, 12, 12, outline='', fill='red')

        self.conn_label = ttk.Label(controls, text='Disconnected')
        self.conn_label.grid(row=0, column=15, padx=(0, 12))

        self.mission_label = ttk.Label(controls, text='T+ 00:00:00')
        self.mission_label.grid(row=0, column=16, padx=(0, 12))

        self.connect_btn = ttk.Button(controls, text='Connect', command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=17, padx=(0, 8))

        self.tare_btn = ttk.Button(controls, text='Tare IMU+Alt', command=self.tare_current)
        self.tare_btn.grid(row=0, column=18, padx=(0, 8))

        self.reset_tare_btn = ttk.Button(controls, text='Reset Tare', command=self.reset_tare)
        self.reset_tare_btn.grid(row=0, column=19, padx=(0, 8))

        self.save_buffer_btn = ttk.Button(controls, text='Save Buffer', command=self.save_buffer)
        self.save_buffer_btn.grid(row=0, column=20, padx=(0, 8))

        self.play_pause_btn = ttk.Button(controls, text='Pause', command=self.toggle_playback_pause, state='disabled')
        self.play_pause_btn.grid(row=0, column=21)

        ttk.Label(controls, text='Timeline').grid(row=1, column=0, padx=(0, 6), pady=(8, 0), sticky='w')
        self.playback_pos_var = tk.IntVar(value=0)
        self.playback_scrubber = ttk.Scale(
            controls,
            from_=0,
            to=0,
            orient='horizontal',
            command=self.on_scrub_preview
        )
        self.playback_scrubber.grid(row=1, column=1, columnspan=18, sticky='ew', pady=(8, 0))
        self.playback_scrubber.bind('<ButtonPress-1>', self.on_scrub_start)
        self.playback_scrubber.bind('<ButtonRelease-1>', self.on_scrub_release)

        self.playback_status_label = ttk.Label(controls, text='No playback loaded')
        self.playback_status_label.grid(row=1, column=19, columnspan=3, padx=(8, 0), pady=(8, 0), sticky='w')
        controls.columnconfigure(10, weight=1)

        main = ttk.Frame(self, padding=(12, 0, 12, 12))
        main.grid(row=1, column=0, sticky='nsew')
        main.columnconfigure(0, weight=14)
        main.columnconfigure(1, weight=13)
        main.columnconfigure(2, weight=11)
        main.rowconfigure(0, weight=1)
        main.rowconfigure(1, weight=0)

        self.map_panel = self._panel(main, 'Position')
        self.map_panel.grid(row=0, column=0, sticky='nsew', padx=(0, 12), pady=(0, 12))
        self.map_panel.rowconfigure(2, weight=1)
        self.map_panel.columnconfigure(0, weight=1)

        map_tools = ttk.Frame(self.map_panel, style='Panel.TFrame')
        map_tools.grid(row=1, column=0, sticky='ew', pady=(0, 8))
        ttk.Button(map_tools, text='Zoom +', command=lambda: self.map_canvas.zoom_in()).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(map_tools, text='Zoom -', command=lambda: self.map_canvas.zoom_out()).grid(row=0, column=1, padx=(0, 6))
        ttk.Button(map_tools, text='Recenter', command=lambda: self.map_canvas.recenter_latest()).grid(row=0, column=2, padx=(0, 6))
        ttk.Label(map_tools, text='Drag map to pan • mouse wheel to zoom', style='PanelTitle.TLabel').grid(row=0, column=3, sticky='w')

        self.map_canvas = TileMapCanvas(self.map_panel, lambda: self.theme_name)
        self.map_canvas.grid(row=2, column=0, sticky='nsew')

        self.charts_panel = self._panel(main, 'Altitude & Speed - Last 5 min')
        self.charts_panel.grid(row=0, column=1, sticky='nsew', padx=(0, 12), pady=(0, 12))
        self.charts_panel.rowconfigure(1, weight=1)
        self.charts_panel.rowconfigure(2, weight=1)
        self.charts_panel.columnconfigure(0, weight=1)
        self.alt_fig, self.alt_ax, self.alt_canvas = self._create_chart(self.charts_panel)
        self.alt_canvas.get_tk_widget().grid(row=1, column=0, sticky='nsew', pady=(0, 10))
        self.spd_fig, self.spd_ax, self.spd_canvas = self._create_chart(self.charts_panel)
        self.spd_canvas.get_tk_widget().grid(row=2, column=0, sticky='nsew')

        self.att_panel = self._panel(main, 'Attitude')
        self.att_panel.grid(row=0, column=2, sticky='nsew', pady=(0, 12))
        self.att_panel.columnconfigure(0, weight=1)
        self.att_panel.rowconfigure(1, weight=1)

        self.att3d = tk.Canvas(self.att_panel, height=240, highlightthickness=0)
        self.att3d.grid(row=1, column=0, sticky='nsew', pady=(0, 10))
        ttk.Label(self.att_panel, text='Attitude Indicator', style='PanelTitle.TLabel').grid(row=2, column=0, sticky='w', pady=(6, 6))
        self.horizon = tk.Canvas(self.att_panel, height=190, highlightthickness=0)
        self.horizon.grid(row=3, column=0, sticky='ew')

        readouts = ttk.Frame(self.att_panel)
        readouts.grid(row=4, column=0, sticky='ew', pady=(8, 0))
        self.roll_var = tk.StringVar(value='0.0°')
        self.pitch_var = tk.StringVar(value='0.0°')
        self.yaw_var = tk.StringVar(value='0.0°')
        ttk.Label(readouts, text='Roll:').grid(row=0, column=0, sticky='w')
        ttk.Label(readouts, textvariable=self.roll_var).grid(row=0, column=1, sticky='w', padx=(4, 14))
        ttk.Label(readouts, text='Pitch:').grid(row=0, column=2, sticky='w')
        ttk.Label(readouts, textvariable=self.pitch_var).grid(row=0, column=3, sticky='w', padx=(4, 14))
        ttk.Label(readouts, text='Yaw:').grid(row=0, column=4, sticky='w')
        ttk.Label(readouts, textvariable=self.yaw_var).grid(row=0, column=5, sticky='w', padx=(4, 0))

        self.tiles = ttk.Frame(main)
        self.tiles.grid(row=1, column=0, columnspan=3, sticky='ew')
        for i in range(4):
            self.tiles.columnconfigure(i, weight=1)
        self.rate_tile = self._metric_tile(self.tiles, 'Rate', '0.00 Hz', 0)
        self.rssi_tile = self._metric_tile(self.tiles, 'RSSI', '- dBm', 1)
        self.coords_tile = self._metric_tile(self.tiles, 'Coords', '--', 2)
        self.age_tile = self._metric_tile(self.tiles, 'Pkt Age', '-- ms', 3)

        self._toggle_source_fields()

    def _panel(self, parent, title: str):
        frame = ttk.Frame(parent, padding=12, style='Panel.TFrame')
        ttk.Label(frame, text=title, style='PanelTitle.TLabel').grid(row=0, column=0, sticky='w', pady=(0, 8))
        return frame

    def _metric_tile(self, parent, label: str, value: str, column: int):
        tile = ttk.Frame(parent, padding=12, style='Tile.TFrame')
        tile.grid(row=0, column=column, sticky='ew', padx=(0 if column == 0 else 12, 0))
        lbl1 = ttk.Label(tile, text=label, style='TileLabel.TLabel')
        lbl1.grid(row=0, column=0, sticky='w')
        lbl2_var = tk.StringVar(value=value)
        lbl2 = ttk.Label(tile, textvariable=lbl2_var, style='TileValue.TLabel')
        lbl2.grid(row=1, column=0, sticky='w', pady=(6, 0))
        return lbl2_var

    def _create_chart(self, parent):
        fig = Figure(figsize=(5, 2.4), dpi=100)
        ax = fig.add_subplot(111)
        canvas = FigureCanvasTkAgg(fig, master=parent)
        return fig, ax, canvas

    def _toggle_source_fields(self):
        mode = self.source_var.get()
        serial_state = 'normal' if mode == 'Serial' else 'disabled'
        playback_state = 'normal' if mode == 'Playback' else 'disabled'
        self.port_combo.configure(state=serial_state)
        self.baud_entry.configure(state=serial_state)
        self.refresh_btn.configure(state=serial_state)
        self.playback_btn.configure(state=playback_state)
        self.playback_speed_entry.configure(state=playback_state)

    def refresh_ports(self):
        ports = []
        if list_ports is not None:
            try:
                ports = [p.device for p in list_ports.comports()]
            except Exception:
                ports = []
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def select_playback_file(self):
        path = filedialog.askopenfilename(
            title='Select telemetry log CSV',
            filetypes=[('CSV files', '*.csv'), ('All files', '*.*')]
        )
        if path:
            self.playback_file_var.set(path)
            self.playback_file_label.configure(text=os.path.basename(path))
            try:
                self.load_playback_file(path)
            except Exception as exc:
                messagebox.showerror('Playback load failed', str(exc))

    def _row_float(self, row, *keys, default=0.0):
        for key in keys:
            if key in row and row[key] not in (None, ''):
                try:
                    return float(row[key])
                except Exception:
                    pass
        return float(default)

    def _packet_from_csv_row(self, row) -> TelemetryPacket:
        return TelemetryPacket(
            ts=self._row_float(row, 'ts', default=time.time()),
            roll=self._row_float(row, 'roll_raw', 'roll', default=0.0),
            pitch=self._row_float(row, 'pitch_raw', 'pitch', default=0.0),
            yaw=self._row_float(row, 'yaw_raw', 'yaw', 'yawNav', default=0.0),
            alt_m=self._row_float(row, 'alt_m_raw', 'alt_raw', 'alt_m', 'alt_rel_m', default=0.0),
            spd_mps=self._row_float(row, 'spd_mps', default=0.0),
            lat=self._row_float(row, 'lat', default=0.0),
            lon=self._row_float(row, 'lon', default=0.0),
            rssi_dbm=self._row_float(row, 'rssi_dbm', default=-60.0),
        )

    def load_playback_file(self, path: str):
        packets = []
        with open(path, 'r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                packets.append(self._packet_from_csv_row(row))
        if not packets:
            raise RuntimeError('Selected CSV has no telemetry rows.')
        self.playback_packets = packets
        self.playback_index = 0
        self.playback_scrubber.configure(from_=0, to=max(0, len(packets) - 1))
        self.playback_scrubber.set(0)
        self.playback_status_label.configure(text=f'Loaded {len(packets)} packets')
        self.play_pause_btn.configure(state='disabled', text='Pause')

    def _start_logging(self):
        self._stop_logging()
        log_dir = os.path.join(os.getcwd(), 'telemetry_logs')
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, datetime.now().strftime('telemetry_%Y%m%d_%H%M%S.csv'))
        self.csv_file = open(self.log_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'ts', 'roll_raw', 'pitch_raw', 'yaw_raw', 'alt_m_raw',
            'roll_tared', 'pitch_tared', 'yaw_tared', 'alt_m_tared',
            'spd_mps', 'lat', 'lon', 'rssi_dbm', 'rate_hz', 'server_rx_ts'
        ])
        self.csv_file.flush()

    def _stop_logging(self):
        if self.csv_file is not None:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass
        self.csv_file = None
        self.csv_writer = None

    def toggle_theme(self):
        self.theme_name = 'light' if self.theme_name == 'dark' else 'dark'
        self._update_theme()

    def _update_theme(self):
        t = THEMES[self.theme_name]
        self.configure(bg=t['bg'])
        self.style.configure('TFrame', background=t['bg'])
        self.style.configure('TLabel', background=t['bg'], foreground=t['fg'], font=('Segoe UI', 10))
        self.style.configure('Panel.TFrame', background=t['panel'])
        self.style.configure('Tile.TFrame', background=t['tile'])
        self.style.configure('PanelTitle.TLabel', background=t['panel'], foreground=t['muted'], font=('Segoe UI', 10, 'bold'))
        self.style.configure('TileLabel.TLabel', background=t['tile'], foreground=t['muted'], font=('Segoe UI', 10, 'bold'))
        self.style.configure('TileValue.TLabel', background=t['tile'], foreground=t['fg'], font=('Segoe UI', 18, 'bold'))
        self.style.configure('TButton', background=t['tile'], foreground=t['fg'])
        self.style.map('TButton', background=[('active', t['panel'])])
        self.style.configure('TEntry', fieldbackground=t['tile'], foreground=t['fg'])
        self.style.configure('TCombobox', fieldbackground=t['tile'], background=t['tile'], foreground=t['fg'])

        self.topbar.configure(style='Panel.TFrame')
        self.brand.configure(background=t['panel'], foreground=t['fg'])
        self.theme_btn.configure(text='☀️' if self.theme_name == 'light' else '🌙')
        self.conn_canvas.configure(bg=t['panel'])
        self.conn_canvas.itemconfig(self.conn_dot, fill=t['green'] if self.connected else t['red'])
        self._style_chart(self.alt_fig, self.alt_ax, 'Altitude (m)')
        self._style_chart(self.spd_fig, self.spd_ax, 'Speed (m/s)')
        self._redraw_attitude(self.current.roll, self.current.pitch, self.current.yaw)
        self.map_canvas.redraw(force=True)

    def _style_chart(self, fig, ax, ylabel):
        t = THEMES[self.theme_name]
        fig.patch.set_facecolor(t['panel'])
        ax.set_facecolor(t['panel'])
        ax.tick_params(colors=t['muted'])
        for spine in ax.spines.values():
            spine.set_color(t['grid'])
        ax.grid(True, color=t['grid'])
        ax.set_xlabel('T+ s', color=t['muted'])
        ax.set_ylabel(ylabel, color=t['muted'])
        fig.tight_layout()

    def _tick_clock(self):
        dt = int(time.time() - self.start_time)
        h = dt // 3600
        m = (dt % 3600) // 60
        s = dt % 60
        self.mission_label.configure(text=f'T+ {h:02d}:{m:02d}:{s:02d}')
        self.after(250, self._tick_clock)

    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        try:
            mode = self.source_var.get()
            self._stop_playback_loop()

            if mode == 'Serial':
                port = self.port_var.get().strip()
                if not port:
                    raise RuntimeError('Pick a serial port first.')
                baud = int(self.baud_var.get().strip())
                self.source = SerialSource(self.enqueue_packet, port, baud)
                self._start_logging()
                self.source.start()
            elif mode == 'Playback':
                path = self.playback_file_var.get().strip()
                if not path:
                    raise RuntimeError('Pick a playback CSV file first.')
                if not self.playback_packets:
                    self.load_playback_file(path)
                self.source = None
                self._stop_logging()
            else:
                self.source = SimulatorSource(self.enqueue_packet)
                self._start_logging()
                self.source.start()

            self.connected = True
            self.conn_label.configure(text='Live')
            self.conn_canvas.itemconfig(self.conn_dot, fill=THEMES[self.theme_name]['green'])
            self.connect_btn.configure(text='Disconnect')
            self.start_time = time.time()
            self.stream_t0 = None
            self.clear_series()

            if mode == 'Playback':
                self.playback_index = int(float(self.playback_scrubber.get()))
                self.playback_paused = False
                self.playback_active = True
                self.play_pause_btn.configure(state='normal', text='Pause')
                self._playback_step()
            else:
                self.playback_active = False
                self.play_pause_btn.configure(state='disabled', text='Pause')

        except Exception as exc:
            messagebox.showerror('Connection failed', str(exc))
            self.connected = False
            self.playback_active = False
            self.play_pause_btn.configure(state='disabled', text='Pause')

    def disconnect(self):
        self._stop_playback_loop()
        if self.source is not None:
            try:
                self.source.stop()
            except Exception:
                pass
            self.source = None
        self._stop_logging()
        self.connected = False
        self.conn_label.configure(text='Disconnected')
        self.conn_canvas.itemconfig(self.conn_dot, fill=THEMES[self.theme_name]['red'])
        self.connect_btn.configure(text='Connect')
        self.play_pause_btn.configure(state='disabled', text='Pause')

    def tare_current(self):
        pkt = self.current
        self.tare_offsets['roll'] = float(pkt.roll)
        self.tare_offsets['pitch'] = float(pkt.pitch)
        self.tare_offsets['yaw'] = float(pkt.yaw)
        self.tare_offsets['alt_m'] = float(pkt.alt_m)
        self.tare_active = True
        if self.connected:
            self.conn_label.configure(text='Live (Tared)')
        else:
            self.conn_label.configure(text='Disconnected (Tared)')

    def reset_tare(self):
        self.tare_active = False
        self.tare_offsets = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'alt_m': 0.0}
        self.conn_label.configure(text='Live' if self.connected else 'Disconnected')

    def _stop_playback_loop(self):
        self.playback_active = False
        if self.playback_after_id is not None:
            try:
                self.after_cancel(self.playback_after_id)
            except Exception:
                pass
            self.playback_after_id = None

    def toggle_playback_pause(self):
        if self.source_var.get() != 'Playback' or not self.connected:
            return
        self.playback_paused = not self.playback_paused
        self.play_pause_btn.configure(text='Resume' if self.playback_paused else 'Pause')
        if not self.playback_paused and self.playback_active:
            self._playback_step()

    def on_scrub_start(self, event=None):
        if self.source_var.get() == 'Playback':
            self.playback_paused = True
            self.play_pause_btn.configure(text='Resume')

    def on_scrub_preview(self, value):
        if self.source_var.get() != 'Playback' or not self.playback_packets:
            return
        idx = int(float(value))
        idx = max(0, min(idx, len(self.playback_packets) - 1))
        self.playback_status_label.configure(text=f'Packet {idx + 1}/{len(self.playback_packets)}')

    def on_scrub_release(self, event=None):
        if self.source_var.get() != 'Playback' or not self.playback_packets:
            return
        idx = int(float(self.playback_scrubber.get()))
        self.seek_playback(idx)

    def seek_playback(self, index: int):
        if not self.playback_packets:
            return
        self._stop_playback_loop()
        self.playback_active = self.connected and self.source_var.get() == 'Playback'
        self.playback_index = max(0, min(index, len(self.playback_packets) - 1))
        self.clear_series()

        # Rebuild the view up to the selected packet so charts, track, and attitude match the scrubber.
        for i in range(self.playback_index + 1):
            pkt = self.playback_packets[i]
            pkt.rate_hz = 0.0
            pkt.server_rx_ts = time.time()
            self.current = pkt
            self._apply_packet(pkt, log=False, store_buffer=False)

        self._refresh_map_from_buffer(follow_latest=True)
        self.playback_scrubber.set(self.playback_index)
        self.playback_status_label.configure(text=f'Packet {self.playback_index + 1}/{len(self.playback_packets)}')
        if self.playback_active and not self.playback_paused:
            self._playback_step()

    def _playback_step(self):
        if not self.playback_active or self.playback_paused or not self.connected:
            return
        if self.playback_index >= len(self.playback_packets):
            self.playback_active = False
            self.play_pause_btn.configure(text='Resume')
            self.conn_label.configure(text='Playback complete')
            return

        pkt = self.playback_packets[self.playback_index]
        pkt.rate_hz = 0.0
        pkt.server_rx_ts = time.time()
        self.current = pkt
        self._apply_packet(pkt, log=False, store_buffer=True)
        self.playback_scrubber.set(self.playback_index)
        self.playback_status_label.configure(text=f'Packet {self.playback_index + 1}/{len(self.playback_packets)}')

        delay_ms = 50
        if self.playback_index + 1 < len(self.playback_packets):
            speed = max(0.01, float(self.playback_speed_var.get().strip() or '1.0'))
            dt = max(0.0, self.playback_packets[self.playback_index + 1].ts - pkt.ts)
            delay_ms = int(max(1, min(1000, (dt / speed) * 1000)))

        self.playback_index += 1
        self.playback_after_id = self.after(delay_ms, self._playback_step)

    def _refresh_map_from_buffer(self, follow_latest: bool = True):
        points = []
        for row in self.buffer_rows:
            try:
                lat = float(row.get('lat', 0.0))
                lon = float(row.get('lon', 0.0))
            except Exception:
                continue
            if lat or lon:
                points.append((lat, lon))
        self.map_canvas.set_track(points, follow_latest=follow_latest)

    def save_buffer(self):
        if not self.buffer_rows:
            messagebox.showinfo('Save Buffer', 'No buffered telemetry to save yet.')
            return
        default_name = datetime.now().strftime('telemetry_buffer_%Y%m%d_%H%M%S.csv')
        path = filedialog.asksaveasfilename(
            title='Save telemetry buffer',
            defaultextension='.csv',
            initialfile=default_name,
            filetypes=[('CSV files', '*.csv'), ('All files', '*.*')]
        )
        if not path:
            return
        fields = [
            'ts', 'roll_raw', 'pitch_raw', 'yaw_raw', 'alt_m_raw',
            'roll_tared', 'pitch_tared', 'yaw_tared', 'alt_m_tared',
            'spd_mps', 'lat', 'lon', 'rssi_dbm', 'rate_hz', 'server_rx_ts'
        ]
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.buffer_rows)
        messagebox.showinfo('Save Buffer', f'Saved {len(self.buffer_rows)} rows to:\\n{path}')

    def clear_series(self):
        self.alt_x.clear()
        self.alt_y.clear()
        self.spd_x.clear()
        self.spd_y.clear()
        self.map_canvas.set_track([], follow_latest=True)
        self.buffer_rows.clear()
        self.rate_ema = 0.0
        self.last_pkt_ts = None
        self.rate_tile.set('0.00 Hz')
        self.rssi_tile.set('- dBm')
        self.coords_tile.set('--')
        self.age_tile.set('-- ms')
        self.alt_ax.clear()
        self.spd_ax.clear()
        self._style_chart(self.alt_fig, self.alt_ax, 'Altitude (m)')
        self._style_chart(self.spd_fig, self.spd_ax, 'Speed (m/s)')
        self.alt_canvas.draw_idle()
        self.spd_canvas.draw_idle()
        self.last_readout_draw = 0.0
        self.last_chart_draw = 0.0
        self.last_attitude_draw = 0.0
        self.last_map_draw = 0.0
        self.map_canvas.redraw(force=True)

    def enqueue_packet(self, pkt: TelemetryPacket):
        now = time.time()
        if self.last_pkt_ts is not None:
            dt = max(1e-6, now - self.last_pkt_ts)
            inst = 1.0 / dt
            alpha = 1.0 - math.exp(-1.0 / (15.0 * max(DEFAULT_SIM_HZ, 1)))
            self.rate_ema = (1 - alpha) * self.rate_ema + alpha * inst
        pkt.rate_hz = self.rate_ema
        pkt.server_rx_ts = now
        self.last_pkt_ts = now
        try:
            self.rx_queue.put_nowait(pkt)
        except queue.Full:
            pass

    def _poll_rx_queue(self):
        processed = 0
        while processed < 250:
            try:
                pkt = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self.current = pkt
            try:
                self._apply_packet(pkt)
            except Exception as exc:
                # Keep the polling loop alive even if one UI draw step fails.
                try:
                    self.conn_label.configure(text=f'UI error: {exc}')
                except Exception:
                    pass
            processed += 1
        self.after(15, self._poll_rx_queue)

    def _apply_packet(self, pkt: TelemetryPacket, log: bool = True, store_buffer: bool = True):
        disp_roll = pkt.roll - self.tare_offsets['roll'] if self.tare_active else pkt.roll
        disp_pitch = pkt.pitch - self.tare_offsets['pitch'] if self.tare_active else pkt.pitch
        disp_yaw = pkt.yaw - self.tare_offsets['yaw'] if self.tare_active else pkt.yaw
        disp_alt = pkt.alt_m - self.tare_offsets['alt_m'] if self.tare_active else pkt.alt_m

        if self.stream_t0 is None:
            self.stream_t0 = pkt.ts
        plot_t = pkt.ts - self.stream_t0

        self.alt_x.append(plot_t)
        self.alt_y.append(disp_alt)
        self.spd_x.append(plot_t)
        self.spd_y.append(pkt.spd_mps)

        # Keep only a live sliding window in the plotted data.
        # Full-resolution history is still preserved by CSV logging and buffer_rows.
        cutoff_t = plot_t - CHART_WINDOW_SEC
        if cutoff_t > 0:
            while self.alt_x and self.alt_x[0] < cutoff_t:
                self.alt_x.pop(0)
                self.alt_y.pop(0)
            while self.spd_x and self.spd_x[0] < cutoff_t:
                self.spd_x.pop(0)
                self.spd_y.pop(0)

        # Hard cap as a safety net for very high packet rates.
        if len(self.alt_x) > MAX_POINTS:
            del self.alt_x[:-MAX_POINTS]
            del self.alt_y[:-MAX_POINTS]
        if len(self.spd_x) > MAX_POINTS:
            del self.spd_x[:-MAX_POINTS]
            del self.spd_y[:-MAX_POINTS]

        row = {
            'ts': pkt.ts,
            'roll_raw': pkt.roll,
            'pitch_raw': pkt.pitch,
            'yaw_raw': pkt.yaw,
            'alt_m_raw': pkt.alt_m,
            'roll_tared': disp_roll,
            'pitch_tared': disp_pitch,
            'yaw_tared': disp_yaw,
            'alt_m_tared': disp_alt,
            'spd_mps': pkt.spd_mps,
            'lat': pkt.lat,
            'lon': pkt.lon,
            'rssi_dbm': pkt.rssi_dbm,
            'rate_hz': pkt.rate_hz,
            'server_rx_ts': pkt.server_rx_ts,
        }

        if store_buffer:
            self.buffer_rows.append(row)

        if log and self.csv_writer is not None:
            self.csv_writer.writerow([
                row['ts'], row['roll_raw'], row['pitch_raw'], row['yaw_raw'], row['alt_m_raw'],
                row['roll_tared'], row['pitch_tared'], row['yaw_tared'], row['alt_m_tared'],
                row['spd_mps'], row['lat'], row['lon'], row['rssi_dbm'], row['rate_hz'], row['server_rx_ts']
            ])
            # Flush periodically enough to be safe without murdering FPS.
            if self.csv_file is not None and (time.time() - self.last_readout_draw) > 0.25:
                self.csv_file.flush()

        now = time.time()

        # Lightweight readouts.
        if now - self.last_readout_draw >= self.readout_draw_period:
            self.rate_tile.set(f'{pkt.rate_hz:.2f} Hz')
            self.rssi_tile.set(f'{pkt.rssi_dbm:.0f} dBm')
            self.coords_tile.set(f'{pkt.lat:.6f}, {pkt.lon:.6f}')
            age_ms = max(0.0, (now - (pkt.server_rx_ts or pkt.ts)) * 1000.0)
            self.age_tile.set(f'{age_ms:.0f} ms')
            self.roll_var.set(f'{disp_roll:.1f}°')
            self.pitch_var.set(f'{disp_pitch:.1f}°')
            self.yaw_var.set(f'{disp_yaw:.1f}°')
            self.last_readout_draw = now

        # Chart redraws are the expensive part. Do not redraw matplotlib per packet.
        if now - self.last_chart_draw >= self.chart_draw_period:
            self.alt_ax.clear()
            self._style_chart(self.alt_fig, self.alt_ax, 'Altitude (m)')
            self.alt_ax.plot(self.alt_x, self.alt_y, linewidth=2.0, color=THEMES[self.theme_name]['accent'])
            if self.alt_x:
                self.alt_ax.set_xlim(max(0.0, plot_t - CHART_WINDOW_SEC), max(CHART_WINDOW_SEC, plot_t))
            self.alt_canvas.draw_idle()

            self.spd_ax.clear()
            self._style_chart(self.spd_fig, self.spd_ax, 'Speed (m/s)')
            self.spd_ax.plot(self.spd_x, self.spd_y, linewidth=2.0, color=THEMES[self.theme_name]['accent'])
            if self.spd_x:
                self.spd_ax.set_xlim(max(0.0, plot_t - CHART_WINDOW_SEC), max(CHART_WINDOW_SEC, plot_t))
            self.spd_canvas.draw_idle()
            self.last_chart_draw = now

        # Rocket body-axis view and attitude indicator.
        if now - self.last_attitude_draw >= self.attitude_draw_period:
            self._redraw_attitude(disp_roll, disp_pitch, disp_yaw)
            self.last_attitude_draw = now

        # Map draws are also costly; append all data but redraw at a sane FPS.
        if store_buffer and (now - self.last_map_draw >= self.map_draw_period):
            self.map_canvas.add_point(pkt.lat, pkt.lon)
            self.last_map_draw = now

    def _redraw_attitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        t = THEMES[self.theme_name]
        for canvas in (self.att3d, self.horizon):
            canvas.configure(bg=t['horizon_bg'])
            canvas.delete('all')

        # Rocket body-axis view.
        # This is intentionally a clear 2D/2.5D attitude cue, not a fake physics renderer.
        w = max(10, self.att3d.winfo_width())
        h = max(10, self.att3d.winfo_height())
        cx, cy = w / 2, h / 2
        scale = min(w, h) * 0.22
        roll = math.radians(roll_deg)
        yaw = math.radians(yaw_deg)
        pitch = max(-1.2, min(1.2, pitch_deg / 70.0))

        for gx in range(0, w, 28):
            self.att3d.create_line(gx, 0, gx, h, fill=t['grid'])
        for gy in range(0, h, 28):
            self.att3d.create_line(0, gy, w, gy, fill=t['grid'])

        def project(x, y):
            # Rotate rocket model 90 deg counter-clockwise on screen:
            # +X/nose points upward, matching a rocket body-camera style view.
            x, y = y, -x
            # rotate by roll, add a small yaw skew and pitch vertical bias
            xr = x * math.cos(roll) - y * math.sin(roll)
            yr = x * math.sin(roll) + y * math.cos(roll)
            xr2 = xr * math.cos(yaw * 0.12) - yr * math.sin(yaw * 0.12)
            yr2 = xr * math.sin(yaw * 0.12) + yr * math.cos(yaw * 0.12)
            return cx + xr2 * scale, cy + (yr2 + pitch) * scale

        # Rocket outline: nose points along +X body axis.
        rocket_pts = [
            (2.35, 0.0),      # nose
            (1.55, -0.36),
            (-1.45, -0.36),
            (-1.85, -0.75),   # lower fin
            (-1.65, -0.32),
            (-2.15, -0.22),   # nozzle lower
            (-2.15, 0.22),    # nozzle upper
            (-1.65, 0.32),
            (-1.85, 0.75),    # upper fin
            (-1.45, 0.36),
            (1.55, 0.36),
        ]
        body = []
        for x, y in rocket_pts:
            body.extend(project(x, y))

        self.att3d.create_polygon(body, fill=t['accent'], outline=t['line'], width=2)
        self.att3d.create_text(10, 10, anchor='nw', text='Rocket Body-Axis View (+X/Nose Up)', fill=t['muted'], font=('Segoe UI', 10, 'bold'))

        # Body axes.
        x0, y0 = project(0, 0)
        xx, xy = project(2.8, 0)
        yx, yy = project(0, -1.45)
        zx, zy = project(0.75, 1.15)

        self.att3d.create_line(x0, y0, xx, xy, fill=t['green'], width=3, arrow=tk.LAST)
        self.att3d.create_line(x0, y0, yx, yy, fill=t['red'], width=3, arrow=tk.LAST)
        self.att3d.create_line(x0, y0, zx, zy, fill=t['warn'], width=3, arrow=tk.LAST)

        self.att3d.create_text(xx + 12, xy, text='+X Nose', fill=t['green'], anchor='w', font=('Segoe UI', 9, 'bold'))
        self.att3d.create_text(yx, yy - 10, text='+Y', fill=t['red'], anchor='s', font=('Segoe UI', 9, 'bold'))
        self.att3d.create_text(zx + 8, zy + 8, text='+Z', fill=t['warn'], anchor='nw', font=('Segoe UI', 9, 'bold'))

        # Center reference dot.
        self.att3d.create_oval(x0 - 4, y0 - 4, x0 + 4, y0 + 4, fill=t['line'], outline='')

        # Artificial horizon.
        w = max(10, self.horizon.winfo_width())
        h = max(10, self.horizon.winfo_height())
        cx, cy = w / 2, h / 2
        pitch_px_per_deg = h / 120.0
        y_shift = pitch_deg * pitch_px_per_deg
        # Build a huge rotated sky/ground split.
        corners = [(-w, -h*2 + y_shift), (w, -h*2 + y_shift), (w, y_shift), (-w, y_shift)]
        ground = [(-w, y_shift), (w, y_shift), (w, h*2 + y_shift), (-w, h*2 + y_shift)]
        self._draw_rotated_polygon(self.horizon, corners, roll, cx, cy, fill=t['sky'])
        self._draw_rotated_polygon(self.horizon, ground, roll, cx, cy, fill=t['ground'])
        self._draw_rotated_line(self.horizon, (-w, y_shift), (w, y_shift), roll, cx, cy, fill=t['line'], width=3)
        for p in range(-60, 61, 10):
            if p == 0:
                continue
            yy = y_shift + p * pitch_px_per_deg
            half = w * (0.20 if abs(p) % 30 == 0 else 0.12)
            self._draw_rotated_line(self.horizon, (-half, yy), (half, yy), roll, cx, cy, fill=t['line'], width=2)
        self.horizon.create_line(cx - w*0.18, cy, cx + w*0.18, cy, fill=t['line'], width=3)
        self.horizon.create_line(cx, cy - 10, cx, cy + 10, fill=t['line'], width=3)
        self.horizon.create_text(10, 10, anchor='nw', text='Body/Nose View Attitude', fill=t['muted'], font=('Segoe UI', 10, 'bold'))

    def _draw_rotated_polygon(self, canvas, points, angle, cx, cy, **kwargs):
        out = []
        for x, y in points:
            xr = x * math.cos(-angle) - y * math.sin(-angle)
            yr = x * math.sin(-angle) + y * math.cos(-angle)
            out.extend((cx + xr, cy + yr))
        canvas.create_polygon(out, **kwargs)

    def _draw_rotated_line(self, canvas, p1, p2, angle, cx, cy, **kwargs):
        x1, y1 = p1
        x2, y2 = p2
        x1r = x1 * math.cos(-angle) - y1 * math.sin(-angle)
        y1r = x1 * math.sin(-angle) + y1 * math.cos(-angle)
        x2r = x2 * math.cos(-angle) - y2 * math.sin(-angle)
        y2r = x2 * math.sin(-angle) + y2 * math.cos(-angle)
        canvas.create_line(cx + x1r, cy + y1r, cx + x2r, cy + y2r, **kwargs)

    def on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == '__main__':
    app = TelemetryApp()
    app.mainloop()
