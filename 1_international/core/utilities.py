from core.shared_imports import mp, cv2, np, time, os, socket, getpass, subprocess, shutil

username = getpass.getuser()
hostname = socket.gethostname()
user_at_host = f"{username}@{hostname}"

# =========================
# Session / Paths (SD card)
# =========================

SESSION_ROOT = "sessions"  # parent folder on SD (relative to cwd)
SESSION_DIR = None         # set by _init_session()
META_PATH = None           # timestamps.csv path
_meta_file = None          # csv file handle
_session_inited = False

def _init_session():
    """
    Create a new session folder like sessions/2025-10-04_19-42-31/
    and a timestamps.csv file inside it.
    """
    global SESSION_DIR, META_PATH, _meta_file, _session_inited

    if _session_inited:
        return

    os.makedirs(SESSION_ROOT, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
    SESSION_DIR = os.path.join(SESSION_ROOT, stamp)
    os.makedirs(SESSION_DIR, exist_ok=True)

    META_PATH = os.path.join(SESSION_DIR, "timestamps.csv")
    _meta_file = open(META_PATH, "w", buffering=1)  # line-buffered
    _meta_file.write("timestamp,filename\n")
    _session_inited = True

def _close_session():
    global _meta_file
    try:
        if _meta_file is not None:
            _meta_file.flush()
            _meta_file.close()
    except Exception:
        pass
    _meta_file = None


# =========================
# Pi Health (CPU/MEM/TEMP)
# =========================

_health_state = {
    "last_timestamp": 0.0,
    "prev_idle": None,
    "prev_total": None,
    "cpu": None,
    "ram": None,
    "temp": None,
}

def read_cpu_percent() -> float:
    with open("/proc/stat", "r") as f:
        fields = f.readline().split()[1:8]
    user, nice, system, idle, iowait, irq, softirq = map(int, fields)
    idle_all = idle + iowait
    non_idle = user + nice + system + irq + softirq
    total = idle_all + non_idle

    prev_idle = _health_state["prev_idle"]
    prev_total = _health_state["prev_total"]
    _health_state["prev_idle"] = idle_all
    _health_state["prev_total"] = total

    if prev_idle is None or prev_total is None:
        return 0.0

    delta_idle = idle_all - prev_idle
    delta_total = total - prev_total
    if delta_total <= 0:
        return 0.0

    return int(100 * (1.0 - (delta_idle / delta_total)))

def read_mem_used_total_mb() -> tuple[int, int]:
    mem_total_kb = None
    mem_available_kb = None
    with open("/proc/meminfo", "r") as f:
        for line in f:
            if line.startswith("MemTotal:"):
                mem_total_kb = int(line.split()[1])
            elif line.startswith("MemAvailable:"):
                mem_available_kb = int(line.split()[1])
            if mem_total_kb is not None and mem_available_kb is not None:
                break
    used_mb = (mem_total_kb - mem_available_kb) // 1024
    total_mb = mem_total_kb // 1024
    used_gb  = used_mb / 1024.0
    total_gb = total_mb / 1024.0
    return used_gb, total_gb

def read_temp_celsius() -> float:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return int(f.read()) // 1000
    except Exception:
        return 0.0

def health_tick(debug_list: list[str], every_seconds: float = 0.5) -> None:
    """
    Append a health line every call. If less than `every_seconds` since last
    update, repeat the previous reading instead of skipping.
    """
    now = time.perf_counter()
    # compute a fresh reading if it's time OR if we have none yet
    if _health_state["cpu"] is None or (now - _health_state["last_timestamp"] >= every_seconds):
        _health_state["last_timestamp"] = now
        _health_state["cpu"] = read_cpu_percent()
        _health_state["ram"] = read_mem_used_total_mb()
        _health_state["temp"] = read_temp_celsius()

    # always append the cached line
    debug_list.append(f"CPU: {_health_state['cpu']}%")
    debug_list.append(f"RAM: {_health_state['ram'][0]:0.1f}/{_health_state['ram'][1]:0.1f}GB")
    debug_list.append(f"TEMP: {_health_state['temp']}°C")


# =========================
# Printing / Display
# =========================

# Fixed widths for debug lines: [TRIGGERS, GREEN, TURN, INT, FPS, CPU, MEM, TEMP]
DEBUG_FIXED_WIDTHS = [11, 19, 11, 15, 11, 11, 18, 12]

# Print Functions
def debug(data: list[str], coloumn_widths: list[int], separator: str = "|") -> None:
    formatted_cells = [f"{cell:^{width}}" for cell, width in zip(data, coloumn_widths)]
    print(f" {separator} ".join(formatted_cells))

def debug_lines(entries: list[str], padding: int = 2, separator: str = "|") -> None:
    """
    Fixed-only renderer:
      - First N columns use DEBUG_FIXED_WIDTHS.
      - Extra columns auto-size to len(cell) + padding.
    """
    if not entries:
        return

    n_fixed = len(DEBUG_FIXED_WIDTHS)
    if len(entries) <= n_fixed:
        col_widths = DEBUG_FIXED_WIDTHS[:len(entries)]
    else:
        fixed = DEBUG_FIXED_WIDTHS[:]
        extras = entries[n_fixed:]
        extra_widths = [max(1, len(str(e)) + padding) for e in extras]
        col_widths = fixed + extra_widths

    formatted = [f"{str(cell):^{width}}" for cell, width in zip(entries, col_widths)]
    print(f" {separator} ".join(formatted))


# =========================
# Display & Saving (to SD)
# =========================

_display_process = None
_display_queue = None

# Keep a small in-RAM index of what we saved: list of (filepath, timestamp)
_saved_frames = []  # simple list; tiny memory use

def _display_worker(queue: mp.Queue):
    frame_count = 0
    last_time = time.time()
    while True:
        item = queue.get()
        if item is None:
            break
        if isinstance(item, tuple) and len(item) == 2:
            frame, window_name = item
            if isinstance(frame, np.ndarray):
                cv2.imshow(window_name, frame)
                cv2.waitKey(1)
                frame_count += 1

                current_time = time.time()
                elapsed = current_time - last_time
                if elapsed >= 1.0:
                    # print(f"[FPS] {frame_count} frames/sec")
                    frame_count = 0
                    last_time = current_time
    cv2.destroyAllWindows()

def get_session_dir():
    return SESSION_DIR


def start_display():
    global _display_process, _display_queue
    # ensure session folder + timestamps.csv exist
    _init_session()

    if _display_process is None:
        _display_queue = mp.Queue(maxsize=1)
        _display_process = mp.Process(target=_display_worker, args=(_display_queue,), daemon=True)
        _display_process.start()

def put_text_on_image(image, debug_lines: list[str]):
    origin=[10, 20]
    line_height=20
    font_scale=0.6
    color=(0, 0, 255)
    thickness=1
    font = cv2.FONT_HERSHEY_SIMPLEX
    for i, line in enumerate(debug_lines):
        y = origin[1] + i * line_height
        cv2.putText(
            image,
            line,
            (origin[0], y),
            font,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA
        )

def show(frame: np.ndarray, name: str = "Display", display: bool = True, debug_lines: list[str] = None):
    """
    - Renders optional debug overlay
    - Shows frame (async display process)
    - Saves EVERY frame directly to SD as .npy
    - Appends a tiny (path, timestamp) record to _saved_frames and timestamps.csv
    """
    global _display_queue, _meta_file

    if debug_lines is not None:
        put_text_on_image(frame, debug_lines)

    frame = np.uint8(frame)

    # async UI display
    if _display_queue is not None and display:
        while not _display_queue.empty():
            try:
                _display_queue.get_nowait()
            except:
                break
        _display_queue.put((frame, name))

    # save to SD (lossless, low CPU)
    ts = time.perf_counter()
    fname = f"f_{int(ts*1e6):016d}.npy"
    fpath = os.path.join(SESSION_DIR, fname)
    np.save(fpath, frame)  # synchronous write (simple & reliable)

    # tiny RAM index + timestamps.csv entry
    _saved_frames.append((fpath, ts))
    if _meta_file is not None:
        try:
            _meta_file.write(f"{ts:.6f},{fname}\n")
        except Exception:
            pass

def stop_display():
    global _display_process, _display_queue
    if _display_queue:
        _display_queue.put(None)
    if _display_process:
        _display_process.join()
    _display_process = None
    _display_queue = None
    _close_session()

def get_saved_frames():
    """
    Returns the in-RAM index of saved files: list of (filepath, timestamp)
    """
    return _saved_frames