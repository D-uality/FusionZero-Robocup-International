from core.shared_imports import GPIO, board, time, busio, digitalio, adafruit_vl53l1x, threading, traceback, Optional
from core.utilities   import debug
from hardware.motors  import Motors

class LaserSensors:
    _NEW_ADDRS = (0x30, 0x31)              # third stays 0x29

    def __init__(self, motors=None, *, xshut_pins=(board.D23, board.D24, board.D25),
                 timing_budget_ms=20, inter_measurement_ms=25, poll_delay=0.001,
                 max_none=5, enable_watchdog=True):

        # ───────── shared state
        self._motors  = motors
        self._wd_on   = enable_watchdog and motors is not None
        self._max_none, self._poll = max_none, poll_delay
        self._latest  = [None, None, None]
        self._nonecnt = [0, 0, 0]
        self._paused  = False
        self._alive   = True
        self._lock    = threading.Lock()

        # ───────── sensor boot
        self.i2c = busio.I2C(board.SCL, board.SDA)                    # :contentReference[oaicite:3]{index=3}
        self._xshut = [digitalio.DigitalInOut(p) for p in xshut_pins]
        for p in self._xshut: p.switch_to_output(False)               # hold reset
        self._sns = []
        for i, p in enumerate(self._xshut):
            p.value = True; time.sleep(0.12)                          # ≥100 ms boot :contentReference[oaicite:4]{index=4}
            s = adafruit_vl53l1x.VL53L1X(self.i2c)
            if i < len(self._NEW_ADDRS): s.set_address(self._NEW_ADDRS[i])
            s.timing_budget = timing_budget_ms                        # accuracy-speed table :contentReference[oaicite:5]{index=5}
            s.inter_measurement = inter_measurement_ms
            s.start_ranging()                                         # continuous mode
            self._sns.append(s)

        # ───────── worker thread
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

        debug( ["INITIALISATION", "LASER SENSORS", "✓"], [25, 25, 50] )

    # ----------------------------- worker
    def _worker(self):
        while self._alive:
            all_ok = True
            for i, s in enumerate(self._sns):
                try:
                    if s.data_ready:
                        d = s.distance
                        s.clear_interrupt()
                    else:
                        d = None
                except OSError:            # e.g. bus closed / sensor off
                    d = None
                except Exception:
                    traceback.print_exc()
                    d = None

                with self._lock:
                    if d is not None:
                        self._latest[i] = d
                        self._nonecnt[i] = 0
                    else:
                        self._nonecnt[i] += 1
                    if self._nonecnt[i] > self._max_none:
                        all_ok = False

            # optional watchdog
            if self._wd_on:
                with self._lock:
                    if not self._paused and not all_ok:
                        self._motors.run(0, 0)
                        self._paused = True
                    elif self._paused and all_ok:
                        self._paused = False
            time.sleep(self._poll)

    # ----------------------------- public read
    def read(self, idx=None):
        with self._lock:
            snap = self._latest.copy()
        if idx is None:       return snap
        if isinstance(idx,int): return snap[idx]
        return [snap[i] for i in idx]

    # ----------------------------- graceful shutdown
    def close(self):
        """Stop thread, power-down sensors, release GPIO & I²C."""
        self._alive = False
        self._thread.join(timeout=1.0)               # wait worker out :contentReference[oaicite:6]{index=6}

        for s in self._sns:          # stop driver first (thread already dead)
            try: s.stop_ranging()                     # API call :contentReference[oaicite:7]{index=7}
            except Exception: pass

        for p in self._xshut:        # then cut power
            try: p.switch_to_output(value=False)      # LOW = shutdown :contentReference[oaicite:8]{index=8}
            except Exception: pass
            try: p.deinit()
            except Exception: pass

        try: self.i2c.deinit()                        # releases /dev/i2c-1 :contentReference[oaicite:9]{index=9}
        except Exception: pass

    # ----------------------------- context-manager
    def __enter__(self): return self
    def __exit__(self, *_): self.close()
