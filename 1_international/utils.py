# Print Function
def debug(data: list[str], coloumn_widths: list[int], separator: str = "|"):
    formatted_cells = [f"{cell:^{width}}" for cell, width in zip(data, coloumn_widths)]
    print(f" {separator} ".join(formatted_cells))

# Display Function
import cv2
import multiprocessing as mp
import numpy as np

_display_process = None
_display_queue = None

def _display_worker(queue: mp.Queue):
    window_last_frame_time = {}

    while True:
        item = queue.get()
        if item is None:
            break
        if isinstance(item, tuple) and len(item) == 2:
            frame, window_name = item
            if isinstance(frame, np.ndarray):
                cv2.imshow(window_name, frame)
                cv2.waitKey(1)
                window_last_frame_time[window_name] = True

    cv2.destroyAllWindows()

def start_display():
    global _display_process, _display_queue
    if _display_process is None:
        _display_queue = mp.Queue(maxsize=10)
        _display_process = mp.Process(target=_display_worker, args=(_display_queue,), daemon=True)
        _display_process.start()

def show(frame: np.ndarray, name: str = "Display"):
    global _display_queue
    if _display_queue is not None and _display_queue.qsize() < 10:
        _display_queue.put((frame, name))

def stop_display():
    global _display_process, _display_queue
    if _display_queue:
        _display_queue.put(None)
    if _display_process:
        _display_process.join()
    _display_process = None
    _display_queue = None
