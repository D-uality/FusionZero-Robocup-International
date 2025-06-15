from core.shared_imports import mp, cv2, np, time, os
import shutil
import subprocess

# Print Function
def debug(data: list[str], coloumn_widths: list[int], separator: str = "|") -> None:
    formatted_cells = [f"{cell:^{width}}" for cell, width in zip(data, coloumn_widths)]
    print(f" {separator} ".join(formatted_cells))

# Global variables for display and frame storage
_display_process = None
_display_queue = None
_manager = mp.Manager()
_saved_frames = _manager.list()  # Stores (frame, timestamp) tuples

def _display_worker(queue: mp.Queue):
    window_last_frame_time = {}
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
                    print(f"[FPS] {frame_count} frames/sec")
                    frame_count = 0
                    last_time = current_time

    cv2.destroyAllWindows()
    
def start_display():
    global _display_process, _display_queue
    if _display_process is None:
        _display_queue = mp.Queue(maxsize=10)
        _display_process = mp.Process(target=_display_worker, args=(_display_queue,), daemon=True)
        _display_process.start()

def show(frame: np.ndarray, display, name: str = "Display"):
    global _display_queue
    if _display_queue is not None and display:
        # Clear queue to prioritize the latest frame
        while not _display_queue.empty():
            try:
                _display_queue.get_nowait()
            except:
                break
        _display_queue.put((frame, name))
    
    # Save EVERY frame with its timestamp
    _saved_frames.append((frame.copy(), time.perf_counter()))

def stop_display():
    global _display_process, _display_queue
    if _display_queue:
        _display_queue.put(None)
    if _display_process:
        _display_process.join()
    _display_process = None
    _display_queue = None

def get_saved_frames():
    return _saved_frames  # Returns list of (frame, timestamp) tuples

def save_vfr_video(frames_with_timestamps: list[tuple[np.ndarray, float]], filename: str = "output_vfr.mp4"):
    if not frames_with_timestamps:
        print("[save_vfr_video] No frames to save.")
        return

    # Step 1: Write frames to temporary directory
    temp_dir = "temp_frames"
    os.makedirs(temp_dir, exist_ok=True)
    
    for i, (frame, _) in enumerate(frames_with_timestamps):
        cv2.imwrite(f"{temp_dir}/frame_{i:04d}.png", frame)

    # Step 2: Generate timestamps.txt for FFmpeg
    with open("timestamps.txt", "w") as f:
        for i in range(len(frames_with_timestamps) - 1):
            frame, timestamp = frames_with_timestamps[i]
            next_timestamp = frames_with_timestamps[i + 1][1]
            duration = next_timestamp - timestamp
            f.write(f"file '{temp_dir}/frame_{i:04d}.png'\n")
            f.write(f"duration {duration:.6f}\n")
        # Last frame (no duration)
        f.write(f"file '{temp_dir}/frame_{len(frames_with_timestamps)-1:04d}.png'\n")

    name, ext = os.path.splitext(filename)
    counter = 0
    while True:
        new_name = f"{name}_{counter}{ext}"
        if not os.path.exists(new_name):
            filename = new_name
            break
        counter += 1

    # Step 3: Run FFmpeg to create VFR video
    ffmpeg_cmd = [
        "ffmpeg",
        "-f", "concat",
        "-i", "timestamps.txt",
        "-vsync", "vfr",
        "-pix_fmt", "yuv420p",
        "-c:v", "libx264",
        "-crf", "18",
        filename
    ]
    subprocess.run(ffmpeg_cmd, check=True)

    # Cleanup
    shutil.rmtree(temp_dir)
    os.remove("timestamps.txt")
    print(f"[save_vfr_video] VFR video saved to {filename}")