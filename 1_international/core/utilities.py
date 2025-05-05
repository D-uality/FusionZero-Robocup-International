from core.shared_imports import mp, cv2, np

# Print Function
def debug(data: list[str], coloumn_widths: list[int], separator: str = "|"):
    formatted_cells = [f"{cell:^{width}}" for cell, width in zip(data, coloumn_widths)]
    print(f" {separator} ".join(formatted_cells))

# Display Function

class DisplayManager():
    def __init__(self):
        self.__display_process = None
        self.__display_queue   = None

    def __display_worker(self, queue: mp.Queue):
        window_last_frame_time = {}
        
        while True:
            item = queue.get()
            
            if item is None: break
            
            if isinstance(item, tuple) and len(item) == 2:
                frame, window_name = item
                
                if isinstance(frame, np.ndarray):
                    cv2.imshow(window_name, frame)
                    cv2.waitKey(1)
                    window_last_frame_time[window_name] = True
                    
        cv2.destroyAllWindows()

    def start(self):
        if self.__display_process is None:
            self.__display_queue   = mp.Queue(maxsize=10)
            self.__display_process = mp.Process(target=self.__display_worker, args=(self.__display_queue,), daemon=True)
            
            self.__display_process.start()

    def show(self, frame: np.ndarray, name: str = "Display"):
        if self.__display_queue is not None and self.__display_queue.qsize() < 10:
            self.__display_queue.put((frame, name))

    def stop(self):
        if self.__display_queue:   self.__display_queue.put(None)
        if self.__display_process: self.__display_process.join()
        
        self.__display_process = None
        self.__display_queue = None
