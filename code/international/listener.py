import threading

class C_MODE_LISTENER():
    def __init__(self, initial_mode: int = 0) -> None:
        self.mode = initial_mode
        self.mode_lock = threading.Lock()
        self.exit_event = threading.Event()
        self.listener_thread = threading.Thread(target=self._input_listener, daemon=True)
        
        
    def start(self) -> None:
        self.listener_thread.start()
        
    def _input_listener(self):
        valid_modes = {"0", "1", "9"}

        while not self.exit_event.is_set():
            print("[0] Nothing")
            print("[1] Line Follow only")
            print("[9] Exit program")
            mode = input("Enter mode: ")
            mode = int(mode) if mode in valid_modes else 0                
            
            with self.mode_lock: self.mode = mode
                
            if mode == 9:
                self.exit_event.set()
                break
    
    def get_mode(self) -> int:
        with self.mode_lock: return self.mode
        
    def reset_exit_event(self) -> None:
        self.exit_event.clear()
        
    def has_exited(self) -> bool:
        return self.exit_event.is_set()