import threading
import RPi.GPIO as GPIO
import time

class C_MODE_LISTENER():
    def __init__(self, initial_mode: int = 0) -> None:
        self.button_pin = 22

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        self.mode = initial_mode
        self.mode_lock = threading.Lock()
        self.exit_event = threading.Event()
        self.listener_thread = threading.Thread(target=self._input_listener, daemon=True)
        # self.listener_thread = threading.Thread(target=self._button_listener, daemon=True)
        
    def start(self) -> None:
        self.listener_thread.start()
        
    def _input_listener(self):
        valid_modes = {"0", "1", "2", "9"}

        while not self.exit_event.is_set():
            print("[0] Nothing")
            print("[1] Line Follow")
            print("[9] Exit program")
            mode = input("Enter mode: ")
            mode = int(mode) if mode in valid_modes else 1                
            
            with self.mode_lock: self.mode = mode
                
            if mode == 9:
                self.exit_event.set()
                break

    # def _button_listener(self) -> None:
    #     start_time = time.perf_counter()

    #     while not self.exit_event.is_set():
    #         if time.perf_counter() - start_time > 0.5:
    #             start_time = time.perf_counter()

    #             print(GPIO.input(self.button_pin))

    #             if GPIO.input(self.button_pin) == GPIO.LOW:
    #                 with self.mode_lock: self.mode = 1
    #             else:
    #                 with self.mode_lock: self.mode = 0
    
    def get_mode(self) -> int:
        with self.mode_lock: return self.mode
        
    def reset_exit_event(self) -> None:
        self.exit_event.clear()
        
    def has_exited(self) -> bool:
        return self.exit_event.is_set()