import multiprocessing as mp
import time
from RPi import GPIO

class C_MODE_LISTENER():
    BUTTON_PIN = 22

    def __init__(self, initial_mode: int = 0) -> None:
        # shared state
        self.mode       = mp.Value("i", initial_mode)
        self.exit_event = mp.Event()

        # create—but don’t start—child processes
        self._process_console = mp.Process(target=self.__input_listener,  daemon=True)
        self._process_button  = mp.Process(target=self.__button_listener, daemon=True)
        
    def start(self) -> None:
        # Start the child processes
        self._process_console.start()
        self._process_button.start()

    def join(self) -> None:
        # Wait for the child processes to finish
        self._process_console.join()
        self._process_button.join()

    def stop(self) -> None:
        # Stop the child processes
        self.exit_event.set()
        self.join()

    def get_mode(self) -> int:
        return self.mode.value

    def has_exited(self) -> bool:
        return self.exit_event.is_set()

    def run(self) -> None:
        # Start the listener and wait for it to finish
        try:
            self.start()
        finally:
            self.stop()

    def __input_listener(self) -> None:
        valid = {"0", "1", "2", "9"}
        while not self.exit_event.is_set():
            print("[0] Nothing\n[1] Line Follow only\n[9] Exit program")
            mode = input("Enter mode: ")
            mode = int(mode) if mode in valid else 1
            self.mode.value = mode
            if mode == 9:
                self.exit_event.set()

    def __button_listener(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        start_time = time.perf_counter()

        while not self.exit_event.is_set():
            current_time = time.perf_counter()
            
            if current_time - start_time >= 1:
                start_time = current_time
                
                pressed = GPIO.input(self.BUTTON_PIN) == GPIO.LOW 
                self.mode.value = 1 if pressed else 0
