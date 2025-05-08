from core.shared_imports import mp, time, GPIO, Thread

class ModeListener():
    def __init__(self, initial_mode: int = 0) -> None:
        self.__button_pin = 22
        
        # shared state
        self.mode        = mp.Value("i", initial_mode)
        self.exit_event  = mp.Event()
        self.input_queue = mp.Queue()

        # create - but don’t start—child processes
        self.__process_console = mp.Process(target=self.__input_listener,  daemon=True)
        self.__process_button  = mp.Process(target=self.__button_listener, daemon=True)
        
    def start(self) -> None:
        # Start the child processes
        self.__process_console.start()
        self.__process_button.start()

    def stop(self) -> None:
        # Stop the child processes
        self.exit_event.set()

    def has_exited(self) -> bool:
        return self.exit_event.is_set()

    def run(self) -> None:
        # Start the listener and wait for it to finish
        try:
            self.__start_input_thread()
            self.start()
        finally:
            self.__stop()

    def __start_input_thread(self) -> None:
        # Start a thread in the main process to handle user input
        def input_thread() -> None:
            while not self.exit_event.is_set():
                print("[0] Nothing")
                print("[1] Line follow")
                print("[9] Exit program")
                
                mode = input("Enter mode: ")
                self.input_queue.put(mode)

        Thread(target=input_thread, daemon=True).start()

    def __input_listener(self) -> None:
        valid = {"0", "1", "2", "9"}

        while not self.exit_event.is_set():
            if not self.input_queue.empty():
                mode = self.input_queue.get()
                
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
