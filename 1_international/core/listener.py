from core.shared_imports import mp, time, GPIO, Thread

class ModeListener():
    def __init__(self, initial_mode: int = 0) -> None:
        self.__button_pin = 22
        
        self.mode        = mp.Value("i", initial_mode)
        self.exit_event  = mp.Event()
        self.input_queue = mp.Queue()

        self.__process_console = mp.Process(target=self.__input_listener,  daemon=True)
        self.__process_button  = mp.Process(target=self.__button_listener, daemon=True)
        
    def start(self) -> None:
        self.__process_console.start()
        self.__process_button.start()

    def has_exited(self) -> bool:
        return self.exit_event.is_set()

    def run(self) -> None:
        self.__start_input_thread()
        self.start()

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
        try:
            valid = {"0", "1", "2", "3", "9"}
            while not self.exit_event.is_set():
                if not self.input_queue.empty():
                    mode = self.input_queue.get()
                    
                    mode = int(mode) if mode in valid else 1                    
                    self.mode.value = mode
                    
                    if mode == 9: self.exit_event.set()
                    
        except KeyboardInterrupt:
            pass

    def __button_listener(self) -> None:
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.__button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            prev_pressed = (GPIO.input(self.__button_pin) == GPIO.LOW)

            while not self.exit_event.is_set():
                pressed = (GPIO.input(self.__button_pin) == GPIO.LOW)
                
                if pressed != prev_pressed:
                    self.mode.value = 1 if pressed else 0
                    prev_pressed = pressed

                time.sleep(0.05)
        
        except KeyboardInterrupt:
            pass