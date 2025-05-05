from core.shared_imports import GPIO, time
start_time = time.perf_counter()
from core.listener import ModeListener
from core.utilities import debug, DisplayManager

listener = ModeListener()
display_manager = DisplayManager()

def main() -> None:
    try:
        listener.run()
        display_manager.start()
        
        debug( ["INITIALISATION", f"{time.perf_counter() - start_time}"], [24, 50] )
        
        while listener.has_exited() == False:
            if listener.mode == 9:
                listener.stop()
                
            elif listener.mode == 1:
                
                
        
    finally:
        GPIO.cleanup()

if __name__ == "main": main()