import RPi.GPIO as GPIO
import time
import subprocess

GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

running = False
main_process = None

try:
    while True:
        running = True if GPIO.input(27) == GPIO.LOW else False

        # If running is True and the competition script is not running, start it.
        if running and main_process is None:
            print("Starting competition_main.py...")
            # Launch the competition_main.py script
            main_process = subprocess.Popen(["python3", "competition_main.py"])
        
        # If running is False and the competition script is running, kill it.
        elif not running and main_process is not None:
            print("Terminating competition_main.py...")
            subprocess.call(["sudo", "kill", "-9", str(main_process.pid)])
            main_process = None

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
    if main_process is not None:
        subprocess.call(["sudo", "kill", "-9", str(main_process.pid)])
        
finally:
    GPIO.cleanup()
