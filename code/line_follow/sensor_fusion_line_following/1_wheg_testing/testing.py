import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
modules_dir = os.path.abspath(os.path.join(current_dir, 'modules'))
if modules_dir not in sys.path: sys.path.insert(0, modules_dir)

import config
import motors
import colour

def run_input() -> None:
    values = input("[v1, v2]: ").split()
    while True:        
        if len(values) == 0: break
        
        v1, v2 = map(int, values)
        motors.run(v1, v2)

        colour_values = colour.read()
        config.update_log(["TESTING", ", ".join(list(map(str, colour_values)))], [24, 30])
        print()

if __name__ == "__main__": run_input()