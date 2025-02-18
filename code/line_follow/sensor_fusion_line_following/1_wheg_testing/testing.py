import motors

def run_input() -> None:
    while True:
        values = input("[v1, v2]: ").split()
        
        if len(values) == 0: break
        
        v1, v2 = map(int, values)
        motors.run(v1, v2)