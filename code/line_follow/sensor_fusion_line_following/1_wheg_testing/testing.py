import motors

def main():
    while True:
        v1, v2 = map(int, input("[v1 v2]: ").split())

        motors.run_uphill(v1, v2)

def run_input():
    while True:
        values = input("[v1, v2]: ").split()
        
        if len(values) == 0: break
        
        v1, v2 = map(int, values)
        motors.run(v1, v2)