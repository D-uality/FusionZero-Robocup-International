import motors

def main():
    while True:
        v1, v2 = map(int, input("[v1 v2]: ").split())

        motors.run_uphill(v1, v2)
