import motors

def main():
    while True:
        v1, v2 = list(map(int, input("[v1 v2]: ").split()))

        motors.run(v1, v2)