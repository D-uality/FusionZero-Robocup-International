import VL53L1X

tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()

try:
    tof.start_ranging(1)

    while True:
        distance = tof.get_distance()

        print(distance)

except KeyboardInterrupt:
    print("Exiting Gracefully")

finally:
    tof.stop_ranging()