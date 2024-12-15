import smbus2
import time

bus = smbus2.SMBus(1)
ADC_ADDRESS = 0x4a

def read_adc(channel):
    command = 0x40 | (channel << 4)
    bus.write_byte(ADC_ADDRESS, command)
    time.sleep(0.001)
    return bus.read_byte(ADC_ADDRESS)

try:
    while True:
        adc_values = []
        for ch in range(8):
            adc_values.append(read_adc(ch))
            time.sleep(0.01)
        print(", ".join([f"adc{idx} = {val}" for idx, val in enumerate(adc_values)]))
except KeyboardInterrupt:
    print("\nExiting...")
