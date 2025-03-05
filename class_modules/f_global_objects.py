from colour_sensors import c_colour_sensors
from gyroscope import c_gyroscope
from laser_sensors import c_laser_sensor
from touch_sensors import c_touch_sensor
from motors import c_motors, c_claw

x_shut_pins = [17, 27, 22]
laser_sensors = [c_laser_sensor(x_shut_pin, 0x30 + i) for i, x_shut_pin in enumerate(x_shut_pins)]

stop_angles = [88, 89, 88, 89]
servo_pins = [14, 13, 12, 10]
motors = c_motors(stop_angles=stop_angles, servo_pins=servo_pins)
claw = c_claw(claw_pin=9)

colour_sensors = c_colour_sensors()
gyroscope = c_gyroscope(delay=0.01)

touch_pins = [22, 5]
touch_sensor = [c_touch_sensor(touch_pin) for touch_pin in touch_pins]