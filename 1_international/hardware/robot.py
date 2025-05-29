from hardware.laser_sensors import LaserSensors
from hardware.touch_sensors import TouchSensors
from hardware.motors import Motors
from hardware.camera import Camera
from hardware.colour_sensors import ColourSensors
from hardware.gyroscope_sensor import Gyroscope
from hardware.led import LED
from hardware.evacuation_camera import EvacuationCamera
from hardware.claw import Claw

motors = Motors()
laser_sensors = LaserSensors()
touch_sensors = TouchSensors()
camera = Camera()
colour_sensors = ColourSensors()
gyroscope = Gyroscope()
led = LED()
evac_camera = EvacuationCamera()
claw = Claw()