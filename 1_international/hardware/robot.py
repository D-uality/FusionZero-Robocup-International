from hardware.laser_sensors import LaserSensors
from hardware.touch_sensors import TouchSensors
from hardware.motors import Motors
from hardware.camera import Camera
from hardware.colour_sensors import ColourSensors
from hardware.gyroscope_sensor import Gyroscope
from hardware.led import LED
from hardware.evacuation_camera import EvacuationCamera
from hardware.claw import Claw
from hardware.silver_sensor import SilverSensor

camera = Camera()
motors = Motors(camera)
laser_sensors = LaserSensors(motors)
touch_sensors = TouchSensors()
colour_sensors = ColourSensors()
gyroscope = Gyroscope()
led = LED()
evac_camera = EvacuationCamera()
claw = Claw()
silver_sensor = SilverSensor()